/*
 * avr_controller.c
 * Modified: 21-May-2025  (telemetry via Timer-3 @ 50 Hz)
 * Author   : Endeavor360
 */

#include "config.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>
#include <stdbool.h>
#include <math.h>

#include "motors.h"
#include "m_usb.h"
#include "bno055_ll.h"
#include "analog.h"
#include "encoder.h"
#include "profiler.h"
#include "systime.h"

#define RX_BUF_SIZE 64

// Receive buffer
static char rx_buf[RX_BUF_SIZE];
static uint8_t rx_index = 0;

// Parsed parameters
static float rx_distance;     // mm
static float rx_angle;        // degrees
static uint16_t rx_max_vel;   // mm/s
static uint16_t rx_max_omega; // deg/s
static uint16_t rx_last_vel;
static uint16_t rx_last_omega;
static float rx_lin_acc; // mm/s
static float rx_ang_acc; // deg/s

uint8_t id;

typedef enum
{
    AUTONOMOUS = 0,
    TELEOPERATOR = 1,
    VELOCITY_STREAM = 2,
} controlMode;
static uint8_t control_mode = AUTONOMOUS;

typedef enum
{
    DEBUG_OFF = 0,
    MOTION_DEBUG = 1,
    RX_ECHO = 2,
    MD_AND_ECHO = 3,
} debugMode;
static uint8_t debug_mode = MOTION_DEBUG;

typedef enum
{
    CONSTANT = 0,
    DECELERATING = 1,
    ACCELERATING = 2,
    NONETELEOP = 3,
} teleOpStates;
static uint8_t teleStates = NONETELEOP;

static uint8_t last_command = 0;
static uint8_t current_command =0;

static bool f, b, l, r;

/* ---------- VELOCITY_STREAM state ---------- */
static float vs_target_lin  = 0.0f;   /* target linear vel  (mm/s)  */
static float vs_target_ang  = 0.0f;   /* target angular vel (deg/s) */
static float vs_current_lin = 0.0f;   /* ramped linear cmd  (mm/s)  */
static float vs_current_ang = 0.0f;   /* ramped angular cmd (deg/s) */
static uint64_t vs_last_cmd_us = 0;   /* timestamp of last valid cmd */

/* ---------- VELOCITY_STREAM helpers ---------- */
/* Ramp 'current' toward 'target' by at most 'max_step' per call. */
static float slew_rate_limit(float current, float target, float max_step)
{
    float diff = target - current;
    if (diff >  max_step) diff =  max_step;
    if (diff < -max_step) diff = -max_step;
    return current + diff;
}

static void vs_reset(void)
{
    vs_target_lin  = 0.0f;
    vs_target_ang  = 0.0f;
    vs_current_lin = 0.0f;
    vs_current_ang = 0.0f;
}

/* -------------------------------  PROTOTYPES ----------------------------*/
static void timer4_init(void);                            /* sets up periodic IRQ            */
static void send_telemetry(bool emerg, bool profileDone); /* heavy USB / sensor work         */
static void usb_send_ram(const char *s);
static uint8_t parse_jetson(const char *line);
static void receive_from_jetson(void);

static void send_debug(void);
static void send_cmd_echo(void);

/*  ------------------------------ GLOBAL FLAG ------------------------*/
static volatile uint8_t loop_execute = 0; /* set in ISR, cleared in main */

bool emerg = false;
bool profile_done = true;

typedef enum
{
    NONE = 0,
    FORWARD = 1,
    ROTATION = 2,
    BOTH = 3,
} MoveType;

MoveType determineFinishnes = NONE;

/* ====================================================*/
int main(void)
{
    /* ---- initialize everything ---- */
    _delay_ms(100);

    m_usb_init();
    systime_init();
    motors_init();
    encoder_init();
    analog_init();
    twi_init();

    _delay_ms(100);
    bno055_gpio_reset();
    _delay_ms(100);

    if (!bno055_init())
    {
        m_usb_tx_string("IMU Failed\r\n");
    }

    /* ---- start 125 Hz timer & enable global IRQs ---- */
    timer4_init(); /* Timer-3 compare-match every 8 ms        */
    sei();         /* global interrupt enable                 */

    /* ---------------- MAIN LOOP ---------------------- */
    motion_reset_drive_system(); // initialize the encoder counts, velocities, omegas, distances, angles to 0
    while (1)
    {
        /* ---------- Emergency Button press status ---------- */
        emerg = encoder_emergency_hit();

        if (loop_execute)
        {
            loop_execute = 0;

            receive_from_jetson();
            encoder_odometry_update();
            motion_update();

            // bool imu_ok = bno055_read8(0x00, &id) && (id == 0xA0);
            // if (imu_ok) bno055_gpio_reset();

            if (!emerg)
            {
                if (control_mode == AUTONOMOUS)
                {
                    if (determineFinishnes == BOTH && motion_turn_finished() && motion_move_finished())
                    {
                        profile_done = true;
                        motion_reset_drive_system();
                        determineFinishnes = NONE;
                    }
                    else if (determineFinishnes == FORWARD && motion_move_finished())
                    {
                        profile_done = true;
                        motion_reset_drive_system();
                        determineFinishnes = NONE;
                    }
                    else if (determineFinishnes == ROTATION && motion_turn_finished())
                    {
                        profile_done = true;
                        motion_reset_drive_system();
                        determineFinishnes = NONE;
                    }
                    else
                    {
                        motors_update(motion_velocity(), motion_omega());
                    }
                }
                else if (control_mode == TELEOPERATOR)
                {
                    if ((determineFinishnes == FORWARD && motion_move_finished()) ||  (determineFinishnes == ROTATION && motion_turn_finished()))
                    {
                        profile_done = true;

                        if (teleStates == ACCELERATING)
                        {
                            teleStates = CONSTANT;
                        }
                        else if (teleStates == DECELERATING)
                        {
                            teleStates = NONETELEOP;
							motion_reset_drive_system();
                        }
                    }

                    else
                    {
	                    motors_update(motion_velocity(), motion_omega());
                    }
                }
                else if (control_mode == VELOCITY_STREAM)
                {
                    /* ---- watchdog: check command freshness ---- */
                    uint64_t age_us = micros64() - vs_last_cmd_us;
                    bool cmd_fresh = (age_us < (uint64_t)VSTREAM_TIMEOUT_MS * 1000ULL);

                    if (!cmd_fresh)
                    {
                        /* stale command -> ramp targets to zero */
                        vs_target_lin = 0.0f;
                        vs_target_ang = 0.0f;
                    }

                    /* ---- slew-rate-limited ramping ---- */
                    float dt_s   = (float)LOOP_TIME * 0.001f;  /* 10 ms */
                    float lin_step = VSTREAM_LIN_ACC * dt_s;
                    float ang_step = VSTREAM_ANG_ACC * dt_s;

                    vs_current_lin = slew_rate_limit(vs_current_lin,
                                                     vs_target_lin, lin_step);
                    vs_current_ang = slew_rate_limit(vs_current_ang,
                                                     vs_target_ang, ang_step);

                    /* ---- drive motors ---- */
                    motors_update(vs_current_lin, vs_current_ang);

                    /* ---- if fully stopped after timeout, disable ---- */
                    if (!cmd_fresh
                        && fabsf(vs_current_lin) < 0.5f
                        && fabsf(vs_current_ang) < 0.5f)
                    {
                        vs_current_lin = 0.0f;
                        vs_current_ang = 0.0f;
                        motors_stop_all();
                    }
                }
            }
            else
            {
                motors_stop_all();
            }

            if (debug_mode == MOTION_DEBUG || debug_mode == MD_AND_ECHO)
            {
                send_debug();
            }

            send_telemetry(emerg, profile_done);
        }
    }
}

/* -------------------- TIMER-4 INITIALISATION (CTC, 50 Hz) --------------------
   16 MHz / 1024 = 15625 Hz
  15625 Hz × 0.01 s = 156  → OCR4A = 100 ⇒ 100Hz         */
static void timer4_init(void)
{
    /* reset all Timer-4 control registers (mandatory for this timer) */
    TCCR4A = 0;
    TCCR4B = 0;
    TCCR4C = 0;
    TCCR4D = 0; // set WGM40 and WGM41 to normal mode

    TCNT4 = 0;   /* start from zero                        */
    OCR4A = 156; /* compare after 156 counts (~10 ms)      */

    TIMSK4 |= _BV(OCIE4A); /* enable Compare-A interrupt             */

    /* start clock: prescaler = 1024 →  CS43 | CS41                        */
    TCCR4B |= _BV(CS43) | _BV(CS41) | _BV(CS40);
}

/* ----------------------- TIMER-4 COMPARE ISR --------------------------- */
ISR(TIMER4_COMPA_vect)
{
    TCNT4 = 0;        /* emulate CTC                */
    loop_execute = 1; /* signal main loop           */
}

/* ------------------- TELEMETRY SENDER (called from main) ----------------- */
static void send_telemetry(bool emerg, bool profileDone)
{
    char line[180];

    /* --- orientation --- */
    int16_t h16, r16, p16;
    bno055_get_euler(&h16, &r16, &p16);

    /* --- angular rate --- */
    int16_t gx16, gy16, gz16;
    bno055_get_omega(&gx16, &gy16, &gz16);

    /* --- linear acceleration --- */
    int16_t ax16, ay16, az16;
    bno055_get_accel(&ax16, &ay16, &az16);

    float h = h16 / 16.0f;
    float r = r16 / 16.0f;
    float p = p16 / 16.0f;

    const float wx = gx16 / 16.0f; /* °/s */
    const float wy = gy16 / 16.0f;
    const float wz = gz16 / 16.0f;

    const float ax = ax16 / 100.0f; /* m/s² */
    const float ay = ay16 / 100.0f;
    const float az = az16 / 100.0f;

    // uint8_t cal = bno055_is_fully_calibrated() ? 1u : 0u;

    /* ---------- ADC ---------- */
    uint16_t vbat_1 = analog_get_battery_1_mV();
    uint16_t vbat_2 = analog_get_battery_2_mV();
    uint16_t cliffL = analog_get_cliff_left();
    uint16_t cliffC = analog_get_cliff_front();
    uint16_t cliffR = analog_get_cliff_right();

    /* ---------- Encoders ---------- */
    int32_t encL = encoder_get_left();
    int32_t encR = encoder_get_right();

    /* ---------- Format & ship ---------- */
    /* Packet Structure: { Yaw Roll Pitch IMUOmega accrX accrY encoderLeft encoderRight bat1Voltage bat2Voltage LeftCliff CenterCliff RightCliff emergencyFlag }  */
    snprintf(line, sizeof(line),
             "%3.2f %3.2f %3.2f %+3.2f %+3.2f %+3.2f %+3.2f %+3.2f %+3.2f %10ld %10ld %u %u %u %u %u %u %u\r\n", h, r, p, wx, wy, wz, ax, ay, az, (long)encL, (long)encR, vbat_1, vbat_2, cliffL, cliffC, cliffR, emerg, profileDone);

    usb_send_ram(line);
    m_usb_tx_push();
}

// SpdL:%+6.1f SpdR: %+6.1f Vel: %+6.1f Omg: %+5.1f dist: %+8.1f ang: %+7.1f dt: %7.1f
static void send_debug(void)
{
    char line[128];

    /* grab data */
    const float sl = encoder_left_speed_mm_s();
    const float sr = encoder_right_speed_mm_s();
    const float v = encoder_robot_speed_mm_s();
    const float w = encoder_robot_omega_dps();
    const float d = encoder_robot_distance_mm();
    const float a = encoder_robot_angle_deg();
    const float dt = encoder_loop_time_us();

    snprintf(line, sizeof(line),
             "%+6.1f %+6.1f %+6.1f %+5.1f %+8.1f %+7.1f %7.1f  ",
             sl, sr, v, w, d, a, dt);

    usb_send_ram(line);
    m_usb_tx_push();
}

// just for debugging
static void send_cmd_echo(void)
{
    /* use the existing globals filled by parse_jetson_auto() */
    extern float rx_distance, rx_angle, rx_lin_acc, rx_ang_acc;
    extern uint16_t rx_max_vel, rx_max_omega,
        rx_last_vel, rx_last_omega;

    char buf[150];
    /* Format:  “CMD d=500.0 a=90.0 vmax=300 wmax=120 vend=0 wend=0 acc=500.0 aacc=360.0” */
    snprintf(buf, sizeof(buf),
             "CMD d=%g a=%g vmax=%u wmax=%u vend=%u wend=%u acc=%g aacc=%g\r\n",
             rx_distance, rx_angle,
             rx_max_vel, rx_max_omega,
             rx_last_vel, rx_last_omega,
             rx_lin_acc, rx_ang_acc);

    usb_send_ram(buf);
    m_usb_tx_push();
}

/* ------------------- Tiny helper ------------------------- */
static void usb_send_ram(const char *s)
{
    while (*s)
        m_usb_tx_char(*s++);
}

static uint8_t parse_jetson(const char *line)
{

    extern float rx_distance, rx_angle, rx_lin_acc, rx_ang_acc;
    extern uint16_t rx_max_vel, rx_max_omega, rx_last_vel, rx_last_omega;
    extern bool f, b, l, r;
    extern uint8_t control_mode, debug_mode;

    uint16_t temp1, temp2, temp3, temp4;

    // Note: "%f" for floats, "%u" for uint16_t on AVR
    int cnt = sscanf(line,
                     "%hhu,%hhu,%f,%f,%u,%u,%u,%u,%f,%f",
                     &control_mode,
                     &debug_mode,
                     &rx_distance,
                     &rx_angle,
                     &temp1,
                     &temp2,
                     &temp3,
                     &temp4,
                     &rx_lin_acc,
                     &rx_ang_acc);

    if (control_mode == AUTONOMOUS)
    {
        rx_max_vel = temp1;
        rx_max_omega = temp2;
        rx_last_vel = temp3;
        rx_last_omega = temp4;
    }
    else if (control_mode == TELEOPERATOR)
    {
        f = temp1;
        b = temp2;
        l = temp3;
        r = temp4;
    }
    else if (control_mode == VELOCITY_STREAM)
    {
        /* Re-parse with the simpler 4-field format:
           "2,<debug>,<lin_mmps>,<ang_dps>"             */
        float lin_f = 0.0f, ang_f = 0.0f;
        uint8_t cm_tmp, dm_tmp;
        int n = sscanf(line, "%hhu,%hhu,%f,%f",
                       &cm_tmp, &dm_tmp, &lin_f, &ang_f);
        if (n >= 4)
        {
            /* clamp to safe limits */
            if (lin_f >  VSTREAM_MAX_LIN_VEL) lin_f =  VSTREAM_MAX_LIN_VEL;
            if (lin_f < -VSTREAM_MAX_LIN_VEL) lin_f = -VSTREAM_MAX_LIN_VEL;
            if (ang_f >  VSTREAM_MAX_ANG_VEL) ang_f =  VSTREAM_MAX_ANG_VEL;
            if (ang_f < -VSTREAM_MAX_ANG_VEL) ang_f = -VSTREAM_MAX_ANG_VEL;

            vs_target_lin  = lin_f;
            vs_target_ang  = ang_f;
            vs_last_cmd_us = micros64();
        }
    }

    return cnt;
}

static void receive_from_jetson(void)
{
    
    while (m_usb_rx_available())
    {
        char c = m_usb_rx_char();
        // start parsing if new line detected
        if (c == '\n' || c == '\r')
        {
            if (rx_index > 0)
            {
                rx_buf[rx_index] = '\0';
                if (parse_jetson(rx_buf))
                {

                    if (debug_mode == RX_ECHO || debug_mode == MD_AND_ECHO)
                    {
                        send_cmd_echo();
                    }

                    if (control_mode == AUTONOMOUS)
                    {
                        vs_reset(); /* clear velocity-stream state on mode switch */

                        if (rx_distance != 0 && rx_angle != 0)
                        {
                            determineFinishnes = BOTH;
                            motion_reset_drive_system();
                            motion_start_move(rx_distance, rx_max_vel, rx_last_vel, rx_lin_acc);
                            motion_start_turn(rx_angle, rx_max_omega, rx_last_omega, rx_ang_acc);
                        }
                        else if (rx_distance != 0)
                        {
                            determineFinishnes = FORWARD;
                            motion_reset_drive_system();
                            motion_start_move(rx_distance, rx_max_vel, rx_last_vel, rx_lin_acc);
                        }
                        else if (rx_angle != 0)
                        {
                            determineFinishnes = ROTATION;
                            motion_reset_drive_system();
                            motion_start_turn(rx_angle, rx_max_omega, rx_last_omega, rx_ang_acc);
                        }
                        else
                        {
                            determineFinishnes = NONE;
                        }

                        profile_done = false;
                    }
                    else if (control_mode == TELEOPERATOR)
                    {
                        vs_reset(); /* clear velocity-stream state on mode switch */

                        current_command = (f << 3) | (b << 2) | (l << 1) | r;
						
						bool need_new_profile =
						(current_command && teleStates == NONETELEOP);

                        if (need_new_profile)
                        {
							teleStates = ACCELERATING;
                            motion_reset_drive_system();
							
                            if (f)
                            {
                                determineFinishnes = FORWARD;
                                motion_start_move(FORWARD_DIST, TELEOP_SPEED, TELEOP_SPEED, TELEOP_ACC);
                            }
                            else if (b)
                            {
                                determineFinishnes = FORWARD;
                                motion_start_move(-BACKWARD_DIST, TELEOP_SPEED, TELEOP_SPEED, TELEOP_ACC);
                            }
                            else if (l)
                            {
                                determineFinishnes = ROTATION;
                                motion_start_turn(LEFT_TURN_ANGLE, TELEOP_OMEGA, TELEOP_OMEGA, TELEOP_ALPHA);
                            }
                            else if (r)
                            {
                                determineFinishnes = ROTATION;
                                motion_start_turn(-RIGHT_TURN_ANGLE, TELEOP_OMEGA, TELEOP_OMEGA, TELEOP_ALPHA);
                            }

                            profile_done = false;
                        }
                        else if ((current_command != last_command) && ((teleStates != DECELERATING) && (teleStates != NONETELEOP)))
                        {
                            teleStates = DECELERATING;
                            motion_SOFT_reset_drive_system();

                            if (last_command & 0b1000)
                            {
                                determineFinishnes = FORWARD;
                                motion_start_move(FORWARD_DIST / 2, TELEOP_SPEED, 0, TELEOP_ACC);
                            }
                            else if (last_command & 0b0100)
                            {
                                determineFinishnes = FORWARD;
                                motion_start_move(-BACKWARD_DIST / 2, TELEOP_SPEED, 0, TELEOP_ACC);
                            }
                            else if (last_command & 0b0010)
                            {
                                determineFinishnes = ROTATION;
                                motion_start_turn(LEFT_TURN_ANGLE / 2, TELEOP_OMEGA, 0, TELEOP_ALPHA);
                            }
                            else if (last_command & 0b0001)
                            {
                                determineFinishnes = ROTATION;
                                motion_start_turn(-RIGHT_TURN_ANGLE / 2, TELEOP_OMEGA, 0, TELEOP_ALPHA);
                            }

                            profile_done = false;
                        }
                        else if ((current_command == last_command) && (teleStates == CONSTANT) && profile_done)
                        {
                            // Maintain constant speed smoothly
                            motors_update(motion_velocity(), motion_omega());
                        }

                        last_command = current_command;
                    }
                    else if (control_mode == VELOCITY_STREAM)
                    {
                        /* targets & timestamp already set in parse_jetson.
                           Ensure motors are enabled on entry.              */
                        motors_enable_all(true);
                    }
                }

                rx_index = 0;
            }
        }
        else if (rx_index < (RX_BUF_SIZE - 1))
        {
            rx_buf[rx_index++] = c;
        }
    }
}