/*
 * config.h
 *
 * Created: 5/6/2025 4:35:44 AM
 *  Author: Endeavor360
 */

#ifndef CONFIG_H_
#define CONFIG_H_

// MCU clock
#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <stdint.h>


//----------------------------------------------MOTORS--------------------------------------------------

// Steps per revolution (full-step mode)
#define STEPS_PER_REV 200U

/* ---------- LEFT motor (M1)  now on Timer-3 / OC3A (PC6 / Arduino D5) ---------- */
//  PUL- -> D5  = PC6 / OC3A
//  DIR- -> D12 = PD6
//  ENA- -> D10 = PB6   (active-HIGH in inverted logic)
#define LEFT_PUL_DDR   DDRC
#define LEFT_PUL_PORT  PORTC
#define LEFT_PUL_BIT   PC6   /* OC3A */

#define LEFT_DIR_DDR   DDRD
#define LEFT_DIR_PORT  PORTD
#define LEFT_DIR_BIT   PD6

#define LEFT_ENA_DDR   DDRB
#define LEFT_ENA_PORT  PORTB
#define LEFT_ENA_BIT   PB6

// Right motor pins (Arduino Leonardo) M2
//  PUL- -> D9  = PB5 / OC1A
//  DIR- -> D8  = PB4
//  ENA- -> D6 = PD7  (active High; explained above)
#define RIGHT_PUL_DDR   DDRB
#define RIGHT_PUL_PORT  PORTB
#define RIGHT_PUL_BIT   PB5

#define RIGHT_DIR_DDR   DDRD
#define RIGHT_DIR_PORT  PORTD
#define RIGHT_DIR_BIT   PD4

#define RIGHT_ENA_DDR   DDRD
#define RIGHT_ENA_PORT  PORTD
#define RIGHT_ENA_BIT   PD7  

#ifndef _BV //this is just to silence the shitty linter in microchip studio
#define _BV(bit) (1 << (bit))
#endif

/* ------------------- timer prescaler / clock divisors --------------------------- */
#define CLOCK_DIVISOR_TIMER1          1024U
#define PRE_SCALE_TIMER1       (_BV(CS12) | _BV(CS10))

#define CLOCK_DIVISOR_TIMER3   1024U             
#define PRE_SCALE_TIMER3       (_BV(CS32) | _BV(CS30))



//----------------------------------------------USB-CDC--------------------------------------------------
// constants corresponding to the various serial parameters
#define USB_SERIAL_DTR 0x01
#define USB_SERIAL_RTS 0x02
#define USB_SERIAL_1_STOP 0
#define USB_SERIAL_1_5_STOP 1
#define USB_SERIAL_2_STOP 2
#define USB_SERIAL_PARITY_NONE 0
#define USB_SERIAL_PARITY_ODD 1
#define USB_SERIAL_PARITY_EVEN 2
#define USB_SERIAL_PARITY_MARK 3
#define USB_SERIAL_PARITY_SPACE 4
#define USB_SERIAL_DCD 0x01
#define USB_SERIAL_DSR 0x02
#define USB_SERIAL_BREAK 0x04
#define USB_SERIAL_RI 0x08
#define USB_SERIAL_FRAME_ERR 0x10
#define USB_SERIAL_PARITY_ERR 0x20
#define USB_SERIAL_OVERRUN_ERR 0x40

#define EP_TYPE_CONTROL 0x00
#define EP_TYPE_BULK_IN 0x81
#define EP_TYPE_BULK_OUT 0x80
#define EP_TYPE_INTERRUPT_IN 0xC1
#define EP_TYPE_INTERRUPT_OUT 0xC0
#define EP_TYPE_ISOCHRONOUS_IN 0x41
#define EP_TYPE_ISOCHRONOUS_OUT 0x40
#define EP_SINGLE_BUFFER 0x02
#define EP_DOUBLE_BUFFER 0x06
#define EP_SIZE(s) ((s) == 64 ? 0x30 : ((s) == 32 ? 0x20 : ((s) == 16 ? 0x10 : 0x00)))

#define MAX_ENDPOINT 4

#define LSB(n) (n & 255)
#define MSB(n) ((n >> 8) & 255)

#define HW_CONFIG() (UHWCON = 0x01)

#ifdef M1
#define PLL_CONFIG() (PLLCSR = 0x02) // fixed to 8MHz clock
#else
#define PLL_CONFIG() (PLLCSR = 0x12) // 0001 0010 For a 16MHz clock
#endif

#define USB_CONFIG() (USBCON = ((1 << USBE) | (1 << OTGPADE)))
#define USB_FREEZE() (USBCON = ((1 << USBE) | (1 << FRZCLK)))

// standard control endpoint request types
#define GET_STATUS 0
#define CLEAR_FEATURE 1
#define SET_FEATURE 3
#define SET_ADDRESS 5
#define GET_DESCRIPTOR 6
#define GET_CONFIGURATION 8
#define SET_CONFIGURATION 9
#define GET_INTERFACE 10
#define SET_INTERFACE 11
// HID (human interface device)
#define HID_GET_REPORT 1
#define HID_GET_PROTOCOL 3
#define HID_SET_REPORT 9
#define HID_SET_IDLE 10
#define HID_SET_PROTOCOL 11
// CDC (communication class device)
#define CDC_SET_LINE_CODING 0x20
#define CDC_GET_LINE_CODING 0x21
#define CDC_SET_CONTROL_LINE_STATE 0x22



//----------------------------------------------IMU--------------------------------------------------
/* BNO055 I2C addresses (BOOT pin low = ADDR = 0x28, high = 0x29) */
#define BNO055_ADDR_A 0x28u
#define BNO055_ADDR_B 0x29u
#define BNO055_I2C_ADDR BNO055_ADDR_A /**< change if needed        */

/* Desired I2C speed (Hz) */
#define TWI_SCL_HZ 100000UL 

/* ------------------  hard reset via MCU GPIO ------------------ */
#define BNO_RST_DDR   DDRF
#define BNO_RST_PORT  PORTF
#define BNO_RST_PIN   PF7


//---------------------------------------------ANALOG-------------------------------------------------
/* 0-7  = PF0�PF7,   8-13 = PB4�PB7                       */
#define ADC_CH_BAT_MAIN     0   // PF0  � main battery divider
#define ADC_CH_BAT_AUX      1   // PF1  � aux battery divider
#define ADC_CH_CLIFF_LEFT   4   // PF4  � Sharp IR left
#define ADC_CH_CLIFF_FRONT  5   // PF5  � Sharp IR centre
#define ADC_CH_CLIFF_RIGHT  6   // PF6  � Sharp IR right

/* ---------- Conversion parameters --------------------- */
#define ADC_NUM_SAMPLES     4   // simple software average
#define ADC_PRESCALER_BITS  ((1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)) /* �128 */

/* ---------- Battery-scaling maths --------------------- *
 *   VBAT =  ADCraw * (AVcc / 1023) * (R1+R2) / R2
 *   Put the resistor ratios here so the driver can give
 *   you millivolts directly.  Example 100 k? / 33 k?.     */
#define BAT_DIV_FACTOR_NUM  (1)   // (R1+R2)*100 / R2
#define BAT_DIV_FACTOR_DEN  (1)



/* ================= ENCODERS (LEFT & RIGHT) ================= */

/* ---- LEFT encoder  ---- */
#define ENC_L_A_DDR    DDRD
#define ENC_L_A_PORT   PORTD
#define ENC_L_A_PINREG PIND
#define ENC_L_A_BIT    2    // PD2 -> INT0 ((D1)

#define ENC_L_B_DDR    DDRD
#define ENC_L_B_PORT   PORTD
#define ENC_L_B_PINREG PIND
#define ENC_L_B_BIT    3    // PD3 -> INT1  (D0)

/* ---- RIGHT encoder ---- */
#define ENC_R_B_DDR    DDRE
#define ENC_R_B_PORT   PORTE
#define ENC_R_B_PINREG PINE
#define ENC_R_B_BIT    6    // PE6 -> INT6  (D7)

#define ENC_R_A_DDR    DDRB
#define ENC_R_A_PORT   PORTB
#define ENC_R_A_PINREG PINB
#define ENC_R_A_BIT    4   // PB4 -> PCINT4  (D8)

/* ---- Emergency button (shared PCINT) ---- */
#define EMG_BTN_DDR    DDRB
#define EMG_BTN_PORT   PORTB
#define EMG_BTN_PINREG PINB
#define EMG_BTN_BIT    7    // PB7 -> PCINT7 (D11)

/* ================= PROFILER PARAMETERS ================= */

#define ENCODER_PPR         1000U        // quadrature pulses per channel
#define WHEEL_DIAMETER_MM   200.0f        // wheel diameter [mm]
#define WHEEL_BASE_MM       500.0f       // track width: distance between wheels [mm] 
#define GEAR_RATIO 10   

#define MM_PER_ROTATION  (M_PI * WHEEL_DIAMETER_MM)
#define DEG_PER_MM_DIFF  (180.0f / (M_PI * WHEEL_BASE_MM))

#define LOOP_TIME 10

// Teleoperator Mode - Distance and Angles
#define FORWARD_DIST         400.0f    // mm - Forward movement per command
#define BACKWARD_DIST        400.0f    // mm - Backward movement per command
#define LEFT_TURN_ANGLE      45.0f     // degrees - CCW rotation per command
#define RIGHT_TURN_ANGLE     45.0f     // degrees - CW rotation per command

// Teleoperator Mode - Speeds
#define TELEOP_SPEED         300.0f    // mm/s - Linear speed
#define TELEOP_OMEGA         50.0f    // deg/s - Angular speed

// Teleoperator Mode - Accelerations
#define TELEOP_ACC           200.0f    // mm/s� - Linear acceleration
#define TELEOP_ALPHA         30.0f    // deg/s� - Angular acceleration

/* ================= VELOCITY STREAM MODE (ROS 2 prep) ================= */
#define VSTREAM_TIMEOUT_MS       250      /* stop if no command for 250 ms   */
#define VSTREAM_MAX_LIN_VEL      500.0f   /* mm/s  - absolute clamp         */
#define VSTREAM_MAX_ANG_VEL      120.0f   /* deg/s - absolute clamp         */
#define VSTREAM_LIN_ACC          400.0f   /* mm/s^2 - slew-rate limit        */
#define VSTREAM_ANG_ACC          200.0f   /* deg/s^2 - slew-rate limit       */

#endif // CONFIG_H