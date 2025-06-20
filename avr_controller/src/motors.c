/*
 * motors.c
 *
 * Created: 5/6/2025 4:26:05 AM
 * Author: Endeavor360
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <stdint.h>
#include "motors.h"
#include "config.h"

/* private state ----------------------------------------------------------- */
static uint8_t left_top;
static uint16_t right_top;

/* ----------- hardware‐assisted edge (toggle) counters & ISRs ------------   */

volatile uint32_t left_edge_cnt  = 0;
volatile uint32_t right_edge_cnt = 0;

/* helpers ----------------------------------------------------------------- */
static inline uint32_t rpm_to_freq(uint16_t rpm)
{
	/* steps per minute ? steps per second = rpm * STEPS_PER_REV / 60 */
	return ((uint32_t)rpm * STEPS_PER_REV) / 60U;
}

/* public functions -------------------------------------------------------- */
void motors_init(void)
{
	/* I/O direction ------------------------------------------------------- */
	LEFT_PUL_DDR |= _BV(LEFT_PUL_BIT);
	LEFT_DIR_DDR |= _BV(LEFT_DIR_BIT);
	LEFT_ENA_DDR |= _BV(LEFT_ENA_BIT);

	RIGHT_PUL_DDR |= _BV(RIGHT_PUL_BIT);
	RIGHT_DIR_DDR |= _BV(RIGHT_DIR_BIT);
	RIGHT_ENA_DDR |= _BV(RIGHT_ENA_BIT);

	/* default levels � pulses high, forward, drivers disabled ------------ */
	LEFT_PUL_PORT |= _BV(LEFT_PUL_BIT);
	LEFT_DIR_PORT |= _BV(LEFT_DIR_BIT);
	LEFT_ENA_PORT &= ~_BV(LEFT_ENA_BIT);

	RIGHT_PUL_PORT |= _BV(RIGHT_PUL_BIT);
	RIGHT_DIR_PORT |= _BV(RIGHT_DIR_BIT);
	RIGHT_ENA_PORT &= ~_BV(RIGHT_ENA_BIT);

	// — Timer4 (10-bit) for LEFT motor PUL on OC4D (PD7) —
	// Configure Timer4 for PWM mode with proper frequency
	TCCR4A = 0; // No PWM mode for channel A and B
	// COM4D1:0 = 01 for toggle OC4D on compare match
	TCCR4C = _BV(COM4D0);
	// PWM4D = 0 (default) disables PWM mode for OC4D
	// WGM41:0 = 01 for PWM mode with OCR4C as TOP
	TCCR4D = _BV(WGM40);
	// No clock yet, will be started when setting speed
	TCCR4B = 0;

	// — Timer1 (16-bit) for RIGHT motor PUL on OC1A (PB5) —
	TCCR1A = _BV(COM1A0); // toggle OC1A on compare
	TCCR1B = _BV(WGM12);  // CTC mode, no clock yet
	
	
	/* enable OC‐compare interrupts for step counting */
	TIMSK4 |= _BV(OCIE4D);   // Timer4 Compare‐D
	TIMSK1 |= _BV(OCIE1A);   // Timer1 Compare‐A
}

/* ISR: each toggle = one edge */
ISR(TIMER4_COMPD_vect) {
	 left_edge_cnt++;
	  }
ISR(TIMER1_COMPA_vect) { 
	right_edge_cnt++; 
	}

void motors_enable_left(bool en)
{
	(en ? (LEFT_ENA_PORT |= _BV(LEFT_ENA_BIT))
		: (LEFT_ENA_PORT &= ~_BV(LEFT_ENA_BIT)));
}

void motors_enable_right(bool en)
{
	(en ? (RIGHT_ENA_PORT |= _BV(RIGHT_ENA_BIT))
		: (RIGHT_ENA_PORT &= ~_BV(RIGHT_ENA_BIT)));
}

void motors_enable_all(bool en)
{
	motors_enable_left(en);
	motors_enable_right(en);
}

void motors_set_dir_left(bool fwd)
{
	(fwd ? (LEFT_DIR_PORT |= _BV(LEFT_DIR_BIT))
		 : (LEFT_DIR_PORT &= ~_BV(LEFT_DIR_BIT)));
}

void motors_set_dir_right(bool fwd)
{
	(fwd ? (RIGHT_DIR_PORT |= _BV(RIGHT_DIR_BIT))
		 : (RIGHT_DIR_PORT &= ~_BV(RIGHT_DIR_BIT)));
}

void motors_set_speed_left(uint16_t rpm)
{
	uint32_t freq = rpm_to_freq(rpm);
	TCCR4B &= ~(_BV(CS43) | _BV(CS42) | _BV(CS41) | _BV(CS40));
	if (rpm > 500)
	{
		// for higher rpm, set shorter lower prescaler with higher pulse frequency to go faster
		left_top = (uint16_t)(F_CPU / (2UL * freq * CLOCK_DIVISOR_TIMER4_HIGH) - 1UL);

		if (left_top > 255)
			left_top = 255; // Timer4 is 8-bit in this mode

		OCR4C = left_top;	  // Set TOP value
		OCR4D = left_top / 2; // Set compare value for 50% duty cycle

		TCCR4B |= PRE_SCALE_TIMER4_HIGH;
	}
	else
	{
		// for lower rpm, set shorter higher prescaler wit lower pulse frequency to go slower
		left_top = (uint16_t)(F_CPU / (2UL * freq * CLOCK_DIVISOR_TIMER4_LOW) - 1UL);

		if (left_top > 255)
			left_top = 255; // Timer4 is 8-bit in this mode

		OCR4C = left_top;	  // Set TOP value
		OCR4D = left_top / 2; // Set compare value for 50% duty cycle

		TCCR4B |= PRE_SCALE_TIMER4_LOW;
	}
}

void motors_set_speed_right(uint16_t rpm)
{
	uint32_t freq = rpm_to_freq(rpm);
	uint32_t top = (F_CPU / (2UL * freq * CLOCK_DIVISOR)) - 1UL;

	right_top = (uint16_t)top;
	OCR1A = right_top;

	/* start Timer-1 with /1024 pre-scale */
	TCCR1B &= ~(_BV(CS12) | _BV(CS11) | _BV(CS10)); /* clear first   */
	TCCR1B |= PRE_SCALE_TIMER1;
}

void motors_set_speed_both(uint16_t rpm_left, uint16_t rpm_right)
{
	motors_set_speed_left(rpm_left);
	motors_set_speed_right(rpm_right);
}

void motors_move_left(int32_t steps)
{
	motors_enable_left(true);
	if (steps < 0)
	{
		steps = -steps;
		motors_set_dir_left(false);
	}
	else
		motors_set_dir_left(true);

	for (int32_t i = 0; i < steps; ++i)
	{
		LEFT_PUL_PORT |= _BV(LEFT_PUL_BIT);
		_delay_us(5);
		LEFT_PUL_PORT &= ~_BV(LEFT_PUL_BIT);
		_delay_us(5);
	}
}

void motors_move_right(int32_t steps)
{
	motors_enable_right(true);
	if (steps < 0)
	{
		steps = -steps;
		motors_set_dir_right(false);
	}
	else
		motors_set_dir_right(true);

	for (int32_t i = 0; i < steps; ++i)
	{
		RIGHT_PUL_PORT |= _BV(RIGHT_PUL_BIT);
		_delay_us(5);
		RIGHT_PUL_PORT &= ~_BV(RIGHT_PUL_BIT);
		_delay_us(5);
	}
}

void motors_stop_all(void)
{
	/* disable drivers */
	motors_enable_all(false);

	/* stop timers � clear prescaler bits */
	TCCR1B &= ~(_BV(CS12) | _BV(CS11) | _BV(CS10));
	TCCR4B &= ~(_BV(CS43) | _BV(CS42) | _BV(CS41) | _BV(CS40));
}


/* — API to reset & read counts atomically — */
void motors_reset_edge_counts(void)
{
	uint8_t oldSREG = SREG; cli();
	left_edge_cnt = right_edge_cnt = 0;
	SREG = oldSREG;
}

uint32_t motors_get_edge_count_left(void)
{
	uint32_t c; uint8_t oldSREG = SREG; cli();
	c = left_edge_cnt;
	SREG = oldSREG;
	return c;
}

uint32_t motors_get_edge_count_right(void)
{
	uint32_t c; uint8_t oldSREG = SREG; cli();
	c = right_edge_cnt;
	SREG = oldSREG;
	return c;
}

uint32_t motors_get_step_count_left(void)
{
	return motors_get_edge_count_left() >> 1;
}

uint32_t motors_get_step_count_right(void)
{
	return motors_get_edge_count_right() >> 1;
}