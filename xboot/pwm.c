/*
 * pwm.c - pulse width modulation drivers
 * This file is part of the TinyG project
 *
 * Copyright (c) 2012 - 2013 Alden S. Hart, Jr.
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * As a special exception, you may use this file as part of a software library without
 * restriction. Specifically, if other files instantiate templates or use macros or
 * inline functions from this file, or you compile this file and link it with  other
 * files to produce an executable, this file does not by itself cause the resulting
 * executable to be covered by the GNU General Public License. This exception does not
 * however invalidate any other reasons why the executable file might be covered by the
 * GNU General Public License.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <avr/interrupt.h>
#include <string.h>
#include "pwm.h"

//#include "config.h"		// #2
//#include "hardware.h"
 enum cfgPortBits {			// motor control port bit positions
	STEP_BIT_bp = 0,		// bit 0
	DIRECTION_BIT_bp,		// bit 1
	MOTOR_ENABLE_BIT_bp,	// bit 2
	MICROSTEP_BIT_0_bp,		// bit 3
	MICROSTEP_BIT_1_bp,		// bit 4
	GPIO1_OUT_BIT_bp,		// bit 5 (4 gpio1 output bits; 1 from each axis)
	SW_MIN_BIT_bp,			// bit 6 (4 input bits for homing/limit switches)
	SW_MAX_BIT_bp			// bit 7 (4 input bits for homing/limit switches)
};

#define GPIO1_OUT_BIT_bm	(1<<GPIO1_OUT_BIT_bp)	// spindle and coolant output bits

#define MOTOR_PORT_DIR_gm 0x3F	// dir settings: lower 6 out, upper 2 in
#define MOTOR_ENABLE_BIT_bm (1<<MOTOR_ENABLE_BIT_bp)

#define SPINDLE_PWM			0x02		// spindle PWMs output bit

#define PORT_MOTOR_1	PORTA			// motors mapped to ports
#define PORT_MOTOR_2 	PORTF
#define PORT_MOTOR_3	PORTE
#define PORT_MOTOR_4	PORTD

#define PORT_OUT_V7_X	PORTA			// v7 mapping - Output bits mapped to ports
#define PORT_OUT_V7_Y 	PORTF
#define PORT_OUT_V7_Z	PORTD
#define PORT_OUT_V7_A	PORTE

/* Timer assignments - see specific modules for details) */
#define TIMER_DDA			TCC0		// DDA timer 	(see stepper.h)
#define TIMER_DWELL	 		TCD0		// Dwell timer	(see stepper.h)
#define TIMER_LOAD			TCE0		// Loader timer	(see stepper.h)
#define TIMER_EXEC			TCF0		// Exec timer	(see stepper.h)
#define TIMER_5				TCC1		// unallocated timer
#define TIMER_PWM1			TCD1		// PWM timer #1 (see pwm.c)
#define TIMER_PWM2			TCE1		// PWM timer #2	(see pwm.c)

typedef struct hmSingleton {
	PORT_t *st_port[MOTORS];		// bindings for stepper motor ports (stepper.c)
	PORT_t *out_port[MOTORS];		// bindings for output ports (GPIO1)
} hwSingleton_t;
hwSingleton_t hw;

//#include "gpio.h"
void gpio_set_bit_off(uint8_t b)
{
	if (b & 0x08) { hw.out_port[0]->OUTCLR = GPIO1_OUT_BIT_bm; }
	if (b & 0x04) { hw.out_port[1]->OUTCLR = GPIO1_OUT_BIT_bm; }
	if (b & 0x02) { hw.out_port[2]->OUTCLR = GPIO1_OUT_BIT_bm; }
	if (b & 0x01) { hw.out_port[3]->OUTCLR = GPIO1_OUT_BIT_bm; }
}

void gpio_set_bit_on(uint8_t b)
{
	if (b & 0x08) { hw.out_port[0]->OUTSET = GPIO1_OUT_BIT_bm; }
	if (b & 0x04) { hw.out_port[1]->OUTSET = GPIO1_OUT_BIT_bm; }
	if (b & 0x02) { hw.out_port[2]->OUTSET = GPIO1_OUT_BIT_bm; }
	if (b & 0x01) { hw.out_port[3]->OUTSET = GPIO1_OUT_BIT_bm; }
}


#ifdef __cplusplus
extern "C"{
#endif

void stepper_init();
void stepper_init()
{
	// setup ports
	for (uint8_t i=0; i<MOTORS; i++) {
		hw.st_port[i]->DIR = MOTOR_PORT_DIR_gm;  // sets outputs for motors & GPIO1, and GPIO2 inputs
		hw.st_port[i]->OUT = MOTOR_ENABLE_BIT_bm;// zero port bits AND disable motor
	}
}

// imported tinyg init code to make this PWM module work
void other_init( void );
void other_init( void )
{
	hw.st_port[0] = &PORT_MOTOR_1;
	hw.st_port[1] = &PORT_MOTOR_2;
	hw.st_port[2] = &PORT_MOTOR_3;
	hw.st_port[3] = &PORT_MOTOR_4;

	// if (hw_version > 6.9) {
		hw.out_port[0] = &PORT_OUT_V7_X;
		hw.out_port[1] = &PORT_OUT_V7_Y;
		hw.out_port[2] = &PORT_OUT_V7_Z;
		hw.out_port[3] = &PORT_OUT_V7_A;
	// } else {
	// 	hw.out_port[0] = &PORT_OUT_V6_X;
	// 	hw.out_port[1] = &PORT_OUT_V6_Y;
	// 	hw.out_port[2] = &PORT_OUT_V6_Z;
	// 	hw.out_port[3] = &PORT_OUT_V6_A;
	// }

	stepper_init();
}

// settings_othermill.h
#define P1_PWM_FREQUENCY		100					// in Hz
#define P1_PWM_PHASE_OFF		0.1

void more_other_init( void );
void more_other_init( void )
{
	pwm_set_freq(PWM_1, P1_PWM_FREQUENCY);
	pwm_set_duty(PWM_1, P1_PWM_PHASE_OFF);
}

/***** PWM defines, structures and memory allocation *****/

pwmSingleton_t pwm;

// defines common to all PWM channels
//#define PWM_TIMER_TYPE	TC1_struct	// PWM uses TC1's
#define PWM_TIMER_t	TC1_t				// PWM uses TC1's
#define PWM_TIMER_DISABLE 0				// turn timer off (clock = 0 Hz)
#define PWM_MAX_FREQ (F_CPU/256)		// max frequency with 8-bits duty cycle precision
#define PWM_MIN_FREQ (F_CPU/64/65536)	// min frequency with supported prescaling

// channel specific defines
/* CLKSEL is used to configure default PWM clock operating ranges
 * These can be changed by pwm_freq() depending on the PWM frequency selected
 *
 * The useful ranges (assuming a 32 Mhz system clock) are:
 *	 TC_CLKSEL_DIV1_gc  - good for about 500 Hz to 125 Khz practical upper limit
 *   TC_CLKSEL_DIV2_gc  - good for about 250 Hz to  62 KHz
 *	 TC_CLKSEL_DIV4_gc  - good for about 125 Hz to  31 KHz
 *	 TC_CLKSEL_DIV8_gc  - good for about  62 Hz to  16 KHz
 *	 TC_CLKSEL_DIV64_gc - good for about   8 Hz to   2 Khz
 */
#define PWM1_CTRLA_CLKSEL	TC_CLKSEL_DIV1_gc	// starting clock select value
#define PWM1_CTRLB 			(3 | TC0_CCBEN_bm)	// single slope PWM enabled on channel B
#define PWM1_ISR_vect 		TCD1_CCB_vect		// must match timer assignments in system.h
#define PWM1_INTCTRLB		0					// timer interrupt level (0=off, 1=lo, 2=med, 3=hi)

#define PWM2_CTRLA_CLKSEL 	TC_CLKSEL_DIV1_gc
#define PWM2_CTRLB 			3					// single slope PWM enabled, no output channel
//#define PWM2_CTRLB 		(3 | TC0_CCBEN_bm)	// single slope PWM enabled on channel B
#define PWM2_ISR_vect		TCE1_CCB_vect		// must match timer assignments in system.h
#define PWM2_INTCTRLB		0					// timer interrupt level (0=off, 1=lo, 2=med, 3=hi)

/***** PWM code *****/
/* 
 * pwm_init() - initialize pwm channels
 *
 *	Notes: 
 *	  - Whatever level interrupts you use must be enabled in main()
 *	  - init assumes PWM1 output bit (D5) has been set to output previously (stepper.c)
 *	  - See system.h for timer and port assignments
 *    - Don't do this: memset(&TIMER_PWM1, 0, sizeof(PWM_TIMER_t)); // zero out the timer registers
 */
void pwm_init()
{
	other_init();

	gpio_set_bit_on(SPINDLE_PWM);

	// setup PWM channel 1
	memset(&pwm.p[PWM_1], 0, sizeof(pwmChannel_t));		// clear parent structure 
	pwm.p[PWM_1].timer = &TIMER_PWM1;					// bind timer struct to PWM struct array
	pwm.p[PWM_1].ctrla = PWM1_CTRLA_CLKSEL;				// initialize starting clock operating range
	pwm.p[PWM_1].timer->CTRLB = PWM1_CTRLB;
	pwm.p[PWM_1].timer->INTCTRLB = PWM1_INTCTRLB;		// set interrupt level	

	// setup PWM channel 2
	memset(&pwm.p[PWM_2], 0, sizeof(pwmChannel_t));		// clear all values, pointers and status
	pwm.p[PWM_2].timer = &TIMER_PWM2;
	pwm.p[PWM_2].ctrla = PWM2_CTRLA_CLKSEL;
	pwm.p[PWM_2].timer->CTRLB = PWM2_CTRLB;
	pwm.p[PWM_2].timer->INTCTRLB = PWM2_INTCTRLB;

	more_other_init();
}

/*
 * ISRs for PWM timers
 */

ISR(PWM1_ISR_vect) 
{
	return;
}

ISR(PWM2_ISR_vect) 
{
	return;
}

/* 
 * pwm_set_freq() - set PWM channel frequency
 *
 *	channel	- PWM channel
 *	freq	- PWM frequency in Khz as a float
 *
 *	Assumes 32MHz clock.
 *	Doesn't turn time on until duty cycle is set
 */

stat_t pwm_set_freq(uint8_t chan, float freq)
{
	if (chan > PWMS) { return (STAT_NO_SUCH_DEVICE);}
	if (freq > PWM_MAX_FREQ) { return (STAT_INPUT_VALUE_TOO_LARGE);}
	if (freq < PWM_MIN_FREQ) { return (STAT_INPUT_VALUE_TOO_SMALL);}

	// set the period and the prescaler
	float prescale = F_CPU/65536/freq;	// optimal non-integer prescaler value
	if (prescale <= 1) { 
		pwm.p[chan].timer->PER = F_CPU/freq;
		pwm.p[chan].timer->CTRLA = TC_CLKSEL_DIV1_gc;
	} else if (prescale <= 2) { 
		pwm.p[chan].timer->PER = F_CPU/2/freq;
		pwm.p[chan].timer->CTRLA = TC_CLKSEL_DIV2_gc;
	} else if (prescale <= 4) { 
		pwm.p[chan].timer->PER = F_CPU/4/freq;
		pwm.p[chan].timer->CTRLA = TC_CLKSEL_DIV4_gc;
	} else if (prescale <= 8) { 
		pwm.p[chan].timer->PER = F_CPU/8/freq;
		pwm.p[chan].timer->CTRLA = TC_CLKSEL_DIV8_gc;
	} else { 
		pwm.p[chan].timer->PER = F_CPU/64/freq;
		pwm.p[chan].timer->CTRLA = TC_CLKSEL_DIV64_gc;
	}
	return (STAT_OK);
}

/* 
 * pwm_set_duty() - set PWM channel duty cycle 
 *
 *	channel	- PWM channel
 *	duty	- PWM duty cycle from 0% to 100%
 *
 *	Setting duty cycle to 0 disables the PWM channel with output low
 *	Setting duty cycle to 100 disables the PWM channel with output high
 *	Setting duty cycle between 0 and 100 enables PWM channel
 *
 *	The frequency must have been set previously
 */

stat_t pwm_set_duty(uint8_t chan, float duty)
{
    if (duty < 0.0) { return (STAT_INPUT_VALUE_TOO_SMALL);}
    if (duty > 1.0) { return (STAT_INPUT_VALUE_TOO_LARGE);}
    
	// Ffrq = Fper/(2N(CCA+1))
	// Fpwm = Fper/((N(PER+1))
	
	float period_scalar = pwm.p[chan].timer->PER;
	pwm.p[chan].timer->CCB = (uint16_t)(period_scalar * duty) + 1;
	return (STAT_OK);
}

#ifdef __cplusplus
}
#endif
