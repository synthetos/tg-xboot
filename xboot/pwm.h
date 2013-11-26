/*
 * pwm.h - pulse width modulation drivers
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

#ifndef PWM_H_ONCE
#define PWM_H_ONCE

//#include "tinyg.h"		// #1
#define MOTORS		4			// number of motors on the board
#define PWMS		2			// number of supported PWM channels

#define PWM_1		0
#define PWM_2		1

#define	STAT_OK 0						// function completed OK
#define	STAT_INPUT_VALUE_TOO_SMALL 44		// input error: value is under minimum
#define	STAT_INPUT_VALUE_TOO_LARGE 45		// input error: value is over maximum

#define	STAT_NO_SUCH_DEVICE 11

 typedef uint8_t stat_t;

#ifdef __cplusplus
extern "C"{
#endif


typedef struct pwmConfigChannel {
	float frequency;				// base frequency for PWM driver, in Hz
	float cw_speed_lo;				// minimum clockwise spindle speed [0..N]
	float cw_speed_hi;				// maximum clockwise spindle speed
	float cw_phase_lo;				// pwm phase at minimum CW spindle speed, clamped [0..1]
	float cw_phase_hi;				// pwm phase at maximum CW spindle speed, clamped [0..1]
	float ccw_speed_lo;				// minimum counter-clockwise spindle speed [0..N]
	float ccw_speed_hi;				// maximum counter-clockwise spindle speed
	float ccw_phase_lo;				// pwm phase at minimum CCW spindle speed, clamped [0..1]
	float ccw_phase_hi;				// pwm phase at maximum CCW spindle speed, clamped
	float phase_off;				// pwm phase when spindle is disabled
} pwmConfigChannel_t;

typedef struct pwmChannel {
	uint8_t ctrla;					// byte needed to active CTRLA (it's dynamic - rest are static)
	TC1_t *timer;					// assumes TC1 flavor timers used for PWM channels
} pwmChannel_t;

typedef struct pwmSingleton {
	pwmConfigChannel_t  c[PWMS];	// array of channel configs
	pwmChannel_t 		p[PWMS];	// array of PWM channels
} pwmSingleton_t;

extern pwmSingleton_t pwm;

/*** function prototypes ***/

void pwm_init(void);
stat_t pwm_set_freq(uint8_t channel, float freq);
stat_t pwm_set_duty(uint8_t channel, float duty);

#ifdef __cplusplus
}
#endif

#endif	// End of include guard: PWM_H_ONCE
