/* LF1000 Pulse Width Modulator (PWM) Driver
 *
 * pwm_hal.h -- PWM hardware abstraction.
 *
 * Scott Esters <sesters@leapfrog.com
 */

#ifndef PWM_HAL_H
#define PWM_HAL_H

/* registers, as offsets */
#define PWM01PRES		0x00
#define	PWM0DUTY		0x02
#define PWM1DUTY		0x04
#define PWM0PERIOD		0x06
#define PWM1PERIOD		0x08
#define PWM2PRES		0x10
#define PWM2DUTY		0x12
#define PWM2PERIOD		0x16
#define PWMCLKENB		0x40
#define PWMCLKGEN		0x44

/* PWM PRESCALER0/1 REGISTER (PWM01PRES) */
#define PWM1POL			15
#define PWM1PRESCALE		8
#define PWM0POL			7
#define PWM0PRESCALE		0

/* PWM PRESCALER 2 REGISTER (PWM2PRES) */
#define PWM2POL			7
#define PWM2PRESCALE		0

/* PWM CLOCK ENABLE REGISTER (PWMCLKENB) */
#define PWMPCLKMODE		3
#define PWMCLKGENENB		2

/* PWM CLOCK GENERATE REGISTER (PWMCLKGEN) */
#define PWMCLKDIV		4
#define PWMCLKSRCSEL		1

#endif

