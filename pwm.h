/*
 * Generic Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _PWM_H_
#define _PWM_H_

// 2039 max duty cycle value
#define DUTY_CYCLE_NORMAL_MAX		2047
#define DUTY_CYCLE_NORMAL_MIN		(2047/2)
#define DUTY_CYCLE_OFF			DUTY_CYCLE_NORMAL_MIN
#define DUTY_CYCLE_INVERTED_MAX		((2047/2) - 1)
#define DUTY_CYCLE_INVERTED_MIN 	0
#define MIN_POSITIVE_DUTY_CYCLE		100
#define MIN_NEGATIVE_DUTY_CYCLE		100

//#define DUTY_CYCLE_MAX_SAFE		(DUTY_CYCLE_MAX - (DUTY_CYCLE_MAX/20)) // 95% for safe margin
//#define DUTY_CYCLE_MIN_SAFE		(DUTY_CYCLE_MAX/20 + DUTY_CYCLE_MAGIC_NUMBER) // 5% for safe margin

#define PWM_DUTY_CYCLE_STEP		1 // step value to increment/decrement duty cycle value

#define PWM_PERIOD_INTERRUPT		TIM1_UP_IRQHandler

void pwm_init (void);
void pwm_set_duty_cycle (int value);
void pwm_update_duty_cycle (void);
void pwm_manage (void);

#endif
