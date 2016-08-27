/*
 * EGG Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015, 2106.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _PWM_H_
#define _PWM_H_

#define DUTY_CYCLE_MAX		2303

#define PWM_PERIOD_INTERRUPT		TIM3_UP_IRQHandler

void enable_phase_a (void);
void enable_phase_b (void);
void enable_phase_c (void);
void set_pwm_phase_a (unsigned int value);
void set_pwm_phase_b (unsigned int value);
void set_pwm_phase_a (unsigned int value);

void pwm_init (void);
void pwm_set_duty_cycle (int value);
void pwm_update_duty_cycle (void);

#endif
