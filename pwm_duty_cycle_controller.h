/*
 * EGG Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015, 2106, 2017.
 *
 * Released under the GPL License, Version 3
 */

//#ifndef _PWM-DUTY_CYCLE_CONTROLLER_H_
//#define _PWM-DUTY_CYCLE_CONTROLLER_H_

#ifndef _PWM_DUTY_CYCLE_CONTROLLER_H_
#define _PWM_DUTY_CYCLE_CONTROLLER_H_

#define PWM_PERIOD_US 			50
#define PWM_VALUE_DUTY_CYCLE_MAX	(1800 - 1)
#define MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX     (PWM_VALUE_DUTY_CYCLE_MAX / 2)

#define PWM_PERIOD_INTERRUPT		TIM3_IRQHandler

#define SVM_TABLE_LEN 360

extern unsigned int svm_table [SVM_TABLE_LEN];
extern int duty_cycle;

void apply_duty_cycle (int duty_cycle_value);
void pwm_duty_cycle_controller (void);
void set_pwm_duty_cycle (int value);
unsigned int get_motor_rotation_direction (void);

#endif
