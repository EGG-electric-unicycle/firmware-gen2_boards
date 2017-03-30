/*
 * EGG Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015, 2106.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _MOTOR_H
#define _MOTOR_H

// direction of motor movement
#define RIGHT 		1
#define LEFT 		2

// phase states
#define OFF 		0
#define NORMAL 		1
#define INVERTED 	2

// State machine
//#define BLDC_NORMAL		0
//#define BLDC_OVER_MAX_CURRENT	1

#define DEGRES_120_IN_RADIANS 	(120.0 * (M_PI/180.0))

#define degrees_to_radiands(angle) qfp_fmul(angle, M_PI/180.0)
#define radians_to_degrees(angle) qfp_fmul(angle, 180.0/M_PI)

extern unsigned int motor_speed_erps;
extern unsigned int PWM_cycles_counter;
extern int motor_rotor_position;
extern int position_correction_value;
extern unsigned int duty_cycle;

extern int adc_phase_a_current;
extern int adc_phase_b_current;
extern int adc_phase_c_current;

extern unsigned int adc_phase_a_current_offset;
extern unsigned int adc_phase_c_current_offset;

void commutation_disable (void);
void hall_sensors_interrupt (void);
void apply_duty_cycle (void);
void motor_set_duty_cycle (unsigned int value);
void motor_calc_current_dc_offset (void);
void motor_set_current (int value);

#endif /* _MOTOR_H_ */
