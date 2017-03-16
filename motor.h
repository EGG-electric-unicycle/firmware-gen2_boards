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
#define LEFT 		0

// phase states
#define OFF 		0
#define NORMAL 		1
#define INVERTED 	2

// State machine
#define BLDC_NORMAL		0
#define BLDC_OVER_MAX_CURRENT	1

struct Bldc_phase_state
{
  unsigned int a;
  unsigned int b;
  unsigned int c;
};

#define degrees_to_radiands(angle) qfp_fmul(angle, M_PI/180.0)
#define radians_to_degrees(angle) qfp_fmul(angle, 180.0/M_PI)

extern struct Bldc_phase_state bldc_phase_state;
extern unsigned int motor_speed_erps;
extern unsigned int motor_inverse_speed__timer;
extern float motor_rotor_position;

extern unsigned int adc_phase_a_current_offset;
extern unsigned int adc_phase_c_current_offset;

void commutation_disable (void);
void commutate (void);
unsigned int get_current_sector (void);
unsigned int increment_sector (unsigned int sector);
unsigned int decrement_sector (unsigned int sector);
void bldc_set_direction (unsigned int direction);
unsigned int bldc_get_direction (void);
void bldc_set_state (unsigned int state);
unsigned int bldc_get_state (void);
void apply_duty_cycle (void);
void commutate_timer (void);
void motor_calc_current_dc_offset (void);

#endif /* _MOTOR_H_ */
