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

extern struct Bldc_phase_state bldc_phase_state;

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

#endif /* _MOTOR_H_ */
