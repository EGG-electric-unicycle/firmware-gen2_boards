/*
 * EGG Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015, 2106.
 *
 * Released under the GPL License, Version 3
 */

#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "gpio.h"
#include "motor.h"

//unsigned int bldc_machine_state = BLDC_NORMAL;
static unsigned int _direction = RIGHT;

static unsigned int svm_table_index_a;
static unsigned int svm_table_index_b;
static unsigned int svm_table_index_c;

struct Bldc_phase_state bldc_phase_state;

int duty_cycle = 0;

// Space Vector Modulation PWMs values, please read this blog message:
// http://www.berryjam.eu/2015/04/driving-bldc-gimbals-at-super-slow-speeds-with-arduino/
// Please see file: BLDC_SPWM_Lookup_tables.ods
unsigned int svm_table [36] =
{
  1280,
  1579,
  1870,
  2143,
  2217,
  2262,
  2277,
  2262,
  2217,
  2143,
  2217,
  2262,
  2277,
  2262,
  2217,
  2143,
  1870,
  1579,
  1280,
  980,
  689,
  416,
  342,
  297,
  282,
  297,
  342,
  416,
  342,
  297,
  282,
  297,
  342,
  416,
  689,
  980
};

void apply_duty_cycle (void)
{
  int _duty_cycle = duty_cycle;
  unsigned int value = 0;

  // invert in the case of negative value
  if (_duty_cycle < 0)
    _duty_cycle *= -1;

  // scale and apply _duty_cycle (integer operations only!)
  value = svm_table[svm_table_index_a];
  value = value * duty_cycle;
  value = value / 1000;
  set_pwm_phase_a (value);

  value = svm_table[svm_table_index_b];
  value = value * duty_cycle;
  value = value / 1000;
  set_pwm_phase_b (value);

  value = svm_table[svm_table_index_c];
  value = value * duty_cycle;
  value = value / 1000;
  set_pwm_phase_c (value);
}

void commutation_AB (void)
{
  svm_table_index_a = 26;
  svm_table_index_b = 2;
  svm_table_index_c = 14;
}

void commutation_AC (void)
{
  svm_table_index_a = 20;
  svm_table_index_b = 32;
  svm_table_index_c = 8;
}

void commutation_BC (void)
{
  svm_table_index_a = 14;
  svm_table_index_b = 26;
  svm_table_index_c = 2;
}

void commutation_BA (void)
{
  svm_table_index_a = 8;
  svm_table_index_b = 20;
  svm_table_index_c = 32;
}

void commutation_CA (void)
{
  svm_table_index_a = 2;
  svm_table_index_b = 14;
  svm_table_index_c = 26;
}

void commutation_CB (void)
{
  svm_table_index_a = 32;
  svm_table_index_b = 8;
  svm_table_index_c = 20;
}

void svm_table_index_dec (void)
{
  if (svm_table_index_a > 0) svm_table_index_a--;
  else svm_table_index_a = 35;

  if (svm_table_index_b > 0) svm_table_index_b--;
  else svm_table_index_b = 35;

  if (svm_table_index_c > 0) svm_table_index_c--;
  else svm_table_index_c = 35;
}

void commutation_disable (void)
{
  TIM_CtrlPWMOutputs (TIM1, DISABLE); // PWM Output Disable
}

unsigned int get_current_stator_angle (void)
{
  #define HALL_SENSORS_MASK (HALL_SENSOR_A__PIN | HALL_SENSOR_B__PIN | HALL_SENSOR_C__PIN)

  static unsigned int hall_sensors = 0;
  unsigned int sector;

  hall_sensors = (GPIO_ReadInputData (HALL_SENSORS__PORT) & (HALL_SENSORS_MASK)); // mask other pins

//  if (_direction == RIGHT)
//  {
//    // IDENTIFY the sector from hall sensors signals
//    //
//    //       cba
//    //  00000001 == 1
//    //  00000011 == 3
//    //  00000010 == 2
//    //  00000110 == 6
//    //  00000100 == 4
//    //  00000101 == 5

    switch (hall_sensors)
    {
      case 1: // right 1, 3, 2, 5, 6, 4
	return  0;
      break;

      case 2:
	return  60;
      break;

      case 3:
	return 120;
      break;

      case 4:
      	return 180;
      break;

      case 5:
      	return 240;
      break;

      case 6:
      	return 300;
      break;

      default:
	return 0;
      break;
    }
//  }
//  else if (_direction == LEFT)
//  {
//    switch (hall_sensors)
//    {
//      case 1: // left 4, 6, 5, 2, 3, 1
//      sector = 4;
//      break;
//
//      case 2:
//      sector = 6;
//      break;
//
//      case 3:
//      sector = 5;
//      break;
//
//      case 4:
//      sector = 2;
//      break;
//
//      case 5:
//      sector = 3;
//      break;
//
//      case 6:
//      sector = 1;
//      break;
//    }
//  }

  return 0;
}

void commutate (void)
{
  volatile unsigned int current_stator_angle;

  current_stator_angle = get_current_stator_angle ();

//  switch (current_stator_angle)
//  {
//    case 0:
//    commutation_AB ();
//    break;
//
//    case 60:
//    commutation_AC ();
//    break;
//
//    case 120:
//    commutation_BC ();
//    break;
//
//    case 180:
//    commutation_BA ();
//    break;
//
//    case 240:
//    commutation_CA ();
//    break;
//
//    case 300:
//    commutation_CB ();
//    break;
//
//    default:
//    commutation_disable ();
//    break;
//  }
//
//  apply_duty_cycle ();
}

void commutate_timer (void)
{
  svm_table_index_dec ();

  apply_duty_cycle ();
}

void bldc_set_direction (unsigned int direction)
{
  _direction = direction;
}

unsigned int bldc_get_direction (void)
{
  return _direction;
}

void bldc_set_state (unsigned int state)
{
//  bldc_machine_state = state;
}

unsigned int bldc_get_state (void)
{
//  return bldc_machine_state;
}

void motor_set_duty_cycle (int value)
{
  duty_cycle = value;
}
