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
  int duty_cycle_ = duty_cycle;
  unsigned int value = 0;

  // invert in the case of negative value
  if (duty_cycle_ < 0)
    duty_cycle_ *= -1;

  // scale and apply _duty_cycle (integer operations only!)
  value = svm_table[svm_table_index_a];
  value = value * duty_cycle_;
  value = value / 1000;
  set_pwm_phase_a (value);

  value = svm_table[svm_table_index_b];
  value = value * duty_cycle_;
  value = value / 1000;
  set_pwm_phase_b (value);

  value = svm_table[svm_table_index_c];
  value = value * duty_cycle_;
  value = value / 1000;
  set_pwm_phase_c (value);
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

void svm_table_index_inc (void)
{
  if (svm_table_index_a < 35) svm_table_index_a++;
  else svm_table_index_a = 0;

  if (svm_table_index_b < 35) svm_table_index_b++;
  else svm_table_index_b = 0;

  if (svm_table_index_c < 35) svm_table_index_c++;
  else svm_table_index_c = 0;
}

void commutation_disable (void)
{
  TIM_CtrlPWMOutputs (TIM1, DISABLE); // PWM Output Disable
}

void commutate (void)
{
  #define HALL_SENSORS_MASK (HALL_SENSOR_A__PIN | HALL_SENSOR_B__PIN | HALL_SENSOR_C__PIN)

  static unsigned int hall_sensors = 0;
  unsigned int sector;

  hall_sensors = (GPIO_ReadInputData (HALL_SENSORS__PORT) & (HALL_SENSORS_MASK)); // mask other pins

  if (duty_cycle > 0)
    _direction = RIGHT;
  else
    _direction = LEFT;

  if (_direction == RIGHT)
  {
    // the next sequence was obtained experimentaly
    switch (hall_sensors)
    {
      case 8192:
      svm_table_index_a = 28; // 1
      svm_table_index_b = 4;
      svm_table_index_c = 16;
      break;

      case 24576:
      svm_table_index_a = 22; // 2
      svm_table_index_b = 34;
      svm_table_index_c = 10;
      break;

      case 16384:
      svm_table_index_a = 16; // 3
      svm_table_index_b = 28;
      svm_table_index_c = 4;
      break;

      case 20480:
      svm_table_index_a = 10; // 4
      svm_table_index_b = 22;
      svm_table_index_c = 34;
      break;

      case 4096:
      svm_table_index_a = 4; // 5
      svm_table_index_b = 16;
      svm_table_index_c = 28;
      break;

      case 12288:
      svm_table_index_a = 34; // 6
      svm_table_index_b = 10;
      svm_table_index_c = 22;
      break;

      default:
      break;
    }
  }
  else if (_direction == LEFT)
  {
    switch (hall_sensors)
     {
      case 8192:
	      svm_table_index_a = 28; // 1
	      svm_table_index_b = 4;
	      svm_table_index_c = 16;
      break;

      case 24576:
	      svm_table_index_a = 34; // 6
	      svm_table_index_b = 10;
	      svm_table_index_c = 22;
      break;

      case 16384:
	      svm_table_index_a = 4; // 5
	      svm_table_index_b = 16;
	      svm_table_index_c = 28;
      break;

      case 20480:
	      svm_table_index_a = 10; // 4
	      svm_table_index_b = 22;
	      svm_table_index_c = 34;
      break;

      case 4096:
	      svm_table_index_a = 16; // 3
	      svm_table_index_b = 28;
	      svm_table_index_c = 4;
      break;

      case 12288:
	      svm_table_index_a = 22; // 2
	      svm_table_index_b = 34;
	      svm_table_index_c = 10;
      break;

      default:
      break;
    }
  }

    apply_duty_cycle ();
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
