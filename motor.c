/*
 * EGG Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015, 2106.
 *
 * Released under the GPL License, Version 3
 */

#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stdio.h"
#include "core_cm3.h"
#include "adc.h"
#include "gpio.h"
#include "pwm.h"
#include "motor.h"
#include "qfplib-m3.h"
#include "math.h"
#include "main.h"
#include "filter.h"
#include "IMU/imu.h"
#include "pwm_duty_cycle_controller.h"

unsigned int motor_speed_erps = 0; // motor speed in electronic rotations per second
unsigned int PWM_cycles_per_SVM_TABLE_step = 0;
unsigned int PWM_cycles_counter = 0;
unsigned int interpolation_PWM_cycles_counter = 0;
int motor_rotor_position = 0; // in degrees
unsigned int motor_rotor_absolute_position = 0; // in degrees
unsigned int interpolation_counter = 0;
int position_correction_value = 0; // in degrees

int adc_phase_a_current = 0;
int adc_phase_b_current = 0;
int adc_phase_c_current = 0;

int adc_phase_a_current1 = 0;
int adc_phase_c_current1 = 0;
int adc_phase_current1_cycles = 0;

int adc_phase_a_current_offset;
int adc_phase_c_current_offset;

// runs every 1ms
void FOC_slow_loop (void)
{
  //---------------------------
  // Clarke transform assuming balanced currents
  // removing DC offset

  __disable_irq();
  adc_phase_a_current = adc_phase_a_current1;
  adc_phase_a_current1 = 0;
  adc_phase_c_current = adc_phase_c_current1;
  adc_phase_c_current1 = 0;
  int temp_adc_phase_current1_cycles = adc_phase_current1_cycles;
  adc_phase_current1_cycles = 0;
  int temp_motor_rotor_position = motor_rotor_position;
  unsigned int temp_direction = get_motor_rotation_direction();
  __enable_irq();

  adc_phase_a_current /= temp_adc_phase_current1_cycles;
  adc_phase_c_current /= temp_adc_phase_current1_cycles;

  adc_phase_a_current = adc_phase_a_current - adc_phase_a_current_offset;
  adc_phase_c_current = adc_phase_c_current - adc_phase_c_current_offset;


  /* Calc phase B current assuming balanced currents
   * considering: a + b + c = 0 ; a + c = -b ; b = -(a + c) ; b = -a -c
   */
//  adc_phase_b_current = -adc_phase_a_current - adc_phase_c_current;
  // change currents as for this motor, the phases are: BAC and not ABC
  adc_phase_b_current = adc_phase_a_current;
  adc_phase_a_current = -adc_phase_b_current - adc_phase_c_current;

  // calc ia and ib in Amps
  float ia = qfp_fmul(adc_phase_a_current, ADC_CURRENT_GAIN_MILLIAMPS);
  float ib = qfp_fmul(adc_phase_b_current, ADC_CURRENT_GAIN_MILLIAMPS);
  float ic = qfp_fmul(adc_phase_c_current, ADC_CURRENT_GAIN_MILLIAMPS);

  float id = 0;
  float iq = 0;

  int temp_motor_rotor_position1 = temp_motor_rotor_position;
  temp_motor_rotor_position = (temp_motor_rotor_position + (270 - 1)) % 360; // this makes the motor to run (almost) smooth in both directions -- id current seems good after this
  float motor_rotor_position_radians = degrees_to_radiands(temp_motor_rotor_position);

  // ABC->dq Park transform
  //------------------------------------------------------------------------
  float temp;
  temp = qfp_fmul(ia, qfp_fcos(motor_rotor_position_radians));
  temp += qfp_fmul(ib, qfp_fcos(motor_rotor_position_radians + DEGRES_120_IN_RADIANS));
  temp += qfp_fmul(ic, qfp_fcos(motor_rotor_position_radians - DEGRES_120_IN_RADIANS));
  id = qfp_fmul(temp, 2.0/3.0);

  temp_motor_rotor_position = (temp_motor_rotor_position1 + (315 - 1)) % 360; // needed for correct value calculation of iq
  motor_rotor_position_radians = degrees_to_radiands(temp_motor_rotor_position);
  temp = qfp_fmul(ia, qfp_fsin(motor_rotor_position_radians));
  temp += qfp_fmul(ib, qfp_fsin((motor_rotor_position_radians) + DEGRES_120_IN_RADIANS));
  temp += qfp_fmul(ic, qfp_fsin((motor_rotor_position_radians) - DEGRES_120_IN_RADIANS));
  iq = qfp_fmul(temp, 2.0/3.0);
  if (temp_direction == RIGHT) { // needed for correct sign value of iq
    iq *= -1.0;
  }
  //------------------------------------------------------------------------

  // Filter Id and Iq currents
  //------------------------------------------------------------------------
  float alpha_idiq = 5.0;
  static float moving_average_id = 0.0;
  static float moving_average_iq = 0.0;
  id = ema_filter_float(&id, &moving_average_id, &alpha_idiq);
  iq = ema_filter_float(&iq, &moving_average_iq, &alpha_idiq);
  // ------------------------------------------------------------------------

  // Calculate phase/angle correction value to try keep id current = 0
  //------------------------------------------------------------------------
  static float correction_value = 0;
  correction_value = qfp_fsub(correction_value, qfp_fmul(K_POSITION_CORRECTION_VALUE, id));

  if (duty_cycle < 5 || motor_speed_erps < 80) // avoid PI controller windup
  { // motor_speed_erps < 80 seems a good value to avoid motor stalling at start up, very low speed
    correction_value = 0.0;
  }
  if (correction_value > 30.0) { correction_value = 30.0; }
  if (correction_value < -30.0) { correction_value = -30.0; }
  position_correction_value = (int) correction_value;
  // ------------------------------------------------------------------------


  static unsigned int loop_timer1 = 0;
  loop_timer1++;
  if (loop_timer1 > 1)
  {
    loop_timer1 = 0;
    balance_controller ();
  }

  static unsigned int loop_timer = 0;
  loop_timer++;
  if (loop_timer > 10)
  {
    loop_timer = 0;

    int motor_speed = (int) motor_speed_erps;
    if (get_motor_rotation_direction() == LEFT) motor_speed *= -1;

    printf ("%d, %d, %.2f, %.2f\n", motor_speed, duty_cycle, angle_log, angle_error_log);
  }
}

// runs every 50us (PWM frequency)
void FOC_fast_loop (void)
{
  // count number of fast loops / PWM cycles
  if (PWM_cycles_counter < PWM_CYCLES_COUNTER_MAX)
  {
    PWM_cycles_counter++;
  }
  else
  {
    PWM_cycles_counter = 0;
    motor_speed_erps = 0;
    PWM_cycles_per_SVM_TABLE_step = PWM_CYCLES_COUNTER_MAX / SVM_TABLE_LEN;
  }

  // calculate the interpolation angle
  // interpolation seems a problem when motor starts, so avoid to do it at very low speed
  if (duty_cycle >= 5 || motor_speed_erps >= 80)
  {
    interpolation_PWM_cycles_counter++;
    if (interpolation_PWM_cycles_counter > PWM_cycles_per_SVM_TABLE_step)
    {
      interpolation_PWM_cycles_counter = 0;
      if (interpolation_counter <= 60) // limit max interpolation value/angle
      {
	interpolation_counter++;

	if (get_motor_rotation_direction() == RIGHT)
	{
	  motor_rotor_position = (motor_rotor_position + 359) % 360; // the same as motor_rotor_position--
	}
	else
	{
	  motor_rotor_position = (motor_rotor_position + 1) % 360;
	}
      }
      else
      {
	// keep this value static over the loops, when interpolation_position >= 60
	interpolation_PWM_cycles_counter = PWM_cycles_per_SVM_TABLE_step;
      }
    }
  }

  pwm_duty_cycle_controller ();

  // measure raw currents A and C and filter
  adc_phase_a_current1 += (int) adc_get_phase_a_current_value ();
  adc_phase_c_current1 += (int) adc_get_phase_c_current_value ();
  adc_phase_current1_cycles++;
}

// calc the DC offset value for the current ADCs
void motor_calc_current_dc_offset (void)
{
  unsigned int i = 0;
  while (i <= 200)
  {
    i++;
    adc_phase_a_current_offset += adc_get_phase_a_current_value ();
    adc_phase_c_current_offset += adc_get_phase_c_current_value ();
    delay_ms (10);
  }
  adc_phase_a_current_offset /= 200;
  adc_phase_c_current_offset /= 200;
}

void hall_sensors_read_and_action (void)
{
  #define HALL_SENSORS_MASK (HALL_SENSOR_A__PIN | HALL_SENSOR_B__PIN | HALL_SENSOR_C__PIN)

  unsigned int hall_sensors = 0;
  static unsigned int flag_count_speed = 0;

  // read hall sensors signal pins and mask other pins
  hall_sensors = (GPIO_ReadInputData (HALL_SENSORS__PORT) & (HALL_SENSORS_MASK));

  if (get_motor_rotation_direction() == LEFT)
  {
    switch (hall_sensors)
    {
      // -15º
      case 8192:
      motor_rotor_absolute_position = 320; // 3
      break;

      case 24576: // transition to positive value of hall sensor A
      motor_rotor_absolute_position = 260; // 4
      break;

      case 16384:
      motor_rotor_absolute_position = 200; // 5
      flag_count_speed = 1;
      break;

      case 20480:
      motor_rotor_absolute_position = 140; // 6
      break;

      case 4096:
      motor_rotor_absolute_position = 80; // 1
      break;

      case 12288:
      motor_rotor_absolute_position = 20; // 2

      // count speed only when motor did rotate half of 1 electronic rotation
      if (flag_count_speed)
      {
	  flag_count_speed = 0;
	  motor_speed_erps = PWM_CYCLES_COUNTER_MAX / PWM_cycles_counter;
	  PWM_cycles_per_SVM_TABLE_step = PWM_cycles_counter / SVM_TABLE_LEN;
	  PWM_cycles_counter = 0;
      }
      break;

      default:
      return;
      break;
    }

    motor_rotor_position = (motor_rotor_absolute_position + position_correction_value) % 360;
    if (motor_rotor_position < 0) { motor_rotor_position *= -1; } // angle value can be negative in some values of position_correction_value negative, need to convert to positive
    interpolation_counter = 0;
    interpolation_PWM_cycles_counter = 0;
  }
  else if (get_motor_rotation_direction() == RIGHT)
  {
    switch (hall_sensors)
    {
      // +15º
      case 8192:
      motor_rotor_absolute_position = 176; // 6
      break;

      case 24576: // transition to positive value of hall sensor A
      motor_rotor_absolute_position = 116; // 1
      break;

      case 16384:
      motor_rotor_absolute_position = 56; // 2
      flag_count_speed = 1;
      break;

      case 20480:
      motor_rotor_absolute_position = 356; // 3
      break;

      case 4096:
      motor_rotor_absolute_position = 296; // 4
      break;

      case 12288:
      motor_rotor_absolute_position = 236; // 5

      if (flag_count_speed)
      {
	  flag_count_speed = 0;
	  motor_speed_erps = PWM_CYCLES_COUNTER_MAX / PWM_cycles_counter;
	  PWM_cycles_per_SVM_TABLE_step = PWM_cycles_counter / SVM_TABLE_LEN;
	  PWM_cycles_counter = 0;
      }
      break;

      default:
      return;
      break;
    }

    motor_rotor_position = (motor_rotor_absolute_position + position_correction_value) % 360;
    if (motor_rotor_position < 0) { motor_rotor_position *= -1; } // angle value can be negative in some values of position_correction_value negative
    interpolation_counter = 0;
    interpolation_PWM_cycles_counter = 0;
  }
}

void hall_sensors_interrupt (void)
{
  hall_sensors_read_and_action ();
  apply_duty_cycle ();
}
