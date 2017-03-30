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
#include "gpio.h"
#include "pwm.h"
#include "motor.h"
#include "qfplib-m3.h"
#include "math.h"
#include "main.h"
#include "filter.h"

unsigned int motor_speed_erps = 0; // motor speed in electronic rotations per second
unsigned int PWM_cycles_per_SVM_TABLE_step = 0;
unsigned int PWM_cycles_counter = 0;
unsigned int interpolation_PWM_cycles_counter = 0;
int motor_rotor_position = 0; // in degrees
unsigned int motor_rotor_absolute_position = 0; // in degrees
unsigned int interpolation_counter = 0;
int position_correction_value = 0; // in degrees

int adc_phase_a_current;
int adc_phase_b_current;
int adc_phase_c_current;

unsigned int adc_phase_a_current_offset;
unsigned int adc_phase_c_current_offset;

static unsigned int _direction = RIGHT;

unsigned int duty_cycle = 0;
unsigned int motor_commanded_iq_current = 0; // in milliamps

// Space Vector Modulation PWMs values, please read this blog message:
// http://www.berryjam.eu/2015/04/driving-bldc-gimbals-at-super-slow-speeds-with-arduino/
// Please see file: BLDC_SPWM_Lookup_tables.ods
#define SVM_TABLE_LEN 360
unsigned int svm_table [SVM_TABLE_LEN] =
{
    923	,
    947	,
    970	,
    994	,
    1017	,
    1041	,
    1064	,
    1087	,
    1111	,
    1134	,
    1157	,
    1180	,
    1203	,
    1226	,
    1249	,
    1271	,
    1294	,
    1316	,
    1339	,
    1361	,
    1383	,
    1405	,
    1427	,
    1448	,
    1470	,
    1491	,
    1512	,
    1533	,
    1554	,
    1574	,
    1581	,
    1587	,
    1594	,
    1600	,
    1606	,
    1611	,
    1617	,
    1622	,
    1627	,
    1632	,
    1636	,
    1640	,
    1644	,
    1648	,
    1652	,
    1655	,
    1659	,
    1661	,
    1664	,
    1667	,
    1669	,
    1671	,
    1673	,
    1674	,
    1676	,
    1677	,
    1677	,
    1678	,
    1678	,
    1678	,
    1678	,
    1678	,
    1677	,
    1677	,
    1676	,
    1674	,
    1673	,
    1671	,
    1669	,
    1667	,
    1664	,
    1661	,
    1659	,
    1655	,
    1652	,
    1648	,
    1644	,
    1640	,
    1636	,
    1632	,
    1627	,
    1622	,
    1617	,
    1611	,
    1606	,
    1600	,
    1594	,
    1587	,
    1581	,
    1574	,
    1581	,
    1587	,
    1594	,
    1600	,
    1606	,
    1611	,
    1617	,
    1622	,
    1627	,
    1632	,
    1636	,
    1640	,
    1644	,
    1648	,
    1652	,
    1655	,
    1659	,
    1661	,
    1664	,
    1667	,
    1669	,
    1671	,
    1673	,
    1674	,
    1676	,
    1677	,
    1677	,
    1678	,
    1678	,
    1678	,
    1678	,
    1678	,
    1677	,
    1677	,
    1676	,
    1674	,
    1673	,
    1671	,
    1669	,
    1667	,
    1664	,
    1661	,
    1659	,
    1655	,
    1652	,
    1648	,
    1644	,
    1640	,
    1636	,
    1632	,
    1627	,
    1622	,
    1617	,
    1611	,
    1606	,
    1600	,
    1594	,
    1587	,
    1581	,
    1574	,
    1554	,
    1533	,
    1512	,
    1491	,
    1470	,
    1448	,
    1427	,
    1405	,
    1383	,
    1361	,
    1339	,
    1316	,
    1294	,
    1271	,
    1249	,
    1226	,
    1203	,
    1180	,
    1157	,
    1134	,
    1111	,
    1087	,
    1064	,
    1041	,
    1017	,
    994	,
    970	,
    947	,
    923	,
    900	,
    876	,
    852	,
    829	,
    805	,
    782	,
    758	,
    735	,
    712	,
    688	,
    665	,
    642	,
    619	,
    596	,
    573	,
    550	,
    528	,
    505	,
    483	,
    460	,
    438	,
    416	,
    394	,
    372	,
    351	,
    329	,
    308	,
    287	,
    266	,
    245	,
    225	,
    218	,
    212	,
    205	,
    199	,
    193	,
    188	,
    182	,
    177	,
    172	,
    167	,
    163	,
    159	,
    155	,
    151	,
    147	,
    144	,
    140	,
    138	,
    135	,
    132	,
    130	,
    128	,
    126	,
    125	,
    123	,
    122	,
    122	,
    121	,
    121	,
    121	,
    121	,
    121	,
    122	,
    122	,
    123	,
    125	,
    126	,
    128	,
    130	,
    132	,
    135	,
    138	,
    140	,
    144	,
    147	,
    151	,
    155	,
    159	,
    163	,
    167	,
    172	,
    177	,
    182	,
    188	,
    193	,
    199	,
    205	,
    212	,
    218	,
    225	,
    218	,
    212	,
    205	,
    199	,
    193	,
    188	,
    182	,
    177	,
    172	,
    167	,
    163	,
    159	,
    155	,
    151	,
    147	,
    144	,
    140	,
    138	,
    135	,
    132	,
    130	,
    128	,
    126	,
    125	,
    123	,
    122	,
    122	,
    121	,
    121	,
    121	,
    121	,
    121	,
    122	,
    122	,
    123	,
    125	,
    126	,
    128	,
    130	,
    132	,
    135	,
    138	,
    140	,
    144	,
    147	,
    151	,
    155	,
    159	,
    163	,
    167	,
    172	,
    177	,
    182	,
    188	,
    193	,
    199	,
    205	,
    212	,
    218	,
    225	,
    245	,
    266	,
    287	,
    308	,
    329	,
    351	,
    372	,
    394	,
    416	,
    438	,
    460	,
    483	,
    505	,
    528	,
    550	,
    573	,
    596	,
    619	,
    642	,
    665	,
    688	,
    712	,
    735	,
    758	,
    782	,
    805	,
    829	,
    852	,
    876	,
    900
};

// runs every 1ms
void FOC_slow_loop (void)
{
  // Get potentiometer value and adapt to desired current value and motor direction
  unsigned int adc_potentiometer_value = adc_get_potentiometer_value ();
  static unsigned int moving_average = 4095 / 2;
  unsigned int alpha = 20;
  adc_potentiometer_value = ema_filter_uint32 (&adc_potentiometer_value, &moving_average, &alpha);
  int value = ((int) adc_potentiometer_value) - 2048;
  value = value * MOTOR_MAX_CURRENT;
  value = value / 2048;
  motor_set_current (value);

  //---------------------------
  // Clarke transform assuming balanced currents

  // measure raw currents A and C and filter
  adc_phase_a_current = adc_get_phase_a_current_value ();
  adc_phase_c_current = adc_get_phase_c_current_value ();

  // removing DC offset
  adc_phase_a_current = adc_phase_a_current - adc_phase_a_current_offset;
  adc_phase_c_current = adc_phase_c_current - adc_phase_c_current_offset;

  /* Calc phase B current assuming balanced currents
   * considering: a + b + c = 0 ; a + c = -b ; b = -(a + c) ; b = -a -c
   */
  adc_phase_b_current = -adc_phase_a_current - adc_phase_c_current;

  // calc ia and ib in milliamps
  float ia = qfp_fmul(adc_phase_a_current, ADC_CURRENT_GAIN_MILLIAMPS);
  float ib = qfp_fmul(adc_phase_b_current, ADC_CURRENT_GAIN_MILLIAMPS);

  float i_alpha = ib;
  float i_beta = qfp_fadd(qfp_fmul(ONE_BY_SQRT3, ib), qfp_fmul(TWO_BY_SQRT3, ia));

  float motor_rotor_position_radians = degrees_to_radiands(motor_rotor_position);
  float id = qfp_fadd(qfp_fmul(ib, qfp_fcos(motor_rotor_position_radians)), qfp_fmul(ia, qfp_fsin(motor_rotor_position_radians)));
  float iq = qfp_fadd(qfp_fmul(-ib, qfp_fsin(motor_rotor_position_radians)), qfp_fmul(ia, qfp_fcos(motor_rotor_position_radians)));

  // Filter Id and Iq currents
  float alpha_idiq = 20.0;
  static float moving_average_id = 0.0;
  static float moving_average_iq = 0.0;
  id = ema_filter_float(&id, &moving_average_id, &alpha_idiq);
  iq = ema_filter_float(&iq, &moving_average_iq, &alpha_idiq);

  // ------------------------------------------------------------------------
  // Calculate phase/angle correction value to try keep id current = 0
  static float correction_value = 0;
  correction_value = qfp_fadd(correction_value, qfp_fmul(K_POSITION_CORRECTION_VALUE, id));

  if (duty_cycle < 5 || motor_speed_erps < 80) // avoid PI controller windup
  { // motor_speed_erps < 80 seems a good value to avoid motor stalling at start up, very low speed
    correction_value = 0.0;
  }
  if (correction_value > 30.0) { correction_value = 30.0; }
  if (correction_value < -30.0) { correction_value = -30.0; }
  position_correction_value = (int) correction_value;
  // ------------------------------------------------------------------------

  // ------------------------------------------------------------------------
  // Calculate target iq current
  static float duty_cycle_f = 0;

  float iq_current_error = qfp_fsub(motor_commanded_iq_current, iq);
  duty_cycle_f = qfp_fadd(duty_cycle_f, qfp_fmul(K_IQ_CURRENT, iq_current_error));
  if (motor_commanded_iq_current < 20) // avoid PI controller windup
  {
    duty_cycle_f = 0.0;
  }

  int temp = (int) duty_cycle_f;
  if (temp > 1000) { temp = 1000; }
  else if (temp < 0) { temp = 0; }

  motor_set_duty_cycle((unsigned int)temp); // -1000 to 1000
  // ------------------------------------------------------------------------

  static unsigned int loop_timer = 0;
  loop_timer++;
  if (loop_timer > 40)
  {
    loop_timer = 0;
    printf ("%d, %.2f; %.2f, %.2f, %.2f\n", motor_speed_erps, id, iq, correction_value, iq_current_error);
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

	if (_direction == RIGHT)
	{
	  motor_rotor_position = (motor_rotor_position - 1) % 360;
	  if (motor_rotor_position < 0) { motor_rotor_position *= -1; }
	}
	else
	{
	  motor_rotor_position = (motor_rotor_position + 1) % 360;
	  if (motor_rotor_position < 0) { motor_rotor_position *= -1; }
	}
      }
      else
      {
	// keep this value static over the loops, when interpolation_position >= 60
	interpolation_PWM_cycles_counter = PWM_cycles_per_SVM_TABLE_step;
      }
    }
  }

  apply_duty_cycle ();
}

// calc the DC offset value for the current ADCs
void motor_calc_current_dc_offset (void)
{
  unsigned int i = 0;
  while (i++ < 200)
  {
    adc_phase_a_current_offset += adc_get_phase_a_current_value ();
    adc_phase_c_current_offset += adc_get_phase_c_current_value ();
    delay_ms (10);
  }
  adc_phase_a_current_offset /= 200;
  adc_phase_c_current_offset /= 200;
}

void apply_duty_cycle (void)
{
  unsigned int duty_cycle_value = duty_cycle;
  unsigned int value = 0;

  // scale and apply _duty_cycle
  int temp;
  temp = (motor_rotor_position + 120 + position_correction_value) % 360;
  if (temp < 0) { temp *= -1; }
  value = svm_table[(unsigned int) temp];
  if (value > MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX)
  {
    value = (value - MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX) * duty_cycle_value;
    value = value / 1000;
    value = MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX + value;
  }
  else
  {
    value = (MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX - value) * duty_cycle_value;
    value = value / 1000;
    value = MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX - value;
  }
  set_pwm_phase_a (value);

  // add 120 degrees and limit
  temp = (motor_rotor_position + position_correction_value) % 360;
  if (temp < 0) { temp *= -1; }
  value = svm_table[(unsigned int) temp];
  if (value > MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX)
  {
    value = (value - MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX) * duty_cycle_value;
    value = value / 1000;
    value = MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX + value;
  }
  else
  {
    value = (MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX - value) * duty_cycle_value;
    value = value / 1000;
    value = MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX - value;
  }
  set_pwm_phase_b (value);

  // subtract 120 degrees and limit
  temp = (motor_rotor_position + 240 + position_correction_value) % 360;
  if (temp < 0) { temp *= -1; }
  value = svm_table[(unsigned int) temp];
  if (value > MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX)
  {
    value = (value - MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX) * duty_cycle_value;
    value = value / 1000;
    value = MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX + value;
  }
  else
  {
    value = (MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX - value) * duty_cycle_value;
    value = value / 1000;
    value = MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX - value;
  }
  set_pwm_phase_c (value);
}

void hall_sensors_interrupt (void)
{
  #define HALL_SENSORS_MASK (HALL_SENSOR_A__PIN | HALL_SENSOR_B__PIN | HALL_SENSOR_C__PIN)

  static unsigned int hall_sensors = 0;
  static unsigned int flag_count_speed = 0;

  // read hall sensors signal pins and mask other pins
  hall_sensors = (GPIO_ReadInputData (HALL_SENSORS__PORT) & (HALL_SENSORS_MASK));

  if (_direction == LEFT)
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
    interpolation_counter = 0;
    interpolation_PWM_cycles_counter = 0;
  }
  else if (_direction == RIGHT)
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
    interpolation_counter = 0;
    interpolation_PWM_cycles_counter = 0;
  }

  apply_duty_cycle ();
}


// 0 < value < 1000
void motor_set_duty_cycle (unsigned int value)
{
  duty_cycle = value;
}

// -MOTOR_MAX_CURRENT < duty_cycle_value < MOTOR_MAX_CURRENT
void motor_set_current (int value)
{
  static unsigned int old_direction = 0;

  if (value > 0)
  {
    _direction = RIGHT; // no torque...
  }
  else
  {
    _direction = LEFT;
    value *= -1; // invert the value, to be positive
  }

  // if direction changes, we need to execute the code of commutation for the new update of the angle other way the motor will block
  if (_direction != old_direction)
  {
    old_direction = _direction;
    hall_sensors_interrupt ();
  }

  motor_commanded_iq_current = (unsigned int) value;
}
