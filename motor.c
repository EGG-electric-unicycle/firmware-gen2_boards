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
#include "pwm.h"
#include "motor.h"
#include "qfplib-m3.h"
#include "math.h"
#include "main.h"

unsigned int motor_speed_erps = 0; // motor speed in electronic rotations per second
unsigned int motor_inverse_speed__timer = 0;
float motor_rotor_position = 0; // in radians

unsigned int adc_phase_a_current_offset;
unsigned int adc_phase_c_current_offset;

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
  1928,
  2396,
  2851,
  3277,
  3392,
  3462,
  3486,
  3462,
  3392,
  3277,
  3392,
  3462,
  3486,
  3462,
  3392,
  3277,
  2851,
  2396,
  1928,
  1459,
  1004,
  578,
  463,
  393,
  369,
  393,
  463,
  578,
  463,
  393,
  369,
  393,
  463,
  578,
  1004,
  1459
};

//void FOC_fast_loop (void)
//{
//  // measure raw currents A and C
//  unsigned int adc_phase_a_current = adc_get_phase_a_current_value ();
//  unsigned int adc_phase_c_current = adc_get_phase_c_current_value ();
//
//  // filtering to remove possible signal noise
//  unsigned int adc_phase_a_current_filtered;
//  unsigned int adc_phase_c_current_filtered;
//  static unsigned int moving_average_a_current = 0;
//  static unsigned int moving_average_c_current = 0;
//  const unsigned int moving_average_current_alpha = 80;
//  adc_phase_a_current_filtered = ema_filter_uint32 (&adc_phase_a_current, &moving_average_a_current, &moving_average_current_alpha);
//  adc_phase_c_current_filtered = ema_filter_uint32 (&adc_phase_c_current, &moving_average_c_current, &moving_average_current_alpha);
//
//  // removing DC offset
//  int adc_phase_a_current_filtered_1;
//  int adc_phase_c_current_filtered_1;
//  adc_phase_a_current_filtered_1 = adc_phase_a_current_filtered - adc_phase_a_current_offset;
//  adc_phase_c_current_filtered_1 = adc_phase_c_current_filtered - adc_phase_c_current_offset;
//
//  /* Calc phase B current assuming balanced currents
//   * considering: a + b + c = 0 ; a + c = -b ; b = -(a + c) ; b = -a -c
//   */
//  int adc_phase_b_current_filtered_1;
//  adc_phase_b_current_filtered_1 = -adc_phase_a_current_filtered_1 - adc_phase_c_current_filtered_1;
//
//  // calc ia and ib in Amps
//  float ia = qfp_fmul(adc_phase_a_current_filtered_1, ADC_CURRENT_GAIN_AMPS);
//  float ib = qfp_fmul(adc_phase_b_current_filtered_1, ADC_CURRENT_GAIN_AMPS);
//
//  // Clarke transform assuming balanced currents
//  float i_alpha = ia;
//  float i_beta = qfp_fadd(qfp_fmul(ONE_BY_SQRT3, ia), qfp_fmul(TWO_BY_SQRT3, ib));
//
//  // calc voltage on each motor phase
//  int adc_v_bus = adc_get_battery_voltage_value ();
//
//  unsigned int adc_v_bus_filtered;
//  static unsigned int moving_average_adc_v_bus = 0;
//  const unsigned int moving_average_v_bus_alpha = 80;
//  adc_v_bus_filtered = ema_filter_uint32 (&adc_v_bus, &moving_average_adc_v_bus, &moving_average_v_bus_alpha); // filtering to remove possible signal noise
//
//  float v_bus = qfp_fmul(adc_v_bus, ADC_BATTERY_VOLTAGE_GAIN_VOLTS); // calc v_bus in volts
//
//  float va = qfp_fdiv(qfp_fmul(v_bus, ((float) phase_a_duty_cycle)), 100000); // needs to be divided by 100000 due to int phase_a_duty_cycle
//  float vb = qfp_fdiv(qfp_fmul(v_bus, ((float) phase_b_duty_cycle)), 100000);
//  float vc = qfp_fdiv(qfp_fmul(v_bus, ((float) phase_c_duty_cycle)), 100000);
//
//  // Clarke transform for the voltages on each phase
//  float v_alpha = qfp_fsub(qfp_fsub(qfp_fmul((2.0 / 3.0), va), qfp_fmul((1.0 / 3.0), vb)), qfp_fmul((1.0 / 3.0), vc));
//  float v_beta = qfp_fsub(qfp_fmul(ONE_BY_SQRT3, vb), qfp_fmul(ONE_BY_SQRT3, vc));
//
//  observer_update(v_alpha,
//		  v_beta,
//		  i_alpha,
//		  i_beta,
//		  MOTOR_PWM_DT,
//		  &m_observer_x1,
//		  &m_observer_x2,
//		  &m_phase_now_observer);
//
//  float angle_degrees = radians_to_degrees(m_phase_now_observer);
//  if (angle_degrees > 0 && angle_degrees < 60)
//  {
//    GPIO_SetBits(BUZZER__PORT, BUZZER__PIN);
//  }
//  else
//  {
//    GPIO_ResetBits(BUZZER__PORT, BUZZER__PIN);
//  }
//
//  // Run PLL for speed estimation
////  pll_run(m_phase_now_observer, MOTOR_PWM_DT, &m_pll_phase, &m_pll_speed);
//
//}


void calc_motor_speed (void)
{
  // calc the motor speed
  if (motor_inverse_speed__timer > 100) // let's impose at least some minimum time
  {
    motor_speed_erps = MOTOR_SPEED__MAX_INVERTED_TIME / motor_inverse_speed__timer;
    motor_inverse_speed__timer = 0;
  }
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
  int duty_cycle_value = duty_cycle;
  int value = 0;

  // invert in the case of negative value
  if (duty_cycle_value < 0)
    duty_cycle_value *= -1;

  // scale and apply _duty_cycle
  value = svm_table[svm_table_index_a];
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

  value = svm_table[svm_table_index_b];
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

  value = svm_table[svm_table_index_c];
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
      motor_rotor_position = degrees_to_radiands(60);
      break;

      case 24576:
      svm_table_index_a = 22; // 2
      svm_table_index_b = 34;
      svm_table_index_c = 10;
      motor_rotor_position = degrees_to_radiands(120);
      break;

      case 16384:
      svm_table_index_a = 16; // 3
      svm_table_index_b = 28;
      svm_table_index_c = 4;
      motor_rotor_position = degrees_to_radiands(180);
      break;

      case 20480:
      svm_table_index_a = 10; // 4
      svm_table_index_b = 22;
      svm_table_index_c = 34;
      motor_rotor_position = degrees_to_radiands(240);
      break;

      case 4096:
      svm_table_index_a = 4; // 5
      svm_table_index_b = 16;
      svm_table_index_c = 28;
      motor_rotor_position = degrees_to_radiands(310);
      break;

      case 12288:
      svm_table_index_a = 34; // 6
      svm_table_index_b = 10;
      svm_table_index_c = 22;
      motor_rotor_position = degrees_to_radiands(0);

      calc_motor_speed ();
      break;

      default:
      return;
      break;
    }
  }
  else if (_direction == LEFT)
  {
    switch (hall_sensors)
     {
      case 8192:
      svm_table_index_a = 10; // 4
      svm_table_index_b = 22;
      svm_table_index_c = 34;
      motor_rotor_position = degrees_to_radiands(240);
      break;

      case 24576:
      svm_table_index_a = 4; // 5
      svm_table_index_b = 16;
      svm_table_index_c = 28;
      motor_rotor_position = degrees_to_radiands(310);
      break;

      case 16384:
      svm_table_index_a = 34; // 6
      svm_table_index_b = 10;
      svm_table_index_c = 22;
      motor_rotor_position = degrees_to_radiands(0);
      break;

      case 20480:
      svm_table_index_a = 28; // 1
      svm_table_index_b = 4;
      svm_table_index_c = 16;
      motor_rotor_position = degrees_to_radiands(60);
      break;

      case 4096:
      svm_table_index_a = 22; // 2
      svm_table_index_b = 34;
      svm_table_index_c = 10;
      motor_rotor_position = degrees_to_radiands(120);
      break;

      case 12288:
      svm_table_index_a = 16; // 3
      svm_table_index_b = 28;
      svm_table_index_c = 4;
      motor_rotor_position = degrees_to_radiands(180);

      calc_motor_speed ();
      break;

      default:
      return;
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

// -999 < duty_cycle_value < 1000
void motor_set_duty_cycle (int duty_cycle_value)
{
  duty_cycle = duty_cycle_value;
}
