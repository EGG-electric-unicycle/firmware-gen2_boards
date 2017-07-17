/*
 * EGG Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015, 2106, 2017.
 *
 * Released under the GPL License, Version 3
 */

#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_adc.h"
#include "main.h"
#include "gpio.h"
#include "pwm.h"
#include "pwm_duty_cycle_controller.h"
#include "motor_foc.h"

// Space Vector Modulation PWMs values, please read this blog message:
// http://www.berryjam.eu/2015/04/driving-bldc-gimbals-at-super-slow-speeds-with-arduino/
// Please see file: BLDC_SPWM_Lookup_tables.ods
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

int duty_cycle_target;
int duty_cycle = 0;
unsigned int _direction;

void apply_duty_cycle (int duty_cycle_value)
{
  int _duty_cycle = duty_cycle_value;
  unsigned int value_a = 0;
  unsigned int value_b = 0;
  unsigned int value_c = 0;
  static unsigned int old_direction = 0;

  if (_duty_cycle > 0)
  {
    _direction = RIGHT;
  }
  else
  {
   _direction = LEFT;
   _duty_cycle *= -1; // invert the value, to be always positive
  }

  // if direction changes, we need to execute the code of commutation for the new update of the angle other way the motor will block
  if (_direction != old_direction)
  {
    old_direction = _direction;
    hall_sensors_read_and_action ();
  }

  // apply over current limit factor
  _duty_cycle *=  motor_max_current_factor;
  _duty_cycle /=  1000;

  // apply minimum duty_cycle value
  // I found that there are a minimum duty_cycle value after the motor start to move, like if there is a gap with low values. This code is to remove that gap and I expect it to improve the quick motor direction change rotation that I think is important for the balance of EUC.
  int temp1 = 1000 - MOTOR_MIN_DUTYCYCLE;
  _duty_cycle = MOTOR_MIN_DUTYCYCLE + (((_duty_cycle * temp1) + MOTOR_MIN_DUTYCYCLE) / 1000);

  // apply limits
  if (_duty_cycle > 1000) _duty_cycle = 1000;
  if (_duty_cycle < 0) _duty_cycle = 0;

  // scale and apply _duty_cycle
  int temp;
  temp = motor_rotor_position;
  value_a = svm_table[(unsigned int) motor_rotor_position];
  if (value_a > MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX)
  {
    value_a = (value_a - MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX) * ((unsigned int) _duty_cycle);
    value_a = value_a / 1000;
    value_a = MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX + value_a;
  }
  else
  {
    value_a = (MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX - value_a) * ((unsigned int) _duty_cycle);
    value_a = value_a / 1000;
    value_a = MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX - value_a;
  }

  // add 120 degrees and limit
  temp = mod_angle_degrees(motor_rotor_position + 120);
  value_b = svm_table[(unsigned int) temp];
  if (value_b > MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX)
  {
    value_b = (value_b - MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX) * ((unsigned int) _duty_cycle);
    value_b = value_b / 1000;
    value_b = MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX + value_b;
  }
  else
  {
    value_b = (MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX - value_b) * ((unsigned int) _duty_cycle);
    value_b = value_b / 1000;
    value_b = MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX - value_b;
  }

  // subtract 120 degrees and limit
  temp = mod_angle_degrees(motor_rotor_position + 240);
  value_c = svm_table[(unsigned int) temp];
  if (value_c > MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX)
  {
    value_c = (value_c - MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX) * ((unsigned int) _duty_cycle);
    value_c = value_c / 1000;
    value_c = MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX + value_c;
  }
  else
  {
    value_c = (MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX - value_c) * ((unsigned int) _duty_cycle);
    value_c = value_c / 1000;
    value_c = MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX - value_c;
  }

#if MOTOR_TYPE == MOTOR_TYPE_EUC1
  set_pwm_phase_a (value_a);
  set_pwm_phase_b (value_b);
  set_pwm_phase_c (value_c);
#elif MOTOR_TYPE == MOTOR_TYPE_EUC2
  set_pwm_phase_a (value_b);
  set_pwm_phase_b (value_a);
  set_pwm_phase_c (value_c);
#elif MOTOR_TYPE == MOTOR_TYPE_MICROWORKS_500W_30KMH
  set_pwm_phase_a (value_b);
  set_pwm_phase_b (value_c);
  set_pwm_phase_c (value_a);
#endif
}

// run at each PWM period = 50us
void pwm_duty_cycle_controller (void)
{
  // limit PWM increase/decrease rate
  static unsigned int counter;
  if (counter++ > PWM_DUTY_CYCLE_CONTROLLER_COUNTER)
  {
    counter = 0;

    // increment or decrement duty_cycle
    if (duty_cycle_target > duty_cycle) { duty_cycle++; }
    else if (duty_cycle_target < duty_cycle) { duty_cycle--; }
  }

  apply_duty_cycle (duty_cycle);
}

void set_pwm_duty_cycle (int value)
{
  duty_cycle_target = value;
}

unsigned int get_motor_rotation_direction (void)
{
  return _direction;
}







