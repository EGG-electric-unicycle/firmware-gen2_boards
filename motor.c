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

// nan check for floats
#define UTILS_IS_NAN(x)		((x) != (x))

// Converts degrees to radians.
#define degrees_to_radians(angle_degrees) (angle_degrees * M_PI / 180.0)

// Converts radians to degrees.
#define radians_to_degrees(angle_radians) (angle_radians * 180.0 / M_PI)

// vars for the Observer
volatile float m_observer_x1;
volatile float m_observer_x2;
volatile float m_phase_now_observer;

static volatile float m_pll_phase;
static volatile float m_pll_speed;

int phase_a_duty_cycle = 0; // needs to be divided by 1000 to get value in [0 -> 1]
int phase_b_duty_cycle = 0;
int phase_c_duty_cycle = 0;

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

void apply_duty_cycle (void)
{
  int duty_cycle_value = duty_cycle;
  int temp1 = 0;

  // invert in the case of negative value
  if (duty_cycle_value < 0)
    duty_cycle_value *= -1;

  /* scale and apply _duty_cycle (integer operations only!) */
  temp1 = svm_table[svm_table_index_a];
  if (temp1 > MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX)
  {
    temp1 = temp1 - MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX;
    temp1 = temp1 * ((unsigned int ) duty_cycle_value);

    // save phase duty_cycle for later usage
    phase_a_duty_cycle = (((unsigned int) temp1) * 500) / MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX;

    temp1 = temp1 / 1000;
    temp1 = MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX + temp1;
  }
  else
  {
    temp1 = MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX - temp1;
    temp1 = temp1 * ((unsigned int ) duty_cycle_value);

    // save phase duty_cycle for later usage
    phase_a_duty_cycle = (((unsigned int) temp1) * 500) / MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX;

    temp1 = temp1 / 1000;
    temp1 = MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX - temp1;
  }
  set_pwm_phase_a (temp1);

  temp1 = svm_table[svm_table_index_b];
  if (temp1 > MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX)
  {
    temp1 = temp1 - MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX;
    temp1 = temp1 * ((unsigned int ) duty_cycle_value);

    // save phase duty_cycle for later usage
    phase_b_duty_cycle = (((unsigned int) temp1) * 500) / MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX;

    temp1 = temp1 / 1000;
    temp1 = MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX + temp1;
  }
  else
  {
    temp1 = MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX - temp1;
    temp1 = temp1 * ((unsigned int ) duty_cycle_value);

    // save phase duty_cycle for later usage
    phase_b_duty_cycle = (((unsigned int) temp1) * 500) / MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX;

    temp1 = temp1 / 1000;
    temp1 = MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX - temp1;
  }
  set_pwm_phase_b (temp1);

  temp1 = svm_table[svm_table_index_c];
  if (temp1 > MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX)
  {
    temp1 = temp1 - MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX;
    temp1 = temp1 * ((unsigned int ) duty_cycle_value);

    // save phase duty_cycle for later usage
    phase_c_duty_cycle = (((unsigned int) temp1) * 500) / MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX;

    temp1 = temp1 / 1000;
    temp1 = MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX + temp1;
  }
  else
  {
    temp1 = MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX - temp1;
    temp1 = temp1 * ((unsigned int ) duty_cycle_value);

    // save phase duty_cycle for later usage
    phase_c_duty_cycle = (((unsigned int) temp1) * 500) / MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX;

    temp1 = temp1 / 1000;
    temp1 = MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX - temp1;
  }
  set_pwm_phase_c (temp1);
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
        svm_table_index_a = 10; // 4
        svm_table_index_b = 22;
        svm_table_index_c = 34;
      break;

      case 24576:
        svm_table_index_a = 4; // 5
        svm_table_index_b = 16;
        svm_table_index_c = 28;
      break;

      case 16384:
        svm_table_index_a = 34; // 6
        svm_table_index_b = 10;
        svm_table_index_c = 22;
      break;

      case 20480:
        svm_table_index_a = 28; // 1
        svm_table_index_b = 4;
        svm_table_index_c = 16;
      break;

      case 4096:
        svm_table_index_a = 22; // 2
        svm_table_index_b = 34;
        svm_table_index_c = 10;
      break;

      case 12288:
        svm_table_index_a = 16; // 3
        svm_table_index_b = 28;
        svm_table_index_c = 4;
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

void observer_update(float v_alpha, float v_beta, float i_alpha, float i_beta,
		float dt, volatile float *x1, volatile float *x2, volatile float *phase)
{
  const float L = MOTOR_L;
  const float R = MOTOR_R;
  const float gamma = MOTOR_GAMA;
  const float linkage = MOTOR_LINKAGE;

  const float Lia = qfp_fmul(L, i_alpha);
  const float Lib = qfp_fmul(L, i_beta);

  float k1 = (qfp_fmul(linkage, linkage)) - (qfp_fmul(qfp_fsub(*x1, Lia), qfp_fsub(*x1, Lia)) + qfp_fmul(qfp_fsub(*x2, Lib), qfp_fsub(*x2, Lib)));

  float x1_dot = 0.0;
  float x2_dot = 0.0;

  x1_dot = qfp_fadd((qfp_fadd(qfp_fmul(-R, i_alpha), v_alpha)), (qfp_fmul(qfp_fmul((gamma / 2.0), (qfp_fsub(*x1, Lia))), k1)));
  x2_dot = qfp_fadd((qfp_fadd(qfp_fmul(-R, i_beta), v_beta)), (qfp_fmul(qfp_fmul((gamma / 2.0), (qfp_fsub(*x2, Lib))), k1)));

  *x1 = qfp_fadd(*x1, qfp_fmul(x1_dot, dt));
  *x2 = qfp_fadd(*x2, qfp_fmul(x2_dot, dt));

  if (fabsf(*x1) > 1e20 || UTILS_IS_NAN(*x1))
    *x1 = 0.0;

  if (fabsf(*x2) > 1e20 || UTILS_IS_NAN(*x2))
    *x2 = 0.0;

  *phase = qfp_fatan2(qfp_fsub(*x2, qfp_fmul(L, i_beta)), qfp_fsub(*x1, qfp_fmul(L, i_alpha)));
}

/**
 * Make sure that -pi <= angle < pi,
 *
 * TODO: Maybe use fmodf instead?
 *
 * @param angle
 * The angle to normalize in radians.
 * WARNING: Don't use too large angles.
 */
void utils_norm_angle_rad (float *angle)
{
  while (*angle < -M_PI)
  {
    *angle += 2.0 * M_PI;
  }

  while (*angle >  M_PI)
  {
    *angle -= 2.0 * M_PI;
  }
}

//static void pll_run (float phase, float dt, volatile float *phase_var, volatile float *speed_var)
//{
//  float delta_theta = phase - *phase_var;
//  utils_norm_angle_rad(&delta_theta);
//  *phase_var += (*speed_var + m_conf->foc_pll_kp * delta_theta) * dt;
//  utils_norm_angle_rad((float*)phase_var);
//  *speed_var += m_conf->foc_pll_ki * delta_theta * dt;
//}

void FOC_control_loop (void)
{
  // measure raw currents A and C
  int adc_phase_a_current = adc_get_phase_a_current_value ();
  int adc_phase_c_current = adc_get_phase_c_current_value ();

  // removing
  adc_phase_a_current -= ADC_CURRENT_OFFSET;
  adc_phase_c_current -= ADC_CURRENT_OFFSET;

  /* Calc phase B current assuming balanced currents
   * considering: a + b + c = 0 ; a + c = -b ; b = -(a + c) ; b = -a -c
   */
  int adc_phase_b_current = -adc_phase_a_current - adc_phase_c_current;

  // calc ia and ib in Amps
  float ia = qfp_fmul(adc_phase_a_current, ADC_CURRENT_GAIN_AMPS);
  float ib = qfp_fmul(adc_phase_b_current, ADC_CURRENT_GAIN_AMPS);

  // Clarke transform assuming balanced currents
  float i_alpha = ia;
  float i_beta = qfp_fadd(qfp_fmul(ONE_BY_SQRT3, ia), qfp_fmul(TWO_BY_SQRT3, ib));

  // calc voltage on each motor phase
  int adc_v_bus = adc_get_battery_voltage_value ();
  float v_bus = qfp_fmul(adc_v_bus, ADC_BATTERY_VOLTAGE_GAIN_VOLTS); // calc v_bus in volts

  float va = qfp_fdiv(qfp_fmul(v_bus, ((float) phase_a_duty_cycle)), 100000); // needs to be divided by 1000 due to int phase_a_duty_cycle
  float vb = qfp_fdiv(qfp_fmul(v_bus, ((float) phase_b_duty_cycle)), 100000);
  float vc = qfp_fdiv(qfp_fmul(v_bus, ((float) phase_c_duty_cycle)), 100000);

  // Clarke transform for the voltages on each phase
  float v_alpha = qfp_fsub(qfp_fsub(qfp_fmul((2.0 / 3.0), va), qfp_fmul((1.0 / 3.0), vb)), qfp_fmul((1.0 / 3.0), vc));
  float v_beta = qfp_fsub(qfp_fmul(ONE_BY_SQRT3, vb), qfp_fmul(ONE_BY_SQRT3, vc));

  observer_update(v_alpha,
		  v_beta,
		  i_alpha,
		  i_beta,
		  MOTOR_PWM_DT,
		  &m_observer_x1,
		  &m_observer_x2,
		  &m_phase_now_observer);

  float angle_degrees = radians_to_degrees(m_phase_now_observer);
  if (angle_degrees > 0 && angle_degrees < 60)
  {
    GPIO_SetBits(BUZZER__PORT, BUZZER__PIN);
  }
  else
  {
    GPIO_ResetBits(BUZZER__PORT, BUZZER__PIN);
  }

  // Run PLL for speed estimation
//  pll_run(m_phase_now_observer, MOTOR_PWM_DT, &m_pll_phase, &m_pll_speed);

}






//void mcpwm_foc_adc_inj_int_handler(void)
//{

//    // Run observer
//    observer_update(m_motor_state.v_alpha, m_motor_state.v_beta,
//			    m_motor_state.i_alpha, m_motor_state.i_beta, MOTOR_PWM_DT,
//			    &m_observer_x1, &m_observer_x2, &m_phase_now_observer);
//
//
//
//
//  }
//
//
//
//  // Run PLL for speed estimation
//  pll_run(m_motor_state.phase, dt, &m_pll_phase, &m_pll_speed);
//
//}
