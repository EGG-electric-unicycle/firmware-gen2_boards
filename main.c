/*
 * EGG Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015, 2106.
 *
 * Released under the GPL License, Version 3
 */

#include "stm32f10x.h"
#include "gpio.h"
#include "main.h"
#include "stdio.h"
#include "leds.h"
#include "filter.h"
#include "math.h"
#include "qfplib-m3.h"

static unsigned int _ms;

void delay_ms (unsigned int ms)
{
  _ms = 1;
  while (ms >= _ms) ;
}

void SysTick_Handler(void) // runs every 1ms
{
  // for delay_ms ()
  _ms++;
}

void initialize (void)
{
  /* Setup SysTick Timer for 1 millisecond interrupts, also enables Systick and Systick-Interrupt */
  if (SysTick_Config(SystemCoreClock / 1000))
  {
    /* Capture error */
    while (1);
  }

//  TIM2_init ();
  gpio_init ();
  adc_init ();
  pwm_init ();
  buzzer_init ();
  usart1_bluetooth_init ();
  hall_sensor_init ();
//  MPU6050_I2C_Init ();
//  MPU6050_Initialize ();
}

// nan and infinity check for floats
#define UTILS_IS_INF(x)		((x) == (1.0 / 0.0) || (x) == (-1.0 / 0.0))
#define UTILS_IS_NAN(x)		((x) != (x))

static volatile float m_observer_x1;
static volatile float m_observer_x2;
static volatile float m_phase_now_observer;

float utils_fast_atan2(float y, float x) {
	float abs_y = fabsf(y) + 1e-10; // kludge to prevent 0/0 condition
	float angle;

	if (x >= 0) {
		float r = (x - abs_y) / (x + abs_y);
		float rsq = r * r;
		angle = ((0.1963 * rsq) - 0.9817) * r + (M_PI / 4.0);
	} else {
		float r = (x + abs_y) / (abs_y - x);
		float rsq = r * r;
		angle = ((0.1963 * rsq) - 0.9817) * r + (3.0 * M_PI / 4.0);
	}

	if (y < 0) {
		return(-angle);
	} else {
		return(angle);
	}
}

// See http://cas.ensmp.fr/~praly/Telechargement/Journaux/2010-IEEE_TPEL-Lee-Hong-Nam-Ortega-Praly-Astolfi.pdf
void observer_update(float v_alpha, float v_beta, float i_alpha, float i_beta,
		float dt, volatile float *x1, volatile float *x2, volatile float *phase) {

	//const float L = (3.0 / 2.0) * m_conf->foc_motor_l;
	const float L = MOTOR_L;
	//const float R = (3.0 / 2.0) * m_conf->foc_motor_r;
	const float R = MOTOR_R;
	//const float gamma = m_conf->foc_observer_gain;
	const float gamma = MOTOR_GAMA;
	//const float linkage = m_conf->foc_motor_flux_linkage;
	const float linkage = MOTOR_LINKAGE;

	const float Lia = L * i_alpha;
	const float Lib = L * i_beta;

	float k1 = (linkage * linkage) - ((*x1 - Lia) * (*x1 - Lia) + (*x2 - Lib) * (*x2 - Lib));
	float x1_dot = 0.0;
	float x2_dot = 0.0;

	x1_dot = -R * i_alpha + v_alpha + ((gamma / 2.0) * (*x1 - Lia)) * k1;
	x2_dot = -R * i_beta + v_beta + ((gamma / 2.0) * (*x2 - Lib)) * k1;
	*x1 += x1_dot * dt;
	*x2 += x2_dot * dt;

	if (fabsf(*x1) > 1e20 || UTILS_IS_NAN(*x1)) {
		*x1 = 0.0;
	}

	if (fabsf(*x2) > 1e20 || UTILS_IS_NAN(*x2)) {
		*x2 = 0.0;
	}

	*phase = utils_fast_atan2(*x2 - L * i_beta, *x1 - L * i_alpha);
}

// See http://cas.ensmp.fr/~praly/Telechargement/Journaux/2010-IEEE_TPEL-Lee-Hong-Nam-Ortega-Praly-Astolfi.pdf
void observer_update_FAST(float v_alpha, float v_beta, float i_alpha, float i_beta,
		float dt, volatile float *x1, volatile float *x2, volatile float *phase)
{
  //const float L = (3.0 / 2.0) * m_conf->foc_motor_l;
  const float L = MOTOR_L;
  //const float R = (3.0 / 2.0) * m_conf->foc_motor_r;
  const float R = MOTOR_R;
  //const float gamma = m_conf->foc_observer_gain;
  const float gamma = MOTOR_GAMA;
  //const float linkage = m_conf->foc_motor_flux_linkage;
  const float linkage = MOTOR_LINKAGE;

  //const float Lia = L * i_alpha;
  const float Lia = qfp_fmul(L, i_alpha);
  //const float Lib = L * i_beta;
  const float Lib = qfp_fmul(L, i_beta);

  //float k1 = (linkage * linkage) - ((*x1 - Lia) * (*x1 - Lia) + (*x2 - Lib) * (*x2 - Lib));
  float k1 = (qfp_fmul(linkage, linkage)) - (qfp_fmul(qfp_fsub(*x1, Lia), qfp_fsub(*x1, Lia)) + qfp_fmul(qfp_fsub(*x2, Lib), qfp_fsub(*x2, Lib)));

  float x1_dot = 0.0;
  float x2_dot = 0.0;

  //x1_dot = -R * i_alpha + v_alpha + ((gamma / 2.0) * (*x1 - Lia)) * k1;
  x1_dot = qfp_fadd((qfp_fadd(qfp_fmul(-R, i_alpha), v_alpha)), (qfp_fmul(qfp_fmul((gamma / 2.0), (qfp_fsub(*x1, Lia))), k1)));
  //x2_dot = -R * i_beta + v_beta + ((gamma / 2.0) * (*x2 - Lib)) * k1;
  x2_dot = qfp_fadd((qfp_fadd(qfp_fmul(-R, i_beta), v_beta)), (qfp_fmul(qfp_fmul((gamma / 2.0), (qfp_fsub(*x2, Lib))), k1)));

  //*x1 += x1_dot * dt;
  *x1 = qfp_fadd(*x1, qfp_fmul(x1_dot, dt));
  //*x2 += x2_dot * dt;
  *x2 = qfp_fadd(*x2, qfp_fmul(x2_dot, dt));

  if (fabsf(*x1) > 1e20 || UTILS_IS_NAN(*x1))
    *x1 = 0.0;

  if (fabsf(*x2) > 1e20 || UTILS_IS_NAN(*x2))
    *x2 = 0.0;

  //*phase = utils_fast_atan2(*x2 - L * i_beta, *x1 - L * i_alpha);
  *phase = qfp_fatan2(qfp_fsub(*x2, qfp_fmul(L, i_beta)), qfp_fsub(*x1, qfp_fmul(L, i_alpha)));
}

int main(void)
{
  initialize ();

  /* needed for printf */
  // turn off buffers, so IO occurs immediately
  setvbuf(stdin, NULL, _IONBF, 0);
  setvbuf(stdout, NULL, _IONBF, 0);
  setvbuf(stderr, NULL, _IONBF, 0);

  //static int value;
  static float value, v1;

//  enable_phase_a ();
//  enable_phase_b ();
//  enable_phase_c ();

  while (1)
  {
    while (1)
    {
      GPIO_ResetBits(BUZZER__PORT, BUZZER__PIN);
      GPIO_SetBits(BUZZER__PORT, BUZZER__PIN);
      GPIO_ResetBits(BUZZER__PORT, BUZZER__PIN);

//      void observer_update_FAST(float v_alpha, float v_beta, float i_alpha, float i_beta,
//      		float dt, volatile float *x1, volatile float *x2, volatile float *phase)

      observer_update(20.554,
		      24.5,
		      -4.432,
		      -7.123,
		      0.0001,
		      &m_observer_x1,
		      &m_observer_x2,
		      &m_phase_now_observer);

      m_observer_x1 = 20;
      m_observer_x2 = -3;

      GPIO_ResetBits(BUZZER__PORT, BUZZER__PIN);
      GPIO_SetBits(BUZZER__PORT, BUZZER__PIN);
      GPIO_ResetBits(BUZZER__PORT, BUZZER__PIN);

      observer_update_FAST(20.554,
		      24.5,
		      -4.432,
		      -7.123,
		      0.0001,
		      &m_observer_x1,
		      &m_observer_x2,
		      &m_phase_now_observer);

      m_observer_x1 = 20;
      m_observer_x2 = -3;

      GPIO_SetBits(BUZZER__PORT, BUZZER__PIN);
      GPIO_ResetBits(BUZZER__PORT, BUZZER__PIN);
      GPIO_SetBits(BUZZER__PORT, BUZZER__PIN);


      printf("pot: %d\n", m_observer_x1);
      printf("pot: %d\n", m_observer_x2);
      printf("pot: %d\n", m_phase_now_observer);

//	printf("pot: %d\n", value);
    }


//    delay_ms (10);
//
//    value = adc_get_potentiometer_value ();
//    value = ema_filter (value);
//
//    //value = (value * 1000) / 4096;
//
//    value = value - 2048;
//    value = value * 1000;
//    value = value / 2048;
//
//    motor_set_duty_cycle (value);
//    printf("pot: %d\n", value);


//    apply_duty_cycle ();

//    balance_controller ();

//    value = (adc_get_phase_a_current_value ());
//    printf("adc phase a: %d\n", value);
//    //printf("voltage adc phase a: %d\n\n", ((value * K_ADC_VOLTAGE) / 100));
//

//
//    value = (adc_get_battery_voltage_value ());
//    printf("battery voltage: %d\n", value);
//    //printf("voltage adc phase c: %d\n\n", ((value * K_ADC_VOLTAGE) / 100));
  }
}

