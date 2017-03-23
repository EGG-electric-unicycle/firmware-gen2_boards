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
#include "filter.h"
#include "stm32f10x_tim.h"
#include "timer.h"
#include "adc.h"
#include "motor.h"
#include "usart.h"

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

int main(void)
{
  initialize ();

  /* needed for printf */
  // turn off buffers, so IO occurs immediately
  setvbuf(stdin, NULL, _IONBF, 0);
  setvbuf(stdout, NULL, _IONBF, 0);
  setvbuf(stderr, NULL, _IONBF, 0);

  // don't start until the potentiometer is on the left side
  unsigned int duty_cycle_value;
  while (adc_get_potentiometer_value() < (4095/15)) ;

  motor_calc_current_dc_offset ();

  enable_phase_a ();
  enable_phase_b ();
  enable_phase_c ();

  commutate ();

  unsigned int moving_average = 0;
  unsigned int alpha = 20;
  unsigned int tx_timer = 0;
  int value;
  float alpha_idiq = 20.0;
  float moving_average_id = 0.0;
  float moving_average_iq = 0.0;
  float correction_value = 0;
  while (1)
  {
    delay_ms (4);

    duty_cycle_value = adc_get_potentiometer_value ();
    duty_cycle_value = ema_filter_uint32 (&duty_cycle_value, &moving_average, &alpha);
    value = qfp_fdiv((float) duty_cycle_value, 4.096);
//value = 100;
    motor_set_duty_cycle (value);


    /* ********************************************************************
     * FOC slow loop
     */
    // measure raw currents A and C and filter
    int adc_phase_a_current_filtered;
    int adc_phase_b_current_filtered;
    int adc_phase_c_current_filtered;

    adc_phase_a_current_filtered = adc_get_phase_a_current_value ();
    adc_phase_c_current_filtered = adc_get_phase_c_current_value ();

    // removing DC offset
    adc_phase_a_current_filtered = adc_phase_a_current_filtered - adc_phase_a_current_offset;
    adc_phase_c_current_filtered = adc_phase_c_current_filtered - adc_phase_c_current_offset;

    /* Calc phase B current assuming balanced currents
     * considering: a + b + c = 0 ; a + c = -b ; b = -(a + c) ; b = -a -c
     */
    adc_phase_b_current_filtered = -adc_phase_a_current_filtered - adc_phase_c_current_filtered;

    // calc ia, ib and Ic in Amps
    float ia = qfp_fmul(adc_phase_a_current_filtered, ADC_CURRENT_GAIN_AMPS);
    float ib = qfp_fmul(adc_phase_b_current_filtered, ADC_CURRENT_GAIN_AMPS);
//    float ic = qfp_fmul(adc_phase_c_current_filtered, ADC_CURRENT_GAIN_AMPS);

    float id, iq;
    float motor_rotor_position_radians = degrees_to_radiands(motor_rotor_position);
//    // ABC->dq Park transform
    // ------------------------------------------------------------------------
//    float temp;
//    temp = qfp_fmul(ib, qfp_fcos(motor_rotor_position_radians));
//    temp += qfp_fmul(ia, qfp_fcos(motor_rotor_position_radians + DEGRES_120_IN_RADIANS));
//    temp += qfp_fmul(ic, qfp_fcos(motor_rotor_position_radians - DEGRES_120_IN_RADIANS));
//    float id = qfp_fmul(temp, 2.0/3.0);
//
//    motor_rotor_position_radians = degrees_to_radiands((motor_rotor_position + 180) % 360); // invert the angle to get iq current inverted, to have the right signal
//    temp = qfp_fmul(ib, qfp_fsin(motor_rotor_position_radians));
//    temp += qfp_fmul(ia, qfp_fsin((motor_rotor_position_radians) + DEGRES_120_IN_RADIANS));
//    temp += qfp_fmul(ic, qfp_fsin((motor_rotor_position_radians) - DEGRES_120_IN_RADIANS));
//    float iq = qfp_fmul(temp, 2.0/3.0);

//    //---------------------------
//    // Clarke transform assuming balanced currents
//    float i_alpha = ia;
////    float i_beta = qfp_fmul(qfp_fsub(ib, ic), ONE_BY_SQRT3);
//    float i_beta = qfp_fadd(qfp_fmul(ONE_BY_SQRT3, ia), qfp_fmul(TWO_BY_SQRT3, ib));
//
//    id = qfp_fadd(qfp_fmul(ia, qfp_fcos(motor_rotor_position_radians)), qfp_fmul(ib, qfp_fsin(motor_rotor_position_radians)));
//    iq = qfp_fadd(qfp_fmul(-ia, qfp_fsin(motor_rotor_position_radians)), qfp_fmul(ib, qfp_fcos(motor_rotor_position_radians)));

    float i_alpha = ib;
//    float i_beta = qfp_fmul(qfp_fsub(ib, ic), ONE_BY_SQRT3);
    float i_beta = qfp_fadd(qfp_fmul(ONE_BY_SQRT3, ib), qfp_fmul(TWO_BY_SQRT3, ia));

    id = qfp_fadd(qfp_fmul(ib, qfp_fcos(motor_rotor_position_radians)), qfp_fmul(ia, qfp_fsin(motor_rotor_position_radians)));
    iq = qfp_fadd(qfp_fmul(-ib, qfp_fsin(motor_rotor_position_radians)), qfp_fmul(ia, qfp_fcos(motor_rotor_position_radians)));

    // Filter Id and Iq currents
    id = ema_filter_float(&id, &moving_average_id, &alpha_idiq);
    iq = ema_filter_float(&iq, &moving_average_iq, &alpha_idiq);

    // ------------------------------------------------------------------------
    // Calculate angle correction value to try keep id current = 0
    correction_value = qfp_fadd(correction_value, qfp_fmul(K_POSITION_CORRECTION_VALUE, id));
    if (duty_cycle == 0 || motor_speed_erps == 0) // avoid PI controller windup
    {
      correction_value = 0;
    }
    if (correction_value > 30.0) { correction_value = 30.0; }
    if (correction_value < -30.0) { correction_value = -30.0; }
    position_correction_value = (int) correction_value;

    static unsigned int loop_timer = 0;
    loop_timer++;
    if (loop_timer > 20)
    {
      loop_timer = 0;
      printf ("%d, %.2f; %.2f, %.2f\n", motor_speed_erps, id, iq, correction_value);
    }
  }
}

