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
#include "timer.h"
#include "pwm.h"
#include "IMU/imu.h"
#include "qfplib-m3.h"
#include "balance_controller.h"
#include "motor_foc.h"

float kp = 0;
float ki = 0;
float kd = 0;

// called at each 10ms
void balance_controller(void)
{
  float angle_error;
  static float angle_error_old = 0;
  static float duty_cycle_f = 0;
  unsigned int micros_new;
  static unsigned int micros_old = 0;
  float dt;
  float temp;

  angle_error = IMU_get_angle_error ();

  /////////////////////////////

  float p_term = 0;
  float i_term = 0;
  static float i_error = 0;
  float d_term = 0;
  float d_error = 0;
//
//  // calc dt, using micro seconds value
//  micros_new = micros ();
//  dt = qfp_fdiv((float) (micros_new - micros_old), 1000000.0);
//
//  p_term = qfp_fmul(kp, angle_error);
//
//  // Anti-windup
////  if(abs_f(duty_cycle_f) >= 999 && (((angle_error >= 0) && (i_error >= 0)) || ((angle_error < 0) && (i_error < 0))))
////  {
////    i_error = i_error;
////  }
////  else
////  {
//    i_error = qfp_fadd(i_error, qfp_fmul(angle_error, dt));
////  }
//
//  i_term = qfp_fmul(ki, i_error);
//
//  d_error = qfp_fdiv(qfp_fsub(angle_error, angle_error_old), dt);
//  angle_error_old = angle_error;
//  d_term = qfp_fmul(kd, d_error);
//
//  duty_cycle_f = qfp_fadd(qfp_fadd(p_term, i_term), d_term);

  ////////////////////////////

  duty_cycle_f = qfp_fadd(duty_cycle_f, qfp_fmul(ki, angle_error));

  // calc dt, using micro seconds value
  micros_new = micros ();
  dt = qfp_fdiv((float) (micros_new - micros_old), 1000000.0);
  micros_old = micros_new;

  temp = qfp_fmul(qfp_fsub(angle_error, angle_error_old), dt);
  angle_error_old = angle_error;
  duty_cycle_f = qfp_fadd(duty_cycle_f, qfp_fmul(kd, temp));


  ////////////////////////////

  // limit value -- max values [-999; 1000]
  if (duty_cycle_f >= 1000) { duty_cycle_f = 1000; }
  if (duty_cycle_f < -999) { duty_cycle_f = -999; }


// PWM_INPUT
#define PWM_INPUT_BALANCE_CONTROLLER 	0
#define PWM_INPUT_POTENTIOMETER		1
#define PWM_INPUT_FIXED_VALUE		2
#define PWM_INPUT PWM_INPUT_POTENTIOMETER

#if PWM_INPUT == PWM_INPUT_POTENTIOMETER
  unsigned int duty_cycle_value;
  int value;
  static unsigned int moving_average = 4095 / 2;
  unsigned int alpha = 20;
  duty_cycle_value = adc_get_potentiometer_value ();
  duty_cycle_value = ema_filter_uint32 (&duty_cycle_value, &moving_average, &alpha);
  value = ((int) duty_cycle_value) - 2048;
  value = value * 1000;
  value = value / 2048;
  duty_cycle_f = (float) value;
#elif PWM_INPUT == PWM_INPUT_FIXED_VALUE
  duty_cycle_f = -40.0;
#endif

  set_pwm_duty_cycle ((int) duty_cycle_f);
}
