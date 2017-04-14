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
#include "motor.h"
#include "IMU/imu.h"
#include "qfplib-m3.h"
#include "balance_controller.h"

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

  duty_cycle_f = qfp_fadd(duty_cycle_f, qfp_fmul(KI_BALANCE_CONTROLLER, angle_error));

  // calc dt, using micro seconds value
  micros_new = micros ();
  dt = qfp_fdiv((float) (micros_new - micros_old), 1000000.0);
  micros_old = micros_new;

  temp = qfp_fmul(qfp_fsub(angle_error, angle_error_old), dt);
  angle_error_old = angle_error;
  duty_cycle_f = qfp_fadd(duty_cycle_f, qfp_fmul(KD_BALANCE_CONTROLLER, temp));

  // limit value
  if (duty_cycle_f > 1000) { duty_cycle_f = 1000; }
  if (duty_cycle_f < -999) { duty_cycle_f = -999; }

duty_cycle_f = 150;
  set_pwm_duty_cycle ((int) duty_cycle_f);
}
