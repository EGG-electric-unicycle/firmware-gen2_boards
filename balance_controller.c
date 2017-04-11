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
#include "motor.h"
#include "IMU/imu.h"
#include "qfplib-m3.h"
#include "balance_controller.h"

// called at each 10ms
void balance_controller(void)
{
  float angle_error;
  static float duty_cycle_f = 0;

  angle_error = IMU_get_angle_error ();

//  angle_error = qfp_fmul(angle_error, -1.0);

  duty_cycle_f = qfp_fadd(duty_cycle_f, qfp_fmul(K_BALANCE_CONTROLLER, angle_error));

  // limit value
  if (duty_cycle_f > 1000) { duty_cycle_f = 1000; }
  if (duty_cycle_f < -999) { duty_cycle_f = -999; }

  set_pwm_duty_cycle ((int) duty_cycle_f);
}
