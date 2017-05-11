/*
 * EGG Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015, 2106, 2017.
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
#include "IMU/imu.h"

static volatile unsigned int _ms;

int mod_angle_degrees (int a)
{
  int ret = a % 360;
  if(ret < 0)
    ret += 360;
  return ret;
}

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

  gpio_init ();
  IMU_init ();
  TIM2_init ();
  adc_init ();
//  TIM4_init ();
  buzzer_init ();
  usart1_bluetooth_init ();
  hall_sensor_init ();
  pwm_init ();
}

int main(void)
{
  /* needed for printf */
  // turn off buffers, so IO occurs immediately
  setvbuf(stdin, NULL, _IONBF, 0);
  setvbuf(stdout, NULL, _IONBF, 0);
  setvbuf(stderr, NULL, _IONBF, 0);

  initialize ();

  // don't start until the potentiometer is on the middle value --> PWM ~= 0
  unsigned int duty_cycle_value;
//  while ((duty_cycle_value = adc_get_potentiometer_value()) < 1720 ||
//      duty_cycle_value > 1880) ;
  while ((duty_cycle_value = adc_get_potentiometer_value()) < 500) ;

  motor_calc_current_dc_offset ();

  set_pwm_duty_cycle (0);
  enable_phase_a ();
  enable_phase_b ();
  enable_phase_c ();

  hall_sensors_interrupt ();

  static unsigned int moving_average = 4095 / 2;
  unsigned int alpha = 20;
  while (1)
  {
    delay_ms (1);

    FOC_slow_loop ();
  }
}

