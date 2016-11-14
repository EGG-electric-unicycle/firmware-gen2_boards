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


  // don't start until the potentiometer is on the middle value --> PWM ~= 0
  unsigned int duty_cycle_value;
  while ((duty_cycle_value = adc_get_potentiometer_value()) < 1720 ||
      duty_cycle_value > 1880) ;

  motor_calc_current_dc_offset ();

  int value = ((int) duty_cycle_value) - 2048;
  value = value * 1000;
  value = value / 2048;
  motor_set_duty_cycle (value);

  enable_phase_a ();
  enable_phase_b ();
  enable_phase_c ();

  commutate ();

  while (1)
  {
    delay_ms (10);

    duty_cycle_value = adc_get_potentiometer_value ();
    unsigned int alpha = 1;
    unsigned int moving_average = 4095 / 2;
    ema_filter_uint32 (&duty_cycle_value, &moving_average, &alpha);

//    float value = ((float) duty_cycle_value) / 1.138;
//    set_pwm_phase_a ((unsigned int) value);
//    set_pwm_phase_b ((unsigned int) value);
//    set_pwm_phase_c ((unsigned int) value);

    int value = ((int) duty_cycle_value) - 2048;
    value = value * 1000;
    value = value / 2048;
    motor_set_duty_cycle (value);
  }
}

