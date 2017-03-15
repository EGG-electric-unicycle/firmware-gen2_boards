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
//  buzzer_init ();
//  usart1_bluetooth_init ();
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

  int value = (int) (qfp_fsub(adc_get_potentiometer_value(), 4.096));
  motor_set_duty_cycle (value);

  enable_phase_a ();
  enable_phase_b ();
  enable_phase_c ();

  commutate ();

  unsigned int moving_average = 4095 / 2;
  unsigned int alpha = 10;
  unsigned int tx_timer = 0;
  while (1)
  {
    delay_ms (10);

    duty_cycle_value = adc_get_potentiometer_value ();
    duty_cycle_value = ema_filter_uint32 (&duty_cycle_value, &moving_average, &alpha);
    value = qfp_fdiv((float) duty_cycle_value, 4.096);
    motor_set_duty_cycle (value);

//    // Start a new usart/bluetooth transmission at fixed intervals
//    tx_timer = (tx_timer + 1) % TX_INTERVAL;
//    if (tx_timer == 0) { usart1_send_data(); }
  }
}

