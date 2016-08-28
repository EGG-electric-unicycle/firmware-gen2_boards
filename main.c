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

  int value;

  enable_phase_a ();
  enable_phase_b ();
  enable_phase_c ();

  while (1)
  {

    delay_ms (100);

    value = adc_get_potentiometer_value ();
    value = value - 2048;
    value = value * 1000;
    value = value / 2048;

    motor_set_duty_cycle (value);
    printf("pot: %d\n", value);


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

