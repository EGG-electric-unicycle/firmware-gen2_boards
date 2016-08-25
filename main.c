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

  pwm_init ();
  gpio_init (); // configure pins just after PWM init
  buzzer_init ();
  usart1_init ();
  adc_init ();

}

int main(void)
{
  initialize ();

  /* needed for printf */
  // turn off buffers, so IO occurs immediately
  setvbuf(stdin, NULL, _IONBF, 0);
  setvbuf(stdout, NULL, _IONBF, 0);
  setvbuf(stderr, NULL, _IONBF, 0);

  unsigned int value;

  while (1)
  {
    enable_phase_a ();
    enable_phase_b ();
    set_pwm_phase_a (500);
    set_pwm_phase_b (2303 - 500);

    delay_ms (100);

    value = (adc_get_phase_a_current_value () >> 0);
    printf("adc phase a: %d\n", value);
    //printf("voltage adc phase a: %d\n\n", ((value * K_ADC_VOLTAGE) / 100));

    value = (adc_get_phase_c_current_value () >> 0);
    printf("adc phase c: %d\n", value);
    //printf("voltage adc phase c: %d\n\n", ((value * K_ADC_VOLTAGE) / 100));
  }
}

