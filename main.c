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
#include "fix16.h"

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

  gpio_init (); // configure pins just after PWM init
  buzzer_init ();
  usart1_init ();
}

int main(void)
{
  initialize ();

  /* needed for printf */
  // turn off buffers, so IO occurs immediately
  setvbuf(stdin, NULL, _IONBF, 0);
  setvbuf(stdout, NULL, _IONBF, 0);
  setvbuf(stderr, NULL, _IONBF, 0);

  int int_a = 10000;
  int int_b = 2;

  float float_a = 10000;
  float float_b = 1.01;

  fix16_t fix_a = fix16_from_float (float_a);
  fix16_t fix_b = fix16_from_float (float_b);

  while (1)
  {
     delay_ms (100);

     led1_on ();
     int_a = (int_a / int_b) + 10;
     int_a = (int_a * int_b) + 10;
     led1_off ();

     led2_on ();
     float_a = float_a / ((float) int_a) + 10;
     float_a = float_a * ((float) int_a) + 10;
     led2_off ();

     led3_on ();
     fix_a = fix16_div (fix_a, fix_b);
     fix_a = fix16_mul (fix_a, fix_b);
     led3_off ();

     printf("int_a: %d\n", int_a);
     printf("float_a: %f\n", float_a);
     printf("fix_a: %f\n", fix16_to_float (fix_a));
  }
}

