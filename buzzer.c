/*
 * Generic Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015.
 *
 * Released under the GPL License, Version 3
 */

#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "gpio.h"

void buzzer_init (void)
{
  //there is nothing to init
  //the buzzer GPIO out pin is already configure on gpio.c
}

void buzzer_on (void)
{
  GPIO_ResetBits(GPIOB, BUZZER);
}

void buzzer_off (void)
{
  GPIO_SetBits(GPIOB, BUZZER);
}

