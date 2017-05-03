/*
 * EGG Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015, 2106, 2017.
 *
 * Released under the GPL License, Version 3
 */

#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "gpio.h"

void buzzer_init (void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // buzzer pin
  GPIO_InitStructure.GPIO_Pin = BUZZER__PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(BUZZER__PORT, &GPIO_InitStructure);
}

void buzzer_on (void)
{
  GPIO_ResetBits(BUZZER__PORT, BUZZER__PIN);
}

void buzzer_off (void)
{
  GPIO_SetBits(BUZZER__PORT, BUZZER__PIN);
}

