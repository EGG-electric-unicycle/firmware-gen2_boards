/*
 * EGG Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015, 2106.
 *
 * Released under the GPL License, Version 3
 */

#include "stm32f10x.h"
#include "gpio.h"

void leds_init (void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // battery LED indicator pins
  GPIO_InitStructure.GPIO_Pin = LED_1_BATTERY_INDICATOR__PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(LED_1_BATTERY_INDICATOR__PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = LED_2_BATTERY_INDICATOR__PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(LED_2_BATTERY_INDICATOR__PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = LED_3_BATTERY_INDICATOR__PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(LED_3_BATTERY_INDICATOR__PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = LED_4_BATTERY_INDICATOR__PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(LED_4_BATTERY_INDICATOR__PORT, &GPIO_InitStructure);
}

unsigned char led1_on (void)
{
  GPIO_SetBits(LED_1_BATTERY_INDICATOR__PORT, LED_1_BATTERY_INDICATOR__PIN);
}

unsigned char led2_on (void)
{
  GPIO_SetBits(LED_2_BATTERY_INDICATOR__PORT, LED_2_BATTERY_INDICATOR__PIN);
}

unsigned char led3_on (void)
{
  GPIO_SetBits(LED_3_BATTERY_INDICATOR__PORT, LED_3_BATTERY_INDICATOR__PIN);
}

unsigned char led4_on (void)
{
  GPIO_SetBits(LED_4_BATTERY_INDICATOR__PORT, LED_4_BATTERY_INDICATOR__PIN);
}

unsigned char led1_off (void)
{
  GPIO_ResetBits(LED_1_BATTERY_INDICATOR__PORT, LED_1_BATTERY_INDICATOR__PIN);
}

unsigned char led2_off (void)
{
  GPIO_ResetBits(LED_2_BATTERY_INDICATOR__PORT, LED_2_BATTERY_INDICATOR__PIN);
}

unsigned char led3_off (void)
{
  GPIO_ResetBits(LED_3_BATTERY_INDICATOR__PORT, LED_3_BATTERY_INDICATOR__PIN);
}

unsigned char led4_off (void)
{
  GPIO_ResetBits(LED_4_BATTERY_INDICATOR__PORT, LED_4_BATTERY_INDICATOR__PIN);
}
