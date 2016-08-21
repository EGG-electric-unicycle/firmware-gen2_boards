/*
 * EGG Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015, 2106.
 *
 * Released under the GPL License, Version 3
 */

#include "stm32f10x.h"
#include "gpio.h"

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
