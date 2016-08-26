/*
 * EGG Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015, 2106.
 *
 * Released under the GPL License, Version 3
 */

#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "gpio.h"
#include "buzzer.h"

GPIO_InitTypeDef GPIO_InitStructure;

void gpio_init (void)
{
  /* Enable clocks */
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO  |
                          RCC_APB2Periph_GPIOA |
                          RCC_APB2Periph_GPIOB
                          , ENABLE);

  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

  // mosfets drivers pins
  GPIO_InitStructure.GPIO_Pin = PHASE_A_SHUTDOWN__PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(PHASE_A_SHUTDOWN__PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = PHASE_A_HO_LO__PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(PHASE_A_HO_LO__PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = PHASE_B_SHUTDOWN__PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(PHASE_B_SHUTDOWN__PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = PHASE_B_HO_LO__PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(PHASE_B_HO_LO__PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = PHASE_C_SHUTDOWN__PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(PHASE_C_SHUTDOWN__PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = PHASE_C_HO_LO__PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(PHASE_C_HO_LO__PORT, &GPIO_InitStructure);

  // hall sensors pins
  GPIO_InitStructure.GPIO_Pin = HALL_SENSOR_A__PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(HALL_SENSOR_A__PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = HALL_SENSOR_B__PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(HALL_SENSOR_B__PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = HALL_SENSOR_C__PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(HALL_SENSOR_C__PORT, &GPIO_InitStructure);

  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12);
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource13);
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource14);

  // buzzer pin
  GPIO_InitStructure.GPIO_Pin = BUZZER__PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(BUZZER__PORT, &GPIO_InitStructure);

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

  // USART for bluetooth module pins
  GPIO_InitStructure.GPIO_Pin = USART_RX__PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(USART_RX__PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = USART_TX__PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(USART_TX__PORT, &GPIO_InitStructure);
}
