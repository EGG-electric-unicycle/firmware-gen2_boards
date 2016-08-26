/*
 * EGG Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015, 2106.
 *
 * Released under the GPL License, Version 3
 */

#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "main.h"
#include "stdio.h"

volatile unsigned int hall_sensors_time = 0;

#define HALL_SENSORS_MASK ((1 << 0) | (1 << 1) | (1 << 2))

//unsigned int get_hall_sensors_us (void)
//{
//  return hall_sensors_time * 10; // multiply by 10 to get in us
//}

void hall_sensor_init (void)
{
  // refer to datasheet "External interrupt/event line mapping"
  // HALL_SENSOR_A__PIN, HALL_SENSOR_B__PIN, HALL_SENSOR_C__PIN are port bits number 12, 13 and 14
  EXTI_ClearITPendingBit(EXTI_Line12);
  EXTI_ClearITPendingBit(EXTI_Line13);
  EXTI_ClearITPendingBit(EXTI_Line14);

  EXTI_InitTypeDef EXTI_InitStructure;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_Line = EXTI_Line12;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  EXTI_InitStructure.EXTI_Line = EXTI_Line13;
  EXTI_Init(&EXTI_InitStructure);
  EXTI_InitStructure.EXTI_Line = EXTI_Line14;
  EXTI_Init(&EXTI_InitStructure);

  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = HALL_SENSORS_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void EXTI15_10_IRQHandler(void)
{
  while (1) { printf ("EXTI15_10_IRQHandler"); while (1) ; }

  if (EXTI_GetITStatus(EXTI_Line12) != RESET ||
      EXTI_GetITStatus(EXTI_Line13) != RESET ||
      EXTI_GetITStatus(EXTI_Line14) != RESET
      )
  {
    EXTI_ClearITPendingBit(EXTI_Line12);
    EXTI_ClearITPendingBit(EXTI_Line13);
    EXTI_ClearITPendingBit(EXTI_Line14);
  }
}
