/*
 * Generic Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2016.
 *
 * Released under the GPL License, Version 3
 */

#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "main.h"

unsigned int overflow_counter = 0;

unsigned int micros (void)
{
  return (TIM_GetCounter (TIM3) + (overflow_counter * 65536));
}

// This interrupt fire after each TIM3 overflow, 65536us
void TIM3_IRQHandler (void)
{
  overflow_counter++;

  /* Clear TIM3 TIM_IT_Update pending interrupt bit */
  TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
}

void TIM3_init(void)
{
  // enable TIM3 clock
  RCC_APB1PeriphClockCmd (RCC_APB1Periph_TIM3, ENABLE);

  // reset TIM3
  TIM_DeInit (TIM3);

  /* Time base configuration */
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_TimeBaseStructure.TIM_Period = 65535;
  TIM_TimeBaseStructure.TIM_Prescaler = (64 - 1); // 64MHz clock (PCLK1), 64MHz/64 = 1MHz --> 1us each increment of the counter/timer
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit (TIM3, &TIM_TimeBaseStructure);

  /* Enable the TIM3 gloabal Interrupt */
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIM3_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init (&NVIC_InitStructure);

  /* TIM3 TIM_IT_Update enable */
  TIM_ITConfig (TIM3, TIM_IT_Update, ENABLE);

  /* TIM3 counter enable */
  TIM_Cmd (TIM3, ENABLE);
}
