/*
 * EGG Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015, 2106.
 *
 * Released under the GPL License, Version 3
 */

#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "gpio.h"
#include "main.h"

unsigned int overflow_counter = 0;

unsigned int micros (void)
{
  return ((TIM_GetCounter (TIM2)) + (overflow_counter * 65536));
}

void micros_reset (void)
{
  TIM_SetCounter (TIM2, 0);
  overflow_counter = 0;
}

// Used for implementation of micros()
// This interrupt fire after each TIM2 overflow, 65536us
void TIM2_IRQHandler (void)
{
  overflow_counter++;

  /* Clear TIMx TIM_IT_Update pending interrupt bit */
  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}

void TIM2_init(void)
{
  // enable TIMx clock
  RCC_APB1PeriphClockCmd (RCC_APB1Periph_TIM2, ENABLE);

  // reset TIMx
  TIM_DeInit (TIM2);

  /* Time base configuration */
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_TimeBaseStructure.TIM_Period = 65535;
  TIM_TimeBaseStructure.TIM_Prescaler = (72 - 1); // 72MHz clock (PCLK1), 72MHz/72 = 1MHz --> 1us each increment of the counter/timer
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit (TIM2, &TIM_TimeBaseStructure);

  /* Enable the TIMx global Interrupt */
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIM2_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init (&NVIC_InitStructure);

  /* TIMx TIM_IT_Update enable */
  TIM_ITConfig (TIM2, TIM_IT_Update, ENABLE);

  /* TIM2 counter enable */
  TIM_Cmd (TIM2, ENABLE);
}


// Used for implementation of 1kHz (1ms) FOC slow loop
void TIM4_IRQHandler (void)
{
//GPIO_SetBits(BUZZER__PORT, BUZZER__PIN);
  FOC_slow_loop ();

  /* Clear TIMx TIM_IT_Update pending interrupt bit */
  TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
//GPIO_ResetBits(BUZZER__PORT, BUZZER__PIN);
}

void TIM4_init(void)
{
  // enable TIMx clock
  RCC_APB1PeriphClockCmd (RCC_APB1Periph_TIM4, ENABLE);

  // reset TIMx
  TIM_DeInit (TIM4);

  /* Time base configuration */
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_TimeBaseStructure.TIM_Period = 999;
  TIM_TimeBaseStructure.TIM_Prescaler = (72 - 1); // 72MHz clock (PCLK1), 72MHz/72 = 1MHz --> 1us each increment of the counter/timer
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit (TIM4, &TIM_TimeBaseStructure);

  /* Enable the TIMx global Interrupt */
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIM4_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init (&NVIC_InitStructure);

  /* TIMx TIM_IT_Update enable */
  TIM_ITConfig (TIM4, TIM_IT_Update, ENABLE);

  /* TIM4 counter enable */
  TIM_Cmd (TIM4, ENABLE);
}
