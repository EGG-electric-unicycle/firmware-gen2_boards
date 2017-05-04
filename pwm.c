/*
 * EGG Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015, 2106, 2017.
 *
 * Released under the GPL License, Version 3
 */

#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_adc.h"
#include "main.h"
#include "gpio.h"
#include "pwm.h"
#include "motor.h"

unsigned int TIM_DirMode(TIM_TypeDef* TIMx)
{
  if ((TIMx->CR1 & 0x10) == 0x10) // verify DIR bit state
    return 1;
  else
    return 0;
}

// This interrupt fire 2 times on every PWM period - every 50 us
void PWM_PERIOD_INTERRUPT (void)
{
  // execute the next code only 1 time on evey two PWM cycles (when upcounting) - at the middle of PWM cycle, 100us
  if (!TIM_DirMode(TIM3))
  {
    FOC_fast_loop ();

    ADC_SoftwareStartConvCmd(ADC1, ENABLE); // Start ADCs conversions - here at the middle of PWM cycle
  }
  /* Clear TIMx TIM_IT_Update pending interrupt bit */
  TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
}

void pwm_init (void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

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


  RCC_APB1PeriphClockCmd (RCC_APB1Periph_TIM3, ENABLE);

  // reset TIM3
  TIM_DeInit (TIM3);

  /* Time Base configuration */
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
  TIM_TimeBaseStructure.TIM_Period = PWM_VALUE_DUTY_CYCLE_MAX; // 72MHz clock (PCLK1), 72MHz/7200 = 10KHz (BUT PWM center aligned mode needs twice the frequency)
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 1; // will fire the TIMx_UP_IRQHandler at every PWM period (64us)
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* Configures the TIMx Update Request Interrupt source (SETS the CR1->URS bit)*/
  TIM_UpdateRequestConfig(TIM3, TIM_UpdateSource_Regular);

  /* TIMx_ARR register is buffered and so the duty-cycle value is just updated (shadow registers) at Update Event */
  TIM_ARRPreloadConfig(TIM3, ENABLE);

  NVIC_InitTypeDef NVIC_InitStructure;
  /* Configure and enable TIMx interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIM3_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable Update Event interrupt */
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

  /* Channel 1, 2, 3 Configuration in PWM mode */
  TIM_OCInitTypeDef TIM_OCInitStructure;
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  TIM_OCInitStructure.TIM_Pulse = 0; // start with 0% duty cycle
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  TIM_OC4Init(TIM3, &TIM_OCInitStructure);

  TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
  TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
  TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
  TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
  TIM_BDTRInitStructure.TIM_DeadTime = 0;
  TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
  TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;
  TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
  TIM_BDTRConfig(TIM3, &TIM_BDTRInitStructure);

  /* TIM3 counter enable */
  TIM_Cmd (TIM3, ENABLE);

  /* TIM3 Main Output Disable */
  TIM_CtrlPWMOutputs (TIM3, ENABLE);
}

void enable_phase_a (void)
{
  GPIO_SetBits (PHASE_A_SHUTDOWN__PORT, PHASE_A_SHUTDOWN__PIN);
}

void enable_phase_b (void)
{
  GPIO_SetBits (PHASE_B_SHUTDOWN__PORT, PHASE_B_SHUTDOWN__PIN);
}

void enable_phase_c (void)
{
  GPIO_SetBits (PHASE_C_SHUTDOWN__PORT, PHASE_C_SHUTDOWN__PIN);
}

void set_pwm_phase_a (unsigned int value)
{
  TIM_SetCompare4 (TIM3, value);
}

void set_pwm_phase_b (unsigned int value)
{
  TIM_SetCompare3 (TIM3, value);
}

void set_pwm_phase_c (unsigned int value)
{
  TIM_SetCompare1(TIM3, value);
}








