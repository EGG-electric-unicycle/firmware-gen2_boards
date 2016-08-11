/*
 * Generic Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015.
 *
 * Released under the GPL License, Version 3
*/

#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "main.h"
#include "bldc.h"

volatile unsigned int hall_sensors_time = 0;

#define HALL_SENSORS_MASK ((1 << 0) | (1 << 1) | (1 << 2))

void TIM2_IRQHandler(void)
{
  /* "Read" all sensors sequence and execute the BLDC coils commutation */
  TIM_ITConfig (TIM1, TIM_IT_Update, DISABLE); // disable to avoid concurrency access to update of PWM controller duty-cycle values
  commutate ();
  TIM_ITConfig (TIM1, TIM_IT_Update, ENABLE);

  /* Save current time between each hall sensor signal change */
  hall_sensors_time = TIM_GetCapture1 (TIM2);

  // clear interrupt flag
  TIM_ClearITPendingBit (TIM2, TIM_IT_Trigger);
}

unsigned int get_hall_sensors_us (void)
{
  return hall_sensors_time * 10; // multiply by 10 to get in us
}

void hall_sensor_init (void)
{ 
  // Enable clock for TIM2, used by hall sensors
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  // timer base configuration
  // 84 => 655ms till overflow ; 100kHz (10us) TimerClock [24MHz/Prescaler]
  TIM_TimeBaseStructure.TIM_Prescaler = 240;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = 65535;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  // enable hall sensor
  // T1F_ED will be connected to  HallSensors Inputs
  // TIM2_CH1, TIM2_CH2, TIM2_CH3
  TIM_SelectHallSensor(TIM2, ENABLE);

  // HallSensor event is delivered with signal TI1F_ED
  // (this is XOR of the three hall sensor lines)
  // Signal TI1F_ED: falling and rising edge of the inputs is used
  TIM_SelectInputTrigger(TIM2, TIM_TS_TI1F_ED);

  // On every TI1F_ED event the counter is resetted and update is tiggered
  TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);

  // Channel 1 in input capture mode
  // on every TCR edge (build from TI1F_ED which is a HallSensor edge)
  // the timervalue is copied into ccr register and a CCR1 Interrupt
  // TIM_IT_CC1 is fired
  TIM_ICInitTypeDef TIM_ICInitStructure;
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  // listen to T1, the  HallSensorEvent
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_TRC;
  // Div:1, every edge
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);

  /* Enable the TIM2 Trigger Interrupt Request */
  TIM_ITConfig(TIM2, TIM_IT_Trigger, ENABLE);

  NVIC_InitTypeDef NVIC_InitStructure;
  /* Configure and enable TIM2 interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIM2_HALL_SENSORS_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  TIM_Cmd(TIM2, ENABLE);
}
