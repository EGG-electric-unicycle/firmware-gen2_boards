/*
 * Generic Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015.
 *
 * Released under the GPL License, Version 3
 */

#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "main.h"
#include "pwm.h"
#include "bldc.h"

int pwm_duty_cycle = 0;
int pwm_duty_cycle_target = 0;

// Sine wave Space-Vector PWMs values, please read this blog message:
// http://www.berryjam.eu/2015/04/driving-bldc-gimbals-at-super-slow-speeds-with-arduino/
// Please see file: BLDC_SPWM_Lookup_tables.ods
uint16_t sine_table [36] =
{
  299,
  343,
  386,
  427,
  437,
  444,
  446,
  444,
  437,
  427,
  437,
  444,
  446,
  444,
  437,
  427,
  386,
  343,
  299,
  254,
  211,
  171,
  160,
  153,
  151,
  153,
  160,
  171,
  160,
  153,
  151,
  153,
  160,
  171,
  211,
  254
};

// This interrupt fire after the end on the PWM period (64us) - TIM1 UPdate event
void PWM_PERIOD_INTERRUPT (void)
{
  static unsigned int counter = 1;

//  // at each 1.024ms (64us * 16)
//  counter++;
//  if (counter >= 16)2
//  {
//    // manage PWM only if BLDC is in normal state
//    if (bldc_get_state () == BLDC_NORMAL)
//    {
//      pwm_manage (); //manage the increase/decrease rate of PWM duty-cycle and setup new values on the PWM controller
//    }
//
//    counter = 1;
//  }

  // if current is now under the max value, enable PWM signal again
  if ((bldc_get_state () == BLDC_OVER_MAX_CURRENT) && is_current_under_max ())
  {
    TIM_CtrlPWMOutputs (TIM1, ENABLE); // enable PWM signals
    bldc_set_state (BLDC_NORMAL);
  }

  /* Clear TIM1 TIM_IT_Update pending interrupt bit */
  TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
}

void pwm_init (void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

  /* Time Base configuration */
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
  TIM_TimeBaseStructure.TIM_Period = (2048 - 1); // 64MHz clock (PCLK1), 64MHz/4096 = 15.625KHz (BUT PWM center alined mode needs twice the frequency)
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 1; // will fire the TIM1_UP_IRQHandler at every PWM period (64us)
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  /* Configures the TIM1 Update Request Interrupt source (SETS the CR1->URS bit)*/
  TIM_UpdateRequestConfig(TIM1,TIM_UpdateSource_Regular);

  /* TIMx_ARR register is buffered and so the duty-cycle value is just updated (shadow registers) at Update Event */
  TIM_ARRPreloadConfig(TIM1, ENABLE);

  NVIC_InitTypeDef NVIC_InitStructure;
  /* Configure and enable TIM1 interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIM1_UP_PWM_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable Update Event interrupt */
  TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);

  /* Channel 1, 2, 3 Configuration in PWM mode */
  TIM_OCInitTypeDef TIM_OCInitStructure;
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0; // start with 0% duty cycle
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Set;
  TIM_OC1Init(TIM1, &TIM_OCInitStructure);
  TIM_OC2Init(TIM1, &TIM_OCInitStructure);
  TIM_OC3Init(TIM1, &TIM_OCInitStructure);

  TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
  TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
  TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
  TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
  TIM_BDTRInitStructure.TIM_DeadTime = 165; // 3us dead time
  TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
  TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;
  TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;
  TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

  /* TIM1 counter enable */
  TIM_Cmd (TIM1, DISABLE);

  /* TIM1 Main Output Disable */
  TIM_CtrlPWMOutputs (TIM1, DISABLE);
}


unsigned int increment_index (unsigned int index)
{
  if (index < 35)
  {
    index++;
  }
  else
  {
    index = 0;
  }

  return index;
}

// Function to update the duty cycle PWM values on the PWM controller
// also setup the correct values of PWM duty-cycle for each BLDC phase
void pwm_update_duty_cycle (void)
{
  unsigned int duty_cycle_normal;
  unsigned int duty_cycle_inverted;

  static unsigned int index_a = 0;
  static unsigned int index_b = 12;
  static unsigned int index_c = 24;

  static unsigned int pot_value = 0;


  index_a = increment_index (index_a);
  index_b = increment_index (index_b);
  index_c = increment_index (index_c);

  //pot_value = (adc_get_PS_signal_value () >> 2); // filter and the value is now 10 bits --> max 1023.

  // Apply the duty cycle values
  TIM_SetCompare3(TIM1, (sine_table[index_a]));
  TIM_SetCompare1(TIM1, (sine_table[index_b]));
  TIM_SetCompare2(TIM1, (sine_table[index_c]));




//  // Calc the correct value of duty cycle and inverted duty cycle
//  duty_cycle_normal = (unsigned int) ((pwm_duty_cycle + 999) * 1.024); // scale to correct value
//  duty_cycle_inverted = 2047 - duty_cycle_normal;
//
//
//  if (duty_cycle_normal > 0)
//  {
//    set_direction (RIGHT);
//  }
//  else
//  {
//    set_direction (LEFT);
//  }

//  // Apply the duty cycle values
//  if (bldc_phase_state.a == NORMAL)
//  {
//    TIM_SetCompare3(TIM1, duty_cycle_normal); // phase A
//  }
//  else if (bldc_phase_state.a == INVERTED)
//  {
//    TIM_SetCompare3(TIM1, duty_cycle_inverted);
//  }
//  else if (bldc_phase_state.a == OFF)
//  {
//    TIM_SetCompare3(TIM1, DUTY_CYCLE_OFF);
//  }
//
//
//  if (bldc_phase_state.b == NORMAL)
//  {
//    TIM_SetCompare1(TIM1, duty_cycle_normal); // phase B
//  }
//  else if (bldc_phase_state.b == INVERTED)
//  {
//    TIM_SetCompare1(TIM1, duty_cycle_inverted);
//  }
//  else if (bldc_phase_state.b == OFF)
//  {
//    TIM_SetCompare1(TIM1, DUTY_CYCLE_OFF);
//  }
//
//
//  if (bldc_phase_state.c == NORMAL)
//  {
//    TIM_SetCompare2(TIM1, duty_cycle_normal); // phase C
//  }
//  else if (bldc_phase_state.c == INVERTED)
//  {
//    TIM_SetCompare2(TIM1, duty_cycle_inverted);
//  }
//  else if (bldc_phase_state.c == OFF)
//  {
//    TIM_SetCompare2(TIM1, DUTY_CYCLE_OFF);
//  }
}

// Function to set duty cycle PWM value
void pwm_set_duty_cycle (int value)
{
//#define DUTY_CYCLE_MAX_VALUE	1000 //
//#define DUTY_CYCLE_MIN_VALUE	-999 //
#define DUTY_CYCLE_MAX_VALUE	500 //
#define DUTY_CYCLE_MIN_VALUE	-500 //

  // limit the input values
  if (value >= DUTY_CYCLE_MAX_VALUE)
  {
    value = DUTY_CYCLE_MAX_VALUE;
  }
  else if (value <= DUTY_CYCLE_MIN_VALUE)
  {
    value = DUTY_CYCLE_MIN_VALUE;
  }

  pwm_duty_cycle_target = value;


//if (value > 0) value += MIN_POSITIVE_DUTY_CYCLE;
//if (value < 0) value -= MIN_NEGATIVE_DUTY_CYCLE;

pwm_duty_cycle = value;
pwm_update_duty_cycle ();
}

// This function need to be called every 1ms
// manages the increase/decrease of PWM duty-cycle value at a specific rate
void pwm_manage (void)
{
//  if (pwm_duty_cycle == pwm_duty_cycle_target)
//  {
//    return; // nothing to do, return
//  }
//  else if (pwm_duty_cycle < pwm_duty_cycle_target)
//  {
//    pwm_duty_cycle += PWM_DUTY_CYCLE_STEP;
//  }
//  else if (pwm_duty_cycle > pwm_duty_cycle_target)
//  {
//    pwm_duty_cycle -= PWM_DUTY_CYCLE_STEP;
//  }
//
//  pwm_update_duty_cycle ();
}
