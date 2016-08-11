/*
 * Generic Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015.
 *
 * Released under the GPL License, Version 3
 */

#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "adc.h"
#include "bldc.h"
#include "main.h"

/*
 * BATTERY VOLTAGE SIGNAL
 * 30V -> 1180
 * 40V -> 1558
 * 50V -> 1940
 * 60V -> 2320 | each 1V -> 0.0255 * ADC value
 *
 *
 * MOTOR CURRENT SIGNAL:
 * 0 Amps = 1.54V
 * each 1A = +0.0385V
 * 12 bits ADC; 4096 steps
 * 3.3V / 4096 = 0.0008
 *
 */

#define ADC_BATTERY_VOLTAGE_CONSTANT 0.0255 // fine tunned by hand
#define ADC_BATTERY_VOLTAGE_CONSTANT_X2 (ADC_BATTERY_VOLTAGE_CONSTANT * 2)

#define ADC_WATCHDOG_HIGHTHRESHOLD ((1.54 + (0.0385 * 0.8/*amps*/)) / 0.0008)
#define ADC_WATCHDOG_LOWTHRESHOLD  0

unsigned int adc_watchdog_highthreshold = ADC_WATCHDOG_HIGHTHRESHOLD;

static unsigned int adc_values[3];

// ADC_Channel_4 (PA4) to read PS_signal -- used on debug connected to a potentiometer
// ADC_Channel_5 (PA5) to read over battery voltage signal
// ADC_Channel_7 (PA7) to read over motor current signal
void adc_init (void)
{
  /* ADCCLK = PCLK2/8 */
  RCC_ADCCLKConfig(RCC_PCLK2_Div8);

  /* Enable DMA1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  /* Enable GPIOA and ADC1 clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);

  DMA_InitTypeDef DMA_InitStructure;
  /* DMA1 channel1 configuration ----------------------------------------------*/
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(ADC1->DR);
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) &adc_values;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 3;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);

  /* Enable DMA1 channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);

  ADC_InitTypeDef ADC_InitStructure;
  /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 3;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channel4 configuration */
  ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_55Cycles5);
  /* ADC1 regular channel5 configuration */
  ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 2, ADC_SampleTime_55Cycles5);
  /* ADC1 regular channel7 configuration */
  ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 3, ADC_SampleTime_55Cycles5);

  /*
   * ADC analog watchdog used for over current measurement
   */
  /* Configure high and low analog watchdog thresholds */
  ADC_AnalogWatchdogThresholdsConfig(ADC1, adc_watchdog_highthreshold, ADC_WATCHDOG_LOWTHRESHOLD);
  /* Configure channel14 as the single analog watchdog guarded channel */
  ADC_AnalogWatchdogSingleChannelConfig(ADC1, ADC_Channel_7);
  /* Enable analog watchdog on one regular channel */
  ADC_AnalogWatchdogCmd(ADC1, ADC_AnalogWatchdog_SingleRegEnable);

  NVIC_InitTypeDef NVIC_InitStructure;
  /* Configure and enable TIM2 interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = ADC_ANALOG_WATCHDOG_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable AWD interrupt */
  ADC_ITConfig(ADC1, ADC_IT_AWD, ENABLE);

  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);

  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /* Enable ADC1 reset calibration register */
  ADC_ResetCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));

  /* Start ADC1 calibration */
  ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));

  /* Start ADC1 Software Conversion */
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

// This interrupt happens when current is over the defined threshold and will disable the duty-cycle
void MAX_CURRENT_INTERRUPT(void)
{
  if (ADC_GetFlagStatus (ADC1, ADC_FLAG_AWD) == SET)
  {
    TIM_CtrlPWMOutputs (TIM1, DISABLE); //disable PWM signals
    bldc_set_state (BLDC_OVER_MAX_CURRENT);

    /* Clear ADC1 AWD pending interrupt bit */
    ADC_ClearITPendingBit(ADC1, ADC_IT_AWD); // it's the same as ADC_ClearFlagStatus (ADC1, ADC_FLAG_AWD);
  }
}

// output a value 0 - 4095
unsigned int adc_get_PS_signal_value (void)
{
  return adc_values[0];
}

// output a value 0 - 4095
unsigned int adc_get_battery_voltage_value (void)
{
  return adc_values[1];
}

float get_battery_voltage (void)
{
  return ((adc_values[1] >> 1) * ADC_BATTERY_VOLTAGE_CONSTANT_X2);
}

// output a value 0 - 4095
unsigned int adc_get_motor_current_value (void)
{
  return adc_values[2];
}

unsigned int is_current_under_max (void)
{
  if (adc_get_motor_current_value() < adc_watchdog_highthreshold)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}
