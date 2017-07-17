/*
 * EGG Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015, 2106, 2017.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _ADC_H_
#define _ADC_H_

#define PHASE_A_CURRENT_ADC_CHANNEL	ADC_Channel_3
#define PHASE_C_CURRENT_ADC_CHANNEL	ADC_Channel_2
#define BATTERY_VOLTAGE_ADC_CHANNEL	ADC_Channel_4
//#define POTENTIOMETER_ADC_CHANNEL	ADC_Channel_0
#define POTENTIOMETER_ADC_CHANNEL	ADC_Channel_1

//#define MAX_CURRENT_INTERRUPT ADC1_2_IRQHandler


void adc_init (void);
unsigned int adc_get_phase_a_current_value (void);
unsigned int adc_get_phase_c_current_value (void);
unsigned int adc_get_battery_voltage_value (void);
unsigned int adc_get_potentiometer_value (void);

#endif
