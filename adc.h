/*
 * EGG Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015, 2106.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _ADC_H_
#define _ADC_H_

#define MAX_CURRENT_INTERRUPT ADC1_2_IRQHandler

void adc_init (void);
unsigned int adc_get_phase_a_current_value (void);
unsigned int adc_get_phase_c_current_value (void);
unsigned int adc_get_battery_voltage_value (void);
unsigned int adc_get_potentiometer_value (void);

#endif
