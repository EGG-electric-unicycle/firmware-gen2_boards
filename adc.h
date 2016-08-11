/*
 * Generic Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _ADC_H_
#define _ADC_H_

#define MAX_CURRENT_INTERRUPT ADC1_2_IRQHandler

void adc_init (void);
unsigned int adc_get_battery_voltage_value (void);
float get_battery_voltage (void);
unsigned int adc_get_motor_current_value (void);
unsigned int adc_get_PS_signal_value (void);
unsigned int is_current_under_max (void);

#endif
