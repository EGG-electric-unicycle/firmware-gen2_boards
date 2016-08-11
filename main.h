/*
 * Generic Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _MAIN_H_
#define _MAIN_H_

// Define for the NVIC IRQChannel Preemption Priority
// lower number has higher priority
#define ADC_ANALOG_WATCHDOG_PRIORITY		0
#define TIM2_HALL_SENSORS_PRIORITY		1
#define	TIM1_UP_PWM_PRIORITY			2
#define	TIM3_PRIORITY				3

// State machine
#define COAST			0
#define RUNNING 		1
#define OVER_MAX_CURRENT 	2
#define OVER_CURRENT 		3

//#define USART1_DEBUG

extern unsigned int machine_state;
void delay_ms (unsigned int ms);
void printDouble(double v, int decimalDigits);

#endif /* _MAIN_H_ */
