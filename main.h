/*
 * EGG Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015, 2106.
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
#define	TIM4_PRIORITY				3
#define	TIM3_PRIORITY				4

// State machine
#define COAST			0
#define RUNNING 		1
#define OVER_MAX_CURRENT 	2
#define OVER_CURRENT 		3

//#define USART1_DEBUG
//#define DAC_DEBUG

// Motor
#define MOTOR_MAX_CURRENT	1 // Define max motor current (used on adc.c)
#define MOTOR_MAX_SPEED 	20000 // meter per hour
//#define MOTOR_MIN_SPEED 	5000 // meter per hour -- walking speed is 5km/h
#define MOTOR_MIN_SPEED 	0 // meter per hour -- walking speed is 5km/h
#define MOTOR_SPEED_CONVERSION 	0.5574 // convert hall sensor signal period (each 10us) to motor speed

extern unsigned int machine_state;
void delay_ms (unsigned int ms);
void printDouble(double v, int decimalDigits);

#endif /* _MAIN_H_ */
