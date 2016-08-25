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
#define	TIM4_PRIORITY				3
#define	TIM3_PRIORITY				4

// State machine
#define COAST			0
#define RUNNING 		1
#define OVER_MAX_CURRENT 	2
#define OVER_CURRENT 		3

// battery voltage: 60V input = 2.35V at ADC input

// phase current ADC voltage input 0 amps = 2.49V
// Voltage measured values:
//  4.7A | 2.58V
//  2.8A | 2.54V
//    0A | 2.49V
// -2.8A | 2.45V
// -4.7A | 2.4V
// about 20mv for each 1A

#define K_ADC_VOLTAGE 161 // amplified 100k

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
