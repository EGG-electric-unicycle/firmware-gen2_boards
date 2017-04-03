/*
 * EGG Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015, 2106.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _MAIN_H_
#define _MAIN_H_

// Constants
#define ONE_BY_SQRT3			(0.57735026919)
#define TWO_BY_SQRT3			(2.0f * 0.57735026919)
#define SQRT3_BY_2				(0.86602540378)

// Define for the NVIC IRQChannel Preemption Priority
// lower number has higher priority
#define ADC_ANALOG_WATCHDOG_PRIORITY		0
#define HALL_SENSORS_PRIORITY			1
#define	TIM3_PRIORITY				2
#define	TIM4_PRIORITY				3
#define	TIM2_PRIORITY				4
#define	USART1_PRIORITY				5

// State machine
#define COAST			0
#define RUNNING 		1
#define OVER_MAX_CURRENT 	2
#define OVER_CURRENT 		3

// Battery voltage
// voltage measured values:
//  20V | 0.75V
//  30V | 1.12V
//  40V | 1.50V
//  50V | 1.88V
//  60V | 2.25V
// about 38mv for each 1V

// Phase current
// Voltage measured values:
//  4.7A | 1.77V
//  2.8A | 1.74V
//    0A | 1.71V
// -2.8A | 1.68V
// -4.7A | 1.65V
// about 12.7mv for each 1A

#define ADC_CURRENT_OFFSET		2131 // represents 1.71V when current = 0
#define ADC_CURRENT_GAIN_AMPS		0.0634 // 0.0190 original estimated value of 0.0634 but verified to be instead 3.33 less, in 2017.03.17.
#define ADC_BATTERY_VOLTAGE_GAIN_VOLTS	0.0215

// VESC Lizardmech
//#define MOTOR_R		0.20653
//#define MOTOR_L		0.00022277
//#define MOTOR_LINKAGE	0.02436 // 0.1 NOK; 0.01 a velocidade baixa ok mas aumentando começa com picos;
//#define MOTOR_GAMA	4490000 // 60 / L as noted on VESC mcconf_default.h // 1000 Não funciona; 10000 começa a funcionar
//#define MOTOR_PWM_DT	0.0001

#define PWM_CYCLES_COUNTER_MAX	((46000*2) - 1) // estimated as 1 rotation in about 4.6 seconds for the MicroWorks 500W 30km/h (44 magnets)
#define K_POSITION_CORRECTION_VALUE 0.1

#define MOTOR_R		0.5
#define MOTOR_L		0.001
#define MOTOR_LINKAGE	0.0001
#define MOTOR_GAMA	100000 // 60 / L as noted on VESC mcconf_default.h
#define MOTOR_PWM_DT	0.0001

#define MOTOR_MAX_CURRENT	1 // Define max motor current (used on adc.c)
#define MOTOR_MAX_SPEED 	20000 // meter per hour
//#define MOTOR_MIN_SPEED 	5000 // meter per hour -- walking speed is 5km/h
#define MOTOR_MIN_SPEED 	0 // meter per hour -- walking speed is 5km/h
#define MOTOR_SPEED_CONVERSION 	0.5574 // convert hall sensor signal period (each 10us) to motor speed

extern unsigned int machine_state;
void delay_ms (unsigned int ms);
void printDouble(double v, int decimalDigits);



#endif /* _MAIN_H_ */
