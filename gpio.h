/*
 * Generic Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015, 2016.
 *
 * Released under the GPL License, Version 3
 */

/* Connections:
 *
 * Motor PHASE_A: green wire
 * Motor PHASE_B: yellow wire
 * Motor PHASE_B: blue wire
 *
 *
 * PIN				| IN/OUT| Works?|Function
 * ----------------------------------------------------------
 *
 * ??		  		| out	| yes	| Bridge_C-High
 * ??		  		| out	| yes	| Bridge_B-High
 * ??		  		| out	| yes	| Bridge_A-High
 * ??		 		| out	| yes	| Bridge_C-Low (active low)
 * ??		 		| out	| yes	| Bridge_B-Low (active low)
 * ??		 		| out	| yes	| Bridge_A-Low (active low)
 *
 * ??		 		| in	| yes	| Hall_sensor_A
 * ??		 		| in	| yes	| Hall_sensor_B
 * ??		 		| in	| yes	| Hall_sensor_C

 * ??		 		| in	| ??	| BMF_signal-Yellow_A
 * ??		 		| in	| ??	| BMF_signal-Green_B
 * ??		 		| in	| ??	| BMF_signal-Blue_C
 *
 * ??				| in	| yes	| Battery_voltage_signal
 * ??				| in	| yes	| Motor_current_signal
 *
 * ??				| in/out| ??	| IMU_MPU6050-SCL
 * ??				| in/out| ??	| IMU_MPU6050-SDA
 *
 * ??	 			| out	| yes	| LED_1-battery_indicator (active low: float to disable and GND to turn on)
 * ??	 			| out	| yes	| LED_2-battery_indicator (active low: float to disable and GND to turn on)
 * ??	 			| out	| yes	| LED_3-battery_indicator (active low: float to disable and GND to turn on)
 * ??	 			| out	| yes	| LED_4-battery_indicator (active low: float to disable and GND to turn on)
 * ??	 			| out 	| yes	| LED-power_switcher	  (active low: float to disable and GND to turn on)
 *
 * PB3	 			| out	| yes	| Buzzer 		  (active high: push pull)
 * ??	 			| in	| yes	| PS_signal 		  (calibrate_wheel)
 *
 */

#ifndef GPIO_H
#define GPIO_H

#include "stm32f10x_gpio.h"

#define LED_1_BATTERY_INDICATOR
#define LED_2_BATTERY_INDICATOR
#define LED_3_BATTERY_INDICATOR
#define LED_4_BATTERY_INDICATOR
#define LED_POWER_SWITCHER

#define HALL_SENSOR_A
#define HALL_SENSOR_B
#define HALL_SENSOR_C

#define BRIDGE_A_HIGH
#define BRIDGE_B_HIGH
#define BRIDGE_C_HIGH
#define BRIDGE_A_LOW
#define BRIDGE_B_LOW
#define BRIDGE_C_LOW

#define BUZZER				GPIO_Pin_3
#define PS_SIGNAL
#define BATTERY_VOLTAGE_SIGNAL
#define MOTOR_CURRENT_SIGNAL


void gpio_init (void);

#endif /* GPIO_H_ */
