/*
 * EGG Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015, 2106, 2017.
 *
 * Released under the GPL License, Version 3
 */

/* Connections:
 *
 * Motor PHASE_A: yellow wire
 * Motor PHASE_B: blue wire
 * Motor PHASE_C: green wire
 *
 *
 * PIN				        | IN/OUT| Works?|Function
 * ----------------------------------------------------------
 *
 * PA3  (ADC12_IN3)   | in  | ??  | current_phase_A
 * PA2  (ADC12_IN2)   | in  | ??  | current_phase_C
 *
 * PA4  (ADC12_IN4)   | in  | ??  | battery_voltage
 *
 * PB12               | in  | ??  | Hall_sensor_A
 * PB13               | in  | ??  | Hall_sensor_B
 * PB14               | in  | ??  | Hall_sensor_C
 *
 * PB6  (I2C1_SCL)    | in/out| ??  | IMU_MPU6050-SCL
 * PB7  (I2C1_SDA)    | in/out| ??  | IMU_MPU6050-SDA
 *
 * PB1  (TIM3_CH4)    | out | ??  | phase_A-HO_LO
 * PB0  (TIM3_CH3)    | out | ??  | phase_B-HO_LO
 * PA6  (TIM3_CH1)    | out | ??  | phase_C-HO_LO
 * PB2                | out | ??  | phase_A-shutdown
 * PA7                | out | ??  | phase_B-shutdown
 * PA5                | out | ??  | phase_C-shutdown
 *
 * PA9  (USART1_TX)   | out | ??  | usart_tx
 * PA10 (USART1_RX)   | out | ??  | usart_rx
 *
 * PA8                | out | yes | buzzer      (active high: push pull)
 *
 * PB15               | out | ??  | LED_1-battery_indicator (active low: float to disable and GND to turn on)
 * PA11               | out | ??  | LED_2-battery_indicator (active low: float to disable and GND to turn on)
 * PA12               | out | ??  | LED_3-battery_indicator (active low: float to disable and GND to turn on)
 * PB5                | out | ??  | LED_4-battery_indicator (active low: float to disable and GND to turn on)
 *
 */

#ifndef GPIO_H
#define GPIO_H

#include "main.h"

#define CURRENT_PHASE_A__PIN      GPIO_Pin_3
#define CURRENT_PHASE_A__PORT     GPIOA
#define CURRENT_PHASE_C__PIN      GPIO_Pin_2
#define CURRENT_PHASE_C__PORT     GPIOA

#define BATTERY_VOLTAGE__PIN      GPIO_Pin_4
#define BATTERY_VOLTAGE__PORT     GPIOA

#if (MOTOR_TYPE == MOTOR_TYPE_EUC1) || (MOTOR_TYPE == MOTOR_TYPE_EUC2)
  #define HALL_SENSOR_A__PIN        GPIO_Pin_12
  #define HALL_SENSOR_B__PIN        GPIO_Pin_13
  #define HALL_SENSOR_C__PIN        GPIO_Pin_14
#elif MOTOR_TYPE == MOTOR_TYPE_MICROWORKS_500W_30KMH
  #define HALL_SENSOR_A__PIN        GPIO_Pin_14
  #define HALL_SENSOR_B__PIN        GPIO_Pin_13
  #define HALL_SENSOR_C__PIN        GPIO_Pin_12
#endif

#define HALL_SENSOR_A__PORT       GPIOB
#define HALL_SENSOR_B__PORT       GPIOB
#define HALL_SENSOR_C__PORT       GPIOB
#define HALL_SENSORS__PORT        GPIOB
#define HALL_SENSORS_MASK 	  (HALL_SENSOR_A__PIN | HALL_SENSOR_B__PIN | HALL_SENSOR_C__PIN)

#define MPU6050_SCL__PIN          GPIO_Pin_6
#define MPU6050_SCL__PORT         GPIOB
#define MPU6050_SDA__PIN          GPIO_Pin_7
#define MPU6050_SDA__PORT         GPIOB

#define PHASE_A_HO_LO__PIN        GPIO_Pin_1
#define PHASE_A_HO_LO__PORT       GPIOB
#define PHASE_B_HO_LO__PIN        GPIO_Pin_0
#define PHASE_B_HO_LO__PORT       GPIOB
#define PHASE_C_HO_LO__PIN        GPIO_Pin_6
#define PHASE_C_HO_LO__PORT       GPIOA
#define PHASE_A_SHUTDOWN__PIN     GPIO_Pin_2
#define PHASE_A_SHUTDOWN__PORT    GPIOB
#define PHASE_B_SHUTDOWN__PIN     GPIO_Pin_7
#define PHASE_B_SHUTDOWN__PORT    GPIOA
#define PHASE_C_SHUTDOWN__PIN     GPIO_Pin_5
#define PHASE_C_SHUTDOWN__PORT    GPIOA

#define USART_TX__PIN             GPIO_Pin_9
#define USART_TX__PORT            GPIOA
#define USART_RX__PIN             GPIO_Pin_10
#define USART_RX__PORT            GPIOA

#define BUZZER__PIN               GPIO_Pin_8
#define BUZZER__PORT              GPIOA

#define LED_1_BATTERY_INDICATOR__PIN        GPIO_Pin_15
#define LED_1_BATTERY_INDICATOR__PORT       GPIOB
#define LED_2_BATTERY_INDICATOR__PIN        GPIO_Pin_11
#define LED_2_BATTERY_INDICATOR__PORT       GPIOA
#define LED_3_BATTERY_INDICATOR__PIN        GPIO_Pin_12
#define LED_3_BATTERY_INDICATOR__PORT       GPIOA
#define LED_4_BATTERY_INDICATOR__PIN        GPIO_Pin_5
#define LED_4_BATTERY_INDICATOR__PORT       GPIOB

void gpio_init (void);

#endif /* GPIO_H_ */
