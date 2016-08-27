/*
 * Generic Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _HALL_SENSOR_H_
#define _HALL_SENSOR_H_

#define EXTI15_10_IRQHandler HALL_SENSORS_INTERRUPT

void hall_sensor_init (void);
unsigned int get_hall_sensors_us (void);

#endif
