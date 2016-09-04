/*
 * EGG Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015, 2106.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _TIMER_H_
#define _TIMER_H_

/*
 * Returns the value of micro seconds
 * overflows at ~1h
 */
unsigned int micros (void);
void TIM4_set_counter_10us (unsigned int value);
void micros_reset (void);

#endif
