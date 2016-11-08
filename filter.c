/*
 * EGG Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015, 2106.
 *
 * Released under the GPL License, Version 3
 */

#include "filter.h"
#include "pwm.h"

// Exponential Moving Average (EMA)
unsigned int ema_filter (unsigned int current_value)
{
  static unsigned int exponential_average = MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX;

  exponential_average = (EMA_ALPHA * (unsigned int) current_value + (100 - EMA_ALPHA) * (unsigned int) exponential_average) / 100;

  return exponential_average;
}
