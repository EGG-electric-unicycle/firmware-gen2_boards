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
unsigned int ema_filter_uint32 (unsigned int *current_value, unsigned int *exponential_moving_average, unsigned int *ema_alpha)
{
  *exponential_moving_average = ((*current_value * *ema_alpha) + (*exponential_moving_average * (100 - *ema_alpha))) / 100;
  return *exponential_moving_average;
}

float ema_filter_float (float *current_value, float *exponential_moving_average, float *ema_alpha)
{
  *exponential_moving_average = ((*current_value * *ema_alpha) + (*exponential_moving_average * (100 - *ema_alpha))) / 100;
  return *exponential_moving_average;
}
