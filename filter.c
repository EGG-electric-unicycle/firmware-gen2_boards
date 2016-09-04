/*
 * EGG Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015, 2106.
 *
 * Released under the GPL License, Version 3
 */

#include "filter.h"

// Exponential Moving Average (EMA)
unsigned ema_filter (unsigned int current_value)
{
  static int exponential_average = 0;

  exponential_average = (EMA_ALPHA * (unsigned int) current_value + (100 - EMA_ALPHA) * (unsigned int) exponential_average) / 100;

  return exponential_average;
}
