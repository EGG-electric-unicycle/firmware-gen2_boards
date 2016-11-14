/*
 * EGG Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015, 2106.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _FILTER_H_
#define _FILTER_H_

#define EMA_ALPHA 1

unsigned int ema_filter_uint32 (unsigned int *current_value, unsigned int *exponential_moving_average, unsigned int *ema_alpha);

#endif
