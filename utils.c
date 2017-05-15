/*
 * EGG Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015, 2106, 2017.
 *
 * Released under the GPL License, Version 3
 */

int mod_angle_degrees (int a)
{
  int ret = a % 360;
  if(ret < 0)
    ret += 360;
  return ret;
}

float abs_f (float value)
{
  if (value < 0)
  {
//    value = qfp_fmul(value, -1.0); // THIS FAILS qfp_fmul!!!
      value = value * -1.0;
  }

  return value;
}

int abs (int value)
{
  if (value < 0)
    return (value * -1);
  else
    return value;
}

