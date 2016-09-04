/*
    EGG Electric Unicycle firmware

    Copyright (C) 2015 Joerg Hoener
    Copyright (C) Casainho, 2015, 2106.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "stm32f10x.h"
#include "stdio.h"
#include "imu.h"
#include "math.h"
#include "MPU6050/MPU6050.h"

// X axis gives the front/rear inclination of the wheel
// Z axis gives the lateral inclination of the wheel

static s16 accel_gyro[6];
static float accel_gyro_average[6];

BOOL IMU_init(void)
{
  unsigned int i;

  MPU6050_I2C_Init();
  MPU6050_Initialize();

  if (MPU6050_TestConnection())
    return TRUE;
  else
    return FALSE;
}

// called at each 10ms
void balance_controller(void)
{
  float acc_x;
  float acc_y;
  float acc_z;
  static float angle;
  static float old_angle1;
  static float old_angle2;
  static float old_angle3;
  static float gyro_rate;
  float dt;
  unsigned int micros_new;
  static unsigned int micros_old = 0;
  static unsigned int timer_1s = 0;

  float current_error = 0;
  static float old_error = 0;
  float progressive_term = 0;
  static float integrative_term = 0;
  float derivative_term = 0;
  float duty_cycle = 0;

  float speed = 0;

  // read the accel and gyro sensor values
  MPU6050_GetRawAccelGyro (accel_gyro); // takes abut 15ms to be executed!!!

  acc_x = (float) accel_gyro[0];
  acc_y = (float) accel_gyro[1];
  acc_z = (float) accel_gyro[2];
  gyro_rate = (float) (accel_gyro[5] * GYRO_SENSITIVITY);

  // calc dt, using micro seconds value
  micros_new = micros ();

  dt = (micros_new - micros_old) / 1000000.0;
  micros_old = micros_new;

  angle = atan2(acc_x, acc_y); //calc angle between X and Y axis, in rads
  angle = (angle + PI) * RAD_TO_DEG; //convert from rads to degres
  angle = 0.98 * (angle + (gyro_rate * dt)) + 0.02 * (acc_y); //use the complementary filter.

//  angle = (0.25 * angle) + (0.25 * old_angle1) + (0.25 * old_angle2) + (0.25 * old_angle3);
//  old_angle1 = angle;
//  old_angle2 = old_angle1;
//  old_angle3 = old_angle2;

  printf ("a %.0f\n", angle);

}
