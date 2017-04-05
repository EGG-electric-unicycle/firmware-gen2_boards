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
#include "qfplib-m3.h"
#include "motor.h"
#include "main.h"

// X axis gives the front/rear inclination of the wheel
// Z axis gives the lateral inclination of the wheel

static s16 accel_gyro[6];
static float accel_gyro_average[6];

float angle_log = 0;
float angle_error_log = 0;

BOOL IMU_init(void)
{
  unsigned int i;
  static s16 accel_gyro[6];
  static float accel_gyro_average[6];

  MPU6050_I2C_Init();
  MPU6050_Initialize();

  // if the MPU6050 is ready, make "calibration"
  // read the sensor values and average
  if (MPU6050_TestConnection())
  {
    accel_gyro_average[0] = 0;
    accel_gyro_average[1] = 0;
    accel_gyro_average[2] = 0;
    accel_gyro_average[3] = 0;
    accel_gyro_average[4] = 0;
    accel_gyro_average[5] = 0;

    for (i = 0; i <= 10; i++)
    {
      MPU6050_GetRawAccelGyro (accel_gyro);

      accel_gyro_average[0] += accel_gyro[0];
      accel_gyro_average[1] += accel_gyro[1];
      accel_gyro_average[2] += accel_gyro[2];
      accel_gyro_average[3] += accel_gyro[3];
      accel_gyro_average[4] += accel_gyro[4];
      accel_gyro_average[5] += accel_gyro[5];

      delay_ms(50); //wait for 50ms for the gyro to stable
    }

    accel_gyro_average[0] /= 10;
    accel_gyro_average[1] /= 10;
    accel_gyro_average[2] /= 10;
    accel_gyro_average[3] /= 10;
    accel_gyro_average[4] /= 10;
    accel_gyro_average[5] /= 10;

    return TRUE;
  }
  else
  {
    return FALSE;
  }
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

  acc_x = accel_gyro[0];
  acc_y = accel_gyro[1];
  acc_z = accel_gyro[2];
  gyro_rate = accel_gyro[5] * GYRO_SENSITIVITY;

  // calc dt, using micro seconds value
  micros_new = micros ();
  dt = qfp_fdiv((float) (micros_new - micros_old), 1000000.0);
  micros_old = micros_new;

//  angle = qfp_fatan2(acc_x, acc_y); //calc angle between X and Y axis, in rads
  angle = qfp_fatan2(acc_y, acc_z); //calc angle between X and Y axis, in rads
  angle = qfp_fmul(qfp_fadd(angle, PI), RAD_TO_DEG); //convert from rads to degres
//  angle = 0.98 * (angle + (gyro_rate * dt)) + 0.02 * (acc_y); //use the complementary filter.
//  angle = 0.98 * (angle + qfp_fmul(gyro_rate, dt)) + 0.02 * (acc_y); //use the complementary filter.
//  angle = 0.98 * (angle + qfp_fmul(gyro_rate, dt)) + qfp_fmul(0.02, acc_y); //use the complementary filter.
//  angle = qfp_fmul(0.98, (angle + qfp_fmul(gyro_rate, dt))) + qfp_fmul(0.02, acc_y); //use the complementary filter.
//  angle = qfp_fmul(0.98, (qfp_fadd(angle, qfp_fmul(gyro_rate, dt)))) + qfp_fmul(0.02, acc_y); //use the complementary filter.
//  angle = qfp_fadd(qfp_fmul(0.98, (qfp_fadd(angle, qfp_fmul(gyro_rate, dt)))), qfp_fmul(0.02, acc_y)); //use the complementary filter.

  angle = (0.25 * angle) + (0.25 * old_angle1) + (0.25 * old_angle2) + (0.25 * old_angle3);
  old_angle1 = angle;
  old_angle2 = old_angle1;
  old_angle3 = old_angle2;

  // zero value error when the board is on balance
  current_error = qfp_fsub(INITIAL_ANGLE, angle);
  angle_log = angle;
  angle_error_log = current_error;

#define ANGLE_MAX_ERROR 10.0
  if (current_error >= ANGLE_MAX_ERROR) { current_error = ANGLE_MAX_ERROR; }
  if (current_error <= -ANGLE_MAX_ERROR) { current_error = -ANGLE_MAX_ERROR; }


  float current_error_alpha = 2.0;
  static float moving_average_current_error = 0.0;
  current_error = ema_filter_float(&current_error, &moving_average_current_error, &current_error_alpha);
  current_error = moving_average_current_error;

//  float k = ANGLE_MAX_ERROR / (1000.0 - MOTOR_MIN_DUTYCYCLE_LEFT);
  float k = 100.0;

  float output;
  if (current_error > 0)
  {
    output = MOTOR_MIN_DUTYCYCLE_LEFT + (k * current_error);
  }
  else
  {
    output = -MOTOR_MIN_DUTYCYCLE_RIGHT + (k * current_error);
  }

  if (output >= 100) output = 1000;
  if (output <= -999) output = -999;
  motor_set_duty_cycle ((int) output);

//  float kp = 10;
//  float ki = 0.5;
//  float kd = 5;
//  float angle_old;
//
//  progressive_term = qfp_fmul(current_error, kp);
//  integrative_term = qfp_fadd(integrative_term, qfp_fmul(current_error, ki));
//  derivative_term = qfp_fmul(qfp_fsub(current_error, old_error), kd);
//  old_error = current_error;
//
//  duty_cycle = qfp_fadd(qfp_fadd(progressive_term, integrative_term), derivative_term);
//  if (duty_cycle >= 1000) { duty_cycle = 1000; }
//  if (duty_cycle <= -999) { duty_cycle = -999; }
////  motor_set_duty_cycle ((int) duty_cycle); // -999 <-> 1000
//  motor_set_current ((int) duty_cycle); // -999 <-> 1000

}
