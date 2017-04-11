/*
    EGG Electric Unicycle firmware

    Copyright (C) 2015 Joerg Hoener
    Copyright (C) Casainho, 2015, 2106, 2017.

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

#ifndef _IMU_H_
#define _IMU_H_

#define PI 3.1415926535897932384626433832795
#define RAD_TO_DEG 57.2957786

// 16 bits +-16g: -32768 [-16g] <---> 32768 [+16g]
// 1g --> 2048
#define ACCEL_SENSITIVITY (16.0 / 32768.0)
// 16 bits +-2000º/sec: -32768 [-2000] <---> 32768 [+2000]
#define GYRO_SENSITIVITY (2000.0 / 32768.0) // 0.061035156

#define INITIAL_ANGLE 270.0

extern float angle_log;
extern float angle_error_log;

BOOL IMU_init(void);
float IMU_get_angle_error (void);

#endif

