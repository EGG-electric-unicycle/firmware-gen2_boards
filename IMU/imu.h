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

#define PI 3.1415926535897932384626433832795
#define RAD_TO_DEG 57.2957786

// 16 bits +-16g: -32768 [-16g] <---> 32768 [+16g]
// 1g --> 2048
#define ACCEL_SENSITIVITY (16.0 / 32768.0)
// 16 bits +-2000º/sec: -32768 [-2000] <---> 32768 [+2000]
#define GYRO_SENSITIVITY (2000.0 / 32768.0) // 0.061035156

//#define INITIAL_ANGLE 90.0
#define INITIAL_ANGLE 180.0
#define SUM_ERROR_MAX 3.0
#define SUM_ERROR_MIN -3.0
#define KP 200 //30000.0
//#define KI 50000.0
//#define KD 500.0
#define KI 500.0
#define KD 100.0


/**
 * Select one define for data transfer mode
 * and used filter
 */
//#define IMU_MODE_POLL 1
#define IMU_MODE_DMA 1


#define IMU_FILTER_COMPL 
//#define IMU_FILTER_COMPL2
//#define IMU_FILTER_KALMAN

/**
 * Define the used IMU from all that are implemented 
 */
#define IMU_IS_MPU6050
//#define IMU_IS_...

/**
 * Initialize IMU to be used for data transfer
 *
 * @param [in] func*(float*, float*) Callback for new data arrival
 *                            in DMA mode,
 *           @param [out] float* Angle in rad of forward backward lean.
 *           @param [out] * Sideway fall over is indicated as follows:
 *                          0 = Between +-45 degree
 *                          1 = Leaned more than 45 degree right
 *                          2 = Leaned more than 45 degree left 
 *
 * @return BOOL Indicate whether initialisation 
 */
#ifdef IMU_MODE_POLL 
BOOL IMU_init();

void IMU_getData(float* fwBkAngle, uint8_t* sideLean);

#elif IMU_MODE_DMA
BOOL IMU_init(void);

void IMU_startDMAtransfer();

extern float angle_log;
extern float angle_error_log;

#endif

/**
 * Filter interface, implemented as per above filter define
 */
float callFilter(float oldAngle, float newAngle, float newRate,int looptime);

void balance_controller(void);
