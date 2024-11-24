/*
 * This file is part of GB-Drone project (https://github.com/GBeetle/GB-Drone).
 * Copyright (c) 2022 GBeetle.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _MPU_MATH_HPP_
#define _MPU_MATH_HPP_

#include <math.h>
#include <stdint.h>
#include "lis3mdl.h"
#include "mpu_types.h"
#include "sdkconfig.h"

// Use floating point M_PI instead explicitly.
#define M_PIf       3.14159265358979323846f
#define M_LN2f      0.69314718055994530942f
#define M_Ef        2.71828182845904523536f

#define sinPolyCoef3 -1.666568107e-1f
#define sinPolyCoef5  8.312366210e-3f
#define sinPolyCoef7 -1.849218155e-4f
#define sinPolyCoef9  0

#define DEG2RAD        0.017453293f    /* 度转弧度 π/180 */
#define RAD2DEG        57.29578f        /* 弧度转度 180/π */

#define RAD    (M_PIf / 180.0f)

#define DEGREES_TO_CENTIDEGREES(angle) ((angle) * 100)
#define CENTIDEGREES_TO_DEGREES(angle) ((angle) / 100)

#define CENTIDEGREES_TO_DECIDEGREES(angle) ((angle) / 10)
#define DECIDEGREES_TO_CENTIDEGREES(angle) ((angle) * 10)

#define DEGREES_TO_DECIDEGREES(angle) ((angle) * 10)
#define DECIDEGREES_TO_DEGREES(angle) ((angle) / 10)

#define DEGREES_PER_DEKADEGREE 10
#define DEGREES_TO_DEKADEGREES(angle) ((angle) / DEGREES_PER_DEKADEGREE)
#define DEKADEGREES_TO_DEGREES(angle) ((angle) * DEGREES_PER_DEKADEGREE)

#define DEGREES_TO_RADIANS(angle) ((angle) * RAD)
#define RADIANS_TO_DEGREES(angle) ((angle) / RAD)
#define DECIDEGREES_TO_RADIANS(angle) (((angle) / 10.0f) * RAD)
#define RADIANS_TO_DECIDEGREES(angle) (((angle) * 10.0f) / RAD)

#define RADIANS_TO_CENTIDEGREES(angle) (((angle) * 100.0f) / RAD)
#define CENTIDEGREES_TO_RADIANS(angle) (((angle) / 100.0f) * RAD)

#define MIN(a, b)     (((a) < (b)) ? (a) : (b))
#define MAX(a, b)     (((a) > (b)) ? (a) : (b))

#ifndef sq
#define sq(x) ((x)*(x))
#endif

#if defined CONFIG_MPU6500
#define kRoomTempOffset 0        // LSB
#define kCelsiusOffset    21.f    // ºC
#define kTempSensitivity  333.87f  // LSB/ºC
#elif defined CONFIG_MPU6000 || defined CONFIG_MPU6050 || defined CONFIG_MPU9150
#define kRoomTempOffset -521   // LSB
#define kCelsiusOffset    35.f   // ºC
#define kTempSensitivity  340.f  // LSB/ºC
#endif

#define kTempResolution   (98.67f / INT16_MAX)
#define kFahrenheitOffset (kCelsiusOffset * 1.8f + 32)  // ºF

uint8_t accelFSRvalue(const accel_fs_t fs);
uint16_t gyroFSRvalue(const gyro_fs_t fs);
uint16_t accelSensitivity(const accel_fs_t fs);
float gyroSensitivity(const gyro_fs_t fs);
float accelResolution(const accel_fs_t fs);
float gyroResolution(const gyro_fs_t fs);
float accelGravity(const int16_t axis, const accel_fs_t fs);
float_axes_t accelGravity_raw(const raw_axes_t *raw_axes, const accel_fs_t fs);
float gyroDegPerSec(const int16_t axis, const gyro_fs_t fs);
float_axes_t gyroDegPerSec_raw(const raw_axes_t *raw_axes, const gyro_fs_t fs);
float gyroRadPerSec(const int16_t axis, const gyro_fs_t fs);
float_axes_t gyroRadPerSec_raw(const raw_axes_t *raw_axes, const gyro_fs_t fs);

uint8_t magFSRvalue(const lis3mdl_scale_t fs);
float magResolution(const lis3mdl_scale_t fs);
float_axes_t magGauss_raw(const raw_axes_t *raw_axes, const lis3mdl_scale_t fs);

float tempCelsius(const int16_t temp);
float tempFahrenheit(const int16_t temp);

#endif /* end of include guard: _MPU_MATH_HPP_ */
