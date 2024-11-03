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

#include "mpu_math.h"

inline uint8_t accelFSRvalue(const accel_fs_t fs)
{
    return 2 << fs;
}

inline uint16_t gyroFSRvalue(const gyro_fs_t fs)
{
    return 250 << fs;
}

inline uint16_t accelSensitivity(const accel_fs_t fs)
{
    return 16384 >> fs;
}

inline float gyroSensitivity(const gyro_fs_t fs)
{
    return 131.f / (1 << fs);
}

inline float accelResolution(const accel_fs_t fs)
{
    return (float)(accelFSRvalue(fs)) / INT16_MAX;
}

inline float gyroResolution(const gyro_fs_t fs)
{
    return (float)(gyroFSRvalue(fs)) / INT16_MAX;
}

inline float accelGravity(const int16_t axis, const accel_fs_t fs)
{
    return axis * accelResolution(fs);
}

inline float_axes_t accelGravity_raw(const raw_axes_t *raw_axes, const accel_fs_t fs)
{
    float_axes_t axes;
    axes.data.x = raw_axes->data.x * accelResolution(fs);
    axes.data.y = raw_axes->data.y * accelResolution(fs);
    axes.data.z = raw_axes->data.z * accelResolution(fs);
    return axes;
}

inline float gyroDegPerSec(const int16_t axis, const gyro_fs_t fs)
{
    return axis * gyroResolution(fs);
}

inline float_axes_t gyroDegPerSec_raw(const raw_axes_t *raw_axes, const gyro_fs_t fs)
{
    float_axes_t axes;
    axes.data.x = raw_axes->data.x * gyroResolution(fs);
    axes.data.y = raw_axes->data.y * gyroResolution(fs);
    axes.data.z = raw_axes->data.z * gyroResolution(fs);
    return axes;
}

inline float gyroRadPerSec(const int16_t axis, const gyro_fs_t fs)
{
    return (M_PI / 180) * gyroDegPerSec(axis, fs);
}

inline float_axes_t gyroRadPerSec_raw(const raw_axes_t *raw_axes, const gyro_fs_t fs)
{
    float_axes_t axes;
    axes.data.x = (M_PI / 180) * gyroDegPerSec(raw_axes->data.x, fs);
    axes.data.y = (M_PI / 180) * gyroDegPerSec(raw_axes->data.y, fs);
    axes.data.z = (M_PI / 180) * gyroDegPerSec(raw_axes->data.z, fs);
    return axes;
}

#ifdef CONFIG_AUX_LIS3MDL

// 量程 1-±4  return  4
//      2-±8  return 8
//      3-±12 return 12
//      4-±16 return 16
inline uint8_t magFSRvalue(const lis3mdl_scale_t fs)
{
    return 4 * fs;
}
// 每个采样电压值对应多少Gauss
inline float magResolution(const lis3mdl_scale_t fs)
{
    return (float)(magFSRvalue(fs)) / INT16_MAX;
}

inline float_axes_t magGauss_raw(const raw_axes_t *raw_axes, const lis3mdl_scale_t fs)
{
    float_axes_t axes;
    axes.data.x = raw_axes->data.x * magResolution(fs);
    axes.data.y = raw_axes->data.y * magResolution(fs);
    axes.data.z = raw_axes->data.z * magResolution(fs);
    return axes;
}

#endif

inline float tempCelsius(const int16_t temp)
{
    // TEMP_degC = ((TEMP_OUT – RoomTemp_Offset)/Temp_Sensitivity) + DegreesCelsius_Offset
    return (temp - kRoomTempOffset) * kTempResolution + kCelsiusOffset;
}

inline float tempFahrenheit(const int16_t temp)
{
    return (temp - kRoomTempOffset) * kTempResolution * 1.8f + kFahrenheitOffset;
}
