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

#ifndef __VL53L1X_DRIVER_H__
#define __VL53L1X_DRIVER_H__

#include <stdint.h>
#include <stdbool.h>
#include "error_handle.h"
#include "io_define.h"
#include "i2c_bus.h"
#include "vl53l1_api.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define VL53L1X_I2C_ADDRESS 0x29

typedef enum {
    GB_LASER_MODE_SHORT = 1,  // 1.3m
    GB_LASER_MODE_MEDIUM = 2, // 3m
    GB_LASER_MODE_LONG = 3    // 4m
} GB_LASER_MODE_T;

typedef enum {
    GB_LASER_TIMING_15MS = 15,
    GB_LASER_TIMING_20MS = 20,
    GB_LASER_TIMING_33MS = 33,
    GB_LASER_TIMING_50MS = 50,
    GB_LASER_TIMING_100MS = 100,
    GB_LASER_TIMING_200MS = 200,
    GB_LASER_TIMING_500MS = 500
} GB_LASER_TIMING_T;

typedef struct
{
    struct i2c *bus;
    uint8_t i2c_address;
    bool initialized;
    uint16_t cached_distance_mm;
    VL53L1_Dev_t vl53l1_dev;
    GB_LASER_MODE_T mode;
    GB_LASER_TIMING_T timing_budget;
} GB_LASER_DEV_T;

GB_RESULT GB_LASER_InitDesc(GB_LASER_DEV_T *dev, struct i2c *bus, uint8_t addr);

GB_RESULT GB_LASER_Init(GB_LASER_DEV_T *dev, GB_LASER_MODE_T mode, GB_LASER_TIMING_T timing_budget);

GB_RESULT GB_LASER_StartContinuous(GB_LASER_DEV_T *dev, uint32_t period_ms);

GB_RESULT GB_LASER_StopContinuous(GB_LASER_DEV_T *dev);

GB_RESULT GB_LASER_MeasureSingle(GB_LASER_DEV_T *dev, uint16_t *distance_mm);

GB_RESULT GB_LASER_ReadDistance(GB_LASER_DEV_T *dev, uint16_t *distance_mm, uint32_t timeout_ms);

GB_RESULT GB_LASER_GetLastDistance(GB_LASER_DEV_T *dev, uint16_t *distance_mm);

GB_RESULT GB_LASER_GetMeasurement(GB_LASER_DEV_T *dev, VL53L1_RangingMeasurementData_t *ranging_data);

GB_RESULT GB_LASER_CheckDataReady(GB_LASER_DEV_T *dev, uint8_t *data_ready);

void VL53L1_SetI2CBus(struct i2c *bus);

#ifdef __cplusplus
}
#endif

#endif  //__VL53L1X_DRIVER_H__
