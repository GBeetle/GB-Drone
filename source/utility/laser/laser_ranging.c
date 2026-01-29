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

#include "gb_timer.h"
#include "laser_ranging.h"

GB_RESULT GB_LASER_InitDesc(GB_LASER_DEV_T *dev, struct i2c *bus, uint8_t addr)
{
    GB_RESULT res = GB_OK;

    CHK_NULL(dev, GB_LASER_DEVICE_NULL);
    CHK_NULL(bus, GB_LASER_BUS_NULL);

    memset(dev, 0, sizeof(GB_LASER_DEV_T));
    dev->bus = bus;
    dev->i2c_address = addr;
    dev->vl53l1_dev.i2c_slave_address = addr;
    dev->initialized = false;
    dev->cached_distance_mm = 0;

error_exit:
    return res;
}

GB_RESULT GB_LASER_Init(GB_LASER_DEV_T *dev, GB_LASER_MODE_T mode, GB_LASER_TIMING_T timing_budget)
{
    GB_RESULT res = GB_OK;

    CHK_NULL(dev, GB_LASER_DEVICE_NULL);
    CHK_NULL(dev->bus, GB_LASER_BUS_NULL);

    VL53L1_SetI2CBus(dev->bus);

    GB_DEBUGI(LASER_TAG, "Initializing VL53L1X sensor at address 0x%02x", dev->i2c_address);

    CHK_NEG_ERROR(VL53L1_CommsInitialise(&dev->vl53l1_dev, VL53L1_I2C, 400), GB_LASER_INIT_FAIL);
    GB_DEBUGI(LASER_TAG, "VL53L1_CommsInitialise Done");
    CHK_NEG_ERROR(VL53L1_WaitDeviceBooted(&dev->vl53l1_dev), GB_LASER_INIT_FAIL);
    GB_DEBUGI(LASER_TAG, "VL53L1_WaitDeviceBooted Done");
    CHK_NEG_ERROR(VL53L1_DataInit(&dev->vl53l1_dev), GB_LASER_INIT_FAIL);
    GB_DEBUGI(LASER_TAG, "VL53L1_DataInit Done");
    CHK_NEG_ERROR(VL53L1_StaticInit(&dev->vl53l1_dev), GB_LASER_INIT_FAIL);
    GB_DEBUGI(LASER_TAG, "VL53L1_StaticInit Done");

    VL53L1_DistanceModes distance_mode;
    switch (mode)
    {
    case GB_LASER_MODE_SHORT:
        distance_mode = VL53L1_DISTANCEMODE_SHORT;
        break;
    case GB_LASER_MODE_MEDIUM:
        distance_mode = VL53L1_DISTANCEMODE_MEDIUM;
        break;
    case GB_LASER_MODE_LONG:
    default:
        distance_mode = VL53L1_DISTANCEMODE_LONG;
        break;
    }

    CHK_NEG_ERROR(VL53L1_SetDistanceMode(&dev->vl53l1_dev, distance_mode), GB_LASER_CFG_FAIL);
    GB_DEBUGI(LASER_TAG, "VL53L1_SetDistanceMode Done");
    CHK_NEG_ERROR(VL53L1_SetMeasurementTimingBudgetMicroSeconds(&dev->vl53l1_dev, (uint32_t)timing_budget * 1000),
                  GB_LASER_CFG_FAIL);
    GB_DEBUGI(LASER_TAG, "VL53L1_SetMeasurementTimingBudgetMicroSeconds Done");

    dev->mode = mode;
    dev->timing_budget = timing_budget;
    dev->initialized = true;

    GB_DEBUGI(LASER_TAG, "VL53L1X sensor initialized successfully");

error_exit:
    return res;
}

GB_RESULT GB_LASER_StartContinuous(GB_LASER_DEV_T *dev, uint32_t period_ms)
{
    GB_RESULT res = GB_OK;

    CHK_NULL(dev, GB_LASER_DEVICE_NULL);
    CHK_BOOL(dev->initialized);

    CHK_NEG_ERROR(VL53L1_SetInterMeasurementPeriodMilliSeconds(&dev->vl53l1_dev, period_ms), GB_LASER_START_FAIL);
    CHK_NEG_ERROR(VL53L1_StartMeasurement(&dev->vl53l1_dev), GB_LASER_START_FAIL);

    GB_DEBUGI(LASER_TAG, "Continuous ranging stared with period %d ms", period_ms);

error_exit:
    return res;
}

GB_RESULT GB_LASER_StopContinuous(GB_LASER_DEV_T *dev)
{
    GB_RESULT res = GB_OK;

    CHK_NULL(dev, GB_LASER_DEVICE_NULL);
    CHK_BOOL(dev->initialized);

    CHK_NEG_ERROR(VL53L1_StopMeasurement(&dev->vl53l1_dev), GB_LASER_STOP_FAIL);

    GB_DEBUGI(LASER_TAG, "Continuous ranging stopped");

error_exit:
    return res;
}

GB_RESULT GB_LASER_MeasureSingle(GB_LASER_DEV_T *dev, uint16_t *distance_mm)
{
    GB_RESULT res = GB_OK;
    VL53L1_RangingMeasurementData_t ranging_data;

    CHK_NULL(dev, GB_LASER_DEVICE_NULL);
    CHK_NULL(distance_mm, GB_LASER_DEVICE_NULL);
    CHK_BOOL(dev->initialized);

    CHK_NEG_ERROR(VL53L1_SetInterMeasurementPeriodMilliSeconds(&dev->vl53l1_dev, 0), GB_LASER_MEASURE_FAIL);
    CHK_NEG_ERROR(VL53L1_StartMeasurement(&dev->vl53l1_dev), GB_LASER_START_FAIL);

    uint8_t data_ready = 0;
    uint32_t timeout = 1000;  // 1 second timeout
    while (data_ready == 0 && timeout > 0) {
        CHK_NEG_ERROR(VL53L1_GetMeasurementDataReady(&dev->vl53l1_dev, &data_ready), GB_LASER_MEASURE_FAIL);
        if (data_ready == 0) {
            GB_SleepMs(1);
            timeout--;
        }
    }

    if (data_ready == 0) {
        CHK_GB_ERROR(GB_LASER_TIMEOUT);
    }

    CHK_NEG_ERROR(VL53L1_GetRangingMeasurementData(&dev->vl53l1_dev, &ranging_data), GB_LASER_READ_FAIL);
    CHK_NEG_ERROR(VL53L1_ClearInterruptAndStartMeasurement(&dev->vl53l1_dev), GB_LASER_MEASURE_FAIL);
    CHK_NEG_ERROR(VL53L1_StopMeasurement(&dev->vl53l1_dev), GB_LASER_MEASURE_FAIL);

    *distance_mm = ranging_data.RangeMilliMeter;

error_exit:
    return res;
}

GB_RESULT GB_LASER_ReadDistance(GB_LASER_DEV_T *dev, uint16_t *distance_mm, uint32_t timeout_ms)
{
    GB_RESULT res = GB_OK;
    VL53L1_RangingMeasurementData_t ranging_data;

    CHK_NULL(dev, GB_LASER_DEVICE_NULL);
    CHK_NULL(distance_mm, GB_LASER_DEVICE_NULL);
    CHK_BOOL(dev->initialized);

    uint8_t data_ready = 0;
    uint32_t elapsed_time = 0;
    while (data_ready == 0 && elapsed_time < timeout_ms) {
        CHK_NEG_ERROR(VL53L1_GetMeasurementDataReady(&dev->vl53l1_dev, &data_ready), GB_LASER_MEASURE_FAIL);
        if (data_ready == 0) {
            GB_SleepMs(1);
            elapsed_time++;
        }
    }

    if (data_ready == 0) {
        CHK_GB_ERROR(GB_LASER_TIMEOUT);
    }

    CHK_NEG_ERROR(VL53L1_GetRangingMeasurementData(&dev->vl53l1_dev, &ranging_data), GB_LASER_READ_FAIL);
    CHK_NEG_ERROR(VL53L1_ClearInterruptAndStartMeasurement(&dev->vl53l1_dev), GB_LASER_MEASURE_FAIL);

    *distance_mm = ranging_data.RangeMilliMeter;

error_exit:
    return res;
}

GB_RESULT GB_LASER_GetLastDistance(GB_LASER_DEV_T *dev, uint16_t *distance_mm)
{
    GB_RESULT res = GB_OK;
    uint8_t data_ready = 0;
    VL53L1_RangingMeasurementData_t ranging_data;

    CHK_NULL(dev, GB_LASER_DEVICE_NULL);
    CHK_NULL(distance_mm, GB_LASER_DEVICE_NULL);
    CHK_BOOL(dev->initialized);

    CHK_NEG_ERROR(VL53L1_GetMeasurementDataReady(&dev->vl53l1_dev, &data_ready), GB_LASER_MEASURE_FAIL);
    if (data_ready) {
        CHK_NEG_ERROR(VL53L1_GetRangingMeasurementData(&dev->vl53l1_dev, &ranging_data), GB_LASER_READ_FAIL);
        CHK_NEG_ERROR(VL53L1_ClearInterruptAndStartMeasurement(&dev->vl53l1_dev), GB_LASER_MEASURE_FAIL);
    }

    *distance_mm = ranging_data.RangeMilliMeter;

error_exit:
    return res;
}

GB_RESULT GB_LASER_GetMeasurement(GB_LASER_DEV_T *dev, VL53L1_RangingMeasurementData_t *ranging_data)
{
    GB_RESULT res = GB_OK;

    CHK_NULL(dev, GB_LASER_DEVICE_NULL);
    CHK_NULL(ranging_data, GB_LASER_DEVICE_NULL);
    CHK_BOOL(dev->initialized);

    CHK_NEG_ERROR(VL53L1_GetRangingMeasurementData(&dev->vl53l1_dev, ranging_data), GB_LASER_READ_FAIL);
    CHK_NEG_ERROR(VL53L1_ClearInterruptAndStartMeasurement(&dev->vl53l1_dev), GB_LASER_MEASURE_FAIL);

error_exit:
    return res;
}

GB_RESULT GB_LASER_CheckDataReady(GB_LASER_DEV_T *dev, uint8_t *data_ready)
{
    GB_RESULT res = GB_OK;

    CHK_NULL(dev, GB_LASER_DEVICE_NULL);
    CHK_NULL(data_ready, GB_LASER_DEVICE_NULL);
    CHK_BOOL(dev->initialized);

    CHK_NEG_ERROR(VL53L1_GetMeasurementDataReady(&dev->vl53l1_dev, data_ready), GB_LASER_MEASURE_FAIL);

error_exit:
    return res;
}
