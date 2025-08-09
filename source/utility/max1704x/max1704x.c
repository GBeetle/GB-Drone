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

#include <string.h>
#include "max1704x.h"
#include "io_define.h"

#define I2C_FREQ_HZ 400000

/**
 * MAX1704X registers
 */
#define MAX1704X_REGISTER_VCELL         0x02
#define MAX1704X_REGISTER_SOC           0x04
#define MAX1704X_REGISTER_MODE          0x06
#define MAX1704X_REGISTER_VERSION       0x08
#define MAX1704X_REGISTER_HIBRT         0x0A
#define MAX1704X_REGISTER_CONFIG        0x0C
#define MAX1704X_REGISTER_VALRT         0x14    // MAX17048/MAX17049 only
#define MAX1704X_REGISTER_CRATE         0x16    // MAX17048/MAX17049 only
#define MAX1704X_REGISTER_VRESET        0x18    // MAX17048/MAX17049 only
#define MAX1704X_REGISTER_STATUS        0x1A    // MAX17048/MAX17049 only
#define MAX1704X_REGISTER_COMMAND       0xFE

/**
 * MAX1704X modes
 */

#define MAX1704X_RESET_COMMAND          0x5400
#define MAX1704X_QUICKSTART_MODE        0x4000

/**
 * MAX1704X config register bits
 */

#define MAX1704X_CONFIG_ALRT_BIT        ((1U << 5))
#define MAX1704X_CONFIG_ALSC_BIT        ((1U << 6))
#define MAX1704X_CONFIG_SLEEP_BIT       ((1U << 7))
#define MAX1704X_CONFIG_ATHD_MASK       0x1F
#define MAX1704X_CONFIG_ATHD_SHIFT      0
#define MAX1704X_STATUS_RI_BIT          ((1U << 0))
#define MAX1704X_STATUS_VH_BIT          ((1U << 1))
#define MAX1704X_STATUS_VL_BIT          ((1U << 2))
#define MAX1704X_STATUS_VR_BIT          ((1U << 3))
#define MAX1704X_STATUS_SL_BIT          ((1U << 4))
#define MAX1704X_STATUS_SC_BIT          ((1U << 5))
#define MAX1704X_STATUS_VRA_BIT         ((1U << 6))

/**
 * MAX1704X precision constants:
 *
 * Precision calculated is by per bit, not per LSB.
 */

#define MAX17043_MV_PRECISION           1.25f        // 1.25mV precision
#define MAX17048_MV_PRECISION           0.078125f    // 0.078125mV precision
#define MAX1704X_CRATE_PRECISION        0.208f       // 0.208% precision
#define MAX1704X_VALRT_PRECISION        20.0f        // 20mV precision
#define MAX1704X_HIBRT_VCELL_PRECISION  1.25f        // 1.25mV precision
#define MAX1704X_HIBRT_CRATE_PRECISION  0.208f       // 0.208% precision

/**
 * Private functions
 */

int16_t be16_to_cpu_signed(const uint8_t data[2])
{
    int16_t r;
    uint16_t u = (unsigned)data[1] | ((unsigned)data[0] << 8);
    memcpy(&r, &u, sizeof r);
    return r;
}

int16_t le16_to_cpu_signed(const uint8_t data[2])
{
    int16_t r;
    uint16_t u = (unsigned)data[0] | ((unsigned)data[1] << 8);
    memcpy(&r, &u, sizeof r);
    return r;
}

/**
 * Public functions
 */

GB_RESULT GB_Max1704xInit(GB_MAX1704X_DEV_T *dev)
{
    GB_RESULT res = GB_OK;

    CHK_NULL(dev, GB_INVALID_ARGUMENT);

    dev->i2c_dev = &i2c1;

    CHK_RES(dev->i2c_dev->begin(&i2c1, MAX_SDA, MAX_SCL, I2C_FREQ_HZ));
    CHK_RES(dev->i2c_dev->addDevice(&i2c1, MAX1704X_I2C_ADDR, I2C_FREQ_HZ));

error_exit:
    return res;
}

GB_RESULT GB_Max1704xFree(GB_MAX1704X_DEV_T *dev)
{
    GB_RESULT res = GB_OK;

    CHK_NULL(dev, GB_INVALID_ARGUMENT);
    CHK_RES(dev->i2c_dev->close(dev->i2c_dev));

error_exit:
    return res;
}

GB_RESULT GB_Max1704xStart(GB_MAX1704X_DEV_T *dev)
{
    GB_RESULT res = GB_OK;
    uint8_t data[2] = { 0x40, 0x00 };

    CHK_NULL(dev, GB_INVALID_ARGUMENT);
    CHK_RES(dev->i2c_dev->writeBytes(dev->i2c_dev, MAX1704X_I2C_ADDR, MAX1704X_REGISTER_MODE, 2, data));

    GB_DEBUGI(BATTERY_TAG, "MAX1704X Quickstart");

error_exit:
    return res;
}

GB_RESULT GB_Max1704xGetVoltage(GB_MAX1704X_DEV_T *dev, float *voltage)
{
    GB_RESULT res = GB_OK;
    uint8_t data[2];
    int value;

    CHK_NULL(dev, GB_INVALID_ARGUMENT);
    CHK_NULL(voltage, GB_INVALID_ARGUMENT);

    CHK_RES(dev->i2c_dev->readBytes(dev->i2c_dev, MAX1704X_I2C_ADDR, MAX1704X_REGISTER_VCELL, 2, data));

    if (dev->model == MAX17043_4) {
        value = (data[0] << 4) | (data[1] >> 4);
        *voltage = ((float)value * MAX17043_MV_PRECISION) / 1000;
    } else {
        *voltage = (((float)data[0] * 256 + (float)data[1]) * MAX17048_MV_PRECISION) / 1000;
    }

error_exit:
    return res;
}

GB_RESULT GB_Max1704xGetSoc(GB_MAX1704X_DEV_T *dev, float *soc)
{
    GB_RESULT res = GB_OK;
    uint8_t data[2];

    CHK_NULL(dev, GB_INVALID_ARGUMENT);
    CHK_NULL(soc, GB_INVALID_ARGUMENT);
    CHK_RES(dev->i2c_dev->readBytes(dev->i2c_dev, MAX1704X_I2C_ADDR, MAX1704X_REGISTER_SOC, 2, data));

    *soc = (float)data[0] + ((float)data[1]) / 256;
    if (*soc > 100) {
        *soc = 100.0f;
    }
error_exit:
    return res;
}

GB_RESULT GB_Max1704xGetCrate(GB_MAX1704X_DEV_T *dev, float *crate)
{
    GB_RESULT res = GB_OK;
    int16_t crate_value;
    uint8_t data[2];

    CHK_NULL(dev, GB_INVALID_ARGUMENT);
    CHK_NULL(crate, GB_INVALID_ARGUMENT);
    CHK_RES(dev->i2c_dev->readBytes(dev->i2c_dev, MAX1704X_I2C_ADDR, MAX1704X_REGISTER_CRATE, 2, data));

    if (dev->model == MAX17043_4) {
        GB_DEBUGI(BATTERY_TAG, "MAX1704X_REGISTER_CRATE is not supported by MAX17043");
    } else {
        crate_value = be16_to_cpu_signed(data);
        GB_DEBUGI(BATTERY_TAG, "crateValue: %d", (int)crate_value);
        *crate = ((float)crate_value * MAX1704X_CRATE_PRECISION);
    }
error_exit:
    return res;
}

GB_RESULT GB_Max1704xGetVersion(GB_MAX1704X_DEV_T *dev, uint16_t *version)
{
    GB_RESULT res = GB_OK;
    uint8_t data[2];

    CHK_NULL(dev, GB_INVALID_ARGUMENT);
    CHK_NULL(version, GB_INVALID_ARGUMENT);
    CHK_RES(dev->i2c_dev->readBytes(dev->i2c_dev, MAX1704X_I2C_ADDR, MAX1704X_REGISTER_VERSION, 2, data));

    *version = (data[0] << 8) | data[1];
error_exit:
    return res;
}

GB_RESULT GB_Max1704xGetConfig(GB_MAX1704X_DEV_T *dev)
{
    GB_RESULT res = GB_OK;
    uint8_t data[2];

    CHK_NULL(dev, GB_INVALID_ARGUMENT);
    CHK_RES(dev->i2c_dev->readBytes(dev->i2c_dev, MAX1704X_I2C_ADDR, MAX1704X_REGISTER_CONFIG, 2, data));

    dev->config.rcomp = data[0];
    dev->config.sleep_mode = (data[1] & MAX1704X_CONFIG_SLEEP_BIT) ? true : false;
    dev->config.soc_change_alert = (data[1] & MAX1704X_CONFIG_ALSC_BIT) ? true : false;
    dev->config.alert_status = (data[1] & MAX1704X_CONFIG_ALRT_BIT) ? true : false;
    dev->config.empty_alert_thresh = 32 - ((data[1] & MAX1704X_CONFIG_ATHD_MASK) >> MAX1704X_CONFIG_ATHD_SHIFT);

error_exit:
    return res;
}

GB_RESULT GB_Max1704xSetConfig(GB_MAX1704X_DEV_T *dev, GB_MAX1704X_CONFIG_T *config)
{
    GB_RESULT res = GB_OK;
    uint8_t data[2];

    CHK_NULL(dev, GB_INVALID_ARGUMENT);
    CHK_NULL(config, GB_INVALID_ARGUMENT);

    if (config->rcomp) {
       data[0] = config->rcomp;
       dev->config.rcomp = config->rcomp;
    }

    data[1] = 0;
    dev->config.sleep_mode = config->sleep_mode;
    if (config->sleep_mode) {
        data[1] |= MAX1704X_CONFIG_SLEEP_BIT;
    }

    dev->config.soc_change_alert = config->soc_change_alert;
    if (config->soc_change_alert) {
        data[1] |= MAX1704X_CONFIG_ALSC_BIT;
    }

    dev->config.alert_status = config->alert_status;
    if (config->alert_status) {
        data[1] |= MAX1704X_CONFIG_ALRT_BIT;
    }

    dev->config.empty_alert_thresh = 32 - config->empty_alert_thresh;
    data[1] |= (32 - config->empty_alert_thresh) << MAX1704X_CONFIG_ATHD_SHIFT;

    CHK_RES(dev->i2c_dev->writeBytes(dev->i2c_dev, MAX1704X_I2C_ADDR, MAX1704X_REGISTER_CONFIG, 2, data));

error_exit:
    return res;
}

GB_RESULT GB_Max1704xGetStatus(GB_MAX1704X_DEV_T *dev)
{
    GB_RESULT res = GB_OK;
    uint8_t data[2];

    CHK_NULL(dev, GB_INVALID_ARGUMENT);
    if (dev->model == MAX17043_4) {
        GB_DEBUGE(BATTERY_TAG, "MAX1704X STATUS is not supported by MAX17043");
        return GB_INVALID_ARGUMENT;
    }

    CHK_RES(dev->i2c_dev->readBytes(dev->i2c_dev, MAX1704X_I2C_ADDR, MAX1704X_REGISTER_STATUS, 2, data));

    dev->status.reset_indicator = (data[0] & MAX1704X_STATUS_RI_BIT) ? true : false;
    dev->status.voltage_high = (data[0] & MAX1704X_STATUS_VH_BIT) ? true : false;
    dev->status.voltage_low = (data[0] & MAX1704X_STATUS_VL_BIT) ? true : false;
    dev->status.voltage_reset = (data[0] & MAX1704X_STATUS_VR_BIT) ? true : false;
    dev->status.soc_low = (data[0] & MAX1704X_STATUS_SL_BIT) ? true : false;
    dev->status.soc_change = (data[0] & MAX1704X_STATUS_SC_BIT) ? true : false;
    dev->status.vreset_alert = (data[0] & MAX1704X_STATUS_VRA_BIT) ? true : false;

error_exit:
    return res;
}

GB_RESULT GB_Max1704xSetStatus(GB_MAX1704X_DEV_T *dev, GB_MAX1704X_STATUS_T *status)
{
    GB_RESULT res = GB_OK;
    uint8_t data[2];

    CHK_NULL(dev, GB_INVALID_ARGUMENT);
    CHK_NULL(status, GB_INVALID_ARGUMENT);

    if (dev->model == MAX17043_4) {
        GB_DEBUGE(BATTERY_TAG, "MAX1704X STATUS is not supported by MAX17043");
        return GB_INVALID_ARGUMENT;
    }

    data[0] = 0;
    if (status->reset_indicator) {
        data[0] |= MAX1704X_STATUS_RI_BIT;
    }
    if (status->voltage_high) {
        data[0] |= MAX1704X_STATUS_VH_BIT;
    }
    if (status->voltage_low) {
        data[0] |= MAX1704X_STATUS_VL_BIT;
    }
    if (status->voltage_reset) {
        data[0] |= MAX1704X_STATUS_VR_BIT;
    }
    if (status->soc_low) {
        data[0] |= MAX1704X_STATUS_SL_BIT;
    }
    if (status->soc_change) {
        data[0] |= MAX1704X_STATUS_SC_BIT;
    }
    if (status->vreset_alert) {
        data[0] |= MAX1704X_STATUS_VRA_BIT;
    }
    data[1] = 0;

    CHK_RES(dev->i2c_dev->writeBytes(dev->i2c_dev, MAX1704X_I2C_ADDR, MAX1704X_REGISTER_STATUS, 2, data));

error_exit:
    return res;
}
