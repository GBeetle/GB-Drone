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
#ifndef _MAX1704X_H__
#define _MAX1704X_H__

#include <stdbool.h>
#include "results.h"
#include "i2c_bus.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX1704X_I2C_ADDR 0x36

/**
 * Device model
 */

typedef enum
{
    MAX17043_4 = 0,
    MAX17048_9,
} GB_MAX1704X_T;

/**
 * Alert Status structure
 */
typedef struct
{
    bool reset_indicator;           //!< Reset indicator
    bool voltage_high;              //!< Voltage high alert
    bool voltage_low;               //!< Voltage low alert
    bool voltage_reset;             //!< Voltage reset alert
    bool soc_low;                   //!< SOC low alert, set when SOC cross empty_alert_thresh
    bool soc_change;                //!< SOC change alert, set when SOC change is at least 1%
    bool vreset_alert;              //!< Set to enable voltage reset alert under conditions specified in the valert register
} GB_MAX1704X_STATUS_T;

/**
 * MAX1704X configuration structure
 */
typedef struct
{
    uint8_t rcomp;                  //!< RCOMP register value - default 0x97
    bool sleep_mode;                //!< Sleep mode - set to true to enter sleep mode
    bool soc_change_alert;          //!< SOC change alert - enable/disable SOC change alert
    bool alert_status;              //!< Alert status - read to check if alert has been triggered
    uint8_t empty_alert_thresh;     //!< Empty alert threshold - default 0x1C (4%, 32 - ATHD)
    uint8_t active_threshold;       //!< Exits hibernation when IOCV-CELLI above this threshold
    uint8_t hibernate_threshold;    //!< Enters Hibernation when CRATE falls below this threshold
} GB_MAX1704X_CONFIG_T;

/**
 * Device descriptor
 */
typedef struct
{
    struct i2c *i2c_dev;
    GB_MAX1704X_T model;
    GB_MAX1704X_CONFIG_T config;
    GB_MAX1704X_STATUS_T status;
} GB_MAX1704X_DEV_T;

/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @param port I2C port
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return `GB_OK` on success
 */
GB_RESULT GB_Max1704xInit(GB_MAX1704X_DEV_T *dev);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `GB_OK` on success
 */
GB_RESULT GB_Max1704xFree(GB_MAX1704X_DEV_T *dev);

/**
 * @brief Quickstart battery fuel gauge
 *
 * @param dev Device descriptor
 * @return `GB_OK` on success
 */
GB_RESULT GB_Max1704xStart(GB_MAX1704X_DEV_T *dev);

/**
 * @brief Get battery voltage
 *
 * @param dev Device descriptor
 * @param voltage Battery voltage
 * @return `GB_OK` on success
 */
GB_RESULT GB_Max1704xGetVoltage(GB_MAX1704X_DEV_T *dev, float *voltage);

/**
 * @brief Get state of charge
 *
 * @param dev Device descriptor
 * @param soc State of charge
 * @return `GB_OK` on success
 */
GB_RESULT GB_Max1704xGetSoc(GB_MAX1704X_DEV_T *dev, float *soc);

/**
 * @brief Get rate of battery charge or discharge
 *
 * @param dev Device descriptor
 * @param crate Rate of charge or discharge
 * @return `GB_OK` on success
 */
GB_RESULT GB_Max1704xGetCrate(GB_MAX1704X_DEV_T *dev, float *crate);

/**
 * @brief Get the production version of the chip
 *
 * @param dev Device descriptor
 * @param version Production version
 * @return `GB_OK` on success
 */
GB_RESULT GB_Max1704xGetVersion(GB_MAX1704X_DEV_T *dev, uint16_t *version);

/**
 * @brief Get the configuration register
 *
 * @param dev Device descriptor
 * @return `GB_OK` on success
 */
GB_RESULT GB_Max1704xGetConfig(GB_MAX1704X_DEV_T *dev);

/**
 * @brief Set the configuration register
 *
 * @param dev Device descriptor
 * @param config Configuration register
 * @return `GB_OK` on success
 */
GB_RESULT GB_Max1704xSetConfig(GB_MAX1704X_DEV_T *dev, GB_MAX1704X_CONFIG_T *config);

/**
 * @brief Get the status register
 *
 * @param dev Device descriptor
 * @return `GB_OK` on success
 */
GB_RESULT GB_Max1704xGetStatus(GB_MAX1704X_DEV_T *dev);

/**
 * @brief Set the status register
 *
 * @param dev Device descriptor
 * @param status Status register
 * @return `GB_OK` on success
 *
 * @note Use this function to clear alert flags after servicing the alert
 */
GB_RESULT GB_Max1704xSetStatus(GB_MAX1704X_DEV_T *dev, GB_MAX1704X_STATUS_T *status);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* end of include guard: _MAX1704X_H__ */
