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

#ifndef __MS5611_H__
#define __MS5611_H__

#include <stdint.h>
#include "sdkconfig.h"
#include "results.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined CONFIG_BARO_I2C
#include "i2c_bus.h"
typedef struct i2c baro_bus_t;
#elif defined CONFIG_BARO_SPI
#include "spi_bus.h"
typedef struct spi baro_bus_t;
#else
#error "No protocol defined for barometer"
#endif
typedef uint8_t baro_addr_t;

#define MS5611_ADDR_CSB_HIGH    0x76
#define MS5611_ADDR_CSB_LOW     0x77

/**
 * Oversampling ratio
 */
typedef enum
{
    MS5611_OSR_256  = 0x00, //!< 256 samples per measurement
    MS5611_OSR_512  = 0x02, //!< 512 samples per measurement
    MS5611_OSR_1024 = 0x04, //!< 1024 samples per measurement
    MS5611_OSR_2048 = 0x06, //!< 2048 samples per measurement
    MS5611_OSR_4096 = 0x08  //!< 4096 samples per measurement
} ms5611_osr_t;

/**
 * Configuration data
 */
typedef struct
{
    uint16_t menu;
    uint16_t sens;       //!< C1 Pressure sensitivity                             | SENS_t1
    uint16_t off;        //!< C2 Pressure offset                                  | OFF_t1
    uint16_t tcs;        //!< C3 Temperature coefficient of pressure sensitivity  | TCS
    uint16_t tco;        //!< C4 Temperature coefficient of pressure offset       | TCO
    uint16_t t_ref;      //!< C5 Reference temperature                            | T_ref
    uint16_t tempsens;   //!< C6 Temperature coefficient of the temperature       | TEMPSENSE
    uint16_t crc;
} ms5611_config_data_t;

/**
 * Device descriptor
 */
typedef struct
{
    baro_bus_t* bus;
    baro_addr_t addr;
    ms5611_osr_t osr;                 //!< Oversampling setting
    ms5611_config_data_t config_data; //!< Device configuration, filled upon initialize
} ms5611_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @param addr I2C address, `MS5611_ADDR_CSB_HIGH` or `MS5611_ADDR_CSB_LOW`
 * @param port I2C port
 * @param sda_gpio GPIO pin for SDA
 * @param scl_gpio GPIO pin for SCL
 * @return `GB_OK` on success
 */
GB_RESULT ms5611_init_desc(ms5611_t *dev, baro_bus_t* bus, baro_addr_t addr);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `GB_OK` on success
 */
GB_RESULT ms5611_free_desc(ms5611_t *dev);

/**
 * @brief Init MS5611-01BA03
 *
 * Reset device and read calibration data
 *
 * @param dev Device descriptor
 * @param osr Oversampling ratio
 * @return `GB_OK` on success
 */
GB_RESULT ms5611_init(ms5611_t *dev, ms5611_osr_t osr);

/**
 * @brief Measure pressure and temperature
 *
 * @param dev Device descriptor
 * @param[out] pressure Pressure, Pa
 * @param[out] temperature Temperature, degrees Celsius
 * @return `GB_OK` on success
 */
GB_RESULT ms5611_get_sensor_data(ms5611_t *dev, float *pressure, float *temperature);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __MS5611_H__ */