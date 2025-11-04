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

#ifndef __QMC5883L_H__
#define __QMC5883L_H__

#include <stdint.h>
#include <stdbool.h>
#include "results.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CONFIG_COMPASS_I2C
#include "i2c_bus.h"
typedef struct i2c compass_bus_t;
#elif CONFIG_COMPASS_SPI
#include "spi_bus.h"
typedef struct spi compass_bus_t;
#endif
typedef uint8_t compass_addr_t;

/**
 * Default I2C address
 */
#define QMC5883L_I2C_ADDR_DEF 0x0d

/**
 * Output data rate
 */
typedef enum {
    QMC5883L_DR_10 = 0, //!< 10Hz
    QMC5883L_DR_50,     //!< 50Hz
    QMC5883L_DR_100,    //!< 100Hz
    QMC5883L_DR_200,    //!< 200Hz
} qmc5883l_odr_t;

/**
 * Oversampling rate
 */
typedef enum {
    QMC5883L_OSR_64 = 0, //!< 64 samples
    QMC5883L_OSR_128,    //!< 128 samples
    QMC5883L_OSR_256,    //!< 256 samples
    QMC5883L_OSR_512,    //!< 512 samples
} qmc5883l_osr_t;

/**
 * Field range
 */
typedef enum {
    QMC5883L_RNG_2 = 0,//!< -2G..+2G
    QMC5883L_RNG_8     //!< -8G..+8G
} qmc5883l_range_t;

/**
 * Mode
 */
typedef enum {
    QMC5883L_MODE_STANDBY = 0, //!< Standby low power mode, no measurements
    QMC5883L_MODE_CONTINUOUS   //!< Continuous measurements
} qmc5883l_mode_t;


/**
 * Raw measurement result
 */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} qmc5883l_raw_data_t;

/**
 * Measurement result, milligauss
 */
typedef struct
{
    float x;
    float y;
    float z;
} qmc5883l_data_t;

/**
 * Device descriptor
 */
typedef struct {
    compass_bus_t *bus;
    compass_addr_t addr;
    qmc5883l_range_t range;
} qmc5883l_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @param port I2C port number
 * @param addr I2C address
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return `GB_OK` on success
 */
GB_RESULT qmc5883l_init_desc(qmc5883l_t *dev, compass_bus_t *bus, compass_addr_t addr);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `GB_OK` on success
 */
GB_RESULT qmc5883l_free_desc(qmc5883l_t *dev);

/**
 * @brief Reset device
 *
 * @param dev Device descriptor
 * @return `GB_OK` on success
 */
GB_RESULT qmc5883l_reset(qmc5883l_t *dev);

/**
 * @brief Read chip ID
 *
 * @param dev Device descriptor
 * @param[out] id Chip ID
 * @return `GB_OK` on success
 */
GB_RESULT qmc5883l_get_chip_id(qmc5883l_t *dev, uint8_t *id);

/**
 * @brief Set device mode
 *
 * @param dev Device descriptor
 * @param mode Mode
 * @return `GB_OK` on success
 */
GB_RESULT qmc5883l_set_mode(qmc5883l_t *dev, qmc5883l_mode_t mode);

/**
 * @brief Read current device mode
 *
 * @param dev Device descriptor
 * @param[out] mode Mode
 * @return `GB_OK` on success
 */
GB_RESULT qmc5883l_get_mode(qmc5883l_t *dev, qmc5883l_mode_t *mode);

/**
 * @brief Set device configuration
 *
 * @param dev Device descriptor
 * @param odr Output data rate
 * @param osr Oversampling
 * @param rng Field range
 * @return `GB_OK` on success
 */
GB_RESULT qmc5883l_set_config(qmc5883l_t *dev, qmc5883l_odr_t odr, qmc5883l_osr_t osr, qmc5883l_range_t rng);

/**
 * @brief Read current device configuration
 *
 * @param dev Device descriptor
 * @param[out] odr Output data rate
 * @param[out] osr Oversampling
 * @param[out] rng Field range
 * @return `GB_OK` on success
 */
GB_RESULT qmc5883l_get_config(qmc5883l_t *dev, qmc5883l_odr_t *odr, qmc5883l_osr_t *osr, qmc5883l_range_t *rng);

/**
 * @brief Enable/disable interrupt pin
 *
 * @param dev Device descriptor
 * @param enable Enable interrupt if true
 * @return `GB_OK` on success
 */
GB_RESULT qmc5883l_set_int(qmc5883l_t *dev, bool enable);

/**
 * @brief Get interrupt pin state
 *
 * @param dev Device descriptor
 * @param[out] enable Interrupt pin enabled if true
 * @return `GB_OK` on success
 */
GB_RESULT qmc5883l_get_int(qmc5883l_t *dev, bool *enable);

/**
 * @brief Get magnetic data state
 *
 * @param dev Device descriptor
 * @param[out] ready Magnetic data ready to read if true
 * @return `GB_OK` on success
 */
GB_RESULT qmc5883l_data_ready(qmc5883l_t *dev, bool *ready);

/**
 * @brief Read raw magnetic data
 *
 * @param dev Device descriptor
 * @param[out] raw Raw magnetic data
 * @return `GB_OK` on success
 */
GB_RESULT qmc5883l_get_raw_data(qmc5883l_t *dev, qmc5883l_raw_data_t *raw);

/**
 * @brief Convert raw magnetic data to milligauss
 *
 * @param dev Device descriptor
 * @param raw Raw magnetic data
 * @param[out] data Magnetic data in mG
 * @return `GB_OK` on success
 */
GB_RESULT qmc5883l_raw_to_mg(qmc5883l_t *dev, qmc5883l_raw_data_t *raw, qmc5883l_data_t *data);

/**
 * @brief Read magnetic data in milligauss
 *
 * @param dev Device descriptor
 * @param[out] data Magnetic data in mG
 * @return `GB_OK` on success
 */
GB_RESULT qmc5883l_get_data(qmc5883l_t *dev, qmc5883l_data_t *data);

/**
 * @brief Read raw temperature data (see datasheet)
 *
 * @param dev Device descriptor
 * @param[out] temp Raw temperature data
 * @return `GB_OK` on success
 */
GB_RESULT qmc5883l_get_raw_temp(qmc5883l_t *dev, int16_t *temp);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __QMC5883L_H__ */
