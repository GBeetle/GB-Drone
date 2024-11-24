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

#ifndef __LIS3MDL_TYPES_H__
#define __LIS3MDL_TYPES_H__

#include "stdint.h"
#include "stdbool.h"

#ifdef __cplusplus
extern "C"
{
#endif


/**
 * @brief   Operation mode (OM) and output data rates (ODR)
 */
typedef enum {

    lis3mdl_lpm_0_625 = 0,  // low power mode at 0.625 Hz
    lis3mdl_lpm_1_25,       // low power mode at 1.25 Hz
    lis3mdl_lpm_2_5,        // low power mode at 2.5 Hz
    lis3mdl_lpm_5,          // low power mode at 5 Hz
    lis3mdl_lpm_10,         // low power mode at 10 Hz
    lis3mdl_lpm_20,         // low power mode at 20 Hz
    lis3mdl_lpm_40,         // low power mode at 40 Hz
    lis3mdl_lpm_80,         // low power mode at 80 Hz
    lis3mdl_lpm_1000,       // low power mode at 1000 Hz
    lis3mdl_mpm_560,        // medium performance mode at 560 Hz
    lis3mdl_hpm_300,        // high performance mode at 300 Hz
    lis3mdl_uhpm_155,       // ultra high performance mode at 155 Hz
    lis3mdl_low_power,      // low power mode at 0.625 Hz
} lis3mdl_mode_t;

typedef lis3mdl_mode_t mag_mode_t;

/**
 * @brief   measurement mode
 */
typedef enum {
    lis3mdl_power_down,
    lis3mdl_single_measurement,
    lis3mdl_continuous_measurement
} lis3mdl_measurement_mode_t;


/**
 * @brief   Full scale measurement range in Gauss
 */
typedef enum {

    lis3mdl_scale_4_Gs = 1,  //!< +/- 4 g  -> 8.192 LSB/g
    lis3mdl_scale_8_Gs,      //!< +/- 8 g  -> 4.096 LSB/g
    lis3mdl_scale_12_Gs,     //!< +/- 12 g  -> 2.731 LSB/g
    lis3mdl_scale_16_Gs      //!< +/- 16 g  -> 2.048 LSB/g

} lis3mdl_scale_t;

/**
 * @brief   Magnetic threshold interrupt configuration for INT signal
 */
typedef struct {

    uint16_t threshold; // threshold used for interrupt generation

    bool     x_enabled; // true - x exceeds threshold on positive side
    bool     y_enabled; // true - y exceeds threshold on positive side
    bool     z_enabled; // true - z exceeds threshold on positive side

    bool     latch;     // true - latch the interrupt until the interrupt
                        //        source has been read
    enum
    {
        lis3mdl_low_active  = 0,
        lis3mdl_high_active = 1

    } signal_level;     // level of interrupt signal

} lis3mdl_int_config_t;


/**
 * @brief   Magnetic threshold interrupt source of INT signal
 */
typedef struct {

    bool x_pos :1;     // true - x exceeds threshold on positive side
    bool y_pos :1;     // true - y exceeds threshold on positive side
    bool z_pos :1;     // true - z exceeds threshold on positive side

    bool x_neg :1;     // true - x exceeds threshold on negative side
    bool y_neg :1;     // true - y exceeds threshold on negative side
    bool z_neg :1;     // true - z exceeds threshold on negative side

    bool mroi  :1;     // true - internal measurement range overflow
    bool active:1;     // true - interrupt event occured

} lis3mdl_int_source_t;


/**
 * @brief   Raw data set as two's complements
 */
typedef struct {

    int16_t mx; // magnetic value on x axis
    int16_t my; // magnetic value on y axis
    int16_t mz; // magnetic value on z axis

} lis3mdl_raw_data_t;


/**
 * @brief   Floating point output value set in Gauss
 */
typedef struct {

    float mx;   // magnetic value on x axis
    float my;   // magnetic value on y axis
    float mz;   // magnetic value on z axis

} lis3mdl_float_data_t;


/**
 * @brief   LIS3MDL sensor device data structure type
 */
typedef struct {

    int       error_code;       // error code of last operation

    uint8_t   bus;              // I2C = x, SPI = 1
    uint8_t   addr;             // I2C = slave address, SPI = 0

    uint8_t   cs;               // ESP8266, ESP32: GPIO used as SPI CS
                                // __linux__: device index

    lis3mdl_scale_t  scale;     // full range scale (default 4 Gauss)

} lis3mdl_sensor_t;


#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* __LIS3MDL_TYPES_H__ */
