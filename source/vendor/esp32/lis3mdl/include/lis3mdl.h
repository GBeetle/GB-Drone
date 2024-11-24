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

#ifndef __LIS3MDL_H__
#define __LIS3MDL_H__

#include "lis3mdl_types.h"

// Uncomment one of the following defines to enable debug output
// #define LIS3MDL_DEBUG_LEVEL_1    // only error messages
// #define LIS3MDL_DEBUG_LEVEL_2    // debug and error messages

// LIS3MDL addresses
#define LIS3MDL_I2C_ADDRESS_1           0x1c  // SDO pin is low
#define LIS3MDL_I2C_ADDRESS_2           0x1e  // SDO pin is high

// LIS3MDL chip id
#define LIS3MDL_CHIP_ID                 0x3d  // LIS3MDL_REG_WHO_AM_I<7:0>

// Definition of error codes
#define LIS3MDL_OK                      0
#define LIS3MDL_NOK                     -1

#define LIS3MDL_INT_ERROR_MASK          0x000f
#define LIS3MDL_DRV_ERROR_MASK          0xfff0

// Error codes for I2C and SPI interfaces ORed with LIS3MDL driver error codes
#define LIS3MDL_I2C_READ_FAILED         1
#define LIS3MDL_I2C_WRITE_FAILED        2
#define LIS3MDL_I2C_BUSY                3
#define LIS3MDL_SPI_WRITE_FAILED        4
#define LIS3MDL_SPI_READ_FAILED         5
#define LIS3MDL_SPI_BUFFER_OVERFLOW     6

// LIS3MDL driver error codes ORed with error codes for I2C and SPI interfaces
#define LIS3MDL_WRONG_CHIP_ID              ( 1 << 8)
#define LIS3MDL_GET_RAW_DATA_FAILED        ( 2 << 8)
#define LIS3MDL_CONFIG_INT_FAILED          ( 3 << 8)
#define LIS3MDL_INT_SOURCE_FAILED          ( 4 << 8)
#define LIS3MDL_GET_ADC_DATA_FAILED        ( 5 << 8)

// register addresses
#define LIS3MDL_REG_WHO_AM_I      0x0f
#define LIS3MDL_REG_CTRL1         0x20
#define LIS3MDL_REG_CTRL2         0x21
#define LIS3MDL_REG_CTRL3         0x22
#define LIS3MDL_REG_CTRL4         0x23
#define LIS3MDL_REG_CTRL5         0x24
#define LIS3MDL_REG_STATUS        0x27
#define LIS3MDL_REG_OUT_X_L       0x28
#define LIS3MDL_REG_OUT_X_H       0x29
#define LIS3MDL_REG_OUT_Y_L       0x2a
#define LIS3MDL_REG_OUT_Y_H       0x2b
#define LIS3MDL_REG_OUT_Z_L       0x2c
#define LIS3MDL_REG_OUT_Z_H       0x2d
#define LIS3MDL_REG_TEMP_OUT_L    0x2e
#define LIS3MDL_REG_TEMP_OUT_H    0x2f
#define LIS3MDL_REG_INT_CFG       0x30
#define LIS3MDL_REG_INT_SRC       0x31
#define LIS3MDL_REG_INT_THS_L     0x32
#define LIS3MDL_REG_INT_THS_H     0x33

#endif /* __LIS3MDL_H__ */
