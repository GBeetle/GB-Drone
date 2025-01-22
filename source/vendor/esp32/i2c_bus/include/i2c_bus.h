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

#ifndef _I2CBUS_HPP_
#define _I2CBUS_HPP_

#include <stdint.h>
#include "error_handle.h"

#define GB_MAX_I2C_DEV_NUMS 10

typedef int GB_I2C_Port;

typedef struct GB_I2C_DevCell {
    uint8_t devAddress;
    void **devHandle;
} GB_I2C_DevCell;

// i2c class definition
struct i2c {
    GB_I2C_Port port;            /*!< i2c port: I2C_NUM_0 or I2C_NUM_1 */
    uint32_t ticksToWait;       /*!< Timeout in ticks for read and write */
    void **busHandle;
    GB_I2C_DevCell device[GB_MAX_I2C_DEV_NUMS];

    /** *** i2c Begin ***
     * @brief  Config i2c bus and Install Driver
     * @param  sda_io_num    [GPIO number for SDA line]
     * @param  scl_io_num    [GPIO number for SCL line]
     * @param  sda_pullup_en [Enable internal pullup on SDA line]
     * @param  scl_pullup_en [Enable internal pullup on SCL line]
     * @param  clk_speed     [i2c clock frequency for master mode, (no higher than 1MHz for now), Default 100KHz]
     *                       @see "driver/i2c.h"
     * @return               - GB_OK   Success
     *                       - GB_I2C_CFG_FAIL Parameter error
     *                       - GB_I2C_INS_FAIL Driver install error
     */
    GB_RESULT (*begin)(struct i2c *i2c, uint32_t sda_io_num, uint32_t scl_io_num, uint32_t clk_speed);

    GB_RESULT (*addDevice)(struct i2c *i2c, uint8_t devAddr, uint32_t clk_speed);

    /**
     * Stop i2c bus and unninstall driver
     */
    GB_RESULT (*close)(struct i2c *i2c);

    /**
     * *** WRITING interface ***
     * @brief  i2c commands for writing to a 8-bit slave device register.
     *         All of them returns standard esp_err_t codes. So it can be used
     *         with CHK_RES();
     * @param  devAddr   [i2c slave device register]
     * @param  regAddr   [Register address to write to]
     * @param  bitNum    [Bit position number to write to (bit 7~0)]
     * @param  bitStart  [Start bit number when writing a bit-sequence (MSB)]
     * @param  data      [Value(s) to be write to the register]
     * @param  length    [Number of bytes to write (should be within the data buffer size)]
     *                   [writeBits() -> Number of bits after bitStart (including)]
     * @param  timeout   [Custom timeout for the particular call]
     * @return  - GB_OK Success
     *          - GB_I2C_RW_FAIL   i2c read/wirte failed
     */
    GB_RESULT (*writeBit)(struct i2c *i2c, uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
    GB_RESULT (*writeBits)(struct i2c *i2c, uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
    GB_RESULT (*writeByte)(struct i2c *i2c, uint8_t devAddr, uint8_t regAddr, uint8_t data);
    GB_RESULT (*writeBytes)(struct i2c *i2c, uint8_t devAddr, uint8_t regAddr, size_t length, const uint8_t *data);

    /**
     * *** READING interface ***
     * @breif  i2c commands for reading a 8-bit slave device register.
     *         All of them returns standard esp_err_t codes.So it can be used
     *         with CHK_RES();
     * @param  devAddr   [i2c slave device register]
     * @param  regAddr   [Register address to read from]
     * @param  bitNum    [Bit position number to write to (bit 7~0)]
     * @param  bitStart  [Start bit number when writing a bit-sequence (MSB)]
     * @param  data      [Buffer to store the read value(s)]
     * @param  length    [Number of bytes to read (should be within the data buffer size)]
     * @param  timeout   [Custom timeout for the particular call]
     * @return  - GB_OK Success
     *          - GB_I2C_RW_FAIL   i2c read/wirte failed
     */
    GB_RESULT (*readBit)(struct i2c *i2c, uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data);
    GB_RESULT (*readBits)(struct i2c *i2c, uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
    GB_RESULT (*readByte)(struct i2c *i2c, uint8_t devAddr, uint8_t regAddr, uint8_t *data);
    GB_RESULT (*readBytes)(struct i2c *i2c, uint8_t devAddr, uint8_t regAddr, size_t length, uint8_t *data);
};

// Default Objects
extern struct i2c i2c0;        /*!< port: I2C_NUM_0 */
extern struct i2c i2c1;        /*!< port: I2C_NUM_1 */

#endif /* end of include guard: _I2CBUS_H_ */
