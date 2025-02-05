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

#include <stdio.h>
#include <stdint.h>
#include "i2c_bus.h"
#include "driver/gpio.h"
#include "esp_err.h"

#include "driver/i2c_master.h"
#include "sdkconfig.h"
#include "log_sys.h"

#define I2C_MASTER_TIMEOUT_MS 1000

#define I2C_MASTER_ACK_EN   true    /*!< Enable ack check for master */
#define I2C_MASTER_ACK_DIS  false   /*!< Disable ack check for master */

static GB_RESULT begin(struct i2c *i2c, uint32_t sda_io_num, uint32_t scl_io_num, uint32_t clk_speed);
static GB_RESULT addDevice(struct i2c *i2c, uint8_t devAddr, uint32_t clk_speed);
static GB_RESULT close(struct i2c *i2c);
/*******************************************************************************
 * WRITING
 ******************************************************************************/
static GB_RESULT writeBit(struct i2c *i2c, uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
static GB_RESULT writeBits(struct i2c *i2c, uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
static GB_RESULT writeByte(struct i2c *i2c, uint8_t devAddr, uint8_t regAddr, uint8_t data);
static GB_RESULT writeBytes(struct i2c *i2c, uint8_t devAddr, uint8_t regAddr, size_t length, const uint8_t *data);

/*******************************************************************************
 * READING
 ******************************************************************************/
static GB_RESULT readBit(struct i2c *i2c, uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data);
static GB_RESULT readBits(struct i2c *i2c, uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
static GB_RESULT readByte(struct i2c *i2c, uint8_t devAddr, uint8_t regAddr, uint8_t *data);
static GB_RESULT readBytes(struct i2c *i2c, uint8_t devAddr, uint8_t regAddr, size_t length, uint8_t *data);

/*******************************************************************************
 * OBJECTS
 ******************************************************************************/
struct i2c i2c0 = {
    .deviceNum = 0,
    .port = I2C_NUM_0,
    .begin = &begin,
    .addDevice = &addDevice,
    .close = &close,
    .writeBit = &writeBit,
    .writeBits = &writeBits,
    .writeByte = &writeByte,
    .writeBytes = &writeBytes,
    .readBit = &readBit,
    .readBits = &readBits,
    .readByte = &readByte,
    .readBytes = &readBytes,
};

struct i2c i2c1 = {
    .deviceNum = 0,
    .port = I2C_NUM_1,
    .begin = &begin,
    .addDevice = &addDevice,
    .close = &close,
    .writeBit = &writeBit,
    .writeBits = &writeBits,
    .writeByte = &writeByte,
    .writeBytes = &writeBytes,
    .readBit = &readBit,
    .readBits = &readBits,
    .readByte = &readByte,
    .readBytes = &readBytes,
};

static uint32_t findDevice(struct i2c *i2c, uint8_t devAddr)
{
    uint32_t i = 0;
    uint8_t newDevice = 1;

    for (i = 0; i < i2c->deviceNum; ++i)
    {
        if (i2c->device[i].devAddress == devAddr)
        {
            newDevice = 0;
            break;
        }
    }

    if (newDevice)
        i2c->deviceNum++;

    GB_DEBUGI(GB_INFO, "I2C BUS findDevice: %d", i);

    return i;
}

static GB_RESULT begin(struct i2c *i2c, uint32_t sda_io_num, uint32_t scl_io_num, uint32_t clk_speed)
{
    GB_RESULT res = GB_OK;
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = i2c->port,
        .scl_io_num = scl_io_num,
        .sda_io_num = sda_io_num,
        .flags.enable_internal_pullup = true,
    };

    CHK_ESP_ERROR(i2c_new_master_bus(&i2c_mst_config, (i2c_master_bus_handle_t*)(&(i2c->busHandle))), GB_I2C_CFG_FAIL);

error_exit:
    return res;
}

static GB_RESULT addDevice(struct i2c *i2c, uint8_t devAddr, uint32_t clk_speed)
{
    GB_RESULT res = GB_OK;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = devAddr,
        .scl_speed_hz = clk_speed,
    };
    uint32_t deviceIdx = findDevice(i2c, devAddr);

    CHK_ESP_ERROR(i2c_master_bus_add_device((i2c_master_bus_handle_t)(i2c->busHandle),
                                             &dev_cfg,
                                             (i2c_master_dev_handle_t*)(&(i2c->device[deviceIdx].devHandle))), GB_I2C_INS_FAIL);
    i2c->device[deviceIdx].devAddress = devAddr;

error_exit:
    return res;
}

static GB_RESULT close(struct i2c *i2c) {
    GB_RESULT res = GB_OK;

    for (int i = 0; i < GB_MAX_I2C_DEV_NUMS; i++)
    {
        if (0 != i2c->device[i].devAddress)
        {
            CHK_ESP_ERROR(i2c_master_bus_rm_device((i2c_master_dev_handle_t)(i2c->device[i].devHandle)), GB_I2C_RMV_FAIL);
        }
    }
    CHK_ESP_ERROR(i2c_del_master_bus((i2c_master_bus_handle_t)(i2c->busHandle)), GB_I2C_RMV_FAIL);

error_exit:
    return res;
}

/*******************************************************************************
 * WRITING
 ******************************************************************************/
static GB_RESULT writeBit(struct i2c *i2c, uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t buffer;
    GB_RESULT res = GB_OK;

    CHK_RES(i2c->readByte(i2c, devAddr, regAddr, &buffer));
    buffer = data ? (buffer | (1 << bitNum)) : (buffer & ~(1 << bitNum));
    CHK_RES(i2c->writeByte(i2c, devAddr, regAddr, buffer));
error_exit:
    return res;
}

static GB_RESULT writeBits(struct i2c *i2c, uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
    uint8_t buffer;
    GB_RESULT res = GB_OK;

    CHK_RES(i2c->readByte(i2c, devAddr, regAddr, &buffer));
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1);
    data &= mask;
    buffer &= ~mask;
    buffer |= data;
    CHK_RES(i2c->writeByte(i2c, devAddr, regAddr, buffer));
error_exit:
    return res;
}

static GB_RESULT writeByte(struct i2c *i2c, uint8_t devAddr, uint8_t regAddr, uint8_t data) {
    GB_RESULT res = GB_OK;

    CHK_RES(i2c->writeBytes(i2c, devAddr, regAddr, 1, &data));
error_exit:
    return res;
}

static GB_RESULT writeBytes(struct i2c *i2c, uint8_t devAddr, uint8_t regAddr, size_t length, const uint8_t *data) {
    GB_RESULT res = GB_OK;

    i2c_master_transmit_multi_buffer_info_t i2c_buffer[3] = {
        {.write_buffer = &regAddr, .buffer_size = 1},
        {.write_buffer = data, .buffer_size = length},
    };

    CHK_ESP_ERROR(i2c_master_multi_buffer_transmit((i2c_master_dev_handle_t)(i2c->device[findDevice(i2c, devAddr)].devHandle),
                                                    i2c_buffer,
                                                    sizeof(i2c_buffer) / sizeof(i2c_master_transmit_multi_buffer_info_t),
                                                    -1), GB_I2C_RW_FAIL);

error_exit:
    return res;
}


/*******************************************************************************
 * READING
 ******************************************************************************/
static GB_RESULT readBit(struct i2c *i2c, uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data) {
    GB_RESULT res = GB_OK;

    CHK_RES(i2c->readBits(i2c, devAddr, regAddr, bitNum, 1, data));
error_exit:
    return res;
}

static GB_RESULT readBits(struct i2c *i2c, uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data) {
    uint8_t buffer;
    GB_RESULT res = GB_OK;

    CHK_RES(i2c->readByte(i2c, devAddr, regAddr, &buffer));
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    buffer &= mask;
    buffer >>= (bitStart - length + 1);
    *data = buffer;
error_exit:
    return res;
}

static GB_RESULT readByte(struct i2c *i2c, uint8_t devAddr, uint8_t regAddr, uint8_t *data) {
    GB_RESULT res = GB_OK;

    CHK_RES(i2c->readBytes(i2c, devAddr, regAddr, 1, data));
error_exit:
    return res;
}

static GB_RESULT readBytes(struct i2c *i2c, uint8_t devAddr, uint8_t regAddr, size_t length, uint8_t *data) {
    GB_RESULT res = GB_OK;

    GB_DEBUGI(GB_INFO, "I2C BUS readBytes start");
    CHK_ESP_ERROR(i2c_master_transmit_receive((i2c_master_dev_handle_t)(i2c->device[findDevice(i2c, devAddr)].devHandle),
                                              &regAddr, 1,
                                              data, length,
                                              -1), GB_I2C_RW_FAIL);
    GB_DEBUGI(GB_INFO, "I2C BUS readBytes end");
error_exit:
    return res;
}
