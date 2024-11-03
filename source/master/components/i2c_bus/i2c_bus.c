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
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_err.h"

#include "i2c_bus.h"
#include "sdkconfig.h"
#include "log_sys.h"

#define I2C_MASTER_TIMEOUT_MS 1000

#define I2C_MASTER_ACK_EN   true    /*!< Enable ack check for master */
#define I2C_MASTER_ACK_DIS  false   /*!< Disable ack check for master */

const uint32_t kDefaultClockSpeed = 100000;  /*!< Clock speed in Hz, default: 100KHz */
const uint32_t kDefaultTimeout = 1000;       /*!< Timeout in milliseconds, default: 1000ms */

static GB_RESULT begin(struct i2c *i2c, uint32_t sda_io_num, uint32_t scl_io_num, uint32_t clk_speed);
static GB_RESULT beginPullEnable(struct i2c *i2c, uint32_t sda_io_num, uint32_t scl_io_num, uint32_t sda_pullup_en,
                                 uint32_t scl_pullup_en, uint32_t clk_speed);
static GB_RESULT close(struct i2c *i2c);
static void setTimeout(struct i2c *i2c, uint32_t ms);
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
 * UTILS
 ******************************************************************************/
static GB_RESULT testConnection(struct i2c *i2c, uint8_t devAddr, int32_t timeout);
static void scanner(struct i2c *i2c);

/*******************************************************************************
 * OBJECTS
 ******************************************************************************/
struct i2c i2c0 = {
    .port = I2C_NUM_0,
    .ticksToWait = pdMS_TO_TICKS(kDefaultTimeout),
    .begin = &begin,
    .beginPullEnable = &beginPullEnable,
    .close = &close,
    .setTimeout = &setTimeout,
    .writeBit = &writeBit,
    .writeBits = &writeBits,
    .writeByte = &writeByte,
    .writeBytes = &writeBytes,
    .readBit = &readBit,
    .readBits = &readBits,
    .readByte = &readByte,
    .readBytes = &readBytes,
    .testConnection = &testConnection,
    .scanner = &scanner,
};

struct i2c i2c1 = {
    .port = I2C_NUM_1,
    .ticksToWait = pdMS_TO_TICKS(kDefaultTimeout),
    .begin = &begin,
    .beginPullEnable = &beginPullEnable,
    .close = &close,
    .setTimeout = &setTimeout,
    .writeBit = &writeBit,
    .writeBits = &writeBits,
    .writeByte = &writeByte,
    .writeBytes = &writeBytes,
    .readBit = &readBit,
    .readBits = &readBits,
    .readByte = &readByte,
    .readBytes = &readBytes,
    .testConnection = &testConnection,
    .scanner = &scanner,
};

/* ^^^^^^
 * I2Cbus
 * ^^^^^^ */
static GB_RESULT begin(struct i2c *i2c, uint32_t sda_io_num, uint32_t scl_io_num, uint32_t clk_speed) {
    GB_RESULT res = GB_OK;
    CHK_RES(i2c->beginPullEnable(i2c, sda_io_num, scl_io_num, GPIO_PULLUP_ENABLE, GPIO_PULLUP_ENABLE, clk_speed));
error_exit:
    return res;
}

static GB_RESULT beginPullEnable(struct i2c *i2c, uint32_t sda_io_num, uint32_t scl_io_num, uint32_t sda_pullup_en,
        uint32_t scl_pullup_en, uint32_t clk_speed) {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda_io_num;
    conf.sda_pullup_en = sda_pullup_en;
    conf.scl_io_num = scl_io_num;
    conf.scl_pullup_en = scl_pullup_en;
    conf.master.clk_speed = clk_speed;
    conf.clk_flags = 0;
    esp_err_t err = i2c_param_config(i2c->port, &conf);
    if (!err)
        err = i2c_driver_install(i2c->port, conf.mode, 0, 0, 0);
    else
        return GB_I2C_CFG_FAIL;
    if (err != ESP_OK)
        return GB_I2C_INS_FAIL;
    return GB_OK;
}

static GB_RESULT close(struct i2c *i2c) {
    if (i2c_driver_delete(i2c->port) != ESP_OK)
        return GB_I2C_RMV_FAIL;
    else
        return GB_OK;
}

static void setTimeout(struct i2c *i2c, uint32_t ms) {
    i2c->ticksToWait = pdMS_TO_TICKS(ms);
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
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    int32_t timeout = I2C_MASTER_TIMEOUT_MS;
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK_EN);
    i2c_master_write_byte(cmd, regAddr, I2C_MASTER_ACK_EN);
    i2c_master_write(cmd, (uint8_t*) data, length, I2C_MASTER_ACK_EN);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(i2c->port, cmd, (timeout < 0 ? i2c->ticksToWait : pdMS_TO_TICKS(timeout)));
    i2c_cmd_link_delete(cmd);
    if (err != ESP_OK) {
        GB_DEBUGE("[port:%d, slave:0x%X] Failed to write %d bytes to__ register 0x%X, error: 0x%X",
            i2c->port, devAddr, length, regAddr, err);
        return GB_I2C_RW_FAIL;
    }
    return GB_OK;
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
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    int32_t timeout = I2C_MASTER_TIMEOUT_MS;
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK_EN);
    i2c_master_write_byte(cmd, regAddr, I2C_MASTER_ACK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_READ, I2C_MASTER_ACK_EN);
    i2c_master_read(cmd, data, length, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(i2c->port, cmd, (timeout < 0 ? i2c->ticksToWait : pdMS_TO_TICKS(timeout)));
    i2c_cmd_link_delete(cmd);
    if (err != ESP_OK) {
        GB_DEBUGE(ERROR_TAG, "[i2c->port:%d, slave:0x%X] Failed to read %d bytes from register 0x%X, error: %s\n",
            i2c->port, devAddr, length, regAddr, esp_err_to_name(err));
        return GB_I2C_RW_FAIL;
    }
    return GB_OK;
}


/*******************************************************************************
 * UTILS
 ******************************************************************************/
static GB_RESULT testConnection(struct i2c *i2c, uint8_t devAddr, int32_t timeout) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK_EN);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(i2c->port, cmd, (timeout < 0 ? i2c->ticksToWait : pdMS_TO_TICKS(timeout)));
    i2c_cmd_link_delete(cmd);
    if (err != ESP_OK)
        return GB_I2C_CONNECT_FAIL;
    return GB_OK;
}

static void scanner(struct i2c *i2c) {
    const int32_t scanTimeout = 20;
    GB_DEBUGE(ERROR_TAG, "\n>> i2c scanning ... \n");
    uint8_t count = 0;
    for (size_t i = 0x3; i < 0x78; i++) {
        if (testConnection(i2c, i, scanTimeout) == ESP_OK) {
            GB_DEBUGE(ERROR_TAG, "- Device found at address 0x%X%s, i \n");
            count++;
        }
    }
    if (count == 0)
        GB_DEBUGE(ERROR_TAG, "- No i2c devices found! \n");
    GB_DEBUGE(ERROR_TAG, "\n");
}
