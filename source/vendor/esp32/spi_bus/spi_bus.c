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
#include "sdkconfig.h"
#include "log_sys.h"
#include "esp_intr_alloc.h"
#include "spi_bus.h"

// Defaults
#define SPIBUS_READ     (0x80)  /*!< addr | SPIBUS_READ  */
#define SPIBUS_WRITE    (0x7F)  /*!< addr & SPIBUS_WRITE */

static GB_RESULT begin(struct spi *spi, int mosi_io_num, int miso_io_num, int sclk_io_num, int max_transfer_sz);
static GB_RESULT close(struct spi *spi);
static GB_RESULT addDevice(struct spi *spi, uint8_t address_len, uint8_t mode, uint8_t flag, uint32_t clock_speed_hz, int cs_io_num);
static GB_RESULT addDeviceCfg(struct spi *spi, uint8_t address_len, spi_device_interface_config_t *dev_config);
static GB_RESULT removeDevice(struct spi *spi);
/*******************************************************************************
 * WRITING
 ******************************************************************************/
static GB_RESULT writeBit(struct spi *spi, uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
static GB_RESULT writeBits(struct spi *spi, uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
static GB_RESULT writeByte(struct spi *spi, uint8_t devAddr, uint8_t regAddr, uint8_t data);
static GB_RESULT writeBytes(struct spi *spi, uint8_t devAddr, uint8_t regAddr, size_t length, const uint8_t *data);
/*******************************************************************************
 * READING
 ******************************************************************************/
static GB_RESULT readBit(struct spi *spi, uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data);
static GB_RESULT readBits(struct spi *spi, uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
static GB_RESULT readByte(struct spi *spi, uint8_t devAddr, uint8_t regAddr, uint8_t *data);
static GB_RESULT readBytes(struct spi *spi, uint8_t devAddr, uint8_t regAddr, size_t length, uint8_t *data);
static GB_RESULT readWriteBytes(struct spi *spi, uint8_t devAddr, uint8_t regAddr, size_t length, uint8_t *r_data, uint8_t *w_data);

/*******************************************************************************
 * OBJECTS
 ******************************************************************************/
struct spi fspi = {
    .host           = SPI2_HOST,
    .begin          = &begin,
    .close          = &close,
    .addDevice      = &addDevice,
    .addDeviceCfg   = &addDeviceCfg,
    .removeDevice   = &removeDevice,
    .writeBit       = &writeBit,
    .writeBits      = &writeBits,
    .writeByte      = &writeByte,
    .writeBytes     = &writeBytes,
    .readBit        = &readBit,
    .readBits       = &readBits,
    .readByte       = &readByte,
    .readBytes      = &readBytes,
    .readWriteBytes = &readWriteBytes,
};
struct spi hspi = {
    .host           = SPI3_HOST,
    .begin          = &begin,
    .close          = &close,
    .addDevice      = &addDevice,
    .addDeviceCfg   = &addDeviceCfg,
    .removeDevice   = &removeDevice,
    .writeBit       = &writeBit,
    .writeBits      = &writeBits,
    .writeByte      = &writeByte,
    .writeBytes     = &writeBytes,
    .readBit        = &readBit,
    .readBits       = &readBits,
    .readByte       = &readByte,
    .readBytes      = &readBytes,
    .readWriteBytes = &readWriteBytes,
};

/*******************************************************************************
 * SETUP
 ******************************************************************************/
static GB_RESULT begin(struct spi *spi, int mosi_io_num, int miso_io_num, int sclk_io_num, int max_transfer_sz) {
    spi_bus_config_t config = {0};
    config.mosi_io_num = mosi_io_num;
    config.miso_io_num = miso_io_num;
    config.sclk_io_num = sclk_io_num;
    config.quadwp_io_num = -1;  // -1 not used
    config.quadhd_io_num = -1;  // -1 not used
    config.max_transfer_sz = max_transfer_sz;
    config.flags = SPICOMMON_BUSFLAG_MASTER;
    esp_err_t err = spi_bus_initialize(spi->host, &config, SPI_DMA_DISABLED);
    if(err != ESP_OK) {
        GB_DEBUGE(ERROR_TAG, "spi init failed, error: %s\n", esp_err_to_name(err));
        return GB_SPI_INI_FAIL;
    }  // 0 DMA not used
    return GB_OK;
}

static GB_RESULT close(struct spi *spi) {
    if(spi_bus_free(spi->host) != ESP_OK) {
        return GB_SPI_FREE_FAIL;
    }
    return GB_OK;
}

static GB_RESULT addDevice(struct spi *spi, uint8_t address_len, uint8_t mode, uint8_t flag, uint32_t clock_speed_hz, int cs_io_num) {
    spi_device_interface_config_t dev_config;
    dev_config.command_bits     = 0;
    dev_config.address_bits     = address_len;
    dev_config.dummy_bits       = 0;
    dev_config.mode             = mode;
    dev_config.duty_cycle_pos   = 128;             // default 128 = 50%/50% duty
    dev_config.cs_ena_pretrans  = 0;               // 0 not used
    dev_config.cs_ena_posttrans = 0;               // 0 not used
    dev_config.clock_speed_hz   = clock_speed_hz;
    dev_config.spics_io_num     = cs_io_num;
    dev_config.flags            = flag;            // 0 not used
    dev_config.queue_size       = 1;
    dev_config.pre_cb           = NULL;
    dev_config.post_cb          = NULL;
    if (spi_bus_add_device(spi->host, &dev_config, &(spi->deviceHandle)) != ESP_OK) {
        return GB_SPI_CFG_FAIL;
    }
    return GB_OK;
}

static GB_RESULT addDeviceCfg(struct spi *spi, uint8_t address_len, spi_device_interface_config_t *dev_config) {
    dev_config->address_bits = address_len;  // must be set, SPIbus uses this 8-bits to send the regAddr
    if (spi_bus_add_device(spi->host, dev_config, &(spi->deviceHandle)) != ESP_OK) {
        return GB_SPI_CFG_FAIL;
    }
    return GB_OK;
}

static GB_RESULT removeDevice(struct spi *spi) {
    if (spi_bus_remove_device(spi->deviceHandle) != ESP_OK) {
        return GB_SPI_RMV_FAIL;
    }
    return GB_OK;
}


/*******************************************************************************
 * WRITING
 ******************************************************************************/
static GB_RESULT writeBit(struct spi *spi, uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t buffer;
    GB_RESULT res = GB_OK;

    CHK_RES(spi->readByte(spi, 0X00, regAddr, &buffer));
    buffer = data ? (buffer | (1 << bitNum)) : (buffer & ~(1 << bitNum));
    CHK_RES(spi->writeByte(spi, 0X00, regAddr, buffer));
error_exit:
    return res;
}

static GB_RESULT writeBits(struct spi *spi, uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
    uint8_t buffer;
    GB_RESULT res = GB_OK;

    CHK_RES(spi->readByte(spi, 0X00, regAddr, &buffer));
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1);
    data &= mask;
    buffer &= ~mask;
    buffer |= data;
    CHK_RES(spi->writeByte(spi, 0X00, regAddr, buffer));
error_exit:
    return res;
}

static GB_RESULT writeByte(struct spi *spi, uint8_t devAddr, uint8_t regAddr, uint8_t data) {
    GB_RESULT res = GB_OK;
    CHK_RES(spi->writeBytes(spi, 0X00, regAddr, 1, &data));
error_exit:
    return res;
}

static GB_RESULT writeBytes(struct spi *spi, uint8_t devAddr, uint8_t regAddr, size_t length, const uint8_t *data) {
    spi_transaction_t transaction;
    transaction.flags = 0;
    transaction.cmd = 0;
    transaction.addr = regAddr & SPIBUS_WRITE;
    transaction.length = length * 8;
    transaction.rxlength = 0;
    transaction.user = NULL;
    transaction.tx_buffer = data;
    transaction.rx_buffer = NULL;
    esp_err_t err = spi_device_transmit(spi->deviceHandle, &transaction);
    if (err != ESP_OK) {
        char str[length*5+1];
        for(size_t i = 0; i < length; i++)
            sprintf(str+i*5, "0x%s%X ", (data[i] < 0x10 ? "0" : ""), data[i]);
        GB_DEBUGE(ERROR_TAG, "[%s, handle:0x%X] Write %d bytes to__ register 0x%X, data: %s\n", (spi->host == 1 ? "FSPI" : "HSPI"), (uint32_t)spi->deviceHandle, length, regAddr, str);
        return GB_SPI_RW_FAIL;
    }
    return GB_OK;
}


/*******************************************************************************
 * READING
 ******************************************************************************/
static GB_RESULT readBit(struct spi *spi, uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data) {
    GB_RESULT res = GB_OK;
    CHK_RES(spi->readBits(spi, 0X00, regAddr, bitNum, 1, data));
error_exit:
    return res;
}

static GB_RESULT readBits(struct spi *spi, uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data) {
    uint8_t buffer;
    GB_RESULT res = GB_OK;

    CHK_RES(spi->readByte(spi, 0X00, regAddr, &buffer));
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    buffer &= mask;
    buffer >>= (bitStart - length + 1);
    *data = buffer;
error_exit:
    return res;
}

static GB_RESULT readByte(struct spi *spi, uint8_t devAddr, uint8_t regAddr, uint8_t *data) {
    GB_RESULT res = GB_OK;

    CHK_RES(spi->readBytes(spi, 0X00, regAddr, 1, data));
error_exit:
    return res;
}

static GB_RESULT readBytes(struct spi *spi, uint8_t devAddr, uint8_t regAddr, size_t length, uint8_t *data) {
    if(length == 0) return GB_SPI_INVALID_SIZE;
    spi_transaction_t transaction;
    transaction.flags = 0;
    transaction.cmd = 0;
    transaction.addr = regAddr | SPIBUS_READ;
    transaction.length = length * 8;
    transaction.rxlength = length * 8;
    transaction.user = NULL;
    transaction.tx_buffer = NULL;
    transaction.rx_buffer = data;
    esp_err_t err = spi_device_transmit(spi->deviceHandle, &transaction);
    if (err != ESP_OK) {
        char str[length*5+1];
        for(size_t i = 0; i < length; i++)
            sprintf(str+i*5, "0x%s%X ", (data[i] < 0x10 ? "0" : ""), data[i]);
        GB_DEBUGE(ERROR_TAG, "[%s, handle:0x%X] Read_ %d bytes from register 0x%X, data: %s\n", (spi->host == 1 ? "FHPI" : "HSPI"), (uint32_t)spi->deviceHandle, length, regAddr, str);
        return GB_SPI_RW_FAIL;
    }
    return GB_OK;
}

static GB_RESULT readWriteBytes(struct spi *spi, uint8_t devAddr, uint8_t regAddr, size_t length, uint8_t *r_data, uint8_t *w_data) {
    if(length == 0) return GB_SPI_INVALID_SIZE;
    spi_transaction_t transaction;
    transaction.flags = 0;
    transaction.cmd = 0;
    transaction.addr = regAddr;
    transaction.length = length * 8;
    transaction.rxlength = length * 8;
    transaction.user = NULL;
    transaction.tx_buffer = w_data;
    transaction.rx_buffer = r_data;
    esp_err_t err = spi_device_transmit(spi->deviceHandle, &transaction);
    if (err != ESP_OK) {
        char rstr[length*5+1];
        char wstr[length*5+1];
        for(size_t i = 0; i < length; i++) {
            sprintf(rstr+i*5, "0x%s%X ", (r_data[i] < 0x10 ? "0" : ""), r_data[i]);
            sprintf(wstr+i*5, "0x%s%X ", (w_data[i] < 0x10 ? "0" : ""), w_data[i]);
        }
        GB_DEBUGE(ERROR_TAG, "[%s, handle:0x%X] Read_ %d bytes from register 0x%X, data: %s\n", (spi->host == 1 ? "FHPI" : "HSPI"), (uint32_t)spi->deviceHandle, length, regAddr, rstr);
        GB_DEBUGE(ERROR_TAG, "[%s, handle:0x%X] Write %d bytes to__ register 0x%X, data: %s\n", (spi->host == 1 ? "FSPI" : "HSPI"), (uint32_t)spi->deviceHandle, length, regAddr, wstr);
        return GB_SPI_RW_FAIL;
    }
    return GB_OK;
}

