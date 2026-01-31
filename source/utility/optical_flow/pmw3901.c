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
#include "pmw3901.h"
#include "log_sys.h"
#include "gb_timer.h"
#include "gpio_setting.h"
#include "results.h"
#include "error_handle.h"

#define PMW3901_REG_PRODUCT_ID      0x00
#define PMW3901_REG_INV_PRODUCT_ID  0x5F
#define PMW3901_REG_MOTION_BURST    0x16
#define PMW3901_REG_POWER_RESET     0x3A

static GB_RESULT pmw3901_write_register(GB_PMW3901_DEV_T *dev, uint8_t reg, uint8_t value)
{
    GB_RESULT res = GB_OK;
    uint8_t tx_data[2] = {0};

    tx_data[0] = reg | 0x80;
    tx_data[1] = value;

    GB_GPIO_Set(dev->cs_pin, 0);
    GB_SleepUs(50);
    CHK_RES(dev->bus->writeBytes(dev->bus, dev->dev_id, 0, 1, &tx_data[0]));
    GB_SleepUs(50);
    CHK_RES(dev->bus->writeBytes(dev->bus, dev->dev_id, 0, 1, &tx_data[1]));
    GB_SleepUs(50);
    GB_GPIO_Set(dev->cs_pin, 1);

error_exit:
    return res;
}

static GB_RESULT pmw3901_read_register(GB_PMW3901_DEV_T *dev, uint8_t reg, uint8_t *value)
{
    GB_RESULT res = GB_OK;
    uint8_t tx_data[2] = {0};
    uint8_t rx_data[2] = {0};

    tx_data[0] = reg & ~0x80;

    GB_GPIO_Set(dev->cs_pin, 0);
    GB_SleepUs(50);
    CHK_RES(dev->bus->writeBytes(dev->bus, dev->dev_id, 0, 1, &tx_data[0]));
    GB_SleepUs(500);
    CHK_RES(dev->bus->readWriteBytes(dev->bus, dev->dev_id, 0, 1, &rx_data[1], &tx_data[1]));
    GB_SleepUs(50);
    GB_GPIO_Set(dev->cs_pin, 1);

    *value = rx_data[1];
    GB_SleepUs(200);

error_exit:
    return res;
}

static GB_RESULT pmw3901_init_registers(GB_PMW3901_DEV_T *dev)
{
    GB_RESULT res = GB_OK;

    CHK_RES(pmw3901_write_register(dev, 0x7F, 0x00));
    CHK_RES(pmw3901_write_register(dev, 0x61, 0xAD));
    CHK_RES(pmw3901_write_register(dev, 0x7F, 0x03));
    CHK_RES(pmw3901_write_register(dev, 0x40, 0x00));
    CHK_RES(pmw3901_write_register(dev, 0x7F, 0x05));
    CHK_RES(pmw3901_write_register(dev, 0x41, 0xB3));
    CHK_RES(pmw3901_write_register(dev, 0x43, 0xF1));
    CHK_RES(pmw3901_write_register(dev, 0x45, 0x14));
    CHK_RES(pmw3901_write_register(dev, 0x5B, 0x32));
    CHK_RES(pmw3901_write_register(dev, 0x5F, 0x34));
    CHK_RES(pmw3901_write_register(dev, 0x7B, 0x08));
    CHK_RES(pmw3901_write_register(dev, 0x7F, 0x06));
    CHK_RES(pmw3901_write_register(dev, 0x44, 0x1B));
    CHK_RES(pmw3901_write_register(dev, 0x40, 0xBF));
    CHK_RES(pmw3901_write_register(dev, 0x4E, 0x3F));
    CHK_RES(pmw3901_write_register(dev, 0x7F, 0x08));
    CHK_RES(pmw3901_write_register(dev, 0x65, 0x20));
    CHK_RES(pmw3901_write_register(dev, 0x6A, 0x18));
    CHK_RES(pmw3901_write_register(dev, 0x7F, 0x09));
    CHK_RES(pmw3901_write_register(dev, 0x4F, 0xAF));
    CHK_RES(pmw3901_write_register(dev, 0x5F, 0x40));
    CHK_RES(pmw3901_write_register(dev, 0x48, 0x80));
    CHK_RES(pmw3901_write_register(dev, 0x49, 0x80));
    CHK_RES(pmw3901_write_register(dev, 0x57, 0x77));
    CHK_RES(pmw3901_write_register(dev, 0x60, 0x78));
    CHK_RES(pmw3901_write_register(dev, 0x61, 0x78));
    CHK_RES(pmw3901_write_register(dev, 0x62, 0x08));
    CHK_RES(pmw3901_write_register(dev, 0x63, 0x50));
    CHK_RES(pmw3901_write_register(dev, 0x7F, 0x0A));
    CHK_RES(pmw3901_write_register(dev, 0x45, 0x60));
    CHK_RES(pmw3901_write_register(dev, 0x7F, 0x00));
    CHK_RES(pmw3901_write_register(dev, 0x4D, 0x11));
    CHK_RES(pmw3901_write_register(dev, 0x55, 0x80));
    CHK_RES(pmw3901_write_register(dev, 0x74, 0x1F));
    CHK_RES(pmw3901_write_register(dev, 0x75, 0x1F));
    CHK_RES(pmw3901_write_register(dev, 0x4A, 0x78));
    CHK_RES(pmw3901_write_register(dev, 0x4B, 0x78));
    CHK_RES(pmw3901_write_register(dev, 0x44, 0x08));
    CHK_RES(pmw3901_write_register(dev, 0x45, 0x50));
    CHK_RES(pmw3901_write_register(dev, 0x64, 0xFF));
    CHK_RES(pmw3901_write_register(dev, 0x65, 0x1F));
    CHK_RES(pmw3901_write_register(dev, 0x7F, 0x14));
    CHK_RES(pmw3901_write_register(dev, 0x65, 0x67));
    CHK_RES(pmw3901_write_register(dev, 0x66, 0x08));
    CHK_RES(pmw3901_write_register(dev, 0x63, 0x70));
    CHK_RES(pmw3901_write_register(dev, 0x7F, 0x15));
    CHK_RES(pmw3901_write_register(dev, 0x48, 0x48));
    CHK_RES(pmw3901_write_register(dev, 0x7F, 0x07));
    CHK_RES(pmw3901_write_register(dev, 0x41, 0x0D));
    CHK_RES(pmw3901_write_register(dev, 0x43, 0x14));
    CHK_RES(pmw3901_write_register(dev, 0x4B, 0x0E));
    CHK_RES(pmw3901_write_register(dev, 0x45, 0x0F));
    CHK_RES(pmw3901_write_register(dev, 0x44, 0x42));
    CHK_RES(pmw3901_write_register(dev, 0x4C, 0x80));
    CHK_RES(pmw3901_write_register(dev, 0x7F, 0x10));
    CHK_RES(pmw3901_write_register(dev, 0x5B, 0x02));
    CHK_RES(pmw3901_write_register(dev, 0x7F, 0x07));
    CHK_RES(pmw3901_write_register(dev, 0x40, 0x41));
    CHK_RES(pmw3901_write_register(dev, 0x70, 0x00));

    GB_SleepMs(10);

    CHK_RES(pmw3901_write_register(dev, 0x32, 0x44));
    CHK_RES(pmw3901_write_register(dev, 0x7F, 0x07));
    CHK_RES(pmw3901_write_register(dev, 0x40, 0x40));
    CHK_RES(pmw3901_write_register(dev, 0x7F, 0x06));
    CHK_RES(pmw3901_write_register(dev, 0x62, 0xF0));
    CHK_RES(pmw3901_write_register(dev, 0x63, 0x00));
    CHK_RES(pmw3901_write_register(dev, 0x7F, 0x0D));
    CHK_RES(pmw3901_write_register(dev, 0x48, 0xC0));
    CHK_RES(pmw3901_write_register(dev, 0x6F, 0xD5));
    CHK_RES(pmw3901_write_register(dev, 0x7F, 0x00));
    CHK_RES(pmw3901_write_register(dev, 0x5B, 0xA0));
    CHK_RES(pmw3901_write_register(dev, 0x4E, 0xA8));
    CHK_RES(pmw3901_write_register(dev, 0x5A, 0x50));
    CHK_RES(pmw3901_write_register(dev, 0x40, 0x80));

    CHK_RES(pmw3901_write_register(dev, 0x7F, 0x00));
    CHK_RES(pmw3901_write_register(dev, 0x5A, 0x10));
    CHK_RES(pmw3901_write_register(dev, 0x54, 0x00));

error_exit:
    return res;
}

GB_RESULT GB_PMW3901_InitDesc(GB_PMW3901_DEV_T *dev, struct spi *bus, GB_SPI_DEV_T dev_id, uint8_t cs_pin)
{
    GB_RESULT res = GB_OK;

    CHK_NULL(dev, GB_OFLOW_DEVINE_NULL);
    CHK_NULL(bus, GB_OFLOW_BUS_NULL);

    memset(dev, 0, sizeof(GB_PMW3901_DEV_T));
    dev->bus = bus;
    dev->dev_id = dev_id;
    dev->cs_pin = cs_pin;
    dev->initialized = false;

error_exit:
    return res;
}

GB_RESULT GB_PMW3901_Init(GB_PMW3901_DEV_T *dev)
{
    GB_RESULT res = GB_OK;
    uint8_t chip_id = 0;
    uint8_t inv_chip_id = 0;
    uint8_t dummy = 0;

    CHK_NULL(dev, GB_OFLOW_DEVINE_NULL);
    CHK_NULL(dev->bus, GB_OFLOW_BUS_NULL);

    GB_DEBUGI(OPF_TAG, "Initializing PMW3901 optical flow sensor");

    GB_GPIO_SetDirection(dev->cs_pin, GB_GPIO_OUTPUT);
    GB_GPIO_Set(dev->cs_pin, 1);

    GB_SleepMs(40);

    GB_GPIO_Set(dev->cs_pin, 1);
    GB_SleepMs(2);
    GB_GPIO_Set(dev->cs_pin, 0);
    GB_SleepMs(2);
    GB_GPIO_Set(dev->cs_pin, 1);
    GB_SleepMs(2);

    CHK_RES(pmw3901_read_register(dev, PMW3901_REG_PRODUCT_ID, &chip_id));
    CHK_RES(pmw3901_read_register(dev, PMW3901_REG_INV_PRODUCT_ID, &inv_chip_id));

    GB_DEBUGI(OPF_TAG, "Motion chip ID: 0x%02X : 0x%02X", chip_id, inv_chip_id);

    if (chip_id != PMW3901_CHIP_ID || inv_chip_id != PMW3901_INV_CHIP_ID) {
        GB_DEBUGE(OPF_TAG, "Invalid chip ID, chip ID: 0x%02X : 0x%02X", chip_id, inv_chip_id);
        CHK_RES(GB_OFLOW_CHIP_ID_FAIL);
    }

    CHK_RES(pmw3901_write_register(dev, PMW3901_REG_POWER_RESET, 0x5A));
    GB_SleepMs(5);

    pmw3901_read_register(dev, 0x02, &dummy);
    pmw3901_read_register(dev, 0x03, &dummy);
    pmw3901_read_register(dev, 0x04, &dummy);
    pmw3901_read_register(dev, 0x05, &dummy);
    pmw3901_read_register(dev, 0x06, &dummy);
    GB_SleepMs(1);

    CHK_RES(pmw3901_init_registers(dev));

    dev->initialized = true;
    GB_DEBUGI(OPF_TAG, "PMW3901 initialized successfully");

error_exit:
    return res;
}

GB_RESULT GB_PMW3901_ReadMotion(GB_PMW3901_DEV_T *dev, GB_PMW3901_MOTION_T *motion)
{
    GB_RESULT res = GB_OK;
    uint8_t address = PMW3901_REG_MOTION_BURST;
    uint8_t tx_data[sizeof(GB_PMW3901_MOTION_T) + 1];
    uint8_t rx_data[sizeof(GB_PMW3901_MOTION_T) + 1];

    CHK_NULL(dev, GB_OFLOW_DEVINE_NULL);
    CHK_NULL(motion, GB_OFLOW_BUS_NULL);
    CHK_BOOL(dev->initialized);

    memset(tx_data, 0, sizeof(tx_data));
    tx_data[0] = address & ~0x80;

    GB_GPIO_Set(dev->cs_pin, 0);
    CHK_RES(dev->bus->readWriteBytes(dev->bus, dev->dev_id, 0, sizeof(GB_PMW3901_MOTION_T) + 1, rx_data, tx_data));
    GB_GPIO_Set(dev->cs_pin, 1);

    memcpy(motion, &rx_data[1], sizeof(GB_PMW3901_MOTION_T));

    uint16_t realShutter = (motion->shutter >> 8) & 0x00FF;
    realShutter |= (motion->shutter & 0x00FF) << 8;
    motion->shutter = realShutter;

error_exit:
    return res;
}

GB_RESULT GB_PMW3901_GetMotion(GB_PMW3901_DEV_T *dev, int16_t *deltaX, int16_t *deltaY)
{
    GB_RESULT res = GB_OK;
    GB_PMW3901_MOTION_T motion;

    CHK_NULL(dev, GB_OFLOW_DEVINE_NULL);
    CHK_NULL(deltaX, GB_OFLOW_BUS_NULL);
    CHK_NULL(deltaY, GB_OFLOW_BUS_NULL);

    CHK_RES(GB_PMW3901_ReadMotion(dev, &motion));

    *deltaX = motion.deltaX;
    *deltaY = motion.deltaY;

error_exit:
    return res;
}

GB_RESULT GB_PMW3901_CheckMotion(GB_PMW3901_DEV_T *dev, bool *motion_occurred)
{
    GB_RESULT res = GB_OK;
    GB_PMW3901_MOTION_T motion;

    CHK_NULL(dev, GB_OFLOW_DEVINE_NULL);
    CHK_NULL(motion_occurred, GB_OFLOW_BUS_NULL);

    CHK_RES(GB_PMW3901_ReadMotion(dev, &motion));

    *motion_occurred = motion.motionOccured;

error_exit:
    return res;
}
