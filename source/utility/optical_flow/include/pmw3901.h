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

#ifndef __PMW3901_DRIVER_H__
#define __PMW3901_DRIVER_H__

#include <stdint.h>
#include <stdbool.h>
#include "error_handle.h"
#include "spi_bus.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define PMW3901_CHIP_ID      0x49
#define PMW3901_INV_CHIP_ID  0xB6

typedef struct motionBurst_s {
    union {
        uint8_t motion;
        struct {
            uint8_t frameFrom0    : 1;
            uint8_t runMode       : 2;
            uint8_t reserved1     : 1;
            uint8_t rawFrom0      : 1;
            uint8_t reserved2     : 2;
            uint8_t motionOccured : 1;
        };
    };

    uint8_t observation;
    int16_t deltaX;
    int16_t deltaY;

    uint8_t squal;

    uint8_t rawDataSum;
    uint8_t maxRawData;
    uint8_t minRawData;

    uint16_t shutter;
} __attribute__((packed)) GB_PMW3901_MOTION_T;

typedef struct
{
    struct spi *bus;
    GB_SPI_DEV_T dev_id;
    uint8_t cs_pin;
    bool initialized;
} GB_PMW3901_DEV_T;

GB_RESULT GB_PMW3901_InitDesc(GB_PMW3901_DEV_T *dev, struct spi *bus, GB_SPI_DEV_T dev_id, uint8_t cs_pin);

GB_RESULT GB_PMW3901_Init(GB_PMW3901_DEV_T *dev);

GB_RESULT GB_PMW3901_ReadMotion(GB_PMW3901_DEV_T *dev, GB_PMW3901_MOTION_T *motion);

GB_RESULT GB_PMW3901_GetMotion(GB_PMW3901_DEV_T *dev, int16_t *deltaX, int16_t *deltaY);

GB_RESULT GB_PMW3901_CheckMotion(GB_PMW3901_DEV_T *dev, bool *motion_occurred);

#ifdef __cplusplus
}
#endif

#endif  //__PMW3901_DRIVER_H__
