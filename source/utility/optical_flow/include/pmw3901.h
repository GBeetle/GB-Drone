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
} __attribute__((packed)) motionBurst_t;

/**
 * Initialize the PMW3901 sensor
 *
 * @param csPin Chip Select pin as defined in deck pinout driver.
 *
 * @return  true if init successful else false.
 */
bool pmw3901Init(uint32_t csPin);

/**
 * Read the current accumulated motion.
 *
 * @param csPin   Chip Select pin as defined in deck pinout driver.
 * @param motion  A filled in motionBurst_t structure with the latest motion information
 */
void pmw3901ReadMotion(uint32_t csPin, motionBurst_t *motion);

#endif  //__PMW3901_DRIVER_H__
