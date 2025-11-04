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

#ifndef __BARO_DRIVER_H__
#define __BARO_DRIVER_H__

#include <inttypes.h>
#include <stdbool.h>

#if defined CONFIG_BMP280
#include "bmp280.h"
typedef bmp280_t baro_dev_t;
#elif defined CONFIG_MS5611
#include "ms5611.h"
typedef ms5611_t baro_dev_t;
#endif

int32_t applyBarometerMedianFilter(int32_t newPressureReading);
void performBaroCalibrationCycle(float baroPressureSamp);

bool isBaroCalibrationFinished();
float getBaroGroundAltitude();
float pressureToAltitude(const float pressure);

#endif  //__BARO_DRIVER_H__
