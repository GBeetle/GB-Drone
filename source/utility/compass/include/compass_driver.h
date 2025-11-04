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

#ifndef __COMPASS_DRIVER_H__
#define __COMPASS_DRIVER_H__

#include <inttypes.h>
#include <stdbool.h>

#if defined CONFIG_QMC5883L
#include "qmc5883l.h"
typedef qmc5883l_t compass_dev_t;
#elif defined CONFIG_LIS3MDL
#include "lis3mdl.h"
#endif

int32_t applyBarometerMedianFilter(int32_t newPressureReading);
void performBaroCalibrationCycle(float baroPressureSamp);

bool isBaroCalibrationFinished();
float getBaroGroundAltitude();
float pressureToAltitude(const float pressure);

#endif  //__COMPASS_DRIVER_H__
