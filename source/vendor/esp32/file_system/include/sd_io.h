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

#ifndef _FILE_SD_IO__
#define _FILE_SD_IO__

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    const char **names;
    const int *pins;
} pin_configuration_t;

void check_sd_card_pins(pin_configuration_t *config, const int pin_count);

#ifdef __cplusplus
}
#endif

#endif