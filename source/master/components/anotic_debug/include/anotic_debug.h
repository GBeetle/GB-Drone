/*
 * This file is part of welkin project (https://github.com/GBeetle/welkin).
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

#ifndef _ANOTIC_DEBUG__
#define _ANOTIC_DEBUG__

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdarg.h>
#include "log_sys.h"

uint8_t __bswap_8(uint8_t value);
uint16_t float2int16(float value);
int anotc_init_data(uint8_t *send_buffer, uint8_t f_id, uint32_t arg_nums, ...);

#endif /* end of include guard: _ANOTIC_DEBUG__ */
