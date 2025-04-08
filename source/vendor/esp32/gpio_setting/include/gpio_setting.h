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
#ifndef _GB_GPIO_SETTING__
#define _GB_GPIO_SETTING__

#include "results.h"

typedef enum {
    GB_GPIO_DISABLE = 0,
    GB_GPIO_INPUT,
    GB_GPIO_OUTPUT,
} GB_GPIO_MODE;

typedef enum {
    GB_GPIO_PULL_DISABLE = 0,
    GB_GPIO_PULLUP,
    GB_GPIO_PULLDOWN,
} GB_GPIO_STATUS;

GB_RESULT GB_GPIO_Init(uint32_t pin, GB_GPIO_MODE mode, GB_GPIO_STATUS staus);
GB_RESULT GB_GPIO_Reset(uint32_t pin);
GB_RESULT GB_GPIO_Set(uint32_t pin, uint32_t level);
GB_RESULT GB_GPIO_SetDirection(uint32_t pin, GB_GPIO_MODE mode);
GB_RESULT GB_GPIO_Get(uint32_t pin, uint32_t *level);

#endif /* end of include guard: _GB_GPIO_SETTING__ */
