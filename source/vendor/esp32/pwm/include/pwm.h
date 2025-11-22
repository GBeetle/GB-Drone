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
#ifndef _GB_PWM__
#define _GB_PWM__

#include "results.h"

typedef struct {
    uint8_t timer;
    uint8_t channel;
    uint8_t resolution;
    uint8_t io;
    uint32_t freq;
} GB_PWM_TYPE_T;

GB_RESULT GB_PWM_Init(GB_PWM_TYPE_T *dev);
GB_RESULT GB_PWM_Start(GB_PWM_TYPE_T dev);

#endif /* end of include guard: _GB_PWM__ */
