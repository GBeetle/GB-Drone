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

#ifndef _ESC_SYSTEM__
#define _ESC_SYSTEM__

#include "results.h"
#include "driver/mcpwm.h"

typedef struct{
    mcpwm_unit_t unit;
    mcpwm_io_signals_t ports;
    mcpwm_timer_t timer;
    int pin;
} ESC_MOTOR_T;

GB_RESULT esc_system_init(int gpio_s0, int gpio_s1, int gpio_s2, int gpio_s3);
GB_RESULT esc_set_duty(int idx, float duty);

#endif /* end of include guard: _ESC_SYSTEM__ */
