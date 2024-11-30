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
#ifndef _GB_TIMER__
#define _GB_TIMER__

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "log_sys.h"

typedef TickType_t GB_TickType;

GB_RESULT GB_GetTimerMs(uint64_t *timer);
GB_RESULT GB_GetTimerUs(uint64_t *timer);
GB_RESULT GB_SleepMs(uint64_t time);
GB_RESULT GB_GetTicks(GB_TickType *ticks);
GB_RESULT GB_MsToTick(uint64_t timeMs, GB_TickType *ticks);

#endif /* end of include guard: _GB_TIMER__ */
