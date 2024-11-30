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

#include <esp_timer.h>
#include "gb_timer.h"

GB_RESULT GB_GetTimerMs(uint64_t *timer)
{
    *timer = esp_timer_get_time() / 1000;
    return GB_OK;
}

GB_RESULT GB_GetTimerUs(uint64_t *timer)
{
    *timer = esp_timer_get_time();
    return GB_OK;
}

GB_RESULT GB_SleepMs(uint64_t time)
{
    vTaskDelay(pdMS_TO_TICKS(time));
    return GB_OK;
}

GB_RESULT GB_GetTicks(GB_TickType *ticks)
{
    *ticks = xTaskGetTickCount();
    return GB_OK;
}

GB_RESULT GB_MsToTick(uint64_t timeMs, GB_TickType *ticks)
{
    *ticks = pdMS_TO_TICKS(timeMs);
    return GB_OK;
}
