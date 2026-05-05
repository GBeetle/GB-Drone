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

#ifndef _TASK_MANAGER__
#define _TASK_MANAGER__

#include "lora_state.h"

void gui_task(void *pvParameter);
void controller_task(void *pvParameter);
void rf_loop(void *pvParameter);
void gui_pid_table_get(GB_PID_TABLE_T *out);
void gui_pid_table_set(const GB_PID_TABLE_T *in);

extern SemaphoreHandle_t xGuiSemaphore;

#endif /* end of include guard: _TASK_MANAGER__ */
