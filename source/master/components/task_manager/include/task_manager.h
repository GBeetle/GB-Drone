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

#include "isr_manager.h"
#include "imu_driver.h"
#include "anotic_debug.h"

#define MPU_DATA_QUEUE_SIZE 10

void gb_sensor_fusion(void* arg);
void gb_read_sensor_data(void* arg);
void uart_rx_task(void *arg);
void nrf24_interrupt_func(void *arg);
void GB_MutexInitialize();

extern struct imu imu;
extern SemaphoreHandle_t mpuSensorReady;

#endif /* end of include guard: _TASK_MANAGER__ */
