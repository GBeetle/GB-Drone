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

#include <stdio.h>
#include "log_sys.h"
#include "imu_driver.h"
#include "task_manager.h"
#include "lora_state.h"
#include "file_system.h"
#include "gb_timer.h"
#include "buzzer.h"
#include "ms5611.h"
#include "spi_bus.h"
#include "io_define.h"
#include "gpio_setting.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gps_driver.h"
#include "uart_bus.h"

void app_main(void)
{
    GB_LogSystemInit();
    GB_MutexInitialize();

    GB_SleepMs(5000);     // Waiting USB Log Ready

    GB_GPS_Init();
    GB_GPS_INFO_T gpsInfo = {
        .latitude = 0.0f,
        .longitude = 0.0f,
        .altitude = 0.0f,
    };

    while (1)
    {
        memset(&gpsInfo, 0x00, sizeof(GB_GPS_INFO_T));
        GB_GPS_ReadData(&gpsInfo);
        GB_DEBUGI(GB_INFO, "latitude: %f, longitude: %f, altitude: %f", gpsInfo.latitude, gpsInfo.longitude, gpsInfo.altitude);

        GB_SleepMs(1000);
    }
    return;
}
