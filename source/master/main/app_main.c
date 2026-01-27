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
#include "i2c_bus.h"
#include "laser_ranging.h"

void app_main(void)
{
    GB_LogSystemInit();
    GB_MutexInitialize();

    GB_SleepMs(5000);     // Waiting USB Log Ready
    if (GB_OK == GB_SDCardFileSystem_Init())
    {
        GB_DEBUGI(GB_INFO, "Setting Log to SD Card");
        GB_Log2fileEnable();
    }

    GB_LASER_DEV_T laser;

    CHK_EXIT(i2c0.begin(&i2c0, COMPASS_I2C_SDA, COMPASS_I2C_SCL, COMPASS_I2C_CLOCK_SPEED));
    CHK_EXIT(i2c0.addDevice(&i2c0, VL53L1X_I2C_ADDRESS, COMPASS_I2C_CLOCK_SPEED));

    CHK_EXIT(GB_LASER_InitDesc(&laser, &i2c0, VL53L1X_I2C_ADDRESS));
    CHK_EXIT(GB_LASER_Init(&laser, GB_LASER_MODE_LONG, GB_LASER_TIMING_50MS));
    CHK_EXIT(GB_LASER_StartContinuous(&laser, 1000));   // 1s

    while (1) {
        uint16_t ground_distance_mm;

        if (GB_LASER_GetLastDistance(&laser, &ground_distance_mm) == GB_OK) {
            GB_DEBUGI(LASER_TAG, "Altitude :%.2f cm", ground_distance_mm / 100.0f);
        }

        GB_SleepMs(1000);
    }

    return;
}