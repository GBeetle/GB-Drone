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
#include "mpu_driver.h"
#include "task_manager.h"
#include "gb_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void)
{
    GB_LogSystemInit();
    GB_SleepMs(10000);

    mpuDataQueueReady = xSemaphoreCreateBinary();
    mpuSensorReady = xSemaphoreCreateBinary();
    gyroQueue = xQueueCreate(MPU_DATA_QUEUE_SIZE, sizeof(raw_axes_t));
    accelQueue = xQueueCreate(MPU_DATA_QUEUE_SIZE, sizeof(raw_axes_t));
    magQueue = xQueueCreate(MPU_DATA_QUEUE_SIZE, sizeof(raw_axes_t));
    baroQueue = xQueueCreate(MPU_DATA_QUEUE_SIZE, sizeof(baro_t));

    GB_DEBUGI(GB_INFO, "Taks Create Start");

    // TEST IO
    //gpio_reset_pin( TEST_IMU_IO );
    //gpio_set_direction( TEST_IMU_IO, GPIO_MODE_OUTPUT );

    xTaskCreatePinnedToCore( gb_sensor_fusion, "gb_sensor_fusion", 5120, NULL, configMAX_PRIORITIES - 1, NULL, tskNO_AFFINITY );
    xTaskCreatePinnedToCore( gb_read_sensor_data, "gb_read_sensor_data", 4096, NULL, configMAX_PRIORITIES - 2, &mpu_isr_handle, tskNO_AFFINITY );
    xTaskCreatePinnedToCore( uart_rx_task, "uart_rx_task", 4096, NULL, 2 | portPRIVILEGE_BIT, NULL, 1 );

    GB_DEBUGI(GB_INFO, "Taks Create DONE");

    return;
}
