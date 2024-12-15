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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void)
{
    GB_LogSystemInit();

    xTaskCreatePinnedToCore( mpu_get_sensor_data, "mpu_get_sensor_data", 5120, NULL, configMAX_PRIORITIES - 1, &mpu_isr_handle, tskNO_AFFINITY );
    xTaskCreatePinnedToCore( uart_rx_task, "uart_rx_task", 4096, NULL, 2 | portPRIVILEGE_BIT, NULL, 1 );

    GB_DEBUGI(GB_INFO, "Taks Create DONE");

    return;
}
