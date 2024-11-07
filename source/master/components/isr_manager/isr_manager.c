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

#include "isr_manager.h"

TaskHandle_t mpu_isr_handle = NULL;
TaskHandle_t nrf24_isr_handle = NULL;

isr_manager_t mpu_isr_manager = {
    .mpu_isr_status = DATA_NOT_READY,
    .mpu_gyro_data_status = DATA_NOT_READY,
    .mpu_accel_data_status = DATA_NOT_READY,
    .mpu_mag_data_status = DATA_NOT_READY
};

void IRAM_ATTR mpu_dmp_isr_handler(void* arg)
{
    //gpio_set_level( TEST_NRF24_IO, 1 );
    mpu_isr_manager.mpu_isr_status = DATA_READY;
    //ets_printf("isr before:[%s] stat:[%d] prid:[%d]\n", pcTaskGetTaskName(mpu_isr_handle), eTaskGetState(mpu_isr_handle), uxTaskPriorityGetFromISR(mpu_isr_handle));
    /* Notify the task that the transmission is complete. */
    if(mpu_isr_handle)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(mpu_isr_handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    //gpio_set_level( TEST_NRF24_IO, 0 );
}

void IRAM_ATTR nrf24_interrupt_handler(void* arg)
{
    //ets_printf("NRF24 ISR\n");
    // notify task
    if (nrf24_isr_handle)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(nrf24_isr_handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
