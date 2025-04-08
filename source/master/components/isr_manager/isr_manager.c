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
#include "rom/ets_sys.h"
#include "driver/gpio.h"   // TODO: USE gpio_setting component

TaskHandle_t mpu_isr_handle = NULL;
TaskHandle_t nrf24_isr_handle = NULL;

extern SemaphoreHandle_t mpuSensorReady;

void IRAM_ATTR mpu_dmp_isr_handler(void* arg)
{
    //GB_GPIO_Set( TEST_NRF24_IO, 1 );
    //ets_printf("isr before:[%s] stat:[%d] prid:[%d]\n", pcTaskGetName(mpu_isr_handle), eTaskGetState(mpu_isr_handle), uxTaskPriorityGetFromISR(mpu_isr_handle));
    /* Notify the task that the transmission is complete. */
    if(mpu_isr_handle)
    {
        xSemaphoreGiveFromISR(mpuSensorReady, NULL);
    }
    //GB_GPIO_Set( TEST_NRF24_IO, 0 );
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

GB_RESULT mpu_isr_register()
{
    // mpu interrupt configuration for ESP32
    gpio_config_t mpu_io_conf;
    mpu_io_conf.intr_type    = GPIO_INTR_POSEDGE;
    mpu_io_conf.pin_bit_mask = MPU_GPIO_INPUT_PIN_SEL;
    mpu_io_conf.mode         = GPIO_MODE_INPUT;
    mpu_io_conf.pull_up_en   = 0;
    mpu_io_conf.pull_down_en = 1;
    gpio_config(&mpu_io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(MPU_DMP_INT, mpu_dmp_isr_handler, (void*) MPU_DMP_INT);

    return GB_OK;
}
