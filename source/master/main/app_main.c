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
#include "lora_state.h"
#include "file_system.h"
#include "gb_timer.h"
#include "buzzer.h"
#include "max1704x.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void test_max17704x()
{
    GB_MAX1704X_DEV_T dev = {0 };
    GB_MAX1704X_CONFIG_T config = { 0 };
    GB_MAX1704X_STATUS_T status = { 0 };
    uint16_t version = 0;
    float voltage = 0;
    float soc_percent = 0;
    float rate_change = 0;

    /**
     * Set up I2C bus to communicate with MAX1704X
     */

    dev.model = MAX17043_4;

    CHK_EXIT(GB_Max1704xInit(&dev));
    CHK_EXIT(GB_Max1704xStart(&dev));
    CHK_EXIT(GB_Max1704xGetVersion(&dev, &version));
    GB_DEBUGI(BATTERY_TAG, "Version: %d\n", version);

    /**
     * Get MAX1704X configuration
     */
    GB_DEBUGI(BATTERY_TAG, "--- MAX1704X config register ---");
    CHK_EXIT(GB_Max1704xGetConfig(&dev));
    GB_DEBUGI(BATTERY_TAG, "Alert Status: %d", dev.config.alert_status);
    GB_DEBUGI(BATTERY_TAG, "Sleep Mode: %d", dev.config.sleep_mode);
    GB_DEBUGI(BATTERY_TAG, "SOC Change Alert Mode: %d", dev.config.soc_change_alert);
    GB_DEBUGI(BATTERY_TAG, "Empty Alert Threshold: %d%%", dev.config.empty_alert_thresh);
    GB_DEBUGI(BATTERY_TAG, "RCOMP Value: %d (%x)", dev.config.rcomp, dev.config.rcomp);
    GB_DEBUGI(BATTERY_TAG, "--- End Configuration ---\n");

    // Change configuration settings
    config.soc_change_alert = true;
    config.empty_alert_thresh = 10;
    GB_DEBUGI(BATTERY_TAG, "Setting new MAX1704X configuration");
    CHK_EXIT(GB_Max1704xSetConfig(&dev, &config));

    GB_DEBUGI(BATTERY_TAG, "--- MAX1704X config register after updating configurations ---");
    CHK_EXIT(GB_Max1704xGetConfig(&dev));
    GB_DEBUGI(BATTERY_TAG, "Alert Status: %d", dev.config.alert_status);
    GB_DEBUGI(BATTERY_TAG, "Sleep Mode: %d", dev.config.sleep_mode);
    GB_DEBUGI(BATTERY_TAG, "SOC Change Alert Mode: %d", dev.config.soc_change_alert);
    GB_DEBUGI(BATTERY_TAG, "Empty Alert Threshold: %d%%", dev.config.empty_alert_thresh);
    GB_DEBUGI(BATTERY_TAG, "RCOMP Value: %d (%x)", dev.config.rcomp, dev.config.rcomp);
    GB_DEBUGI(BATTERY_TAG, "--- End Configuration ---\n");

    /**
     * Get current MAX1704X status
     */
    GB_DEBUGI(BATTERY_TAG, "--- MAX1704X status register ---");
    if (GB_OK == GB_Max1704xGetStatus(&dev)) {
        GB_DEBUGI(BATTERY_TAG, "Reset Indicator: %d", dev.status.reset_indicator);
        GB_DEBUGI(BATTERY_TAG, "Voltage High Alert: %d", dev.status.voltage_high);
        GB_DEBUGI(BATTERY_TAG, "Voltage Low Alert: %d", dev.status.voltage_low);
        GB_DEBUGI(BATTERY_TAG, "Voltage Reset Alert: %d", dev.status.voltage_reset);
        GB_DEBUGI(BATTERY_TAG, "SOC Low Alert: %d", dev.status.soc_low);
        GB_DEBUGI(BATTERY_TAG, "SOC Change Alert: %d", dev.status.soc_change);
        GB_DEBUGI(BATTERY_TAG, "Voltage Reset Alert Enabled: %d", dev.status.vreset_alert);
        GB_DEBUGI(BATTERY_TAG, "--- End Status ---\n");

        // Update MAX1704X status register to clear the reset indicator
        GB_DEBUGI(BATTERY_TAG, "Setting the status register to clear reset indicator");
        status.reset_indicator = false;

        CHK_EXIT(GB_Max1704xSetStatus(&dev, &status));
        GB_DEBUGI(BATTERY_TAG, "--- MAX1704X status register after updating status register ---");
        CHK_EXIT(GB_Max1704xGetStatus(&dev));
        GB_DEBUGI(BATTERY_TAG, "Reset Indicator: %d", dev.status.reset_indicator);
        GB_DEBUGI(BATTERY_TAG, "Voltage High Alert: %d", dev.status.voltage_high);
        GB_DEBUGI(BATTERY_TAG, "Voltage Low Alert: %d", dev.status.voltage_low);
        GB_DEBUGI(BATTERY_TAG, "Voltage Reset Alert: %d", dev.status.voltage_reset);
        GB_DEBUGI(BATTERY_TAG, "SOC Low Alert: %d", dev.status.soc_low);
        GB_DEBUGI(BATTERY_TAG, "SOC Change Alert: %d", dev.status.soc_change);
        GB_DEBUGI(BATTERY_TAG, "Voltage Reset Alert Enabled: %d", dev.status.vreset_alert);
        GB_DEBUGI(BATTERY_TAG, "--- End Status ---\n");

    }

    /**
     * Get current MAX1704X voltage, SOC, and rate of change every 5 seconds
     */

    while (1)
    {
        CHK_EXIT(GB_Max1704xGetVoltage(&dev, &voltage));
        GB_DEBUGI(BATTERY_TAG, "Voltage: %.2fV", voltage);

        CHK_EXIT(GB_Max1704xGetSoc(&dev, &soc_percent));
        GB_DEBUGI(BATTERY_TAG, "SOC: %.2f%%", soc_percent);

        //CHK_EXIT(GB_Max1704xGetCrate(&dev, &rate_change));
        //GB_DEBUGI(BATTERY_TAG, "SOC rate of change: %.2f%%", rate_change);

        printf("\n");
        GB_SleepMs(5000);
    }
}

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

    test_max17704x();

    // TEST IO
    //GB_GPIO_Reset( TEST_IMU_IO );
    //GB_GPIO_SetDirection( TEST_IMU_IO, GB_GPIO_OUTPUT );

    GB_DEBUGI(GB_INFO, "Taks Create Start");
    xTaskCreatePinnedToCore( gb_sensor_fusion, "gb_sensor_fusion", 5120, NULL, configMAX_PRIORITIES - 1, NULL, tskNO_AFFINITY );
    xTaskCreatePinnedToCore( gb_read_sensor_data, "gb_read_sensor_data", 4096, NULL, configMAX_PRIORITIES - 2, &mpu_isr_handle, tskNO_AFFINITY );
    xTaskCreatePinnedToCore( nrf24_interrupt_func, "nrf24 interrupt", 4096, NULL, configMAX_PRIORITIES - 1, &nrf24_isr_handle, tskNO_AFFINITY);
    xTaskCreatePinnedToCore( uart_rx_task, "uart_rx_task", 4096, NULL, 2 | portPRIVILEGE_BIT, NULL, 1 );
    GB_DEBUGI(GB_INFO, "Taks Create DONE");

    return;
}
