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

#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "log_sys.h"
#include "disp_driver.h"
#include "task_manager.h"
#include "lvgl_example.h"
#include "file_system.h"
#include "lora_state.h"
#include "io_define.h"
#include "gpio_setting.h"

void app_main(void)
{
    GB_LogSystemInit();
    GB_FileSystem_Init("storage");

    disp_driver_init();

    // TEST IO
    GB_GPIO_Reset( TEST_IMU_IO );
    GB_GPIO_SetDirection( TEST_IMU_IO, GB_GPIO_OUTPUT );

    // xTaskCreate(draw_loop, "draw_loop", 5120, NULL, 4 | portPRIVILEGE_BIT, NULL);
    xTaskCreate(controller_task, "controller", 1024 * 5, NULL, 2 | portPRIVILEGE_BIT, NULL);
    xTaskCreatePinnedToCore(gui_task, "gui", 1024 * 10,
#if defined CONFIG_LV_USE_DEMO_WIDGETS
                            lv_demo_widgets,
#elif defined CONFIG_LV_USE_DEMO_KEYPAD_AND_ENCODER
                            lv_demo_keypad_encode,
#elif defined CONFIG_LV_USE_DEMO_BENCHMARK
                            lv_demo_benchmark,
#elif defined CONFIG_LV_USE_DEMO_STRESS
                            lv_demo_stress,
#else
                            NULL,
#endif
                            1 | portPRIVILEGE_BIT, NULL, tskNO_AFFINITY);
    xTaskCreate(rf_loop, "nrf24_loop", 4096, NULL, 3 | portPRIVILEGE_BIT, NULL);
}
