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
#include "tft_sprite.h"
#include "file_system.h"
#include "disp_driver.h"

struct TFT_eSPI tft;
struct TFT_eSprite sprite;

void draw_loop()
{
    uint32_t buffer_size = LV_HOR_RES_MAX * LV_VER_RES_MAX * 2;
    uint16_t *buffer = (uint16_t *)heap_caps_malloc(buffer_size, MALLOC_CAP_DMA);

    for (int i = 0; i < 60; i++) {
        char filename[20];
        sprintf(filename, "O_%d.RAW", i);

        GB_FileSystem_Read(filename, (uint8_t*)buffer, buffer_size);
        tft.pushImage(&tft, 0, 0, LV_HOR_RES_MAX, LV_VER_RES_MAX, buffer);

        vTaskDelay(50 / portTICK_PERIOD_MS);

        GB_DEBUGI(DISP_TAG, "Display %s", filename);
    }
}

void app_main(void)
{
    GB_LogSystemInit();
    GB_FileSystem_Init("storage");

    disp_driver_init();
    TFT_eSpi_init(&tft, LV_HOR_RES_MAX, LV_VER_RES_MAX, 0);
    tft.setRotation(&tft, 0);
    TFT_eSprite_init(&sprite, &tft);

    xTaskCreate(draw_loop, "draw_loop", 5120, NULL, 4 | portPRIVILEGE_BIT, NULL);
}
