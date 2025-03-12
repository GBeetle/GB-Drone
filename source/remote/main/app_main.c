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

struct TFT_eSPI tft;
struct TFT_eSprite sprite;

void draw_loop()
{
    uint32_t buffer_size = TFT_WIDTH * TFT_HEIGHT * 2;
    uint16_t *buffer = (uint16_t *)malloc(buffer_size);

    for (int i = 1; i < 62; i++) {
        char filename[20];
        sprintf(filename, "O_%d.RAW", i);

        GB_FileSystem_Read(filename, (uint8_t*)buffer, buffer_size);
        tft.pushImageRam(&tft, 0, 0, TFT_WIDTH, TFT_HEIGHT, buffer);

        GB_DEBUGI(TFT_TAG, "Display %s", filename);
    }
}

void app_main(void)
{
    GB_LogSystemInit();
    GB_FileSystem_Init("storage");

#if 0
    char test[] = "Hello world";
    char out[256] = {0};
    uint32_t out_size = 256;
    GB_FileSystem_Write("Hello", (uint8_t *)test, strlen(test));
    GB_FileSystem_Read("Hello", (uint8_t *)out, strlen(test));
    GB_DEBUGI(FS_TAG, "Read %d bytes, data: %s", out_size, out);
    GB_FileSystem_ListDir();
#endif

    TFT_eSpi_init(&tft, TFT_WIDTH, TFT_HEIGHT, 0);
    tft.setRotation(&tft, 0);
    TFT_eSprite_init(&sprite, &tft);

    xTaskCreate(draw_loop, "draw_loop", 5120, NULL, 4 | portPRIVILEGE_BIT, NULL);
}
