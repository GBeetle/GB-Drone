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

#ifndef ST7701_H
#define ST7701_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "lvgl.h"
#include "../disp_driver.h"

// DSI configuration
#define ST7701_DSI_LANES    1    // Single-lane MIPI DSI
#define ST7701_LANE_BITRATE_MBPS 700   // 700 Mbps bitrate
#define ST7701_PIXEL_CLOCK_MHZ 25    // 25 MHz pixel clock
#define ST7701_REFRESH_RATE_HZ 60    // 60 Hz refresh rate

// DPI video timing parameters (from st7701.h:101-119)
#define ST7701_HSYNC_BACK_PORCH 30
#define ST7701_HSYNC_PULSE_WIDTH 10
#define ST7701_HSYNC_FRONT_PORCH 30
#define ST7701_VSYNC_BACK_PORCH 50
#define ST7701_VSYNC_PULSE_WIDTH 8
#define ST7701_VSYNC_FRONT_PORCH 50

// PCA9536 pin assignments (from main.c:194, 232-236)
#define ST7701_PCA9536_RESET_PIN 0    // PIN0 controls LCD reset
#define ST7701_PCA9536_BACKLIGHT_PIN 1    // PIN1 controls backlight

void st7701_init(void);

void st7701_flush(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, void *color_map);

#ifdef __cplusplus
}
#endif

#endif // ST7701_H
