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

#ifndef DISP_DRIVER_H
#define DISP_DRIVER_H

#include "sdkconfig.h"
#include "io_define.h"
#include "gb_timer.h"
#include "stdbool.h"

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/

#if defined CONFIG_TFT_DISPLAY_CONTROLLER_ILI9341
#include "ili9341.h"
#elif defined CONFIG_TFT_DISPLAY_CONTROLLER_ILI9481
#include "ili9481.h"
#elif defined CONFIG_TFT_DISPLAY_CONTROLLER_ILI9488
#include "ili9488.h"
#elif defined CONFIG_TFT_DISPLAY_CONTROLLER_ST7789
#include "st7789.h"
#elif defined CONFIG_TFT_DISPLAY_CONTROLLER_ST7796S
#include "st7796s.h"
#elif defined CONFIG_TFT_DISPLAY_CONTROLLER_ST7735S
#include "st7735s.h"
#elif defined CONFIG_TFT_DISPLAY_CONTROLLER_HX8357
#include "hx8357.h"
#elif defined CONFIG_TFT_DISPLAY_CONTROLLER_ILI9486
#include "ili9486.h"
#elif defined CONFIG_TFT_DISPLAY_CONTROLLER_SH1107
#include "sh1107.h"
#elif defined CONFIG_TFT_DISPLAY_CONTROLLER_SSD1306
#include "ssd1306.h"
#elif defined CONFIG_TFT_DISPLAY_CONTROLLER_FT81X
#include "FT81x.h"
#elif defined CONFIG_TFT_DISPLAY_CONTROLLER_IL3820
#include "il3820.h"
#elif defined CONFIG_TFT_DISPLAY_CONTROLLER_RA8875
#include "ra8875.h"
#elif defined CONFIG_TFT_DISPLAY_CONTROLLER_GC9A01
#include "GC9A01.h"
#elif defined CONFIG_TFT_DISPLAY_CONTROLLER_JD79653A
#include "jd79653a.h"
#elif defined CONFIG_TFT_DISPLAY_CONTROLLER_UC8151D
#include "uc8151d.h"
#endif

/*********************
 *      DEFINES
 *********************/

/* Maximal horizontal and vertical resolution to support by the library.*/
#ifndef LV_HOR_RES_MAX
#  ifdef CONFIG_LV_HOR_RES_MAX
#    define LV_HOR_RES_MAX CONFIG_LV_HOR_RES_MAX
#  else
#    define  LV_HOR_RES_MAX          (480)
#  endif
#endif
#ifndef LV_VER_RES_MAX
#  ifdef CONFIG_LV_VER_RES_MAX
#    define LV_VER_RES_MAX CONFIG_LV_VER_RES_MAX
#  else
#    define  LV_VER_RES_MAX          (320)
#  endif
#endif

/* DISP_BUF_SIZE value doesn't have an special meaning, but it's the size
 * of the buffer(s) passed to LVGL as display buffers. The default values used
 * were the values working for the contributor of the display controller.
 *
 * As LVGL supports partial display updates the DISP_BUF_SIZE doesn't
 * necessarily need to be equal to the display size.
 *
 * When using RGB displays the display buffer size will also depends on the
 * color format being used, for RGB565 each pixel needs 2 bytes.
 * When using the mono theme, the display pixels can be represented in one bit,
 * so the buffer size can be divided by 8, e.g. see SSD1306 display size. */
#if defined (CONFIG_CUSTOM_DISPLAY_BUFFER_SIZE)
#define DISP_BUF_SIZE   CONFIG_CUSTOM_DISPLAY_BUFFER_BYTES
#else
#if defined (CONFIG_TFT_DISPLAY_CONTROLLER_ST7789)
#define DISP_BUF_SIZE  (LV_HOR_RES_MAX * 40)
#elif defined CONFIG_TFT_DISPLAY_CONTROLLER_ST7735S
#define DISP_BUF_SIZE  (LV_HOR_RES_MAX * 40)
#elif defined CONFIG_TFT_DISPLAY_CONTROLLER_ST7796S
#define DISP_BUF_SIZE  (LV_HOR_RES_MAX * 40)
#elif defined CONFIG_TFT_DISPLAY_CONTROLLER_HX8357
#define DISP_BUF_SIZE  (LV_HOR_RES_MAX * 40)
#elif defined CONFIG_TFT_DISPLAY_CONTROLLER_SH1107
#define DISP_BUF_SIZE  (LV_HOR_RES_MAX * LV_VER_RES_MAX)
#elif defined CONFIG_TFT_DISPLAY_CONTROLLER_ILI9481
#define DISP_BUF_SIZE  (LV_HOR_RES_MAX * 40)
#elif defined CONFIG_TFT_DISPLAY_CONTROLLER_ILI9486
#define DISP_BUF_SIZE  (LV_HOR_RES_MAX * 40)
#elif defined CONFIG_TFT_DISPLAY_CONTROLLER_ILI9488
#define DISP_BUF_SIZE  (LV_HOR_RES_MAX * 40)
#elif defined CONFIG_TFT_DISPLAY_CONTROLLER_ILI9341
#define DISP_BUF_SIZE  (LV_HOR_RES_MAX * 40)
#elif defined CONFIG_TFT_DISPLAY_CONTROLLER_SSD1306

#if defined (CONFIG_THEME_MONO)
#define DISP_BUF_SIZE  (LV_HOR_RES_MAX * (LV_VER_RES_MAX / 8))
#else
#define DISP_BUF_SIZE  (LV_HOR_RES_MAX * LV_VER_RES_MAX)
#endif

#elif defined (CONFIG_TFT_DISPLAY_CONTROLLER_FT81X)
#define DISP_BUF_LINES  40
#define DISP_BUF_SIZE  (LV_HOR_RES_MAX * DISP_BUF_LINES)
#elif defined (CONFIG_TFT_DISPLAY_CONTROLLER_IL3820)
#define DISP_BUF_SIZE (LV_VER_RES_MAX * IL3820_COLUMNS)
#elif defined CONFIG_TFT_DISPLAY_CONTROLLER_RA8875
#define DISP_BUF_SIZE  (LV_HOR_RES_MAX * 40)
#elif defined (CONFIG_TFT_DISPLAY_CONTROLLER_GC9A01)
#define DISP_BUF_SIZE  (LV_HOR_RES_MAX * 40)
#elif defined (CONFIG_TFT_DISPLAY_CONTROLLER_JD79653A)
#define DISP_BUF_SIZE ((LV_VER_RES_MAX * LV_VER_RES_MAX) / 8) // 5KB
#elif defined (CONFIG_TFT_DISPLAY_CONTROLLER_UC8151D)
#define DISP_BUF_SIZE ((LV_VER_RES_MAX * LV_VER_RES_MAX) / 8) // 2888 bytes

#else
#error "No display controller selected"
#endif
#endif

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/* Initialize display */
void disp_driver_init(void);

void disp_driver_set_rotation(uint8_t m);

/* Display flush callback */
void disp_driver_flush(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, void *color_map);

#if 0
/* Display rounder callback, used with monochrome dispays */
void disp_driver_rounder(lv_disp_drv_t * disp_drv, lv_area_t * area);

/* Display set_px callback, used with monochrome dispays */
void disp_driver_set_px(lv_disp_drv_t * disp_drv, uint8_t * buf, lv_coord_t buf_w, lv_coord_t x, lv_coord_t y,
    lv_color_t color, lv_opa_t opa);
#endif

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*DISP_DRIVER_H*/
