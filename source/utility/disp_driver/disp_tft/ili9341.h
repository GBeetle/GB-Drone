/**
 * @file lv_templ.h
 *
 */

#ifndef ILI9341_H
#define ILI9341_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include <stdbool.h>

#include "../disp_driver.h"

/*********************
 *      DEFINES
 *********************/
#define ILI9341_DC   CONFIG_DISP_PIN_DC
#define ILI9341_RST  CONFIG_DISP_PIN_RST
#define ILI9341_BCKL CONFIG_DISP_PIN_BCKL

#define ILI9341_ENABLE_BACKLIGHT_CONTROL CONFIG_ENABLE_BACKLIGHT_CONTROL

#if CONFIG_BACKLIGHT_ACTIVE_LVL
  #define ILI9341_BCKL_ACTIVE_LVL 1
#else
  #define ILI9341_BCKL_ACTIVE_LVL 0
#endif

#define ILI9341_INVERT_COLORS CONFIG_INVERT_COLORS

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/

void ili9341_init(void);
void ili9341_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_map);
void ili9341_enable_backlight(bool backlight);
void ili9341_sleep_in(void);
void ili9341_sleep_out(void);

/**********************
 *      MACROS
 **********************/


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*ILI9341_H*/
