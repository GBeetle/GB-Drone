/**
 * @file disp_driver.h
 */

#ifndef LVGL_DRIVER_H
#define LVGL_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "../lvgl.h"
#endif

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/* Display flush callback */
void lvgl_driver_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_map);

/* Display rounder callback, used with monochrome dispays */
void lvgl_driver_rounder(lv_disp_drv_t * disp_drv, lv_area_t * area);

/* Display set_px callback, used with monochrome dispays */
void lvgl_driver_set_px(lv_disp_drv_t * disp_drv, uint8_t * buf, lv_coord_t buf_w, lv_coord_t x, lv_coord_t y,
    lv_color_t color, lv_opa_t opa);

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*LVGL_DRIVER_H*/
