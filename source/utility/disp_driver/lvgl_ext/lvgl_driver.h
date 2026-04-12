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
void lvgl_driver_flush(lv_display_t * drv, const lv_area_t * area, uint8_t * color_map);

lv_display_t* lvgl_driver_get_disp_drv(void);

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*LVGL_DRIVER_H*/
