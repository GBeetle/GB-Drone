/**
 * @file disp_driver.c
 */

#include "lvgl_driver.h"
#include "disp_driver.h"

static bool lvgl_flash_enable = true;

void lvgl_driver_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_map)
{
    disp_driver_flush(area->x1, area->y1, area->x2, area->y2, (void *)color_map);
    //TODO: only used for synchronous trans
    if (lvgl_flash_enable)
    {
        lv_disp_flush_ready(drv);
    }
}

void lvgl_driver_rounder(lv_disp_drv_t * disp_drv, lv_area_t * area)
{
    // disp_driver_rounder();
}

void lvgl_driver_set_px(lv_disp_drv_t * disp_drv, uint8_t * buf, lv_coord_t buf_w, lv_coord_t x, lv_coord_t y,
    lv_color_t color, lv_opa_t opa)
{
    // disp_driver_set_px();
}

void lvgl_driver_flash_setting(bool state)
{
    lvgl_flash_enable = state;
}
