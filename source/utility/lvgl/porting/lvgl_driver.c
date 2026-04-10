/**
 * @file disp_driver.c
 */

#include "lvgl_driver.h"
#include "disp_driver.h"
#include "sdkconfig.h"

static bool lvgl_flash_enable = true;
static lv_disp_drv_t *g_current_disp_drv = NULL;

void lvgl_driver_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_map)
{
    g_current_disp_drv = drv;

    disp_driver_flush(area->x1, area->y1, area->x2, area->y2, (void *)color_map);
#if defined(CONFIG_TFT_DISPLAY_PROTOCOL_DSI)
    // For DSI, lvgl_driver_flush will be called when DMA completes
    // The callback is registered in disp_driver_init()
#else
    if (lvgl_flash_enable)
    {
        lv_disp_flush_ready(drv);
    }
#endif
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

lv_disp_drv_t * lvgl_driver_get_disp_drv(void)
{
    return g_current_disp_drv;
}
