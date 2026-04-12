/**
 * @file disp_driver.c
 */

#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_ops.h"
#include "lvgl_driver.h"
#include "disp_driver.h"
#include "disp_dsi.h"
#include "sdkconfig.h"

static bool lvgl_flash_enable = true;
static lv_display_t *g_current_disp_drv = NULL;

void lvgl_driver_flush(lv_display_t * drv, const lv_area_t * area, uint8_t * color_map)
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

lv_display_t* lvgl_driver_get_disp_drv(void)
{
    return g_current_disp_drv;
}
