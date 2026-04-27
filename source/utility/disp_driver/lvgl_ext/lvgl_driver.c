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

#if defined(CONFIG_TFT_DISPLAY_PROTOCOL_DSI) && defined(CONFIG_DISPLAY_ORIENTATION_PORTRAIT)
    // For DSI, lvgl_driver_flush will be called when DMA completes
    // The callback is registered in disp_driver_init()
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;

    /* Screen vertical size */
    int32_t hres = lv_display_get_horizontal_resolution(drv);
    int32_t vres = lv_display_get_vertical_resolution(drv);
    lvgl_port_ppa_disp_rotate_t rotate_cfg = {
        .in_buff = color_map,
        .area = {
            .x1 = area->x1,
            .x2 = area->x2,
            .y1 = area->y1,
            .y2 = area->y2,
        },
        .disp_size = {
            .hres = hres,
            .vres = vres,
        },
        .rotation = PPA_SRM_ROTATION_ANGLE_90,
        .ppa_mode = PPA_TRANS_MODE_BLOCKING,
        .swap_bytes = false,
        .user_data = NULL,
        .done_cb = NULL,
    };
    /* Do operation */
    esp_err_t err = lvgl_port_ppa_rotate(disp_dsi_get_ppa_handle(), &rotate_cfg);
    if (err == ESP_OK) {
        color_map = lvgl_port_ppa_get_output_buffer(disp_dsi_get_ppa_handle());
        offsetx1 = rotate_cfg.area.x1;
        offsetx2 = rotate_cfg.area.x2;
        offsety1 = rotate_cfg.area.y1;
        offsety2 = rotate_cfg.area.y2;
    }
    disp_driver_flush(offsetx1, offsety1, offsetx2, offsety2, (void *)color_map);
#else
    disp_driver_flush(area->x1, area->y1, area->x2, area->y2, (void *)color_map);
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
