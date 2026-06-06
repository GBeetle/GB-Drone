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

#include "st7701.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_commands.h"
#include "esp_lcd_mipi_dsi.h"
#include "log_sys.h"
#include "error_handle.h"
#include "../disp_dsi.h"
#include "esp_lcd_panel_interface.h"
#include "esp_check.h"

// Static handles
static esp_lcd_panel_handle_t s_panel_handle = NULL;

/**
 * @brief ST7701 initialization command structure
 */
typedef struct {
    uint8_t cmd;
    const uint8_t *data;
    uint8_t data_len;
    uint8_t delay_ms;
} st7701_init_cmd_t;

static const st7701_init_cmd_t vendor_init_cmds[] = {
    // 2.8 inch
    { 0xFF, (uint8_t[]){0x77, 0x01, 0x00, 0x00, 0x13}, 5, 0},
    // Unknown
    { 0xEF, (uint8_t[]){0x08},1, 0},
    // Command2 BK0 Selection: Disable the BK function of Command2
    { 0xFF, (uint8_t[]){0x77, 0x01, 0x00, 0x00, 0x10},5, 0},
    // Display Line Setting
    { 0xC0, (uint8_t[]){0x4f, 0x00},2, 0},
    // Porch Control
    { 0xC1, (uint8_t[]){0x10, 0x0c},2, 0},
    // Inversion selection & Frame Rate Control
    { 0xC2, (uint8_t[]){0x07, 0x14},2, 0},
    // Unknown
    { 0xCC, (uint8_t[]){0x10}, 1,0},
    // Positive Voltage Gamma Control
    { 0xB0, (uint8_t[]){0x0a, 0x18, 0x1e, 0x12, 0x16, 0x0c, 0x0e, 0x0d, 0x0c,0x29, 0x06, 0x14, 0x13, 0x29, 0x33, 0x1c},16, 0},
    // Negative Voltage Gamma Control
    { 0xB1, (uint8_t[]){0x0a, 0x19, 0x21, 0x0a, 0x0c, 0x00, 0x0c, 0x03, 0x03,0x23, 0x01, 0x0e, 0x0c, 0x27, 0x2b, 0x1c},16, 0},

    // Command2 BK1 Selection: Enable the BK function of Command2
    { 0xFF, (uint8_t[]){0x77, 0x01, 0x00, 0x00, 0x11}, 5, 0},
    // Vop Amplitude setting
    { 0xB0, (uint8_t[]){0x5d},1, 0},
    // VCOM amplitude setting
    { 0xB1, (uint8_t[]){0x61},1, 0},
    // VGH Voltage setting
    { 0xB2, (uint8_t[]){0x84},1, 0},
    // TEST Command Setting
    { 0xB3, (uint8_t[]){0x80},1, 0},
    // VGL Voltage setting
    { 0xB5, (uint8_t[]){0x4d},1, 0},
    // Power Control 1
    { 0xB7, (uint8_t[]){0x85},1, 0},
    // Power Control 2
    { 0xB8, (uint8_t[]){0x20},1, 0},
    // Source pre_drive timing set1
    { 0xC1, (uint8_t[]){0x78},1, 0},
    // Source EQ2 Setting
    { 0xC2, (uint8_t[]){0x78},1, 0},
    // MIPI Setting 1
    { 0xD0, (uint8_t[]){0x88},1, 0},
    // GIP Code
    { 0xE0, (uint8_t[]){0x00, 0x00, 0x02}, 3, 0},
    { 0xE1, (uint8_t[]){0x06, 0xa0, 0x08, 0xa0, 0x05, 0xa0, 0x07, 0xa0, 0x00,0x44, 0x44}, 11, 0},
    { 0xE2, (uint8_t[]){0x20, 0x20, 0x44, 0x44, 0x96, 0xa0, 0x00, 0x00, 0x96,0xa0, 0x00, 0x00}, 12, 0},
    { 0xE3, (uint8_t[]){0x00, 0x00, 0x22, 0x22}, 4, 0},
    { 0xE4, (uint8_t[]){0x44, 0x44}, 2, 0},
    { 0xE5, (uint8_t[]){0x0d, 0x91, 0xa0, 0xa0, 0x0f, 0x93, 0xa0, 0xa0, 0x09,0x8d, 0xa0, 0xa0, 0x0b, 0x8f, 0xa0, 0xa0}, 16, 0},
    { 0xE6, (uint8_t[]){0x00, 0x00, 0x22, 0x22}, 4, 0},
    { 0xE7, (uint8_t[]){0x44, 0x44}, 2, 0},
    { 0xE8, (uint8_t[]){0x0c, 0x90, 0xa0, 0xa0, 0x0e, 0x92, 0xa0, 0xa0, 0x08,0x8c, 0xa0, 0xa0, 0x0a, 0x8e, 0xa0, 0xa0}, 16, 0},
    { 0xE9, (uint8_t[]){0x36, 0x00}, 2, 0},
    { 0xEB, (uint8_t[]){0x00, 0x01, 0xe4, 0xe4, 0x44, 0x88, 0x40}, 7, 0},
    { 0xED, (uint8_t[]){0xff, 0x45, 0x67, 0xfa, 0x01, 0x2b, 0xcf, 0xff, 0xff,0xfc, 0xb2, 0x10, 0xaf, 0x76, 0x54, 0xff}, 16, 0},
    { 0xEF, (uint8_t[]){0x10, 0x0d, 0x04, 0x08, 0x3f, 0x1f}, 6, 0},

    // disable Command2
    { 0xFF, (uint8_t[]){0x77, 0x01, 0x00, 0x00, 0x00}, 5, 0},
    //add
    { 0x11, (uint8_t[]){0x00}, 1, 120}, // Sleep out, delay 120ms
    // enable Command2
    {0x29, (uint8_t[]){0x00}, 1, 20} // Display on, delay 20ms

};

void st7701_sleep_in(void)
{
    esp_lcd_panel_io_handle_t io = disp_dsi_get_dbi_io();
    if (io) {
        esp_lcd_panel_io_tx_param(io, LCD_CMD_SLPIN, NULL, 0);
        vTaskDelay(pdMS_TO_TICKS(120));
        GB_DEBUGE(DISP_TAG, "Entered sleep mode");
    }
}

void st7701_sleep_out(void)
{
    esp_lcd_panel_io_handle_t io = disp_dsi_get_dbi_io();
    if (io) {
        esp_lcd_panel_io_tx_param(io, LCD_CMD_SLPOUT, NULL, 0);
        vTaskDelay(pdMS_TO_TICKS(120));
        GB_DEBUGE(DISP_TAG, "Exited sleep mode");
    }
}

// ========== ST7701 Vendor Panel Wrapper ==========

typedef struct {
    esp_lcd_panel_io_handle_t io;
    uint8_t madctl_val;
    esp_err_t (*orig_del)(esp_lcd_panel_t *panel);
    esp_err_t (*orig_init)(esp_lcd_panel_t *panel);
} st7701_vendor_t;

static esp_err_t st7701_panel_del(esp_lcd_panel_t *panel)
{
    st7701_vendor_t *vendor = (st7701_vendor_t *)panel->user_data;
    panel->del = vendor->orig_del;
    panel->user_data = NULL;
    esp_err_t ret = vendor->orig_del(panel);
    free(vendor);
    return ret;
}

static esp_err_t st7701_panel_init(esp_lcd_panel_t *panel)
{
    st7701_vendor_t *vendor = (st7701_vendor_t *)panel->user_data;
    esp_lcd_panel_io_handle_t io = vendor->io;

    uint8_t id[3] = {0};
    esp_lcd_panel_io_rx_param(io, 0x04, id, 3);
    GB_DEBUGI(DISP_TAG, "Display ID: 0x%02x 0x%02x 0x%02x", id[0], id[1], id[2]);

    esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]){vendor->madctl_val}, 1);

    int cmd_count = sizeof(vendor_init_cmds) / sizeof(st7701_init_cmd_t);
    for (int i = 0; i < cmd_count; i++) {
        const st7701_init_cmd_t *cmd = &vendor_init_cmds[i];
        esp_lcd_panel_io_tx_param(io, cmd->cmd, cmd->data, cmd->data_len);
        if (cmd->delay_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(cmd->delay_ms));
        }
    }

    panel->init = vendor->orig_init;
    esp_err_t ret = vendor->orig_init(panel);
    panel->init = st7701_panel_init;

    return ret;
}

static esp_err_t st7701_panel_reset(esp_lcd_panel_t *panel)
{
    st7701_vendor_t *vendor = (st7701_vendor_t *)panel->user_data;
    if (vendor->io) {
        esp_lcd_panel_io_tx_param(vendor->io, LCD_CMD_SWRESET, NULL, 0);
        vTaskDelay(pdMS_TO_TICKS(120));
    }
    return ESP_OK;
}

static esp_err_t st7701_panel_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    st7701_vendor_t *vendor = (st7701_vendor_t *)panel->user_data;
    uint8_t madctl_val = vendor->madctl_val;
    if (mirror_x) madctl_val |= (1 << 0);
    else madctl_val &= ~(1 << 0);
    if (mirror_y) madctl_val |= (1 << 1);
    else madctl_val &= ~(1 << 1);
    esp_err_t ret = esp_lcd_panel_io_tx_param(vendor->io, LCD_CMD_MADCTL,
        (uint8_t[]){madctl_val}, 1);
    if (ret == ESP_OK) vendor->madctl_val = madctl_val;
    return ret;
}

static esp_err_t st7701_panel_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    st7701_vendor_t *vendor = (st7701_vendor_t *)panel->user_data;
    return esp_lcd_panel_io_tx_param(vendor->io,
        invert_color_data ? LCD_CMD_INVON : LCD_CMD_INVOFF, NULL, 0);
}

static esp_err_t st7701_panel_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
    st7701_vendor_t *vendor = (st7701_vendor_t *)panel->user_data;
    return esp_lcd_panel_io_tx_param(vendor->io,
        on_off ? LCD_CMD_DISPON : LCD_CMD_DISPOFF, NULL, 0);
}

esp_err_t st7701_register_flush_callback(void)
{
    if (!s_panel_handle) {
        GB_DEBUGE(DISP_TAG, "Cannot register callback - panel not initialized");
        return ESP_FAIL;
    }

    // Get the LVGL driver handle from lvgl_driver layer
    // This is set when lvgl_driver_flush() is called
    extern lv_display_t* lvgl_driver_get_disp_drv(void);
    lv_display_t *drv = lvgl_driver_get_disp_drv();
    if (!drv) {
        GB_DEBUGW(DISP_TAG, "LVGL driver not available yet for callback registration");
        return ESP_ERR_INVALID_STATE;
    }

    // Register the DPI callback with LVGL driver as user context
    esp_err_t ret = disp_dsi_register_flush_callback(s_panel_handle, drv);
    if (ret == ESP_OK) {
        GB_DEBUGI(DISP_TAG, "DPI flush callback registered for async operation");
    } else {
        GB_DEBUGE(DISP_TAG, "Failed to register DPI flush callback");
    }

    return ret;
}

void st7701_init(void)
{
    esp_err_t ret;

    GB_DEBUGI(DISP_TAG, "Initializing ST7701 display controller");
    GB_DEBUGI(DISP_TAG, "Resolution: %dx%d, DSI lanes: %d, Bitrate: %d Mbps",
        LV_HOR_RES_MAX, LV_VER_RES_MAX, ST7701_DSI_LANES,
        ST7701_LANE_BITRATE_MBPS);

    // Step 1: Create DSI bus + DBI I/O
    esp_lcd_dsi_bus_config_t bus_config = {
        .bus_id = 0,
        .num_data_lanes = ST7701_DSI_LANES,
        .phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT,
        .lane_bit_rate_mbps = ST7701_LANE_BITRATE_MBPS,
    };

    ret = disp_dsi_create_bus(&bus_config);
    if (ret != ESP_OK) {
        GB_DEBUGE(DISP_TAG, "Failed to create DSI bus");
        return;
    }

    // Step 2: Create DPI panel
    esp_lcd_dpi_panel_config_t dpi_config = {
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
        .dpi_clock_freq_mhz = ST7701_PIXEL_CLOCK_MHZ,
        .virtual_channel = 0,
        .in_color_format = LCD_COLOR_FMT_RGB888,
        .num_fbs = 1,
        .video_timing = {
            .h_size = LV_HOR_RES_MAX,
            .v_size = LV_VER_RES_MAX,
            .hsync_back_porch = ST7701_HSYNC_BACK_PORCH,
            .hsync_pulse_width = ST7701_HSYNC_PULSE_WIDTH,
            .hsync_front_porch = ST7701_HSYNC_FRONT_PORCH,
            .vsync_back_porch = ST7701_VSYNC_BACK_PORCH,
            .vsync_pulse_width = ST7701_VSYNC_PULSE_WIDTH,
            .vsync_front_porch = ST7701_VSYNC_FRONT_PORCH,
        },
        .flags.use_dma2d = true,
    };

    esp_lcd_dsi_bus_handle_t dsi_bus = disp_dsi_get_bus();
    ret = esp_lcd_new_panel_dpi(dsi_bus, &dpi_config, &s_panel_handle);
    if (ret != ESP_OK) {
        GB_DEBUGE(DISP_TAG, "Failed to create DPI panel");
        return;
    }

    disp_dsi_set_dpi_panel(s_panel_handle);
    GB_DEBUGI(DISP_TAG, "DPI panel created successfully");

    // Step 3: Wrap DPI panel with vendor operations
    st7701_vendor_t *vendor = calloc(1, sizeof(st7701_vendor_t));
    if (!vendor) {
        GB_DEBUGE(DISP_TAG, "Failed to allocate vendor struct");
        return;
    }
    vendor->io = disp_dsi_get_dbi_io();
    vendor->madctl_val = 0;
    vendor->orig_del = s_panel_handle->del;
    vendor->orig_init = s_panel_handle->init;

    s_panel_handle->del = st7701_panel_del;
    s_panel_handle->init = st7701_panel_init;
    s_panel_handle->reset = st7701_panel_reset;
    s_panel_handle->mirror = st7701_panel_mirror;
    s_panel_handle->invert_color = st7701_panel_invert_color;
    s_panel_handle->disp_on_off = st7701_panel_disp_on_off;
    s_panel_handle->user_data = vendor;

    // Step 4: Reset (DBI software reset)
    esp_lcd_panel_reset(s_panel_handle);

    // Step 5: Init (sends vendor init cmds + starts DPI video)
    ret = esp_lcd_panel_init(s_panel_handle);
    if (ret != ESP_OK) {
        GB_DEBUGE(DISP_TAG, "Failed to initialize panel");
        return;
    }

    GB_DEBUGI(DISP_TAG, "ST7701 initialized successfully");
}

void st7701_flush(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, void *color_map)
{
    if (!s_panel_handle) {
        GB_DEBUGE(DISP_TAG, "ST7701 panel not initialized");
        return;
    }

    // Register DPI callback on first flush (lazy initialization)
    // At this point, LVGL has called lvgl_driver_flush() so the driver handle is available
    static bool callback_registered = false;
    if (!callback_registered) {
        if (st7701_register_flush_callback() == ESP_OK) {
            callback_registered = true;
        }
        // Continue even if registration fails - will fallback to sync mode
    }

    // Send pixel data to DPI panel via esp_lcd (async DMA transfer)
    // Note: For DSI, lv_disp_flush_ready() is called by DPI callback, not here
    esp_err_t ret = esp_lcd_panel_draw_bitmap(s_panel_handle, x1, y1, x2 + 1, y2 + 1, color_map);

    if (ret != ESP_OK) {
        GB_DEBUGE(DISP_TAG, "Failed to draw bitmap");
    }
}
