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
#include "i2c_bus.h"


#define PCA9536_ADDR         0x41
#define PCA9536_INPUT_REG    0x00
#define PCA9536_OUTPUT_REG   0x01
#define PCA9536_POLARITY_REG 0x02
#define PCA9536_CONFIG_REG   0x03

#define I2C_MASTER_SDA_IO    22    // SDA pin (from screen project)
#define I2C_MASTER_SCL_IO    21    // SCL pin (from screen project)
#define I2C_MASTER_FREQ_HZ   100000    // 100 kHz

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

static esp_err_t pca9536_i2c_init(void)
{
    // Initialize I2C bus using project's i2c_bus wrapper
    GB_RESULT ret = i2c0.begin(&i2c0, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);
    if (ret != GB_OK) {
        GB_DEBUGE(DISP_TAG, "I2C bus initialization failed");
        return ESP_FAIL;
    }

    // Add PCA9536 device to I2C bus
    ret = i2c0.addDevice(&i2c0, PCA9536_ADDR, I2C_MASTER_FREQ_HZ);
    if (ret != GB_OK) {
        GB_DEBUGE(DISP_TAG, "Failed to add PCA9536 device to I2C bus");
        return ESP_FAIL;
    }

    GB_DEBUGI(DISP_TAG, "I2C master initialized (SDA: GPIO%d, SCL: GPIO%d, Freq: %d Hz)",
        I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);

    return ESP_OK;
}

static esp_err_t pca9536_write_reg(uint8_t reg_addr, uint8_t data)
{
    GB_RESULT ret = i2c0.writeByte(&i2c0, PCA9536_ADDR, reg_addr, data);
    if (ret != GB_OK) {
        GB_DEBUGE(DISP_TAG, "Write PCA9536 register 0x%02X failed", reg_addr);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t pca9536_read_reg(uint8_t reg_addr, uint8_t *data)
{
    GB_RESULT ret = i2c0.readByte(&i2c0, PCA9536_ADDR, reg_addr, data);
    if (ret != GB_OK) {
        GB_DEBUGE(DISP_TAG, "Read PCA9536 register 0x%02X failed", reg_addr);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t pca9536_init(void)
{
    esp_err_t ret;

    GB_DEBUGI(DISP_TAG, "Initializing PCA9536 GPIO expander (addr: 0x%02X)", PCA9536_ADDR);

    // Initialize I2C bus if not already done
    ret = pca9536_i2c_init();
        if (ret != ESP_OK) {
        return ret;
    }

    // Configure all pins as outputs (0 = output, 1 = input)
    ret = pca9536_write_reg(PCA9536_CONFIG_REG, 0x00);
    if (ret != ESP_OK) {
        GB_DEBUGE(DISP_TAG, "Failed to configure pins as outputs");
        return ret;
    }

    GB_DEBUGI(DISP_TAG, "PCA9536 initialized successfully (all pins output, low)");

    return ESP_OK;
}

esp_err_t pca9536_set_pin_high(uint8_t pin_mask)
{
    uint8_t current_state;
    esp_err_t ret;

    // Read current output state
    ret = pca9536_read_reg(PCA9536_OUTPUT_REG, &current_state);
        if (ret != ESP_OK) {
        return ret;
    }

    // Set specified pins high
    uint8_t new_state = current_state | pin_mask;
    ret = pca9536_write_reg(PCA9536_OUTPUT_REG, new_state);

    if (ret == ESP_OK) {
        GB_DEBUGI(DISP_TAG, "Set pins 0x%02X high (state: 0x%02X -> 0x%02X)",
            pin_mask, current_state, new_state);
    }

    return ret;
}

esp_err_t pca9536_set_pin_low(uint8_t pin_mask)
{
    uint8_t current_state;
    esp_err_t ret;

    // Read current output state
    ret = pca9536_read_reg(PCA9536_OUTPUT_REG, &current_state);
        if (ret != ESP_OK) {
        return ret;
    }

    // Set specified pins low
    uint8_t new_state = current_state & ~pin_mask;
    ret = pca9536_write_reg(PCA9536_OUTPUT_REG, new_state);

    if (ret == ESP_OK) {
        GB_DEBUGI(DISP_TAG, "Set pins 0x%02X low (state: 0x%02X -> 0x%02X)",
            pin_mask, current_state, new_state);
    }

    return ret;
}

esp_err_t pca9536_read_pins(uint8_t *state)
{
    return pca9536_read_reg(PCA9536_INPUT_REG, state);
}

static esp_err_t st7701_send_init_commands(void)
{
    esp_lcd_panel_io_handle_t io = disp_dsi_get_dbi_io();
    if (!io) {
        GB_DEBUGE(DISP_TAG, "DBI I/O not initialized");
        return ESP_FAIL;
    }

    const int cmd_count = sizeof(vendor_init_cmds) / sizeof(st7701_init_cmd_t);

    GB_DEBUGI(DISP_TAG, "Sending %d initialization commands...", cmd_count);

    for (int i = 0; i < cmd_count; i++) {
        const st7701_init_cmd_t *cmd = &vendor_init_cmds[i];

        // Send command with parameters
        esp_err_t ret = esp_lcd_panel_io_tx_param(io, cmd->cmd,
                                                    cmd->data, cmd->data_len);

        if (ret != ESP_OK) {
            GB_DEBUGE(DISP_TAG, "Failed to send command @%02X (index %d)", cmd->cmd, i);
            return ret;
        }
        //else
        //    GB_DEBUGI(DISP_TAG, "send command @%02X (index %d)", cmd->cmd, i);

        // Delay if required
        if (cmd->delay_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(cmd->delay_ms));
        }
    }

    GB_DEBUGI(DISP_TAG, "Initialization commands sent successfully");
    return ESP_OK;
}

void st7701_enable_backlight(bool enable)
{
    if (enable) {
        pca9536_set_pin_high(1 << ST7701_PCA9536_BACKLIGHT_PIN);
        GB_DEBUGI(DISP_TAG, "Backlight ON");
    } else {
        pca9536_set_pin_low(1 << ST7701_PCA9536_BACKLIGHT_PIN);
        GB_DEBUGI(DISP_TAG, "Backlight OFF");
    }
}

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

static void st7701_hardware_reset(void)
{
    GB_DEBUGI(DISP_TAG, "Performing hardware reset via PCA9536 PIN%d",
            ST7701_PCA9536_RESET_PIN);

    // Reset sequence: high -> low (10ms) -> high (50ms)
    pca9536_set_pin_high(1 << ST7701_PCA9536_RESET_PIN);
    vTaskDelay(pdMS_TO_TICKS(10));

    pca9536_set_pin_low(1 << ST7701_PCA9536_RESET_PIN);
    vTaskDelay(pdMS_TO_TICKS(30));

    pca9536_set_pin_high(1 << ST7701_PCA9536_RESET_PIN);
    vTaskDelay(pdMS_TO_TICKS(50));

    GB_DEBUGI(DISP_TAG, "Hardware reset completed");
}

esp_err_t st7701_register_flush_callback(void)
{
    if (!s_panel_handle) {
        GB_DEBUGE(DISP_TAG, "Cannot register callback - panel not initialized");
        return ESP_FAIL;
    }

    // Get the LVGL driver handle from lvgl_driver layer
    // This is set when lvgl_driver_flush() is called
    extern lv_disp_drv_t* lvgl_driver_get_disp_drv(void);
    lv_disp_drv_t *drv = lvgl_driver_get_disp_drv();
    if (!drv) {
        GB_DEBUGW(DISP_TAG, "LVGL driver not available yet for callback registration");
        return ESP_ERR_INVALID_STATE;
    }

    // Register the DPI callback with LVGL driver as user context
    esp_err_t ret = disp_dsi_register_flush_callback(s_panel_handle, drv);
    if (ret == ESP_OK) {
        GB_DEBUGE(DISP_TAG, "DPI flush callback registered for async operation");
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
        ST7701_HOR_RES, ST7701_VER_RES, ST7701_DSI_LANES,
        ST7701_LANE_BITRATE_MBPS);

    pca9536_init();

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
            .h_size = ST7701_HOR_RES,
            .v_size = ST7701_VER_RES,
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

    // Step 3: Software reset
    {
        esp_lcd_panel_io_handle_t io = disp_dsi_get_dbi_io();
        if (io) {
            esp_lcd_panel_io_tx_param(io, LCD_CMD_SWRESET, NULL, 0);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }

    // Step 4: Hardware reset via PCA9536 (use raw ticks like demo)
    st7701_hardware_reset();

    // Step 5: check display connect
    {
        esp_lcd_panel_io_handle_t io = disp_dsi_get_dbi_io();
        if (io) {
            esp_lcd_panel_io_tx_param(io, LCD_CMD_SWRESET, NULL, 0);
            vTaskDelay(pdMS_TO_TICKS(150));

            esp_lcd_panel_io_tx_param(io, LCD_CMD_SLPOUT, NULL, 0);
            vTaskDelay(pdMS_TO_TICKS(120));

            uint8_t id_data[4] = {0};
            esp_lcd_panel_io_rx_param(io, 0xA1, id_data, 3);

            uint8_t power_mode[2] = {0};
            esp_lcd_panel_io_rx_param(io, 0x0A, power_mode, 1);

            GB_DEBUGI(DISP_TAG, "Display ID: 0x%02x 0x%02x 0x%02x, Power: 0x%02x",
                id_data[0], id_data[1], id_data[2], power_mode[0]);
        }
    }

    // Step 6: Send vendor init commands via DBI (BEFORE creating DPI panel)
    ret = st7701_send_init_commands();
    if (ret != ESP_OK) {
        GB_DEBUGE(DISP_TAG, "Failed to send initialization commands");
        return;
    }

    // Step 7: Initialize DPI panel (starts video stream)
    ret = esp_lcd_panel_init(s_panel_handle);
    if (ret != ESP_OK) {
        GB_DEBUGE(DISP_TAG, "Failed to initialize DPI panel");
        return;
    }

    GB_DEBUGI(DISP_TAG, "DPI panel initialized");

    // Step 8: Enable backlight
    st7701_enable_backlight(true);

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
    esp_err_t ret = esp_lcd_panel_draw_bitmap(s_panel_handle, x1, y1, x2, y2, color_map);

    if (ret != ESP_OK) {
        GB_DEBUGE(DISP_TAG, "Failed to draw bitmap");
    }
}
