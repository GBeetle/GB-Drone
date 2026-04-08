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

#if defined(CONFIG_IDF_TARGET_ESP32P4) && defined(CONFIG_TFT_DISPLAY_CONTROLLER_ST7701)

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_commands.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_log.h"
#include "esp_check.h"
#include "../dispi_dsi.h"
#include "driver/i2c.h"

#define I2C_MASTER_NUM    I2C_NUM_0
#define I2C_MASTER_SDA_IO    22    // SDA pin (from screen project)
#define I2C_MASTER_SCL_IO    21    // SCL pin (from screen project)
#define I2C_MASTER_FREQ_HZ  100000    // 100 kHz
#define I2C_MASTER_TIMEOUT_MS 1000

static const char *TAG = "st7701";

// Static handles
static esp_lcd_panel_handle_t s_panel_handle = NULL;
static lv_disp_drv_t *s_disp_drv = NULL;

/**
 * @brief ST7701 initialization command structure
 */
typedef struct {
    uint8_t cmd;
    const uint8_t *data;
    uint8_t data_len;
    uint8_t delay_ms;
} st7701_init_cmd_t;

static const ST7701_lcd_init_cmd_t vendor_init_cmds[] = {
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
    if (s_i2c_initialized) {
        return ESP_OK;
    }

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_DISABLE, // External pullups on board
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_DISABLE, // External pullups on board
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOG(TAG, "I2C param config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOG(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    s_i2c_initialized = true;
    ESP_LOG(TAG, "I2C master initialized (SDA: GPIO%d, SCL: GPIO%d, Freq: %d Hz)",
    I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);

    return ESP_OK;
}

static esp_err_t pca9536_write_reg(uint8_t reg_addr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9536_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd,
        pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOG(TAG, "Write register 0x%02X failed: %s", reg_addr, esp_err_to_name(ret));
    }

    return ret;
}

static esp_err_t pca9536_read_reg(uint8_t reg_addr, uint8_t *data)
{
    // Write register address
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9536_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd,
        pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOG(TAG, "Write register address 0x%02X failed: %s",
            reg_addr, esp_err_to_name(ret));
        return ret;
    }

    // Read register data
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9536_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd,
        pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOG(TAG, "Read register 0x%02X failed: %s", reg_addr, esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t pca9536_init(void)
{
    esp_err_t ret;

    ESP_LOG(TAG, "Initializing PCA9536 GPIO expander (addr: 0x%02X)", PCA9536_ADDR);

    // Initialize I2C bus if not already done
    ret = pca9536_i2c_init();
        if (ret != ESP_OK) {
        return ret;
    }

    // Configure all pins as outputs (0 = output, 1 = input)
    ret = pca9536_write_reg(PCA9536_CONFIG_REG, 0x00);
    if (ret != ESP_OK) {
        ESP_LOG(TAG, "Failed to configure pins as outputs");
        return ret;
    }

    // Set initial state to all low
    ret = pca9536_write_reg(PCA9536_OUTPUT_REG, 0x00);
    if (ret != ESP_OK) {
        ESP_LOG(TAG, "Failed to set initial output state");
        return ret;
    }

    ESP_LOG(TAG, "PCA9536 initialized successfully (all pins output, low)");

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
    ESP_LOG(TAG, "Set pins 0x%02X high (state: 0x%02X -> 0x%02X)",
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
    ESP_LOG(TAG, "Set pins 0x%02X low (state: 0x%02X -> 0x%02X)",
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
        ESP_LOG(TAG, "DBI I/O not initialized");
        return ESP_FAIL;
    }

    const int cmd_count = sizeof(vendor_init_cmds) / sizeof(st7701_init_cmd_t);

    ESP_LOG(TAG, "Sending %d initialization commands...", cmd_count);

    for (int i = 0; i < cmd_count; i++) {
        const st7701_init_cmd_t *cmd = &vendor_init_cmds[i];

        // Send command with parameters
        esp_err_t ret = esp_lcd_panel_io_tx_param(io, cmd->cmd,
                                                    cmd->data, cmd->data_len);

        if (ret != ESP_OK) {
            ESP_LOG(TAG, "Failed to send command @%02X (index %d)", cmd->cmd, i);
            return ret;
        }

        // Delay if required
        if (cmd->delay_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(cmd->delay_ms));
        }
    }

    ESP_LOG(TAG, "Initialization commands sent successfully");
    return ESP_OK;
}

void st7701_enable_backlight(bool enable)
{
    if (enable) {
        pca9536_set_pin_high(1 << ST7701_PCA9536_BACKLIGHT_PIN);
        ESP_LOG(TAG, "Backlight ON");
    } else {
        pca9536_set_pin_low(1 << ST7701_PCA9536_BACKLIGHT_PIN);
        ESP_LOG(TAG, "Backlight OFF");
    }
}

void st7701_sleep_in(void)
{
    esp_lcd_panel_io_handle_t io = disp_dsi_get_dbi_io();
    if (io) {
        esp_lcd_panel_io_tx_param(io, LCD_CMD_SLPIN, NULL, 0);
        vTaskDelay(pdMS_TO_TICKS(120));
        ESP_LOG(TAG, "Entered sleep mode");
    }
}

void st7701_sleep_out(void)
{
    esp_lcd_panel_io_handle_t io = disp_dsi_get_dbi_io();
    if (io) {
        esp_lcd_panel_io_tx_param(io, LCD_CMD_SLPOUT, NULL, 0);
        vTaskDelay(pdMS_TO_TICKS(120));
        ESP_LOG(TAG, "Exited sleep mode");
    }
}

static void st7701_hardware_reset(void)
{
    ESP_LOG(TAG, "Performing hardware reset via PCA9536 PIN%d",
            ST7701_PCA9536_RESET_PIN);

    // Reset sequence: high -> low (10ms) -> high (50ms)
    pca9536_set_pin_high(1 << ST7701_PCA9536_RESET_PIN);
    vTaskDelay(pdMS_TO_TICKS(10));

    pca9536_set_pin_low(1 << ST7701_PCA9536_RESET_PIN);
    vTaskDelay(pdMS_TO_TICKS(30));

    pca9536_set_pin_high(1 << ST7701_PCA9536_RESET_PIN);
    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_LOG(TAG, "Hardware reset completed");
}


void st7701_init(void)
{
    esp_err_t ret;

    ESP_LOG(TAG, "Initializing ST7701 display controller");
    ESP_LOG(TAG, "Resolution: %dx%d, DSI lanes: %d, Bitrate: %d Mbps",
            ST7701_HOR_RES, ST7701_VER_RES, ST7701_DSI_LANES,
            ST7701_LANE_BITRATE_MBPS);

    pca9536_init();

    // Step 1: Create DSI bus
    esp_lcd_dsi_bus_config_t bus_config = {
        .bus_id = 0,
        .num_data_lanes = ST7701_DSI_LANES,
        .phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT,
        .lane_bit_rate_mbps = ST7701_LANE_BITRATE_MBPS,
    };

    ret = disp_dsi_create_bus(&bus_config);
    if (ret != ESP_OK) {
        ESP_LOG(TAG, "Failed to create DSI bus");
        return;
    }

    // Step 2: Create DPI panel configuration
    esp_lcd_dpi_panel_config_t dpi_config = {
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
        .dpi_clock_freq_mhz = ST7701_PIXEL_CLOCK_MHZ,
        .virtual_channel = 0,
        .pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB888,
        .num_fbs = 1,
        .video_timing = {
            .h_size = ST7701_HOR_RES,
            .v_size = ST7701_VER_RES,
            .hsync_backporch = ST7701_HSYNC_BACK_PORCH,
            .hsync_pulse_width = ST7701_HSYNC_PULSE_WIDTH,
            .hsync_frontporch = ST7701_HSYNC_FRONT_PORCH,
            .vsync_backporch = ST7701_VSYNC_BACK_PORCH,
            .vsync_pulse_width = ST7701_VSYNC_PULSE_WIDTH,
            .vsync_front_porch = ST7701_VSYNC_FRONT_PORCH,
        },
        .flags = {
            .use_dma2d = true, // Enable DMA2D acceleration
        }
    };

    // Step 3: Create DPI panel
    esp_lcd_dsi_bus_handle_t dsi_bus = disp_dsi_get_bus();
    ret = esp_lcd_new_panel_dpi(dsi_bus, &dpi_config, &s_panel_handle);
    if (ret != ESP_OK) {
        ESP_LOG(TAG, "Failed to create DPI panel");
        return;
    }

    // Register panel handle with DSI abstraction layer
    disp_dsi_set_dpi_panel(s_panel_handle);

    ESP_LOG(TAG, "DPI panel created successfully");

    // Step 4: Hardware reset via PCA9536
    st7701_hardware_reset();

    // Step 5: Initialize panel (sends init commands)
    ret = st7701_send_init_commands();
    if (ret != ESP_OK) {
        ESP_LOG(TAG, "Failed to send initialization commands");
        return;
    }

    // Step 6: Initialize DPI panel
    ret = esp_lcd_panel_init(s_panel_handle);
    if (ret != ESP_OK) {
        ESP_LOG(TAG, "Failed to initialize DPI panel");
        return;
    }

    // Step 7: Register flush callback (will be set later by LVGL)
    // The callback is registered in st7701_flush() when first called


    st7701_enable_backlight(true);

    ESP_LOGI(TAG, "ST7701 initialized successfully");
}

void st7701_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    if (!s_panel_handle) {
        ESP_LOG(TAG, "Panel not initialized");
        lv_disp_flush_ready(drv);
        return;
    }

    // Register callback on first flush (drv available now)
    static bool callback_registered = false;
    if (!callback_registered && drv) {
        esp_err_t ret = disp_dsi_register_flush_callback(s_panel_handle, drv);
        if (ret == ESP_OK) {
            callback_registered = true;
            ESP_LOGI(TAG, "Flush callback registered with LVGL driver");
        } else {
            ESP_LOGW(TAG, "Failed to register flush callback");
        }
    }

    // Convert LVGL v7 area to coordinates
    // LVGL v7 uses inclusive coordinates (x2, y2 are part of area)
    // esp_lcd expects exclusive end coordinates
    int x1 = area->x1;
    int y1 = area->y1;
    int x2 = area->x2 + 1;
    int y2 = area->y2 + 1;

    // Send pixel data to DPI panel
    // Note: lv_disp_flush_ready() will be called by DPI callback, not here
    esp_err_t ret = esp_lcd_panel_draw_bitmap(s_panel_handle, x1, y1, x2, y2, color_map);

    if (ret != ESP_OK) {
        ESP_LOG(TAG, "Failed to draw bitmap: %s", esp_err_to_name(ret));
        // Call flush ready to prevent LVGL from hanging
        lv_disp_flush_ready(drv);
    }

    // Store display driver for callback
    s_disp_drv = drv;
}

#endif // CONFIG_IDF_TARGET_ESP32P4 && CONFIG_TFT_DISPLAY_CONTROLLER_ST7701