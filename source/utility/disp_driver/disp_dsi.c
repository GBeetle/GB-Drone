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

#include "disp_dsi.h"

#include "esp_log.h"
#include "esp_check.h"
#include "esp_ldo_regulator.h"
#include "esp_lcd_mipi_dsi.h"
#include "driver/gpio.h"
#include "lvgl.h"
#include "log_sys.h"
#include "error_handle.h"

// Static handles for DSI bus and panels
static esp_lcd_dsi_bus_handle_t s_mipi_dsi_bus = NULL;
static esp_lcd_panel_io_handle_t s_mipi_dbi_io = NULL;
static esp_lcd_panel_handle_t s_mipi_dpi_panel = NULL;
static esp_ldo_channel_handle_t s_ldo_mipi_phy = NULL;

/**
 * @brief DPI panel event callback - notifies LVGL when flush is ready
 */
static bool disp_dsi_notify_lvgl_flush_ready(esp_lcd_panel_handle_t panel,
    esp_lcd_dpi_panel_event_data_t *edata,
    void *user_ctx)
{
    lv_display_t *disp = (lv_display_t *)user_ctx;
    if (disp) {
        lv_disp_flush_ready(disp);
    }
    return false;
}

static esp_err_t disp_dsi_enable_phy_power(void)
{
    esp_ldo_channel_config_t ldo_mipi_phy_config = {
        .chan_id = DISP_DSI_PHY_LDO_CHAN,
        .voltage_mv = DISP_DSI_PHY_LDO_VOLTAGE_MV,
    };

    esp_err_t ret = esp_ldo_acquire_channel(&ldo_mipi_phy_config, &s_ldo_mipi_phy);
    if (ret != ESP_OK) {
        GB_DEBUGE(DISP_TAG, "Failed to acquire LDO channel for MIPI DSI PHY");
        return ret;
    }

    GB_DEBUGI(DISP_TAG, "MIPI DSI PHY powered on (LDO channel %d, %d mV)",
        DISP_DSI_PHY_LDO_CHAN, DISP_DSI_PHY_LDO_VOLTAGE_MV);

    return ESP_OK;
}

esp_err_t disp_dsi_init(void)
{
    esp_err_t ret = ESP_OK;

    // Enable DSI PHY power first
    ret = disp_dsi_enable_phy_power();
    if (ret != ESP_OK) {
        GB_DEBUGE(DISP_TAG, "Failed to enable DSI PHY power");
        return ret;
    }

    // Create MIPI DSI bus (configuration comes from controller driver)
    // Note: The actual bus creation is deferred until controller init
    // because we need controller-specific parameters (lane count, bitrate)

    GB_DEBUGI(DISP_TAG, "DSI bus abstraction layer initialized");
    return ESP_OK;
}

esp_err_t disp_dsi_create_bus(const esp_lcd_dsi_bus_config_t *bus_config)
{
    if (s_mipi_dsi_bus != NULL) {
        GB_DEBUGE(DISP_TAG, "DSI bus already created");
        return ESP_OK;
    }

    esp_err_t ret = esp_lcd_new_dsi_bus(bus_config, &s_mipi_dsi_bus);
    if (ret != ESP_OK) {
        GB_DEBUGE(DISP_TAG, "Failed to create MIPI DSI bus");
        return ret;
    }

    GB_DEBUGI(DISP_TAG, "MIPI DSI bus created (lanes: %d, bitrate: %d Mbps)",
        bus_config->num_data_lanes, bus_config->lane_bit_rate_mbps);

    // Create DBI I/O panel for commands
    esp_lcd_dbi_io_config_t dbi_config = {
        .virtual_channel = 0,
        .lcd_cmd_bits = 8,   // 8-bit command
        .lcd_param_bits = 8, // 8-bit parameters
    };

    ret = esp_lcd_new_panel_io_dbi(s_mipi_dsi_bus, &dbi_config, &s_mipi_dbi_io);
    if (ret != ESP_OK) {
        GB_DEBUGE(DISP_TAG, "Failed to create DBI I/O panel");
        return ret;
    }

    GB_DEBUGI(DISP_TAG, "MIPI DBI I/O panel created");

    return ESP_OK;
}

esp_lcd_panel_io_handle_t disp_dsi_get_dbi_io(void)
{
    return s_mipi_dbi_io;
}

esp_lcd_panel_handle_t disp_dsi_get_dpi_panel(void)
{
    return s_mipi_dpi_panel;
}

esp_lcd_dsi_bus_handle_t disp_dsi_get_bus(void)
{
    return s_mipi_dsi_bus;
}

void disp_dsi_set_dpi_panel(esp_lcd_panel_handle_t panel)
{
    s_mipi_dpi_panel = panel;
    GB_DEBUGI(DISP_TAG, "DPI panel handle registered");
}

esp_err_t disp_dsi_register_flush_callback(esp_lcd_panel_handle_t panel,
    void *user_ctx)
{
    esp_lcd_dpi_panel_event_callbacks_t cbs = {
        .on_color_trans_done = disp_dsi_notify_lvgl_flush_ready,
    };

    esp_err_t ret = esp_lcd_dpi_panel_register_event_callbacks(panel, &cbs, user_ctx);
    if (ret != ESP_OK) {
        GB_DEBUGE(DISP_TAG, "Failed to register DPI panel event callbacks");
        return ret;
    }

    GB_DEBUGI(DISP_TAG, "DPI panel flush callback registered");

    return ESP_OK;
}
