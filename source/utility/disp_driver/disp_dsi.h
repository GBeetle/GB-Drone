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

#ifndef DISP_DSI_H
#define DISP_DSI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"

#if defined(CONFIG_IDF_TARGET_ESP32P4) && defined(CONFIG_TFT_DISPLAY_PROTOCOL_DSI)

#include "esp_lcd_panel_ops.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_io.h"

// DSI PHY power configuration for ESP32-P4
#define DISP_DSI_PHY_LDO_CHAN 3    // LDO_V03 channel
#define DISP_DSI_PHY_LDO_VOLTAGE_MV 2500   // 2.5V for VDD_MIPI_DPHY

esp_err_t disp_dsi_init(void);

esp_err_t disp_dsi_create_bus(const esp_lcd_dsi_bus_config_t *bus_config);

esp_lcd_panel_io_handle_t disp_dsi_get_dbi_io(void);

esp_lcd_panel_handle_t disp_dsi_get_dpi_panel(void);

esp_lcd_dsi_bus_handle_t disp_dsi_get_bus(void);

void disp_dsi_set_dpi_panel(esp_lcd_panel_handle_t panel);

esp_err_t disp_dsi_register_flush_callback(esp_lcd_panel_handle_t panel, void *user_ctx);

#ifdef __cplusplus
}
#endif

#endif // CONFIG_IDF_TARGET_ESP32P4 && CONFIG_TFT_DISPLAY_PROTOCOL_DSI