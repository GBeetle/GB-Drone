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

#ifndef _DASHBOARD_WIDGETS_H__
#define _DASHBOARD_WIDGETS_H__

#include <stdlib.h>
#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************
 *    TYPEDEFS
 ******************************/

typedef enum {
    SENSOR_STATUS_GOOD = 0,
    SENSOR_STATUS_WARNING,
    SENSOR_STATUS_ERROR,
    SENSOR_STATUS_UNKNOWN
} sensor_status_t;

typedef struct {
    float pitch;            // degrees (-90 to +90)
    float roll;             // degrees (-180 to +180)
    float yaw;              // degrees (0-359)
    int altitude;           // meters
    int speed;              // m/s
    int distance_home;      // meters
    int heading;            // degrees (0-359)
    int gps_satellites;     // count
    int rssi;               // dBm
    bool connected;
    bool armed;
    char flight_mode[16];   // "STABILIZE", "ALTITUDE", "POSITION"
    sensor_status_t gps_status;
    sensor_status_t imu_status;
    sensor_status_t mag_status;
    sensor_status_t baro_status;
    sensor_status_t radio_status;
} flight_data_t;

/**
 * @brief Create the complete mixed dashboard
 * @param parent Parent LVGL object (tab content)
 */
void dashboard_create(lv_obj_t *parent);

/**
 * @brief Hide dashboard (when 3D mode is active)
 */
void dashboard_hide(void);

/**
 * @brief Show dashboard (when returning from 3D mode)
 */
void dashboard_show(void);

/**
 * @brief Update dashboard with new flight data
 * @param data Pointer to flight data structure
 */
void dashboard_update(flight_data_t *data);

/**
 * @brief Get current flight data (for external access)
 * @return Pointer to current flight data
 */
flight_data_t* dashboard_get_flight_data(void);

/**
 * @brief Initialize dashboard with demo data (for testing)
 */
void dashboard_init_demo_data(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* end of include guard: _DASHBOARD_WIDGETS_H__ */
