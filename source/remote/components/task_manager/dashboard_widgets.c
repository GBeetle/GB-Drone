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

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "dashboard_widgets.h"
#include "log_sys.h"
#include "sdkconfig.h"
#include "disp_driver.h"
#include "esp_heap_caps.h"

/**********************
 *    DEFINES
 **********************/
#define DISP_TAG "DASHBOARD"

// Color definitions
#define COLOR_SKY lv_color_hex(0x87CEEB)
#define COLOR_GROUND lv_color_hex(0x4B5131)
#define COLOR_HORIZON lv_color_hex(0xFFFFFFFF)
#define COLOR_GOOD lv_color_hex(0x00C853)
#define COLOR_WARNING lv_color_hex(0xFFC107)
#define COLOR_DANGER lv_color_hex(0xF44336)
#define COLOR_NEUTRAL lv_color_hex(0x888888)
#define COLOR_TEXT_PRIMARY lv_color_hex(0xFFFFFFFF)
#define COLOR_TEXT_SECONDARY lv_color_hex(0xCCCCCC)
#define COLOR_BACKGROUND lv_color_hex(0x1E1E1E)

// Layout dimensions
#define ATTITUDE_SIZE 200
#define GAUGE_SIZE 85
#define SENSOR_ICON_SIZE 20

#if defined CONFIG_TFT_DISPLAY_PROTOCOL_DSI
#define ATTITUDE_CF   LV_COLOR_FORMAT_RGB888
#define ATTITUDE_BPP  3
#else
#define ATTITUDE_CF   LV_COLOR_FORMAT_RGB565
#define ATTITUDE_BPP  2
#endif

/**********************
 *    STATIC PROTOTYPES
 **********************/
static void create_attitude_indicator(lv_obj_t *parent);
static lv_obj_t *create_single_gauge(lv_obj_t *row, lv_obj_t **gauge_out,
    lv_obj_t **label_out, const char *text, int range_min, int range_max,
    int bg_start, int bg_end);
static void create_telemetry_gauges(lv_obj_t *parent);
static void create_sensor_indicator(lv_obj_t *parent, const char *label_text,
    const char *icon, lv_obj_t **led_out, lv_obj_t **label_out);
static void create_sensor_status_row(lv_obj_t *parent);
static void create_flight_status_bar(lv_obj_t *parent);
static void update_attitude_indicator(float pitch, float roll);
static void update_telemetry_gauges(flight_data_t *data);
static void update_sensor_status(flight_data_t *data);
static void update_flight_status(flight_data_t *data);
static void update_led_color(lv_obj_t *led, sensor_status_t status);

/**********************
 *    STATIC VARIABLES
 **********************/
static lv_obj_t *dashboard_container = NULL;
static lv_obj_t *attitude_canvas = NULL;
static lv_obj_t *alt_gauge = NULL;
static lv_obj_t *speed_gauge = NULL;
static lv_obj_t *dist_gauge = NULL;
static lv_obj_t *heading_gauge = NULL;
static lv_obj_t *alt_label = NULL;
static lv_obj_t *speed_label = NULL;
static lv_obj_t *dist_label = NULL;
static lv_obj_t *heading_label = NULL;

// Sensor status LEDs
static lv_obj_t *gps_led = NULL;
static lv_obj_t *imu_led = NULL;
static lv_obj_t *mag_led = NULL;
static lv_obj_t *baro_led = NULL;
static lv_obj_t *radio_led = NULL;
static lv_obj_t *gps_detail_label = NULL;
static lv_obj_t *imu_detail_label = NULL;
static lv_obj_t *mag_detail_label = NULL;
static lv_obj_t *baro_detail_label = NULL;
static lv_obj_t *radio_detail_label = NULL;

// Flight status labels
static lv_obj_t *connection_label = NULL;
static lv_obj_t *mode_label = NULL;
static lv_obj_t *arm_label = NULL;
static lv_obj_t *freq_label = NULL;

static flight_data_t current_flight_data = {0};
static lv_color_t *attitude_buffer = NULL;

/**********************
 *    PUBLIC FUNCTIONS
 **********************/
void dashboard_create(lv_obj_t *parent)
{
    GB_DEBUGI(DISP_TAG, "Creating mixed dashboard");

    // Create main dashboard container
    dashboard_container = lv_obj_create(parent);
    lv_obj_set_size(dashboard_container, LV_PCT(100), LV_PCT(100));
    lv_obj_set_align(dashboard_container, LV_ALIGN_TOP_MID);
    lv_obj_set_pos(dashboard_container, 0, 0);
    lv_obj_set_style_bg_color(dashboard_container, COLOR_BACKGROUND, 0);
    lv_obj_set_style_border_width(dashboard_container, 0, 0);
    lv_obj_set_style_pad_all(dashboard_container, 4, 0);
    lv_obj_set_style_pad_row(dashboard_container, 4, 0);
    lv_obj_set_scrollbar_mode(dashboard_container, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(dashboard_container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(dashboard_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(dashboard_container, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    // Create UI sections
    create_attitude_indicator(dashboard_container);
    create_telemetry_gauges(dashboard_container);
    create_sensor_status_row(dashboard_container);
    create_flight_status_bar(dashboard_container);

    // Initialize with demo data
    dashboard_init_demo_data();

    GB_DEBUGI(DISP_TAG, "Mixed dashboard created successfully");
}

void dashboard_hide(void)
{
    if (dashboard_container != NULL) {
        lv_obj_add_flag(dashboard_container, LV_OBJ_FLAG_HIDDEN);
        GB_DEBUGI(DISP_TAG, "Dashboard hidden");
    }
}

void dashboard_show(void)
{
    if (dashboard_container != NULL) {
        lv_obj_clear_flag(dashboard_container, LV_OBJ_FLAG_HIDDEN);
        GB_DEBUGI(DISP_TAG, "Dashboard shown");
    }
}

void dashboard_update(flight_data_t *data)
{
    if (data == NULL) return;

    current_flight_data = *data;

    update_attitude_indicator(data->pitch, data->roll);
    update_telemetry_gauges(data);
    update_sensor_status(data);
    update_flight_status(data);
}

flight_data_t* dashboard_get_flight_data(void)
{
    return &current_flight_data;
}

void dashboard_init_demo_data(void)
{
    current_flight_data.pitch = 0.0f;
    current_flight_data.roll = 0.0f;
    current_flight_data.yaw = 0.0f;
    current_flight_data.altitude = 0;
    current_flight_data.speed = 0;
    current_flight_data.distance_home = 0;
    current_flight_data.heading = 0;
    current_flight_data.gps_satellites = 0;
    current_flight_data.rssi = -100;
    current_flight_data.connected = false;
    current_flight_data.armed = false;
    strcpy(current_flight_data.flight_mode, "INIT");
    current_flight_data.gps_status = SENSOR_STATUS_UNKNOWN;
    current_flight_data.imu_status = SENSOR_STATUS_UNKNOWN;
    current_flight_data.mag_status = SENSOR_STATUS_UNKNOWN;
    current_flight_data.baro_status = SENSOR_STATUS_UNKNOWN;
    current_flight_data.radio_status = SENSOR_STATUS_UNKNOWN;

    dashboard_update(&current_flight_data);
}

/**********************
 *    STATIC FUNCTIONS
 **********************/
static void create_attitude_indicator(lv_obj_t *parent)
{
    // Create canvas for attitude indicator
    attitude_canvas = lv_canvas_create(parent);
    lv_obj_set_size(attitude_canvas, ATTITUDE_SIZE, ATTITUDE_SIZE);

    // Allocate buffer for canvas
    size_t buf_size = ATTITUDE_SIZE * ATTITUDE_SIZE * ATTITUDE_BPP;
    attitude_buffer = (lv_color_t *)heap_caps_aligned_alloc(64, buf_size, MALLOC_CAP_SPIRAM);

    if (attitude_buffer != NULL) {
        lv_canvas_set_buffer(attitude_canvas, attitude_buffer, ATTITUDE_SIZE, ATTITUDE_SIZE, ATTITUDE_CF);
        lv_canvas_fill_bg(attitude_canvas, COLOR_BACKGROUND, LV_OPA_COVER);

        // Draw initial horizon
        update_attitude_indicator(0.0f, 0.0f);

        GB_DEBUGI(DISP_TAG, "Attitude indicator created");
    } else {
        GB_DEBUGE(DISP_TAG, "Failed to allocate attitude canvas buffer");
    }
}

static lv_obj_t *create_single_gauge(lv_obj_t *row, lv_obj_t **gauge_out,
                                     lv_obj_t **label_out, const char *text, int range_min, int range_max,
                                     int bg_start, int bg_end)
{
    // Each gauge is wrapped in a container for label overlay
    lv_obj_t *wrap = lv_obj_create(row);
    lv_obj_set_size(wrap, GAUGE_SIZE, GAUGE_SIZE);
    lv_obj_set_style_bg_opa(wrap, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(wrap, 0, 0);
    lv_obj_set_style_pad_all(wrap, 0, 0);
    lv_obj_set_flex_grow(wrap, 1);

    *gauge_out = lv_arc_create(wrap);
    lv_obj_set_size(*gauge_out, GAUGE_SIZE, GAUGE_SIZE);
    lv_obj_center(*gauge_out);
    lv_arc_set_range(*gauge_out, range_min, range_max);
    lv_arc_set_value(*gauge_out, 0);
    lv_arc_set_bg_angles(*gauge_out, bg_start, bg_end);
    lv_obj_set_style_arc_color(*gauge_out, COLOR_GOOD, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(*gauge_out, 8, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(*gauge_out, 8, LV_PART_MAIN);
    lv_obj_clear_flag(*gauge_out, LV_OBJ_FLAG_CLICKABLE);

    *label_out = lv_label_create(wrap);
    lv_label_set_text(*label_out, text);
    lv_obj_set_style_text_align(*label_out, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(*label_out, COLOR_TEXT_PRIMARY, 0);
    lv_obj_center(*label_out);

    return wrap;
}

static void create_telemetry_gauges(lv_obj_t *parent)
{
    // Row container for all gauges
    lv_obj_t *gauge_row = lv_obj_create(parent);
    lv_obj_set_size(gauge_row, LV_PCT(100), GAUGE_SIZE);
    lv_obj_set_style_bg_opa(gauge_row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(gauge_row, 0, 0);
    lv_obj_set_style_pad_all(gauge_row, 0, 0);
    lv_obj_set_style_pad_column(gauge_row, 5, 0);
    lv_obj_set_flex_flow(gauge_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(gauge_row, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    create_single_gauge(gauge_row, &alt_gauge, &alt_label, "0\nALT", 0, 100, 135, 45);
    create_single_gauge(gauge_row, &speed_gauge, &speed_label, "0m/s\nSPD", 0, 20, 135, 45);
    create_single_gauge(gauge_row, &dist_gauge, &dist_label, "0m\nDIST", 0, 500, 135, 45);
    create_single_gauge(gauge_row, &heading_gauge, &heading_label, "0\nHEAD", 0, 359, 0, 360);

    GB_DEBUGI(DISP_TAG, "Telemetry gauges created");
}

static void create_sensor_indicator(lv_obj_t *parent, const char *label_text, const char *icon,
    lv_obj_t **led_out, lv_obj_t **label_out)
{
    // LED indicator
    *led_out = lv_led_create(parent);
    lv_obj_set_size(*led_out, 16, 16);
    lv_led_set_color(*led_out, COLOR_NEUTRAL);
    lv_led_off(*led_out);

    // Detail label
    *label_out = lv_label_create(parent);
    lv_label_set_text(*label_out, label_text);
    lv_obj_set_style_text_color(*label_out, COLOR_TEXT_SECONDARY, 0);
    lv_obj_set_style_text_font(*label_out, &lv_font_montserrat_16, 0);
}

static void create_sensor_status_row(lv_obj_t *parent)
{
    // Row container for all sensors
    lv_obj_t *sensor_row = lv_obj_create(parent);
    lv_obj_set_size(sensor_row, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_style_bg_color(sensor_row, lv_color_hex(0x2A2A2A), 0);
    lv_obj_set_style_border_color(sensor_row, COLOR_NEUTRAL, 0);
    lv_obj_set_style_border_width(sensor_row, 1, 0);
    lv_obj_set_style_pad_all(sensor_row, 4, 0);
    lv_obj_set_style_pad_column(sensor_row, 6, 0);
    lv_obj_set_flex_flow(sensor_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(sensor_row, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    create_sensor_indicator(sensor_row, "GPS", LV_SYMBOL_GPS, &gps_led, &gps_detail_label);
    create_sensor_indicator(sensor_row, "IMU", LV_SYMBOL_SETTINGS, &imu_led, &imu_detail_label);
    create_sensor_indicator(sensor_row, "MAG", LV_SYMBOL_IMAGE, &mag_led, &mag_detail_label);
    create_sensor_indicator(sensor_row, "BARO", LV_SYMBOL_LIST, &baro_led, &baro_detail_label);
    create_sensor_indicator(sensor_row, "RADIO", LV_SYMBOL_WIFI, &radio_led, &radio_detail_label);

    GB_DEBUGI(DISP_TAG, "Sensor status row created");
}

static void create_flight_status_bar(lv_obj_t *parent)
{
    lv_obj_t *container = lv_obj_create(parent);
    lv_obj_set_size(container, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_style_bg_color(container, lv_color_hex(0x2A2A2A), 0);
    lv_obj_set_style_border_color(container, COLOR_NEUTRAL, 0);
    lv_obj_set_style_border_width(container, 1, 0);
    lv_obj_set_style_pad_all(container, 4, 0);
    lv_obj_set_flex_flow(container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(container, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    connection_label = lv_label_create(container);
    lv_label_set_text(connection_label, LV_SYMBOL_CLOSE " DISC");
    lv_obj_set_style_text_color(connection_label, COLOR_DANGER, 0);

    arm_label = lv_label_create(container);
    lv_label_set_text(arm_label, LV_SYMBOL_STOP " DISARM");
    lv_obj_set_style_text_color(arm_label, COLOR_GOOD, 0);

    mode_label = lv_label_create(container);
    lv_label_set_text(mode_label, LV_SYMBOL_SETTINGS " INIT");
    lv_obj_set_style_text_color(mode_label, COLOR_TEXT_PRIMARY, 0);

    freq_label = lv_label_create(container);
    lv_label_set_text(freq_label, LV_SYMBOL_BLUETOOTH " 433MHz");
    lv_obj_set_style_text_color(freq_label, COLOR_TEXT_SECONDARY, 0);

    GB_DEBUGI(DISP_TAG, "Flight status bar created");
}

static void update_attitude_indicator(float pitch, float roll)
{
    if (attitude_canvas == NULL || attitude_buffer == NULL) return;

    int center_x = ATTITUDE_SIZE / 2;
    int center_y = ATTITUDE_SIZE / 2;
    float pitch_scale = 3.0f;
    int pitch_offset = (int)(pitch * pitch_scale);
    int horizon_y = center_y - pitch_offset;

    if (horizon_y < 0) horizon_y = 0;
    if (horizon_y > ATTITUDE_SIZE) horizon_y = ATTITUDE_SIZE;

    // Fill background
    lv_canvas_fill_bg(attitude_canvas, lv_color_hex(0x000000), LV_OPA_COVER);

    // Use a SINGLE layer for all draw operations (LVGL 9 best practice)
    lv_layer_t layer;
    lv_canvas_init_layer(attitude_canvas, &layer);

    // Draw sky (top half)
    if (horizon_y > 0) {
        lv_draw_rect_dsc_t sky_dsc;
        lv_draw_rect_dsc_init(&sky_dsc);
        sky_dsc.bg_color = COLOR_SKY;
        sky_dsc.bg_opa = LV_OPA_COVER;
        lv_area_t sky_area = {0, 0, ATTITUDE_SIZE - 1, horizon_y - 1};
        lv_draw_rect(&layer, &sky_dsc, &sky_area);
    }

    // Draw ground (bottom half)
    if (horizon_y < ATTITUDE_SIZE) {
        lv_draw_rect_dsc_t ground_dsc;
        lv_draw_rect_dsc_init(&ground_dsc);
        ground_dsc.bg_color = COLOR_GROUND;
        ground_dsc.bg_opa = LV_OPA_COVER;
        lv_area_t ground_area = {0, horizon_y, ATTITUDE_SIZE - 1, ATTITUDE_SIZE - 1};
        lv_draw_rect(&layer, &ground_dsc, &ground_area);
    }

    // Draw horizon line
    lv_draw_line_dsc_t line_dsc;
    lv_draw_line_dsc_init(&line_dsc);
    line_dsc.color = COLOR_HORIZON;
    line_dsc.width = 3;
    line_dsc.p1.x = 0;
    line_dsc.p1.y = horizon_y;
    line_dsc.p2.x = ATTITUDE_SIZE - 1;
    line_dsc.p2.y = horizon_y;
    lv_draw_line(&layer, &line_dsc);

    // Draw center aircraft symbol (yellow)
    line_dsc.color = lv_color_hex(0xFFFF00);
    line_dsc.width = 3;

    // Left wing
    line_dsc.p1.x = center_x - 40;
    line_dsc.p1.y = center_y;
    line_dsc.p2.x = center_x - 10;
    line_dsc.p2.y = center_y;
    lv_draw_line(&layer, &line_dsc);

    // Right wing
    line_dsc.p1.x = center_x + 10;
    line_dsc.p1.y = center_y;
    line_dsc.p2.x = center_x + 40;
    line_dsc.p2.y = center_y;
    lv_draw_line(&layer, &line_dsc);

    // Center dot
    line_dsc.p1.x = center_x;
    line_dsc.p1.y = center_y - 2;
    line_dsc.p2.x = center_x;
    line_dsc.p2.y = center_y + 2;
    lv_draw_line(&layer, &line_dsc);

    // Finish all draw operations at once
    lv_canvas_finish_layer(attitude_canvas, &layer);
}

static void update_telemetry_gauges(flight_data_t *data)
{
    char buf[32];

    // Update altitude
    lv_arc_set_value(alt_gauge, data->altitude);
    snprintf(buf, sizeof(buf), "%d\nALT", data->altitude);
    lv_label_set_text(alt_label, buf);

    if (data->altitude < 30)
        lv_obj_set_style_arc_color(alt_gauge, COLOR_GOOD, LV_PART_INDICATOR);
    else if (data->altitude < 60)
        lv_obj_set_style_arc_color(alt_gauge, COLOR_WARNING, LV_PART_INDICATOR);
    else
        lv_obj_set_style_arc_color(alt_gauge, COLOR_DANGER, LV_PART_INDICATOR);

    // Update speed
    lv_arc_set_value(speed_gauge, data->speed);
    snprintf(buf, sizeof(buf), "%dm/s\nSPD", data->speed);
    lv_label_set_text(speed_label, buf);

    if (data->speed < 10)
        lv_obj_set_style_arc_color(speed_gauge, COLOR_GOOD, LV_PART_INDICATOR);
    else if (data->speed < 15)
        lv_obj_set_style_arc_color(speed_gauge, COLOR_WARNING, LV_PART_INDICATOR);
    else
        lv_obj_set_style_arc_color(speed_gauge, COLOR_DANGER, LV_PART_INDICATOR);

    // Update distance
    lv_arc_set_value(dist_gauge, data->distance_home);
    snprintf(buf, sizeof(buf), "%dm\nDIST", data->distance_home);
    lv_label_set_text(dist_label, buf);

    if (data->distance_home < 100)
        lv_obj_set_style_arc_color(dist_gauge, COLOR_GOOD, LV_PART_INDICATOR);
    else if (data->distance_home < 300)
        lv_obj_set_style_arc_color(dist_gauge, COLOR_WARNING, LV_PART_INDICATOR);
    else
        lv_obj_set_style_arc_color(dist_gauge, COLOR_DANGER, LV_PART_INDICATOR);

    // Update heading
    lv_arc_set_value(heading_gauge, data->heading);
    snprintf(buf, sizeof(buf), "%d°\nHEAD", data->heading);
    lv_label_set_text(heading_label, buf);
}

static void update_led_color(lv_obj_t *led, sensor_status_t status)
{
    switch (status) {
    case SENSOR_STATUS_GOOD:
        lv_led_set_color(led, COLOR_GOOD);
        lv_led_on(led);
        break;
    case SENSOR_STATUS_WARNING:
        lv_led_set_color(led, COLOR_WARNING);
        lv_led_on(led);
        break;
    case SENSOR_STATUS_ERROR:
        lv_led_set_color(led, COLOR_DANGER);
        lv_led_on(led);
        break;
    default:
        lv_led_set_color(led, COLOR_NEUTRAL);
        lv_led_off(led);
        break;
    }
}

static void update_sensor_status(flight_data_t *data)
{
    // GPS
    update_led_color(gps_led, data->gps_status);

    // IMU
    update_led_color(imu_led, data->imu_status);

    // MAG
    update_led_color(mag_led, data->mag_status);

    // BARO
    update_led_color(baro_led, data->baro_status);

    // RADIO
    update_led_color(radio_led, data->radio_status);
}

static void update_flight_status(flight_data_t *data)
{
    char buf[64];

    // Connection status
    if (data->connected) {
        lv_label_set_text(connection_label, LV_SYMBOL_OK " CONN");
        lv_obj_set_style_text_color(connection_label, COLOR_GOOD, 0);
    } else {
        lv_label_set_text(connection_label, LV_SYMBOL_CLOSE " DISC");
        lv_obj_set_style_text_color(connection_label, COLOR_DANGER, 0);
    }

    // Arm status
    if (data->armed) {
        lv_label_set_text(arm_label, LV_SYMBOL_WARNING " ARMED");
        lv_obj_set_style_text_color(arm_label, COLOR_DANGER, 0);
    } else {
        lv_label_set_text(arm_label, LV_SYMBOL_STOP " DISARM");
        lv_obj_set_style_text_color(arm_label, COLOR_GOOD, 0);
    }

    // Flight mode
    snprintf(buf, sizeof(buf), LV_SYMBOL_SETTINGS "%s", data->flight_mode);
    lv_label_set_text(mode_label, buf);
}
