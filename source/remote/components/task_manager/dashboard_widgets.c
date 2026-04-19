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

#ifndef LV_HOR_RES_MAX
#  ifdef CONFIG_LV_HOR_RES_MAX
#    define LV_HOR_RES_MAX CONFIG_LV_HOR_RES_MAX
#  else
#    define  LV_HOR_RES_MAX          (480)
#  endif
#endif
#ifndef LV_VER_RES_MAX
#  ifdef CONFIG_LV_VER_RES_MAX
#    define LV_VER_RES_MAX CONFIG_LV_VER_RES_MAX
#  else
#    define  LV_VER_RES_MAX          (320)
#  endif
#endif

/**********************
 *    DEFINES
 **********************/
#define DISP_TAG "DASHBOARD"

// Color definitions
#define COLOR_SKY lv_color_hex(0x87CEEB)
#define COLOR_GROUND lv_color_hex(0x4B5131) // Fixed: was 0xB4513
#define COLOR_HORIZON lv_color_hex(0xFFFFFFFF)
#define COLOR_GOOD lv_color_hex(0x00C853)
#define COLOR_WARNING lv_color_hex(0xFFC107)
#define COLOR_DANGER lv_color_hex(0xF44336)
#define COLOR_NEUTRAL lv_color_hex(0x888888)
#define COLOR_TEXT_PRIMARY lv_color_hex(0xFFFFFFFF)
#define COLOR_TEXT_SECONDARY lv_color_hex(0xCCCCCC)
#define COLOR_BACKGROUND lv_color_hex(0x1E1E1E)

// Layout dimensions
#define ATTITUDE_SIZE 240
#define GAUGE_SIZE 90
#define SENSOR_ICON_SIZE 20

/**********************
 *    STATIC PROTOTYPES
 **********************/
static void create_attitude_indicator(lv_obj_t *parent);
static void create_telemetry_gauges(lv_obj_t *parent);
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
    lv_obj_set_size(dashboard_container, LV_HOR_RES_MAX, LV_VER_RES_MAX - 30);
    lv_obj_set_pos(dashboard_container, 0, 0);
    lv_obj_set_style_bg_color(dashboard_container, COLOR_BACKGROUND, 0);
    lv_obj_set_style_border_width(dashboard_container, 0, 0);
    lv_obj_set_style_pad_all(dashboard_container, 5, 0);
    lv_obj_clear_flag(dashboard_container, LV_OBJ_FLAG_SCROLLABLE);

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
    lv_obj_align(attitude_canvas, LV_ALIGN_TOP_MID, 0, 10);

    // Allocate buffer for canvas
    size_t buf_size = ATTITUDE_SIZE * ATTITUDE_SIZE * sizeof(lv_color_t);
    attitude_buffer = (lv_color_t *)lv_malloc(buf_size);

    if (attitude_buffer != NULL) {
        lv_canvas_set_buffer(attitude_canvas, attitude_buffer, ATTITUDE_SIZE, ATTITUDE_SIZE, LV_COLOR_FORMAT_RGB565);
        lv_canvas_fill_bg(attitude_canvas, COLOR_BACKGROUND, LV_OPA_COVER);

        // Draw initial horizon
        update_attitude_indicator(0.0f, 0.0f);

        GB_DEBUGI(DISP_TAG, "Attitude indicator created");
    } else {
        GB_DEBUGE(DISP_TAG, "Failed to allocate attitude canvas buffer");
    }
}

static void create_telemetry_gauges(lv_obj_t *parent)
{
    int gauge_y = 270;
    int gauge_spacing = (LV_HOR_RES_MAX - 20) / 4;

    // --- Altitude Gauge ---
    alt_gauge = lv_arc_create(parent);
    lv_obj_set_size(alt_gauge, GAUGE_SIZE, GAUGE_SIZE);
    lv_obj_set_pos(alt_gauge, 10, gauge_y);
    lv_arc_set_range(alt_gauge, 0, 100);
    lv_arc_set_value(alt_gauge, 0);
    lv_arc_set_bg_angles(alt_gauge, 135, 45);
    lv_obj_set_style_arc_color(alt_gauge, COLOR_GOOD, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(alt_gauge, 8, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(alt_gauge, 8, LV_PART_MAIN); // Fixed: was bg_angle
    lv_obj_clear_flag(alt_gauge, LV_OBJ_FLAG_CLICKABLE);

    alt_label = lv_label_create(parent);
    lv_label_set_text(alt_label, "0m\nALT");
    lv_obj_set_style_text_align(alt_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align_to(alt_label, alt_gauge, LV_ALIGN_CENTER, 0, 0);

    // --- Speed Gauge ---
    speed_gauge = lv_arc_create(parent);
    lv_obj_set_size(speed_gauge, GAUGE_SIZE, GAUGE_SIZE);
    lv_obj_set_pos(speed_gauge, 10 + gauge_spacing, gauge_y);
    lv_arc_set_range(speed_gauge, 0, 20);
    lv_arc_set_value(speed_gauge, 0);
    lv_arc_set_bg_angles(speed_gauge, 135, 45);
    lv_obj_set_style_arc_color(speed_gauge, COLOR_GOOD, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(speed_gauge, 8, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(speed_gauge, 8, LV_PART_MAIN); // Fixed: was bg_angle
    lv_obj_clear_flag(speed_gauge, LV_OBJ_FLAG_CLICKABLE);

    speed_label = lv_label_create(parent);
    lv_label_set_text(speed_label, "0m/s\nSPD");
    lv_obj_set_style_text_align(speed_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align_to(speed_label, speed_gauge, LV_ALIGN_CENTER, 0, 0);

    // --- Distance Gauge ---
    dist_gauge = lv_arc_create(parent);
    lv_obj_set_size(dist_gauge, GAUGE_SIZE, GAUGE_SIZE);
    lv_obj_set_pos(dist_gauge, 10 + gauge_spacing * 2, gauge_y);
    lv_arc_set_range(dist_gauge, 0, 500);
    lv_arc_set_value(dist_gauge, 0);
    lv_arc_set_bg_angles(dist_gauge, 135, 45);
    lv_obj_set_style_arc_color(dist_gauge, COLOR_GOOD, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(dist_gauge, 8, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(dist_gauge, 8, LV_PART_MAIN); // Fixed: was dist_gague
    lv_obj_clear_flag(dist_gauge, LV_OBJ_FLAG_CLICKABLE);

    dist_label = lv_label_create(parent);
    lv_label_set_text(dist_label, "0m\nDIST");
    lv_obj_set_style_text_align(dist_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align_to(dist_label, dist_gauge, LV_ALIGN_CENTER, 0, 0);

    // --- Heading Gauge ---
    heading_gauge = lv_arc_create(parent);
    lv_obj_set_size(heading_gauge, GAUGE_SIZE, GAUGE_SIZE);
    lv_obj_set_pos(heading_gauge, 10 + gauge_spacing * 3, gauge_y);
    lv_arc_set_range(heading_gauge, 0, 359);
    lv_arc_set_value(heading_gauge, 0);
    lv_arc_set_bg_angles(heading_gauge, 0, 360);
    lv_obj_set_style_arc_color(heading_gauge, COLOR_GOOD, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(heading_gauge, 8, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(heading_gauge, 8, LV_PART_MAIN); // Fixed: was heading-gauge
    lv_obj_clear_flag(heading_gauge, LV_OBJ_FLAG_CLICKABLE);

    heading_label = lv_label_create(parent);
    //lv_label_set_text(heading_label, "0" LV_SYMBOL_DEGREES "\nHEAD");
    lv_label_set_text(heading_label, "0\nHEAD");
    lv_obj_set_style_text_align(heading_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align_to(heading_label, heading_gauge, LV_ALIGN_CENTER, 0, 0);

    GB_DEBUGI(DISP_TAG, "Telemetry gauges created");
}

static void create_sensor_indicator(lv_obj_t *parent, const char *label_text, const char *icon,
    int x_pos, int y_pos, lv_obj_t **led_out, lv_obj_t **label_out)
{
    lv_obj_t *container = lv_obj_create(parent);
    lv_obj_set_size(container, 90, 90);
    lv_obj_set_pos(container, x_pos, y_pos);
    lv_obj_set_style_bg_opa(container, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(container, 0, 0);
    lv_obj_set_flex_flow(container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    // Icon label
    lv_obj_t *icon_label = lv_label_create(container);
    lv_label_set_text(icon_label, icon);
    lv_obj_set_style_text_font(icon_label, &lv_font_montserrat_16, 0); // Fixed: style_text -> style_text_font

    // LED indicator
    *led_out = lv_led_create(container);
    lv_obj_set_size(*led_out, 16, 16);
    lv_led_set_color(*led_out, COLOR_NEUTRAL);
    lv_led_off(*led_out);

    // Detail label
    *label_out = lv_label_create(container);
    lv_label_set_text(*label_out, label_text);
    lv_obj_set_style_text_color(*label_out, COLOR_TEXT_SECONDARY, 0);
    lv_obj_set_style_text_font(*label_out, &lv_font_montserrat_16, 0);
}

static void create_sensor_status_row(lv_obj_t *parent)
{
    int sensor_y = 410;
    int sensor_spacing = LV_HOR_RES_MAX / 5;

    create_sensor_indicator(parent, "GPS", LV_SYMBOL_GPS, 5, sensor_y, &gps_led, &gps_detail_label);
    create_sensor_indicator(parent, "IMU", LV_SYMBOL_SETTINGS, sensor_spacing, sensor_y, &imu_led, &imu_detail_label);
    create_sensor_indicator(parent, "MAG", LV_SYMBOL_IMAGE, sensor_spacing * 2, sensor_y, &mag_led, &mag_detail_label);
    create_sensor_indicator(parent, "BARO", LV_SYMBOL_LIST, sensor_spacing * 3, sensor_y, &baro_led, &baro_detail_label);
    create_sensor_indicator(parent, "RADIO", LV_SYMBOL_WIFI, sensor_spacing * 4, sensor_y, &radio_led, &radio_detail_label);

    GB_DEBUGI(DISP_TAG, "Sensor status row created");
}

static void create_flight_status_bar(lv_obj_t *parent)
{
    int status_y = 520;

    lv_obj_t *container = lv_obj_create(parent);
    lv_obj_set_size(container, LV_HOR_RES_MAX - 10, 80);
    lv_obj_set_pos(container, 5, status_y);
    lv_obj_set_style_bg_color(container, lv_color_hex(0x2A2A2A), 0);
    lv_obj_set_style_border_color(container, COLOR_NEUTRAL, 0);
    lv_obj_set_style_border_width(container, 1, 0);
    lv_obj_set_style_pad_all(container, 10, 0);
    lv_obj_set_flex_flow(container, LV_FLEX_FLOW_COLUMN); // Fixed: was set_style_flow
    lv_obj_set_style_pad_row(container, 5, 0);

    // Row 1
    lv_obj_t *row1 = lv_obj_create(container);
    lv_obj_set_size(row1, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(row1, LV_OPA_TRANSP, 0); // Fixed: was bg_opaque
    lv_obj_set_style_border_width(row1, 0, 0);
    lv_obj_set_flex_flow(row1, LV_FLEX_FLOW_ROW); // Fixed: was set_style_flow
    lv_obj_set_flex_align(row1, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); // Fixed: was set_style_align

    connection_label = lv_label_create(row1);
    lv_label_set_text(connection_label, LV_SYMBOL_CLOSE " DISC");
    lv_obj_set_style_text_color(connection_label, COLOR_DANGER, 0);

    arm_label = lv_label_create(row1);
    lv_label_set_text(arm_label, LV_SYMBOL_STOP " DISARM");
    lv_obj_set_style_text_color(arm_label, COLOR_GOOD, 0);

    // Row 2
    lv_obj_t *row2 = lv_obj_create(container);
    lv_obj_set_size(row2, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(row2, LV_OPA_TRANSP, 0); // Fixed: was bg_opaque
    lv_obj_set_style_border_width(row2, 0, 0);
    lv_obj_set_flex_flow(row2, LV_FLEX_FLOW_ROW); // Fixed: was set_style_flow
    lv_obj_set_flex_align(row2, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); // Fixed: was set_style_align

    mode_label = lv_label_create(row2);
    lv_label_set_text(mode_label, LV_SYMBOL_SETTINGS " INIT");
    lv_obj_set_style_text_color(mode_label, COLOR_TEXT_PRIMARY, 0);

    freq_label = lv_label_create(row2);
    lv_label_set_text(freq_label, LV_SYMBOL_BLUETOOTH " 433MHz");
    lv_obj_set_style_text_color(freq_label, COLOR_TEXT_SECONDARY, 0);

    GB_DEBUGI(DISP_TAG, "Flight status bar created");
}

void canvas_draw_rect(lv_obj_t *canvas, int32_t x, int32_t y,
                      int32_t w, int32_t h, lv_draw_rect_dsc_t *dsc)
{
    lv_layer_t layer;
    lv_canvas_init_layer(canvas, &layer);

    lv_area_t coords = {
        .x1 = x,
        .y1 = y,
        .x2 = x + w - 1,
        .y2 = y + h - 1
    };

    lv_draw_rect(&layer, dsc, &coords);
    lv_canvas_finish_layer(canvas, &layer);
}

void canvas_draw_line(lv_obj_t *canvas, const lv_point_precise_t *points,
                      uint32_t point_cnt, lv_draw_line_dsc_t *dsc)
{
    if (point_cnt < 2) return;

    lv_layer_t layer;
    lv_canvas_init_layer(canvas, &layer);

    dsc->p1 = points[0];
    dsc->p2 = points[1];

    lv_draw_line(&layer, dsc);
    lv_canvas_finish_layer(canvas, &layer);
}

static void update_attitude_indicator(float pitch, float roll)
{
    if (attitude_canvas == NULL || attitude_buffer == NULL) return;

    int center_x = ATTITUDE_SIZE / 2;
    int center_y = ATTITUDE_SIZE / 2;
    float pitch_scale = 3.0f;
    int pitch_offset = (int)(pitch * pitch_scale);

    // Fill background
    lv_canvas_fill_bg(attitude_canvas, lv_color_hex(0x000000), LV_OPA_COVER);

    // Draw sky (top half)
    lv_draw_rect_dsc_t sky_dsc;
    lv_draw_rect_dsc_init(&sky_dsc);
    sky_dsc.bg_color = COLOR_SKY; // Fixed: was COLOR SKY
    sky_dsc.bg_opa = LV_OPA_COVER; // Fixed: was bg_opaa
    lv_area_t sky_area = {0, 0, ATTITUDE_SIZE - 1, center_y - pitch_offset};
    canvas_draw_rect(attitude_canvas, 0, 0, ATTITUDE_SIZE, center_y - pitch_offset, &sky_dsc);

    // Draw ground (bottom half)
    lv_draw_rect_dsc_t ground_dsc;
    lv_draw_rect_dsc_init(&ground_dsc);
    ground_dsc.bg_color = COLOR_GROUND; // Fixed: was COLOR GROUND
    ground_dsc.bg_opa = LV_OPA_COVER; // Fixed: was bg_opaa
    canvas_draw_rect(attitude_canvas, 0, center_y - pitch_offset, ATTITUDE_SIZE, ATTITUDE_SIZE - (center_y - pitch_offset), &ground_dsc);

    // Draw horizon line
    lv_draw_line_dsc_t line_dsc;
    lv_draw_line_dsc_init(&line_dsc);
    line_dsc.color = COLOR_HORIZON; // Fixed: was COLOR HORIZON
    line_dsc.width = 3;

    lv_point_precise_t points[2] = {
        {0, center_y - pitch_offset},
        {ATTITUDE_SIZE, center_y - pitch_offset}
    };
    canvas_draw_line(attitude_canvas, points, 2, &line_dsc);

    // Draw center aircraft symbol
    line_dsc.color = lv_color_hex(0xFFFF00);
    line_dsc.width = 3;

    lv_point_precise_t left_wing[2] = {{center_x - 40, center_y}, {center_x - 10, center_y}};
    canvas_draw_line(attitude_canvas, left_wing, 2, &line_dsc);

    lv_point_precise_t right_wing[2] = {{center_x + 10, center_y}, {center_x + 40, center_y}};
    canvas_draw_line(attitude_canvas, right_wing, 2, &line_dsc);
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
    snprintf(buf, sizeof(buf), "%dm/s\nSPD", data->speed); // Fixed: was \NSPD
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
    //snprintf(buf, sizeof(buf), "%d" LV_SYMBOL_DEGREES "\nHEAD", data->heading);
    snprintf(buf, sizeof(buf), "%d\nHEAD", data->heading);
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
    char buf[16];

    // GPS
    update_led_color(gps_led, data->gps_status);
    snprintf(buf, sizeof(buf), "%dSAT", data->gps_satellites);
    lv_label_set_text(gps_detail_label, buf);

    // IMU
    update_led_color(imu_led, data->imu_status);
    lv_label_set_text(imu_detail_label, data->imu_status == SENSOR_STATUS_GOOD ? "READY" : "ERR");

    // MAG
    update_led_color(mag_led, data->mag_status);
    lv_label_set_text(mag_detail_label, data->mag_status == SENSOR_STATUS_WARNING ? "CAL" : "READY");

    // BARO
    update_led_color(baro_led, data->baro_status);
    lv_label_set_text(baro_detail_label, data->baro_status == SENSOR_STATUS_GOOD ? "READY" : "ERR");

    // RADIO
    update_led_color(radio_led, data->radio_status);
    snprintf(buf, sizeof(buf), "%dBm", data->rssi);
    lv_label_set_text(radio_detail_label, buf);
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
