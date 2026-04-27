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
#include <stdatomic.h>
#include <esp_heap_caps.h>
#include "log_sys.h"
#include "lvgl.h"
#include "controller.h"
#include "quad_3d.h"
#include "tft_espi.h"
#include "tft_sprite.h"
#include "lvgl_driver.h"
#include "lora_state.h"
#include "gpio_setting.h"
#include "io_define.h"
#include "task_manager.h"
#include "dashboard_widgets.h"

/*********************
 *      DEFINES
 *********************/
#define SHOW_FPS_COUNTER 1

#define FIRST_COL_PAD (-8)
#define TABLE_HEIGHT (9)
#define TABLE_WIDTH 4

/**********************
 *      TYPEDEFS
 **********************/
typedef enum
{
    TABLE_ROW,
    TABLE_COLUMN,
    TABLE_MAX_ITEM,
} TABLE_ITEM;

typedef enum
{
    TABLE_OPERATION_UP,
    TABLE_OPERATION_DOWN,
    TABLE_OPERATION_MAX,
} TABLE_OPERATION;

// extern void sendPIDTblInfo(uint32_t height, uint32_t width, uint16_t pid_tbl[height][width]);
// extern void sendReceivePIDTblInfo();
// extern void getPIDInfoTable(uint32_t height, uint32_t width, uint16_t pid_tbl[height][width], LORA_GB_PID_INIT_T *first_half);
extern atomic_uint_fast32_t lora_send_config;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void welkin_fc_create(lv_obj_t *parent);
static void pid_setting_create(lv_obj_t *parent);
static void color_chg_event_cb(lv_event_t *e);
static void tab_changer_main_page(GB_REMOTE_CONTROL_ID op);
static void _handle_button_event(GB_REMOTE_CONTROL_ID btn_id, uint32_t active_tab);
#if SHOW_FPS_COUNTER
static void create_fps_counter(lv_obj_t *parent);
static void fps_update_task(lv_timer_t *timer);
#endif

/**********************
 *  STATIC VARIABLES
 **********************/
static lv_obj_t *tv = NULL;
static lv_obj_t *t1 = NULL;
static lv_obj_t *t2 = NULL;
static lv_obj_t *status_bar = NULL;

static lv_style_t style_cell_selected;

static lv_style_t style_box;
lv_obj_t *remote_controller;
lv_obj_t *pid_table_obj;

static uint16_t pid_table[TABLE_HEIGHT][TABLE_WIDTH] = {0};
static bool pull_btn = false;
static bool push_btn = false;
static int selected_row = -1;
static int selected_column = -1;

lv_timer_t *draw_task = NULL;
lv_obj_t *model_canvas = NULL;
uint32_t canvas_buffer_size = LV_HOR_RES_MAX * LV_VER_RES_MAX * 2;
uint16_t *canvas_buffer = NULL;
static bool canvas_initializing = false;

// battery
int battery_level = 0;
static lv_obj_t *battery_label;
static lv_obj_t *battery_icon;

// Toggle switch display - using LVGL switch widgets
static lv_obj_t *toggle_sw_widgets[4] = {NULL}; // The switch widgets
static lv_obj_t *toggle_sw_labels[4] = {NULL}; // Text labels for each switch

// Custom styles for toggle switches
static lv_style_t style_switch_on;
static lv_style_t style_switch_off;
static lv_style_t style_switch_knob;

volatile uint32_t g_fps_frame_count = 0;

#if SHOW_FPS_COUNTER
static lv_obj_t *fps_label = NULL;
static uint32_t fps_last_time = 0;
static uint32_t fps_current = 0;
#endif

static void _init_selected_table_cell()
{
    selected_row = -1;
    selected_column = -1;
    pull_btn = false;
    push_btn = false;
}

static bool _check_table_selected(TABLE_ITEM item)
{
    if (item == TABLE_ROW && selected_row != -1)
        return true;
    else if (item == TABLE_COLUMN && selected_column != -1)
        return true;
    return false;
}

static void _start_selecte_table()
{
    selected_row = 0;
    selected_column = 0;
}

static void _run_table_select_raw(TABLE_OPERATION operation)
{
    if (operation == TABLE_OPERATION_UP)
    {
        selected_row += 1;
        if (selected_row >= TABLE_HEIGHT)
        {
            selected_row -= 1;
        }
    }
    else if (operation == TABLE_OPERATION_DOWN)
    {
        selected_row -= 1;
        if (selected_row < 0)
            selected_row = -1;
    }
    else
        GB_DEBUGE(DISP_TAG, "Wrong pid_table_obj cell operation");
}

static void _run_table_select_column(TABLE_OPERATION operation)
{
    do
    {
        if (operation == TABLE_OPERATION_UP)
        {
            selected_column += 1;
            if (selected_column >= TABLE_WIDTH)
                selected_column = 0;
        }
        else if (operation == TABLE_OPERATION_DOWN)
        {
            selected_column -= 1;
            if (selected_column < 0)
                selected_column = TABLE_WIDTH - 1;
        }
        else
            GB_DEBUGE(DISP_TAG, "Wrong pid_table_obj cell operation");
    } while (0);
}

void init_canvas()
{
    if (canvas_initializing)
    {
        GB_DEBUGW(DISP_TAG, "Canvas initialization already in process");
        return;
    }
    canvas_initializing = true;

    if (NULL == canvas_buffer)
    {
        canvas_buffer = (uint16_t *)heap_caps_malloc(canvas_buffer_size, MALLOC_CAP_SPIRAM);
        if (canvas_buffer == NULL)
        {
            GB_DEBUGE(DISP_TAG, "init_canvas allocate buffer failed");
            canvas_initializing = false;
            return;
        }
    }

    if (NULL == model_canvas)
    {
        model_canvas = lv_canvas_create(lv_screen_active());

        lv_obj_set_size(model_canvas, LV_HOR_RES_MAX, LV_VER_RES_MAX - 30);
        lv_obj_align(model_canvas, LV_ALIGN_TOP_LEFT, 0, 30);

        lv_obj_move_foreground(model_canvas);
        if (status_bar)
            lv_obj_move_foreground(status_bar);

        GB_DEBUGI(DISP_TAG, "Canvas created and positioned below status bar");
    }

    canvas_initializing = false;
}

void deinit_canvas()
{
    if (canvas_initializing)
    {
        GB_DEBUGW(DISP_TAG, "Canvas initialization already in process, deferring cleanup");
        return;
    }

    if (model_canvas)
    {
        lv_obj_delete(model_canvas);
        model_canvas = NULL;
    }

    if (canvas_buffer)
    {
        free(canvas_buffer);
        canvas_buffer = NULL;
    }

    GB_DEBUGI(DISP_TAG, "Canvas cleaned up");
}

void gb_remote_single_control(GB_REMOTE_CONTROL_ID button_id)
{
    static GB_REMOTE_CONTROL_ID stored_id;
    stored_id = button_id;
    if (xSemaphoreTake(xGuiSemaphore, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        lv_obj_set_user_data(remote_controller, &stored_id);
        lv_obj_send_event(remote_controller, LV_EVENT_VALUE_CHANGED, NULL);
        xSemaphoreGive(xGuiSemaphore);
    }
    else
    {
        GB_DEBUGW(DISP_TAG, "Failed to acquire GUI semphore for btn_id = %u", button_id);
    }

}

static void _update_pid_table_display(void)
{
    char pid_str[8];

    for (int i = 1; i < TABLE_HEIGHT; i++)
    {
        for (int j = 1; j < TABLE_WIDTH; j++)
        {
            snprintf(pid_str, 8, "%d", pid_table[i][j]);
            lv_table_set_cell_value(pid_table_obj, i, j, pid_str);
        }
    }

    // Update selected cell highlight using lv_table_set_selected_cell
    if (selected_row >= 0 && selected_column >= 0 && selected_row < TABLE_HEIGHT)
    {
        lv_obj_add_state(pid_table_obj, LV_STATE_FOCUSED | LV_STATE_FOCUS_KEY);
        lv_table_set_selected_cell(pid_table_obj, selected_row, selected_column);
    }
    else
    {
        lv_obj_remove_state(pid_table_obj, LV_STATE_FOCUSED | LV_STATE_FOCUS_KEY);
    }
}

static void remote_controller_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_VALUE_CHANGED)
    {
        int *btn_id = (int *)lv_obj_get_user_data(remote_controller);
        uint32_t active_tab = lv_tabview_get_tab_active(tv);

        if (active_tab != 1) // reset pid table item
        {
            _init_selected_table_cell();
        }

        if (btn_id == NULL)
        {
            return;
        }
        GB_DEBUGI(DISP_TAG, "controller callback, btn_id = %u, select[row: %d, column: %d]", *btn_id, selected_row, selected_column);

        if (*btn_id >= BTN_DPAD_UP && *btn_id < GB_CONTROL_ID_MAX)
        {
            _handle_button_event(*btn_id, active_tab);
            _update_pid_table_display();
        }
    }
}

static void _handle_button_event(GB_REMOTE_CONTROL_ID btn_id, uint32_t active_tab)
{
    switch (btn_id)
    {
    case BTN_DPAD_UP:
    case BTN_OP_B:
        if (active_tab == 1 && _check_table_selected(TABLE_ROW))
        {
            _run_table_select_raw(TABLE_OPERATION_DOWN);
        }
        break;

    case BTN_DPAD_DOWN:
    case BTN_OP_Y:
        if (active_tab == 1)
        {
            if (!_check_table_selected(TABLE_ROW))
                _start_selecte_table();
            else
            {
                _run_table_select_raw(TABLE_OPERATION_UP);
            }
        }
        break;

    case BTN_DPAD_LEFT:
    case BTN_OP_X:
        if (active_tab == 1 && _check_table_selected(TABLE_ROW))
        {
            _run_table_select_column(TABLE_OPERATION_DOWN);
        }
        else
        {
            tab_changer_main_page(R_LEFT);
        }
        break;

    case BTN_DPAD_RIGHT:
    case BTN_OP_A:
        if (active_tab == 1 && _check_table_selected(TABLE_ROW))
        {
            _run_table_select_column(TABLE_OPERATION_UP);
        }
        else
        {
            tab_changer_main_page(R_RIGHT);
        }
        break;

    case BTN_DPAD_MID:
        tab_changer_main_page(R_RIGHT);
        break;

    // --- decrement PID value (fast -10) ---
    case BTN_OP_SELECT:
        if (active_tab == 1 && _check_table_selected(TABLE_ROW) && selected_row < TABLE_HEIGHT)
        {
            if (pid_table[selected_row][selected_column] >= 10)
                pid_table[selected_row][selected_column] -= 10;
            else
                pid_table[selected_row][selected_column] = 0;
        }
        break;

    // --- start: switch tabs ---
    case BTN_OP_START:
        if (active_tab == 1 && _check_table_selected(TABLE_ROW) && selected_row < TABLE_HEIGHT)
        pid_table[selected_row][selected_column] += 10;
        break;

    case TOGGLE_SWITHCH_CHANGED:
        GB_DEBUGI(DISP_TAG, "Toggle switch changed event received");
        break;

    // --- decrement PID value ---
    case BTN_OP_OPTION:
        if (active_tab == 1 && _check_table_selected(TABLE_ROW) && selected_row < TABLE_HEIGHT)
        {
            if (pid_table[selected_row][selected_column] > 0)
                pid_table[selected_row][selected_column] -= 1;
        }
        break;

    // --- increase PID value ---
    case BTN_OP_MENU:
        if (active_tab == 1 && _check_table_selected(TABLE_ROW) && selected_row < TABLE_HEIGHT)
            pid_table[selected_row][selected_column] += 1;
        break;
    default:
        break;
    }
}

static void canvas_draw_task(lv_timer_t *timer)
{
    LV_UNUSED(timer);
    GB_GPIO_Set(TEST_IMU_IO, 1);

    if (quad3d_get_image(canvas_buffer))
        lv_canvas_set_buffer(model_canvas, canvas_buffer, LV_HOR_RES_MAX, LV_VER_RES_MAX, LV_COLOR_FORMAT_RGB565);
    GB_GPIO_Set(TEST_IMU_IO, 0);
}

static void battery_update_task(lv_timer_t *timer)
{
    LV_UNUSED(timer);

    static char buf[8];
    snprintf(buf, sizeof(buf), "%d%%", battery_level);
    lv_label_set_text(battery_label, buf);

    if (battery_level > 80)
        lv_label_set_text(battery_icon, LV_SYMBOL_BATTERY_FULL);
    else if (battery_level > 60)
        lv_label_set_text(battery_icon, LV_SYMBOL_BATTERY_3);
    else if (battery_level > 40)
        lv_label_set_text(battery_icon, LV_SYMBOL_BATTERY_2);
    else if (battery_level > 20)
        lv_label_set_text(battery_icon, LV_SYMBOL_BATTERY_1);
    else
        lv_label_set_text(battery_icon, LV_SYMBOL_BATTERY_EMPTY);
}

static void toggle_switch_update_task(lv_timer_t *timer)
{
    LV_UNUSED(timer);
    GB_TOGGLE_SWITCH_STATE toggle_state;
    static GB_TOGGLE_SWITCH_STATE prev_state = {0};
    static bool processing_3d_toggle = false;

    // Get current toggle switch state using thread-safe getter
    controller_get_toggle_state(&toggle_state);

    // Update toggle switch 1 (Mode: UI/FLY)
    if (toggle_sw_widgets[0] != NULL)
    {
        if (toggle_state.sw1_state)
        {
            lv_obj_add_state(toggle_sw_widgets[0], LV_STATE_CHECKED);
            lv_label_set_text(toggle_sw_labels[0], "#888888 UI#|#00C853 FLY#");

        }
        else
        {
            lv_obj_remove_state(toggle_sw_widgets[0], LV_STATE_CHECKED);
            lv_label_set_text(toggle_sw_labels[0], "#00C853 UI#|#888888 FLY#");
        }
    }

    // Update toggle switch 2 (View: UI/3D)
    if (toggle_sw_widgets[1] != NULL)
    {
        if (toggle_state.sw2_state)
        {
            lv_obj_add_state(toggle_sw_widgets[1], LV_STATE_CHECKED);
            lv_label_set_text(toggle_sw_labels[1], "#888888 UI#|#00C853 3D#");
        }
        else
        {
            lv_obj_remove_state(toggle_sw_widgets[1], LV_STATE_CHECKED);
            lv_label_set_text(toggle_sw_labels[1], "#00C853 UI#|#888888 3D#");
        }
    }

    // Update toggle switch 3 (PID1: OFF/PULL)
    if (toggle_sw_widgets[2] != NULL)
    {
        if (toggle_state.sw3_state)
        {
            lv_obj_add_state(toggle_sw_widgets[2], LV_STATE_CHECKED);
            lv_label_set_text(toggle_sw_labels[2], "#888888 _#|#00C853 PULL#");
        }
        else
        {
            lv_obj_remove_state(toggle_sw_widgets[2], LV_STATE_CHECKED);
            lv_label_set_text(toggle_sw_labels[2], "#00C853 _#|#888888 PULL#");
        }
    }

    // Update toggle switch 4 (PID2: OFF/PUSH)
    if (toggle_sw_widgets[3] != NULL)
    {
        if (toggle_state.sw4_state)
        {
            lv_obj_add_state(toggle_sw_widgets[3], LV_STATE_CHECKED);
            lv_label_set_text(toggle_sw_labels[3], "#888888 _#|#00C853 PUSH#");
        }
        else
        {
            lv_obj_remove_state(toggle_sw_widgets[3], LV_STATE_CHECKED);
            lv_label_set_text(toggle_sw_labels[3], "#00C853 _#|#888888 PUSH#");
        }
    }

    // Handle 3D model toggle with debouncing and state protection
    bool sw2_changed = (toggle_state.sw2_state != prev_state.sw2_state);

    if (sw2_changed && !processing_3d_toggle)
    {
        processing_3d_toggle = true;

        if (toggle_state.sw2_state && !draw_task)
        {
            // Switch to 3D model mode
            GB_DEBUGI(DISP_TAG, "Enabling 3D mode");
            // Hide dashboard before showing 3D model
            if (tv) lv_obj_add_flag(tv, LV_OBJ_FLAG_HIDDEN);
            init_canvas();

            // Only create timer if canvas was successfully initialized
            if (model_canvas != NULL)
            {
                draw_task = lv_timer_create(canvas_draw_task, 100, NULL);
                atomic_store(&lora_send_config, LORA_GET_MOTION_STATE);
                GB_DEBUGI(DISP_TAG, "3D mode enabled successfully");
            }
            else
            {
                GB_DEBUGI(DISP_TAG, "Failed to initialize canvas for 3D mode");
                // Show dashboard again if 3D init failed
                if (tv) lv_obj_clear_flag(tv, LV_OBJ_FLAG_HIDDEN);
            }
        }
        else if (!toggle_state.sw2_state && draw_task)
        {
            // Switch back to UI mode
            GB_DEBUGI(DISP_TAG, "Disabling 3D mode");

            lv_timer_delete(draw_task);
            draw_task = NULL;

            deinit_canvas();
            atomic_store(&lora_send_config, LORA_SEND_NA);

            // Show dashboard when returning from 3D mode
            if (tv) lv_obj_clear_flag(tv, LV_OBJ_FLAG_HIDDEN);
            dashboard_show();
            GB_DEBUGI(DISP_TAG, "3D mode disabled successfully");
        }

        processing_3d_toggle = false;
    }

    // Update PID PULL/PUSH state based on switches 3 and 4
    bool sw3_changed = (toggle_state.sw3_state != prev_state.sw3_state);
    bool sw4_changed = (toggle_state.sw4_state != prev_state.sw4_state);

    if (sw3_changed || sw4_changed)
    {
        pull_btn = toggle_state.sw3_state;
        push_btn = toggle_state.sw4_state;

        GB_DEBUGI(DISP_TAG, "Toggle SW3/SW4 changed: PULL=%d, PUSH=%d", pull_btn, push_btn);

        // Update display if on PID tab
        if (tv && lv_tabview_get_tab_active(tv) == 1)
        {
            _update_pid_table_display();
        }
    }

    prev_state = toggle_state;
}

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void welkin_widgets()
{
    // Initialize custom styles for toggle switches
    // Style for switch when ON (checked)
    lv_style_init(&style_switch_on);
    lv_style_set_bg_color(&style_switch_on, lv_color_hex(0x00C853)); // Green when ON
    lv_style_set_border_width(&style_switch_on, 1);

    // Style for switch when OFF (unchecked)
    lv_style_init(&style_switch_off);
    lv_style_set_bg_color(&style_switch_off, lv_color_hex(0xCCCCCC)); // Gray when OFF
    lv_style_set_border_width(&style_switch_off, 1);

    // Style for switch knob
    lv_style_init(&style_switch_knob);
    lv_style_set_bg_color(&style_switch_knob, lv_color_hex(0xFFFFFFFF)); // White knob
    lv_style_set_pad_all(&style_switch_knob, -2); // Make knob slightly larger

    // Create a top status bar container that spans full width
    status_bar = lv_obj_create(lv_screen_active());
    lv_obj_set_size(status_bar, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_align(status_bar, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_layout(status_bar, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(status_bar, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(status_bar, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_bg_opa(status_bar, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_opa(status_bar, LV_OPA_TRANSP, 0);
    lv_obj_set_style_pad_all(status_bar, 4, 0);
    lv_obj_set_style_pad_column(status_bar, 8, 0); // Equal spacing between items

    // Create 4 toggle switch rows (each with: label + switch + state text)
    const char *initial_labels[4] = {"UI", "UI", "_", "_"};

    for (int i = 0; i < 4; i++)
    {
        // Create horizontal container for each switch (switch + label only, no prefix)
        lv_obj_t *sw_group = lv_obj_create(status_bar);
        lv_obj_set_size(sw_group, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_set_layout(sw_group, LV_LAYOUT_FLEX);
        lv_obj_set_flex_flow(sw_group, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(sw_group, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_set_style_bg_opa(sw_group, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_opa(sw_group, LV_OPA_TRANSP, 0);
        lv_obj_set_style_pad_all(sw_group, 0, 0);
        lv_obj_set_style_pad_column(sw_group, 2, 0); // 2px gap between switch and label

        // LVGL switch widget (read-only, reflects hardware state)
        toggle_sw_widgets[i] = lv_switch_create(sw_group);
        lv_obj_set_size(toggle_sw_widgets[i], 24, 14); // Very compact: 24x14px
        lv_obj_remove_flag(toggle_sw_widgets[i], LV_OBJ_FLAG_CLICKABLE); // Read-only

        // Apply custom styles
        lv_obj_add_style(toggle_sw_widgets[i], &style_switch_off, LV_PART_MAIN);
        lv_obj_add_style(toggle_sw_widgets[i], &style_switch_on, LV_PART_MAIN | LV_STATE_CHECKED);
        lv_obj_add_style(toggle_sw_widgets[i], &style_switch_knob, LV_PART_KNOB);

        // State label: shows current option in GREEN (e.g., "UI|FLY")
        toggle_sw_labels[i] = lv_label_create(sw_group);
        lv_label_set_text(toggle_sw_labels[i], initial_labels[i]);
        lv_obj_set_style_text_font(toggle_sw_labels[i], &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(toggle_sw_labels[i], lv_color_hex(0x00C853), 0); // Green for selected
        lv_label_set_recolor(toggle_sw_labels[i], true); // Enable color codes (set once at creation)
    }

    // Battery display - add to status bar
    lv_obj_t *battery_cont = lv_obj_create(status_bar);
    lv_obj_set_size(battery_cont, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_layout(battery_cont, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(battery_cont, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(battery_cont, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_bg_opa(battery_cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_opa(battery_cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_pad_all(battery_cont, 2, 0);

    battery_label = lv_label_create(battery_cont);
    lv_label_set_text(battery_label, "0%");
    lv_obj_set_style_text_font(battery_label, &lv_font_montserrat_16, 0);

    battery_icon = lv_label_create(battery_cont);
    lv_label_set_text(battery_icon, LV_SYMBOL_BATTERY_EMPTY);
    lv_obj_set_style_text_font(battery_icon, &lv_font_montserrat_16, 0);

    // Create tabview AFTER status bar, positioned below it
    tv = lv_tabview_create(lv_screen_active());
    lv_obj_align(tv, LV_ALIGN_TOP_MID, 0, 30); // Offset by status bar height
    lv_obj_set_size(tv, LV_PCT(100), LV_PCT(100) - 30); // Adjust height to account for status bar
    int32_t disp_var = lv_display_get_vertical_resolution(NULL);
    lv_obj_set_size(tv, LV_PCT(100), disp_var - 30);

    t1 = lv_tabview_add_tab(tv, "GB Drone");
    t2 = lv_tabview_add_tab(tv, "PID");

    lv_style_init(&style_box);
    lv_style_set_margin_top(&style_box, 30);

    welkin_fc_create(t1);
    pid_setting_create(t2);

    // Create obj to receive callback
    remote_controller = lv_obj_create(lv_screen_active());
    lv_obj_add_flag(remote_controller, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(remote_controller, remote_controller_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

#if SHOW_FPS_COUNTER
    create_fps_counter(lv_screen_active());
    fps_last_time = lv_tick_get();
#endif

    lv_timer_create(battery_update_task, 500, NULL);
    lv_timer_create(toggle_switch_update_task, 100, NULL);
#if SHOW_FPS_COUNTER
    lv_timer_create(fps_update_task, 50, NULL);
#endif
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

static void welkin_fc_create(lv_obj_t *parent)
{
    dashboard_create(parent);
}

static void lvgl_create_pid_table(lv_obj_t *parent)
{
    pid_table_obj = lv_table_create(parent);
    char pid_str[8];

    GB_DEBUGI(DISP_TAG, "Creating PID table");

    lv_style_init(&style_cell_selected);
    lv_style_set_border_color(&style_cell_selected, lv_color_hex(0xFF0000));
    lv_style_set_border_width(&style_cell_selected, 4);
    lv_style_set_border_opa(&style_cell_selected, LV_OPA_COVER);
    lv_style_set_border_side(&style_cell_selected, LV_BORDER_SIDE_FULL);

    // In LVGL 9, table cells use LV_PART_ITEMS for cell styling
    lv_obj_add_style(pid_table_obj, &style_cell_selected, LV_PART_ITEMS | LV_STATE_FOCUS_KEY);

    lv_table_set_column_count(pid_table_obj, TABLE_WIDTH);
    lv_table_set_row_count(pid_table_obj, TABLE_HEIGHT);

    lv_obj_set_width(pid_table_obj, LV_PCT(100));
    lv_obj_set_height(pid_table_obj, LV_PCT(100));
    lv_obj_align(pid_table_obj, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_clear_flag(pid_table_obj, LV_OBJ_FLAG_SCROLLABLE);

    int32_t table_width = lv_display_get_horizontal_resolution(NULL);
    int32_t first_col_w = table_width / (TABLE_WIDTH + 2);
    int32_t data_col_w = (table_width - first_col_w) / (TABLE_WIDTH - 1);
    lv_table_set_column_width(pid_table_obj, 0, first_col_w);
    for (int i = 1; i < TABLE_WIDTH; i++)
    {
        lv_table_set_column_width(pid_table_obj, i, data_col_w);
    }

    /*Fill the first column*/
    lv_table_set_cell_value(pid_table_obj, 0, 0, "");  // empty top-left cell
    lv_table_set_cell_value(pid_table_obj, 1, 0, "ZS");
    lv_table_set_cell_value(pid_table_obj, 2, 0, "ZH");
    lv_table_set_cell_value(pid_table_obj, 3, 0, "RS");
    lv_table_set_cell_value(pid_table_obj, 4, 0, "PS");
    lv_table_set_cell_value(pid_table_obj, 5, 0, "YS");
    lv_table_set_cell_value(pid_table_obj, 6, 0, "RA");
    lv_table_set_cell_value(pid_table_obj, 7, 0, "PA");
    lv_table_set_cell_value(pid_table_obj, 8, 0, "YA");

    lv_table_set_cell_value(pid_table_obj, 0, 1, "P");
    lv_table_set_cell_value(pid_table_obj, 0, 2, "I");
    lv_table_set_cell_value(pid_table_obj, 0, 3, "D");

    /*Fill the data cells*/
    for (int i = 1; i < TABLE_HEIGHT; i++)
    {
        for (int j = 1; j < TABLE_WIDTH; j++)
        {
        snprintf(pid_str, 8, "%d", pid_table[i][j]);
        lv_table_set_cell_value(pid_table_obj, i, j, pid_str);
        }
    }

    GB_DEBUGI(DISP_TAG, "PID table created successfylly");
}

static void pid_setting_create(lv_obj_t *parent)
{
    // Enable scrolling for the parent container
    lv_obj_set_scrollbar_mode(parent, LV_SCROLLBAR_MODE_AUTO);

    GB_DEBUGI(DISP_TAG, "Creating PID tab content");

    lvgl_create_pid_table(parent);
}

static void color_chg_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_VALUE_CHANGED)
    {
        lv_obj_t *sw = lv_event_get_target(e);
        bool is_dark = lv_obj_has_state(sw, LV_STATE_CHECKED);
        // In LVGL 9, theme switching is handled differently
        // The default theme auto-detects dark/light mode
        LV_UNUSED(is_dark);
    }
}

static void tab_changer_main_page(GB_REMOTE_CONTROL_ID op)
{
    uint32_t act = lv_tabview_get_tab_active(tv);
    if (R_LEFT == op)
    {
        if (act == 0)
            act = 1;
        else
            act--;
    }
    else if (R_RIGHT == op)
    {
        act++;
        if (act >= 2)
            act = 0;
    }
    else
    {
        GB_DEBUGE(DISP_TAG, "Wrong operation for main page tab changer");
    }

    lv_tabview_set_active(tv, act, LV_ANIM_ON);
}

GB_REMOTE_USER_MODE gb_get_user_mode()
{
    if (tv)
        return lv_tabview_get_tab_active(tv);
    else
        return GB_USER_MODE_MAX;
}

#if SHOW_FPS_COUNTER

/**
 * @brief Create FPS counter label at bottom right corner
 * @param parent Parent screen object
 */
static void create_fps_counter(lv_obj_t *parent)
{
    fps_label = lv_label_create(parent);
    lv_label_set_text(fps_label, "FPS: ---");
    lv_obj_set_style_text_color(fps_label, lv_color_hex(0x00FF00), 0); // Green text
    lv_obj_set_style_bg_color(fps_label, lv_color_hex(0x000000), 0); // Black background
    lv_obj_set_style_bg_opa(fps_label, LV_OPA_70, 0); // Semi-transparent
    lv_obj_set_style_pad_all(fps_label, 4, 0);
    lv_obj_set_style_text_font(fps_label, &lv_font_montserrat_14, 0);

    // Position at bottom right corner
    lv_obj_align(fps_label, LV_ALIGN_BOTTOM_RIGHT, -5, -5);

    GB_DEBUGI(DISP_TAG, "FPS counter created at bottom right");
}

static void fps_update_task(lv_timer_t *timer)
{
    LV_UNUSED(timer);

    if (fps_label == NULL) return;

    uint32_t current_time = lv_tick_get();

    // Update FPS every 500ms
    if (current_time - fps_last_time >= 500)
    {
        uint32_t elapsed_ms = current_time - fps_last_time;
        fps_current = (g_fps_frame_count * 1000) / elapsed_ms;

        char buf[16];
        snprintf(buf, sizeof(buf), "FPS:%lu", (unsigned long)fps_current);
        lv_label_set_text(fps_label, buf);

        // Reset counters
        g_fps_frame_count = 0;
        fps_last_time = current_time;

        // Color code based on performance
        if (fps_current >= 30)
            lv_obj_set_style_text_color(fps_label, lv_color_hex(0x00FF00), 0); // Green: Good
        else if (fps_current >= 20)
            lv_obj_set_style_text_color(fps_label, lv_color_hex(0xFFFF00), 0); // Yellow: OK
        else
            lv_obj_set_style_text_color(fps_label, lv_color_hex(0xFF0000), 0); // Red: Poor
    }
}
#endif
