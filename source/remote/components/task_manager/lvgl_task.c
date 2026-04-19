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

/*********************
 *      DEFINES
 *********************/
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

/**********************
 *  STATIC VARIABLES
 **********************/
static lv_obj_t *tv = NULL;
static lv_obj_t *t1 = NULL;
static lv_obj_t *t2 = NULL;

static lv_style_t style_cell_selected;

static lv_style_t style_box;
lv_obj_t *remote_controller;
lv_obj_t *pid_table_obj;

static uint16_t pid_table[TABLE_HEIGHT][TABLE_WIDTH] = {0};
static bool pull_btn = false;
static bool push_btn = false;
static int selected_row = -1;
static int selected_column = -1;

lv_timer_t *draw_task;
lv_obj_t *model_canvas = NULL;
uint32_t canvas_buffer_size = LV_HOR_RES_MAX * LV_VER_RES_MAX * 2;
uint16_t *canvas_buffer = NULL;

// battery
int battery_level = 0;
static lv_obj_t *battery_label;
static lv_obj_t *battery_icon;

// Toggle switch display - using LVGL switch widgets
static lv_obj_t *toggle_sw_widgets[4] = {NULL}; // The switch widgets
static lv_obj_t *toggle_sw_labels[4] = {NULL}; // Text labels for each switch
static lv_obj_t *toggle_sw_state_labels[4] = {NULL}; // State text (UI/FLY, etc.)

// Custom styles for toggle switches
static lv_style_t style_switch_on;
static lv_style_t style_switch_off;
static lv_style_t style_switch_knob;

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
        if (selected_row > TABLE_HEIGHT)
        {
            selected_row -= 1;
        }
        // selected button line
        else if (selected_row == TABLE_HEIGHT)
        {
            if (selected_column < TABLE_WIDTH / 2)
            {
                pull_btn = true;
            }
            else
            {
                push_btn = true;
            }
        }
    }
    else if (operation == TABLE_OPERATION_DOWN)
    {
        selected_row -= 1;
        if (selected_row < 0)
            selected_row = -1;
        // reset button
        if (selected_row < TABLE_HEIGHT)
        {
            pull_btn = false;
            push_btn = false;
        }
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
            if (pull_btn)
            {
                pull_btn = false;
                push_btn = true;
                break;
            }
            else if (push_btn)
            {
                push_btn = false;
                pull_btn = true;
                break;
            }
            selected_column += 1;
            if (selected_column >= TABLE_WIDTH)
                selected_column = 0;
        }
        else if (operation == TABLE_OPERATION_DOWN)
        {
            if (pull_btn)
            {
                pull_btn = false;
                push_btn = true;
                break;
            }
            else if (push_btn)
            {
                push_btn = false;
                pull_btn = true;
                break;
            }
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
    if (NULL == canvas_buffer)
        canvas_buffer = (uint16_t *)heap_caps_malloc(canvas_buffer_size, MALLOC_CAP_DMA);
    if (canvas_buffer == NULL)
    {
        GB_DEBUGE(DISP_TAG, "init_canvas allocate buffer failed");
        return;
    }

    if (NULL == model_canvas)
        model_canvas = lv_canvas_create(lv_screen_active());
}

void deinit_canvas()
{
    free(canvas_buffer);
    canvas_buffer = NULL;
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
        lv_table_set_selected_cell(pid_table_obj, selected_row, selected_column);
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
        if (active_tab == 1 && _check_table_selected(TABLE_ROW))
        {
            _run_table_select_raw(TABLE_OPERATION_DOWN);
            lv_obj_scroll_by(t2, 0, -lv_obj_get_height(t2) / (TABLE_HEIGHT - 2), LV_ANIM_ON);
        }
        break;

    case BTN_DPAD_DOWN:
        if (active_tab == 1)
        {
            if (!_check_table_selected(TABLE_ROW))
                _start_selecte_table();
            else
            {
                _run_table_select_raw(TABLE_OPERATION_UP);
                lv_obj_scroll_by(t2, 0, lv_obj_get_height(t2) / (TABLE_HEIGHT - 2), LV_ANIM_ON);
            }

        }
        break;

    case BTN_DPAD_LEFT:
        if (active_tab == 1 && _check_table_selected(TABLE_ROW))
        {
            _run_table_select_raw(TABLE_OPERATION_DOWN);
        }
        else
        {
            tab_changer_main_page(R_LEFT);
        }
        break;

    case BTN_DPAD_RIGHT:
        if (active_tab == 1 && _check_table_selected(TABLE_ROW))
        {
            _run_table_select_raw(TABLE_OPERATION_UP);
        }
        else
        {
            tab_changer_main_page(R_RIGHT);
        }
        break;

    case BTN_DPAD_MID:
        if (active_tab == 0)
        {
            // TODO
        }
        else if (active_tab == 1)
        {
            // TODO
        }
        break;

    // --- up button: increase PID value ---
    case BTN_OP_B:
        if (active_tab == 1 && _check_table_selected(TABLE_ROW) && selected_row < TABLE_HEIGHT)
            pid_table[selected_row][selected_column] += 1;
        break;

    // --- down button: decrement PID value ---
    case BTN_OP_Y:
        if (active_tab == 1 && _check_table_selected(TABLE_ROW) && selected_row < TABLE_HEIGHT)
        {
            if (pid_table[selected_row][selected_column] > 0)
                pid_table[selected_row][selected_column] -= 1;
        }
        break;

    // --- right button: increment PID value (fast +10) ---
    case BTN_OP_A:
        if (active_tab == 1 && _check_table_selected(TABLE_ROW) && selected_row < TABLE_HEIGHT)
        pid_table[selected_row][selected_column] += 10;
        break;

    // --- left button: decrement PID value (fast -10) ---
    case BTN_OP_X:
    if (active_tab == 1 && _check_table_selected(TABLE_ROW) && selected_row < TABLE_HEIGHT)
    {
        if (pid_table[selected_row][selected_column] >= 10)
            pid_table[selected_row][selected_column] -= 10;
        else
            pid_table[selected_row][selected_column] = 0;
    }
    break;

    // --- select: toggle between pull/push buttons
    case BTN_OP_SELECT:
        if (active_tab == 1 && (pull_btn || push_btn))
        {
            pull_btn = !pull_btn;
            push_btn = !push_btn;
        }
        break;

    // --- start: switch tabs ---
    case BTN_OP_START:
        tab_changer_main_page(R_RIGHT);
        break;

    case BTN_OP_MENU:
    case BTN_OP_OPTION:
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

    // Get current toggle switch state using thread-safe getter
    controller_get_toggle_state(&toggle_state);

    // Update toggle switch 1 (Mode: UI/FLY)
    if (toggle_sw_widgets[0] != NULL)
    {
        if (toggle_state.sw1_state)
        {
            lv_obj_add_state(toggle_sw_widgets[0], LV_STATE_CHECKED);
            lv_label_set_text(toggle_sw_state_labels[0], "FLY");
            lv_obj_set_style_text_color(toggle_sw_state_labels[0], lv_color_hex(0xFF0000), 0); // Red for FLY mode
        }
        else
        {
            lv_obj_remove_state(toggle_sw_widgets[0], LV_STATE_CHECKED);
            lv_label_set_text(toggle_sw_state_labels[0], "UI");
            lv_obj_set_style_text_color(toggle_sw_state_labels[0], lv_color_hex(0x0000FF), 0); // Blue for UI mode
        }
    }

    // Update toggle switch 2 (View: UI/3D)
    if (toggle_sw_widgets[1] != NULL)
    {
        if (toggle_state.sw2_state)
        {
            lv_obj_add_state(toggle_sw_widgets[1], LV_STATE_CHECKED);
            lv_label_set_text(toggle_sw_state_labels[1], "3D");
            lv_obj_set_style_text_color(toggle_sw_state_labels[1], lv_color_hex(0x00C853), 0); // Green for 3D
        }
        else
        {
            lv_obj_remove_state(toggle_sw_widgets[1], LV_STATE_CHECKED);
            lv_label_set_text(toggle_sw_state_labels[1], "UI");
            lv_obj_set_style_text_color(toggle_sw_state_labels[1], lv_color_hex(0x0000FF), 0); // Blue for UI
        }
    }

    // Update toggle switch 3 (PID1: OFF/PULL)
    if (toggle_sw_widgets[2] != NULL)
    {
        if (toggle_state.sw3_state)
        {
            lv_obj_add_state(toggle_sw_widgets[2], LV_STATE_CHECKED);
            lv_label_set_text(toggle_sw_state_labels[2], "PULL");
            lv_obj_set_style_text_color(toggle_sw_state_labels[2], lv_color_hex(0xFFA500), 0); // Orange for PULL
        }
        else
        {
            lv_obj_remove_state(toggle_sw_widgets[2], LV_STATE_CHECKED);
            lv_label_set_text(toggle_sw_state_labels[2], "OFF");
            lv_obj_set_style_text_color(toggle_sw_state_labels[2], lv_color_hex(0x888888), 0); // Gray for OFF
        }
    }

    // Update toggle switch 4 (PID2: OFF/PUSH)
    if (toggle_sw_widgets[3] != NULL)
    {
        if (toggle_state.sw4_state)
        {
            lv_obj_add_state(toggle_sw_widgets[3], LV_STATE_CHECKED);
            lv_label_set_text(toggle_sw_state_labels[3], "PUSH");
            lv_obj_set_style_text_color(toggle_sw_state_labels[3], lv_color_hex(0x9C27B0), 0); // Purple for PUSH
        }
        else
        {
            lv_obj_remove_state(toggle_sw_widgets[3], LV_STATE_CHECKED);
            lv_label_set_text(toggle_sw_state_labels[3], "OFF");
            lv_obj_set_style_text_color(toggle_sw_state_labels[3], lv_color_hex(0x888888), 0); // Gray for OFF
        }
    }

    // Automatically enable/disable 3D model based on switch 2
    if (toggle_state.sw2_state && !draw_task)
    {
        // Switch to 3D model mode
        init_canvas();
        draw_task = lv_timer_create(canvas_draw_task, 100, NULL);
        atomic_store(&lora_send_config, LORA_GET_MOTION_STATE);
    }
    else if (!toggle_state.sw2_state && draw_task)
    {
        // Switch back to UI mode
        lv_timer_delete(draw_task);
        draw_task = NULL;
        deinit_canvas();
        atomic_store(&lora_send_config, LORA_SEND_NA);
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
    tv = lv_tabview_create(lv_screen_active());

    // Initialize custom styles for toggle switches
    // Style for switch when ON (checked)
    lv_style_init(&style_switch_on);
    lv_style_set_bg_color(&style_switch_on, lv_color_hex(0x00C853)); // Green when ON
    lv_style_set_border_color(&style_switch_on, lv_color_hex(0x00A843));
    lv_style_set_border_width(&style_switch_on, 2);

    // Style for switch when OFF (unchecked)
    lv_style_init(&style_switch_off);
    lv_style_set_bg_color(&style_switch_off, lv_color_hex(0xCCCCCC)); // Gray when OFF
    lv_style_set_border_color(&style_switch_off, lv_color_hex(0x999999));
    lv_style_set_border_width(&style_switch_off, 2);

    // Style for switch knob
    lv_style_init(&style_switch_knob);
    lv_style_set_bg_color(&style_switch_knob, lv_color_hex(0xFFFFFFFF)); // White knob
    lv_style_set_border_color(&style_switch_knob, lv_color_hex(0xAAAAAA));
    lv_style_set_border_width(&style_switch_knob, 2);
    lv_style_set_pad_all(&style_switch_knob, -4); // Make knob slightly larger

    // Battery voltage display - top right
    lv_obj_t *battery_cont = lv_obj_create(lv_screen_active());
    lv_obj_set_size(battery_cont, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_align(battery_cont, LV_ALIGN_TOP_RIGHT, 0, -2);
    lv_obj_set_layout(battery_cont, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(battery_cont, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(battery_cont, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_bg_opa(battery_cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_opa(battery_cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_pad_all(battery_cont, 0, 0);

    battery_label = lv_label_create(battery_cont);
    lv_label_set_text(battery_label, "0%");

    battery_icon = lv_label_create(battery_cont);
    lv_label_set_text(battery_icon, LV_SYMBOL_BATTERY_EMPTY);

    // Toggle switches display - top left with LVGL switch widgets
    lv_obj_t *toggle_cont = lv_obj_create(lv_screen_active());
    lv_obj_set_size(toggle_cont, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_align(toggle_cont, LV_ALIGN_TOP_LEFT, 2, 0);
    lv_obj_set_layout(toggle_cont, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(toggle_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(toggle_cont, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_bg_opa(toggle_cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_opa(toggle_cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_pad_all(toggle_cont, 2, 0);
    lv_obj_set_style_pad_row(toggle_cont, 3, 0);

    // Create 4 toggle switch rows (each with: label + switch + state text)
    const char *switch_names[4] = {"Mode:", "View:", "PID1:", "PID2:"};
    const char *initial_states[4] = {"UI", "UI", "OFF", "OFF"};

    for (int i = 0; i < 4; i++)
    {
        // Create horizontal container for this switch row
        lv_obj_t *row = lv_obj_create(toggle_cont);
        lv_obj_set_size(row, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_set_layout(row, LV_LAYOUT_FLEX);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_opa(row, LV_OPA_TRANSP, 0);
        lv_obj_set_style_pad_all(row, 0, 0);
        lv_obj_set_style_pad_row(row, 4, 0);

        // Label for switch name (e.g., "Mode:", "View:")
        toggle_sw_labels[i] = lv_label_create(row);
        lv_label_set_text(toggle_sw_labels[i], switch_names[i]);
        lv_obj_set_style_text_font(toggle_sw_labels[i], &lv_font_montserrat_16, 0);
        lv_obj_set_width(toggle_sw_labels[i], 35); // Fixed width for alignment

        // LVGL switch widget (read-only, reflects hardware state)
        toggle_sw_widgets[i] = lv_switch_create(row);
        lv_obj_set_size(toggle_sw_widgets[i], 30, 16); // Compact switch size
        lv_obj_remove_flag(toggle_sw_widgets[i], LV_OBJ_FLAG_CLICKABLE); // Read-only (hardware controlled)

        // Apply custom styles
        lv_obj_add_style(toggle_sw_widgets[i], &style_switch_off, LV_PART_MAIN);
        lv_obj_add_style(toggle_sw_widgets[i], &style_switch_on, LV_PART_MAIN | LV_STATE_CHECKED);
        lv_obj_add_style(toggle_sw_widgets[i], &style_switch_knob, LV_PART_KNOB);

        // State text label (e.g., "UI", "FLY", "PULL", "OFF")
        toggle_sw_state_labels[i] = lv_label_create(row);
        lv_label_set_text(toggle_sw_state_labels[i], initial_states[i]);
        lv_obj_set_style_text_font(toggle_sw_state_labels[i], &lv_font_montserrat_16, 0);
        lv_obj_set_width(toggle_sw_state_labels[i], 30); // Fixed width

        // Color the state text based on initial state
        if (i < 2) // Mode and View switches
        {
            lv_obj_set_style_text_color(toggle_sw_state_labels[i], lv_color_hex(0x0000FF), 0); // Blue
        }
        else // PID switches
        {
            lv_obj_set_style_text_color(toggle_sw_state_labels[i], lv_color_hex(0x888888), 0); // Gray when OFF
        }
    }

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

    lv_timer_create(battery_update_task, 500, NULL);
    lv_timer_create(toggle_switch_update_task, 100, NULL);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

static void welkin_fc_create(lv_obj_t *parent)
{
    lv_obj_set_layout(parent, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(parent, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

    lv_obj_t *info_label = lv_label_create(parent);
    lv_label_set_text(info_label, "GB-Drone Remote\n\n"
        "Use toggle switches:\n"
        "SW1: UI/FLY mode\n"
        "SW2: UI/3D view\n"
        "SW3: PULL PID\n"
        "SW4: PUSH PID");
    lv_obj_align(info_label, LV_ALIGN_CENTER, 0, 0);
}

static void lvgl_create_pid_table(lv_obj_t *parent, int32_t height, int32_t width)
{
    pid_table_obj = lv_table_create(parent);
    char pid_str[8];

    GB_DEBUGI(DISP_TAG, "Creating PID table: withd = %d, height = %d", width, height);

    lv_style_init(&style_cell_selected);
    lv_style_set_border_color(&style_cell_selected, lv_color_hex(0xFF0000));
    lv_style_set_border_width(&style_cell_selected, 4);
    lv_style_set_border_opa(&style_cell_selected, LV_OPA_50);
    lv_style_set_border_side(&style_cell_selected, LV_BORDER_SIDE_FULL);

    // In LVGL 9, table cells use LV_PART_ITEMS for cell styling
    lv_obj_add_style(pid_table_obj, &style_cell_selected, LV_PART_ITEMS | LV_STATE_PRESSED);

    lv_table_set_column_count(pid_table_obj, TABLE_WIDTH);
    lv_table_set_row_count(pid_table_obj, TABLE_HEIGHT);

    lv_obj_set_size(pid_table_obj, width, LV_SIZE_CONTENT);
    lv_obj_align(pid_table_obj, LV_ALIGN_CENTER, 0, 0);
    lv_obj_clear_flag(pid_table_obj, LV_OBJ_FLAG_SCROLLABLE);

    for (int i = 0; i < TABLE_WIDTH; i++)
    {
    if (0 == i)
        lv_table_set_column_width(pid_table_obj, i, width / TABLE_WIDTH + FIRST_COL_PAD);
    else
        lv_table_set_column_width(pid_table_obj, i, (width - FIRST_COL_PAD) / TABLE_WIDTH);
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

    int32_t grid_w = lv_obj_get_content_width(parent);
    int32_t grid_h = lv_obj_get_content_height(parent);

    GB_DEBUGI(DISP_TAG, "PID tab dimensions: width=%d, height=%d", grid_w, grid_h);

    lvgl_create_pid_table(parent, grid_h, grid_w - 26);
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
