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
extern GB_SEND_CONFIG lora_send_config;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void welkin_fc_create(lv_obj_t *parent);
static void pid_setting_create(lv_obj_t *parent);
static void color_chg_event_cb(lv_event_t *e);
static void tab_content_anim_create(lv_obj_t *parent);
static void tab_changer_main_page(GB_REMOTE_CONTROL_ID op);

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
lv_obj_t *pid_pull_btn, *pid_push_btn, *main_btn, *model_3d_btn;

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
#if 0
    free(canvas_buffer);
    canvas_buffer = NULL;
#endif
}

void gb_remote_single_control(GB_REMOTE_CONTROL_ID button_id)
{
    static GB_REMOTE_CONTROL_ID stored_id;
    stored_id = button_id;
    lv_obj_set_user_data(remote_controller, &stored_id);
    lv_obj_send_event(remote_controller, LV_EVENT_VALUE_CHANGED, NULL);
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

    if (pull_btn)
        lv_obj_set_style_border_color(pid_pull_btn, lv_color_hex(0xFF0000), 0);
    else
        lv_obj_set_style_border_color(pid_pull_btn, lv_color_hex(0xffffff), 0);
    if (push_btn)
        lv_obj_set_style_border_color(pid_push_btn, lv_color_hex(0xFF0000), 0);
    else
        lv_obj_set_style_border_color(pid_push_btn, lv_color_hex(0xffffff), 0);
}

static void remote_controller_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_VALUE_CHANGED)
    {
        int *btn_id = (int *)lv_obj_get_user_data(remote_controller);
        uint32_t active_tab = lv_tabview_get_tab_act(tv);

        if (active_tab != 1) // reset pid table item
        {
            _init_selected_table_cell();
        }

        if (btn_id == NULL)
        {
            return;
        }
        GB_DEBUGI(DISP_TAG, "controller callback, btn_id = %u, select[row: %d, column: %d]", *btn_id, selected_row, selected_column);
        switch (*btn_id)
        {
            case L_DOWN_R_UP:
                if (_check_table_selected(TABLE_ROW) && active_tab == 1)
                {
                    _run_table_select_raw(TABLE_OPERATION_DOWN);
                    lv_obj_scroll_by(t2, 0, lv_obj_get_height(t2) / (TABLE_HEIGHT - 4), LV_ANIM_ON);
                }
                else if (active_tab == 0)
                {
                    if (lv_obj_has_state(main_btn, LV_STATE_CHECKED))
                        lv_obj_remove_state(main_btn, LV_STATE_CHECKED);
                    else
                        lv_obj_add_state(main_btn, LV_STATE_CHECKED);

                    if (lv_obj_has_state(model_3d_btn, LV_STATE_CHECKED))
                        lv_obj_remove_state(model_3d_btn, LV_STATE_CHECKED);
                    else
                        lv_obj_add_state(model_3d_btn, LV_STATE_CHECKED);

                    if (lv_obj_has_state(model_3d_btn, LV_STATE_CHECKED))
                        lv_obj_send_event(model_3d_btn, LV_EVENT_CLICKED, NULL);
                    else
                        lv_obj_send_event(model_3d_btn, LV_EVENT_RELEASED, NULL);
                }
                break;
            case L_DOWN_R_DOWN:
                if (!_check_table_selected(TABLE_ROW) && active_tab == 1)
                {
                    _start_selecte_table();
                }
                else if (_check_table_selected(TABLE_ROW) && active_tab == 1)
                {
                    _run_table_select_raw(TABLE_OPERATION_UP);
                    lv_obj_scroll_by(t2, 0, -lv_obj_get_height(t2) / (TABLE_HEIGHT - 2), LV_ANIM_ON);
                }
                else if (active_tab == 0)
                {
                    if (lv_obj_has_state(main_btn, LV_STATE_CHECKED))
                        lv_obj_remove_state(main_btn, LV_STATE_CHECKED);
                    else
                        lv_obj_add_state(main_btn, LV_STATE_CHECKED);

                    if (lv_obj_has_state(model_3d_btn, LV_STATE_CHECKED))
                        lv_obj_remove_state(model_3d_btn, LV_STATE_CHECKED);
                    else
                        lv_obj_add_state(model_3d_btn, LV_STATE_CHECKED);

                    if (lv_obj_has_state(model_3d_btn, LV_STATE_CHECKED))
                        lv_obj_send_event(model_3d_btn, LV_EVENT_CLICKED, NULL);
                    else
                        lv_obj_send_event(model_3d_btn, LV_EVENT_RELEASED, NULL);
                }
                break;
            case L_UP_R_UP:
                if (_check_table_selected(TABLE_ROW) && active_tab == 1)
                {
                    pid_table[selected_row][selected_column] += 1;
                }
                break;
            case L_UP_R_DOWN:
                if (_check_table_selected(TABLE_ROW) && active_tab == 1)
                {
                    if (0 < pid_table[selected_row][selected_column])
                        pid_table[selected_row][selected_column] -= 1;
                }
                break;
            case R_LEFT:
                if (_check_table_selected(TABLE_ROW) && active_tab == 1)
                {
                    _run_table_select_column(TABLE_OPERATION_DOWN);
                }
                else
                {
                    tab_changer_main_page(R_LEFT);
                }
                break;
            case R_RIGHT:
                if (_check_table_selected(TABLE_ROW) && active_tab == 1)
                {
                    _run_table_select_column(TABLE_OPERATION_UP);
                }
                else
                {
                    tab_changer_main_page(R_RIGHT);
                }
                break;
            case L_UP:
                if (push_btn)
                {
                    if (lv_obj_has_state(pid_push_btn, LV_STATE_CHECKED))
                        lv_obj_remove_state(pid_push_btn, LV_STATE_CHECKED);
                    else
                        lv_obj_add_state(pid_push_btn, LV_STATE_CHECKED);

                    // sendPIDTblInfo(TABLE_HEIGHT, TABLE_WIDTH, pid_table);
                }
                else if (pull_btn)
                {
                    if (lv_obj_has_state(pid_pull_btn, LV_STATE_CHECKED))
                        lv_obj_remove_state(pid_pull_btn, LV_STATE_CHECKED);
                    else
                        lv_obj_add_state(pid_pull_btn, LV_STATE_CHECKED);

                    // sendReceivePIDTblInfo();
                }
                break;
            case L_DOWN:
                break;
            case L_RIGHT:
                break;
            case L_LEFT:
                break;
            case SEND_PID_DONE:
                if (lv_obj_has_state(pid_push_btn, LV_STATE_CHECKED))
                    lv_obj_remove_state(pid_push_btn, LV_STATE_CHECKED);
                else
                    lv_obj_add_state(pid_push_btn, LV_STATE_CHECKED);
                break;
            case FLASH_PID_TBL:
                if (lv_obj_has_state(pid_pull_btn, LV_STATE_CHECKED))
                    lv_obj_remove_state(pid_pull_btn, LV_STATE_CHECKED);
                else
                    lv_obj_add_state(pid_pull_btn, LV_STATE_CHECKED);
                // getPIDInfoTable(TABLE_HEIGHT, TABLE_WIDTH, pid_table, NULL);
                break;
            default:
                GB_DEBUGE(DISP_TAG, "Wrong button id");
        }
        _update_pid_table_display();
    }
}

static void canvas_draw_task(lv_timer_t *timer)
{
    LV_UNUSED(timer);
    GB_GPIO_Set(TEST_IMU_IO, 1);

    quad3d_get_image(canvas_buffer);
    lv_canvas_set_buffer(model_canvas, canvas_buffer, LV_HOR_RES_MAX, LV_VER_RES_MAX, LV_COLOR_FORMAT_RGB565);
    GB_GPIO_Set(TEST_IMU_IO, 0);
}

static void btn_3d_model_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    GB_DEBUGI(DISP_TAG, "3D model button event");
    if (code == LV_EVENT_CLICKED)
    {
        if (!draw_task)
        {
            init_canvas();
            draw_task = lv_timer_create(canvas_draw_task, 100, NULL);
            lora_send_config = LORA_GET_MOTION_STATE;
        }
        else
        {
            lv_timer_delete(draw_task);
            draw_task = NULL;
            deinit_canvas();
            lora_send_config = LORA_SEND_NA;
        }
    }
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

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void welkin_widgets()
{
    tv = lv_tabview_create(lv_screen_active());

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
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

static void welkin_fc_create(lv_obj_t *parent)
{
    lv_obj_set_layout(parent, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(parent, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

    lv_obj_t *h = lv_obj_create(parent);
    lv_obj_set_layout(h, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(h, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(h, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_add_style(h, &style_box, 0);
    lv_obj_set_size(h, lv_pct(90), LV_SIZE_CONTENT);

    main_btn = lv_button_create(h);
    lv_obj_set_size(main_btn, lv_pct(45), LV_SIZE_CONTENT);
    lv_obj_add_flag(main_btn, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_add_state(main_btn, LV_STATE_CHECKED);
    lv_obj_t *label = lv_label_create(main_btn);
    lv_label_set_text(label, "GB Drone");
    lv_obj_center(label);

    model_3d_btn = lv_button_create(h);
    lv_obj_set_size(model_3d_btn, lv_pct(45), LV_SIZE_CONTENT);
    lv_obj_add_flag(model_3d_btn, LV_OBJ_FLAG_CHECKABLE);
    label = lv_label_create(model_3d_btn);
    lv_label_set_text(label, "3D Model");
    lv_obj_center(label);
    lv_obj_add_event_cb(model_3d_btn, btn_3d_model_event_cb, LV_EVENT_CLICKED, NULL);
}

static void lvgl_create_pid_table(lv_obj_t *parent, int32_t height, int32_t width)
{
    pid_table_obj = lv_table_create(parent);
    char pid_str[8];

    lv_style_init(&style_cell_selected);
    lv_style_set_border_color(&style_cell_selected, lv_color_hex(0xFF0000));
    lv_style_set_border_width(&style_cell_selected, 4);
    lv_style_set_border_opa(&style_cell_selected, LV_OPA_50);
    lv_style_set_border_side(&style_cell_selected, LV_BORDER_SIDE_FULL);

    // In LVGL 9, table cells use LV_PART_ITEMS for cell styling
    lv_obj_add_style(pid_table_obj, &style_cell_selected, LV_PART_ITEMS | LV_STATE_PRESSED);

    lv_table_set_column_count(pid_table_obj, TABLE_WIDTH);
    lv_table_set_row_count(pid_table_obj, TABLE_HEIGHT);
    lv_obj_align(pid_table_obj, LV_ALIGN_CENTER, 0, 0);

    for (int i = 0; i < TABLE_WIDTH; i++)
    {
    if (0 == i)
        lv_table_set_column_width(pid_table_obj, i, width / TABLE_WIDTH + FIRST_COL_PAD);
    else
        lv_table_set_column_width(pid_table_obj, i, (width - FIRST_COL_PAD) / TABLE_WIDTH);
    }

    /*Fill the first column*/
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

    pid_pull_btn = lv_button_create(parent);
    lv_obj_set_width(pid_pull_btn, width / 2);
    lv_obj_add_flag(pid_pull_btn, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_align(pid_pull_btn, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_set_style_bg_color(pid_pull_btn, lv_color_hex(0xffffff), 0);
    lv_obj_t *label = lv_label_create(pid_pull_btn);
    lv_label_set_text(label, "PULL");
    lv_obj_center(label);

    pid_push_btn = lv_button_create(parent);
    lv_obj_set_width(pid_push_btn, width / 2);
    lv_obj_add_flag(pid_push_btn, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_align(pid_push_btn, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_set_style_bg_color(pid_push_btn, lv_color_hex(0xffffff), 0);
    label = lv_label_create(pid_push_btn);
    lv_label_set_text(label, "PUSH");
    lv_obj_center(label);
}

static void pid_setting_create(lv_obj_t *parent)
{
    lv_obj_set_layout(parent, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(parent, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

    int32_t grid_w = lv_obj_get_content_width(parent);
    int32_t grid_h = lv_obj_get_content_height(parent);

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
            act = 2;
        else
            act--;
    }
    else if (R_RIGHT == op)
    {
        act++;
        if (act >= 3)
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
