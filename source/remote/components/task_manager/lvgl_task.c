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
#include "lv_widgets/lv_spinbox.h"
#include "controller.h"
#include "quad_3d.h"
#include "tft_espi.h"
#include "tft_sprite.h"
#include "lvgl_driver.h"

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

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void welkin_fc_create(lv_obj_t *parent);
static void pid_setting_create(lv_obj_t *parent);
static void color_chg_event_cb(lv_obj_t *sw, lv_event_t e);
static void tab_content_anim_create(lv_obj_t *parent);
static void tab_changer_main_page(GB_REMOTE_CONTROL_ID op);

/**********************
 *  STATIC VARIABLES
 **********************/
static lv_obj_t *tv = NULL;
static lv_obj_t *t1 = NULL;
static lv_obj_t *t2 = NULL;

static lv_style_t style_cell1;
static lv_style_t style_cell4;

static lv_style_t style_box;
lv_obj_t *remote_controller;
lv_obj_t *pid_table_obj;
lv_obj_t *pid_pull_btn, *pid_push_btn, *main_btn, *model_3d_btn;

static uint16_t pid_table[TABLE_HEIGHT][TABLE_WIDTH] = {0};
static bool pull_btn = false;
static bool push_btn = false;
static int selected_row = -1;
static int selected_column = -1;

lv_task_t *draw_task;
lv_obj_t *model_canvas = NULL;
uint32_t canvas_buffer_size = LV_HOR_RES_MAX * LV_VER_RES_MAX * 2;
uint16_t *canvas_buffer = NULL;

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
        model_canvas = lv_canvas_create(lv_scr_act(), NULL);
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
    lv_obj_set_user_data(remote_controller, &button_id);
    lv_event_send(remote_controller, LV_EVENT_VALUE_CHANGED, NULL);
}

void remote_controller_event_cb(lv_obj_t *remote_controller, lv_event_t event)
{
    char pid_str[8];

    if (event == LV_EVENT_VALUE_CHANGED)
    {
        int *btn_id = (int *)lv_obj_get_user_data(remote_controller);
        uint16_t active_tab = lv_tabview_get_tab_act(tv);

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
                    lv_page_scroll_ver(t2, lv_obj_get_height(t2) / (TABLE_HEIGHT - 4));
                }
                else if (active_tab == 0)
                {
                    lv_btn_toggle(main_btn);
                    lv_btn_toggle(model_3d_btn);

                    if (lv_obj_get_state(model_3d_btn, LV_STATE_CHECKED))
                    {
                        lv_event_send(model_3d_btn, LV_EVENT_CLICKED, NULL);
                    }
#if 0 // can not stop 3d model draw now !
                    else
                    {
                        lv_event_send(model_3d_btn, LV_EVENT_RELEASED, NULL);
                    }
#endif
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
                    lv_page_scroll_ver(t2, -lv_obj_get_height(t2) / (TABLE_HEIGHT - 2));
                }
                else if (active_tab == 0)
                {
                    lv_btn_toggle(main_btn);
                    lv_btn_toggle(model_3d_btn);

                    if (lv_obj_get_state(model_3d_btn, LV_STATE_CHECKED))
                    {
                        lv_event_send(model_3d_btn, LV_EVENT_CLICKED, NULL);
                    }
#if 0 // can not stop 3d model draw now !
                    else
                    {
                        lv_event_send(model_3d_btn, LV_EVENT_RELEASED, NULL);
                    }
#endif
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
                    lv_btn_toggle(pid_push_btn);
                    // sendPIDTblInfo(TABLE_HEIGHT, TABLE_WIDTH, pid_table);
                }
                else if (pull_btn)
                {
                    lv_btn_toggle(pid_pull_btn);
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
                lv_btn_toggle(pid_push_btn);
                break;
            case FLASH_PID_TBL:
                lv_btn_toggle(pid_pull_btn);
                // getPIDInfoTable(TABLE_HEIGHT, TABLE_WIDTH, pid_table, NULL);
                break;
            default:
                GB_DEBUGE(DISP_TAG, "Wrong button id");
        }
        for (int i = 1; i < TABLE_HEIGHT; i++)
        {
            for (int j = 1; j < TABLE_WIDTH; j++)
            {
                snprintf(pid_str, 8, "%d", pid_table[i][j]);
                lv_table_set_cell_value(pid_table_obj, i, j, pid_str);
            }
            // pidInit_t *tmp_pid_tbl = NULL;
            // tmp_pid_tbl = pid_table[i];
            // GB_DEBUGI(DISP_TAG, "PID_TBL[%d]: %d, %d, %d", i, tmp_pid_tbl->kp, tmp_pid_tbl->ki, tmp_pid_tbl->kd);
        }
        for (int i = 0; i < TABLE_HEIGHT; i++)
        {
            for (int j = 0; j < TABLE_WIDTH; j++)
            {
                if (i == selected_row && j == selected_column)
                {
                    lv_table_set_cell_type(pid_table_obj, selected_row, selected_column, LV_TABLE_PART_CELL4);
                }
                else
                {
                    lv_table_set_cell_type(pid_table_obj, i, j, LV_TABLE_PART_CELL1);
                }
            }
        }
        if (pull_btn)
            lv_obj_set_style_local_border_color(pid_pull_btn, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_RED);
        else
            lv_obj_set_style_local_border_color(pid_pull_btn, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, lv_color_hex(0xffffff));
        if (push_btn)
            lv_obj_set_style_local_border_color(pid_push_btn, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_RED);
        else
            lv_obj_set_style_local_border_color(pid_push_btn, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, lv_color_hex(0xffffff));
    }
}

void canvas_draw_task()
{
    lv_draw_img_dsc_t img_dsc;

    lv_draw_img_dsc_init(&img_dsc);

    quad3d_get_image(canvas_buffer);
    lv_canvas_set_buffer(model_canvas, canvas_buffer, LV_HOR_RES_MAX, LV_VER_RES_MAX, LV_IMG_CF_TRUE_COLOR);
}

void btn_3d_model_event_cb(lv_obj_t *sw, lv_event_t e)
{
    GB_DEBUGI(DISP_TAG, "3D model button event");
    if (e == LV_EVENT_CLICKED)
    {
        if (!draw_task)
        {
            init_canvas();
            draw_task = lv_task_create(canvas_draw_task, 10, LV_TASK_PRIO_HIGHEST, NULL);
        }
        else
        {
            lv_task_del(draw_task);
            draw_task = NULL;
            deinit_canvas();
        }
    }
}

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void welkin_widgets()
{
    tv = lv_tabview_create(lv_scr_act(), NULL);
    if (LV_THEME_DEFAULT_INIT == lv_theme_material_init)
    {
        lv_disp_size_t disp_size = lv_disp_get_size_category(NULL);
        if (disp_size >= LV_DISP_SIZE_MEDIUM)
        {
            lv_obj_set_style_local_pad_left(tv, LV_TABVIEW_PART_TAB_BG, LV_STATE_DEFAULT, LV_HOR_RES / 2);
            lv_obj_t *sw = lv_switch_create(lv_scr_act(), NULL);
            if (lv_theme_get_flags() & LV_THEME_MATERIAL_FLAG_DARK)
                lv_switch_on(sw, LV_ANIM_OFF);
            lv_obj_set_event_cb(sw, color_chg_event_cb);
            lv_obj_set_pos(sw, LV_DPX(10), LV_DPX(10));
            lv_obj_set_style_local_value_str(sw, LV_SWITCH_PART_BG, LV_STATE_DEFAULT, "Dark");
            lv_obj_set_style_local_value_align(sw, LV_SWITCH_PART_BG, LV_STATE_DEFAULT, LV_ALIGN_OUT_RIGHT_MID);
            lv_obj_set_style_local_value_ofs_x(sw, LV_SWITCH_PART_BG, LV_STATE_DEFAULT, LV_DPI / 35);
        }
    }

    t1 = lv_tabview_add_tab(tv, "GB Drone");
    t2 = lv_tabview_add_tab(tv, "PID");

    lv_style_init(&style_box);
    lv_style_set_value_align(&style_box, LV_STATE_DEFAULT, LV_ALIGN_OUT_TOP_LEFT);
    lv_style_set_value_ofs_y(&style_box, LV_STATE_DEFAULT, -LV_DPX(10));
    lv_style_set_margin_top(&style_box, LV_STATE_DEFAULT, LV_DPX(30));

    welkin_fc_create(t1);
    pid_setting_create(t2);

    // Create obj to receive callback
    remote_controller = lv_obj_create(lv_scr_act(), NULL);
    lv_obj_set_hidden(remote_controller, true);
    lv_obj_set_event_cb(remote_controller, remote_controller_event_cb);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

static void welkin_fc_create(lv_obj_t *parent)
{
    lv_page_set_scrl_layout(parent, LV_LAYOUT_PRETTY_TOP);

    lv_disp_size_t disp_size = lv_disp_get_size_category(NULL);
    lv_coord_t grid_w = lv_page_get_width_grid(parent, disp_size <= LV_DISP_SIZE_SMALL ? 1 : 2, 1);

    lv_obj_t *h = lv_cont_create(parent, NULL);
    lv_cont_set_layout(h, LV_LAYOUT_PRETTY_MID);
    lv_obj_add_style(h, LV_CONT_PART_MAIN, &style_box);
    lv_obj_set_drag_parent(h, true);

    lv_obj_set_style_local_value_str(h, LV_CONT_PART_MAIN, LV_STATE_DEFAULT, "");

    lv_cont_set_fit2(h, LV_FIT_NONE, LV_FIT_TIGHT);
    lv_obj_set_width(h, grid_w);
    main_btn = lv_btn_create(h, NULL);
    lv_btn_set_fit2(main_btn, LV_FIT_NONE, LV_FIT_TIGHT);
    lv_obj_set_width(main_btn, lv_obj_get_width_grid(h, disp_size <= LV_DISP_SIZE_SMALL ? 1 : 2, 1));
    lv_obj_t *label = lv_label_create(main_btn, NULL);
    lv_label_set_text(label, "GB Drone");
    lv_btn_toggle(main_btn);

    model_3d_btn = lv_btn_create(h, main_btn);
    label = lv_label_create(model_3d_btn, NULL);
    lv_label_set_text(label, "3D Model");
    lv_obj_set_event_cb(model_3d_btn, btn_3d_model_event_cb);

    tab_content_anim_create(parent);
}

static void lvgl_create_pid_table(lv_obj_t *parent, lv_coord_t height, lv_coord_t width)
{
    pid_table_obj = lv_table_create(parent, NULL);
    char pid_str[8];

    lv_style_init(&style_cell1);
    lv_style_set_bg_color(&style_cell1, LV_STATE_DEFAULT, lv_color_hex(0xffffff));
    lv_style_set_text_font(&style_cell1, LV_STATE_DEFAULT, CONFIG_LV_THEME_DEFAULT_FONT_SMALL);

    lv_style_init(&style_cell4);
    lv_style_set_border_color(&style_cell4, LV_STATE_DEFAULT, LV_COLOR_RED);
    lv_style_set_border_width(&style_cell4, LV_STATE_DEFAULT, 4);
    lv_style_set_border_opa(&style_cell4, LV_STATE_DEFAULT, LV_OPA_50);
    lv_style_set_border_side(&style_cell4, LV_STATE_DEFAULT, LV_BORDER_SIDE_FULL);
    lv_style_set_text_font(&style_cell4, LV_STATE_DEFAULT, CONFIG_LV_THEME_DEFAULT_FONT_SMALL);

#if 0

    static lv_style_t style_cell2;
    static lv_style_t style_cell3;

    lv_style_init(&style_cell2);
    lv_style_init(&style_cell3);

    //lv_style_set_bg_opa(&style_cell, LV_STATE_DEFAULT, LV_OPA_50);
    lv_style_set_text_color(&style_cell2, LV_STATE_DEFAULT, LV_COLOR_ORANGE);

    lv_style_set_outline_width(&style_cell3, LV_STATE_DEFAULT, 2);
    lv_style_set_outline_color(&style_cell3, LV_STATE_DEFAULT, LV_COLOR_BLUE);
    lv_style_set_outline_pad(&style_cell3, LV_STATE_DEFAULT, 8);

    lv_obj_add_style(pid_table_obj, LV_TABLE_PART_CELL2, &style_cell2);
    lv_obj_add_style(pid_table_obj, LV_TABLE_PART_CELL3, &style_cell3);
#endif
    lv_obj_add_style(pid_table_obj, LV_TABLE_PART_CELL1, &style_cell1);
    lv_obj_add_style(pid_table_obj, LV_TABLE_PART_CELL4, &style_cell4);

    lv_table_set_col_cnt(pid_table_obj, TABLE_WIDTH);
    lv_table_set_row_cnt(pid_table_obj, TABLE_HEIGHT);
    // lv_obj_set_width(pid_table_obj, width);
    lv_obj_align(pid_table_obj, parent, LV_ALIGN_CENTER, 0, 0);
    // lv_obj_set_height(pid_table_obj, height / TABLE_HEIGHT);

    for (int i = 0; i < TABLE_WIDTH; i++)
    {
        if (0 == i)
            lv_table_set_col_width(pid_table_obj, i, width / TABLE_WIDTH + FIRST_COL_PAD);
        else
            lv_table_set_col_width(pid_table_obj, i, (width - FIRST_COL_PAD) / TABLE_WIDTH);
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

    /*Fill the second column*/
    for (int i = 1; i < TABLE_HEIGHT; i++)
    {
        for (int j = 1; j < TABLE_WIDTH; j++)
        {
            snprintf(pid_str, 8, "%d", pid_table[i][j]);
            lv_table_set_cell_value(pid_table_obj, i, j, pid_str);
        }
    }

    // Create a horizontal layout container
    // lv_obj_t *h_layout = lv_cont_create(parent, NULL);
    // lv_cont_set_layout(h_layout, LV_LAYOUT_ROW_MID); // Set the layout to row and center align items
    // lv_cont_set_fit2(h_layout, LV_FIT_PARENT, LV_FIT_TIGHT);

    pid_pull_btn = lv_btn_create(parent, NULL);
    // lv_btn_set_fit2(btn, LV_FIT_NONE, LV_FIT_TIGHT);
    // lv_obj_set_width(btn, lv_obj_get_width_grid(h, disp_size <= LV_DISP_SIZE_SMALL ? 1 : 2, 1));
    lv_obj_t *label = lv_label_create(pid_pull_btn, NULL);
    lv_obj_set_width(pid_pull_btn, width / 2);
    lv_obj_align(pid_pull_btn, parent, LV_ALIGN_IN_LEFT_MID, 0, 0);
    lv_label_set_text(label, "PULL");
    lv_obj_set_style_local_bg_color(pid_pull_btn, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, lv_color_hex(0xffffff));

    pid_push_btn = lv_btn_create(parent, NULL);
    // lv_btn_toggle(btn);
    label = lv_label_create(pid_push_btn, NULL);
    lv_obj_set_width(pid_push_btn, width / 2);
    lv_obj_align(pid_push_btn, parent, LV_ALIGN_IN_RIGHT_MID, 0, 0);
    lv_label_set_text(label, "PUSH");
    lv_obj_set_style_local_bg_color(pid_push_btn, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, lv_color_hex(0xffffff));
}

static void pid_setting_create(lv_obj_t *parent)
{
    lv_page_set_scrl_layout(parent, LV_LAYOUT_PRETTY_TOP);

    lv_disp_size_t disp_size = lv_disp_get_size_category(NULL);

    lv_coord_t grid_h = lv_page_get_height_grid(parent, 1, 1);
    lv_coord_t grid_w;
    if (disp_size <= LV_DISP_SIZE_SMALL)
        grid_w = lv_page_get_width_grid(parent, 1, 1);
    else
        grid_w = lv_page_get_width_grid(parent, 2, 1);

    lvgl_create_pid_table(parent, grid_h, grid_w - 26);
}

static void color_chg_event_cb(lv_obj_t *sw, lv_event_t e)
{
    if (LV_THEME_DEFAULT_INIT != lv_theme_material_init)
        return;
    if (e == LV_EVENT_VALUE_CHANGED)
    {
        uint32_t flag = LV_THEME_MATERIAL_FLAG_LIGHT;
        if (lv_switch_get_state(sw))
            flag = LV_THEME_MATERIAL_FLAG_DARK;

        LV_THEME_DEFAULT_INIT(lv_theme_get_color_primary(), lv_theme_get_color_secondary(),
                              flag,
                              lv_theme_get_font_small(), lv_theme_get_font_normal(), lv_theme_get_font_subtitle(), lv_theme_get_font_title());
    }
}

static void tab_content_anim_create(lv_obj_t *parent)
{
    lv_anim_t a;
    lv_obj_t *scrl = lv_page_get_scrl(parent);
    lv_coord_t y_start = lv_obj_get_style_pad_top(parent, LV_PAGE_PART_BG);
    lv_coord_t anim_h = lv_obj_get_height(scrl) - lv_obj_get_height_fit(parent);
    uint32_t anim_time = lv_anim_speed_to_time(LV_DPI, 0, anim_h);

    lv_anim_init(&a);
    lv_anim_set_var(&a, scrl);
    lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_y);
    lv_anim_set_values(&a, y_start, y_start - anim_h);
    lv_anim_set_time(&a, anim_time);
    lv_anim_set_playback_time(&a, anim_time);
    lv_anim_set_playback_delay(&a, 200);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_anim_set_repeat_delay(&a, 200);
    lv_anim_start(&a);
}

static void tab_changer_main_page(GB_REMOTE_CONTROL_ID op)
{
    int16_t act = lv_tabview_get_tab_act(tv);
    if (R_LEFT == op)
        act--;
    else if (R_RIGHT == op)
        act++;
    else
    {
        GB_DEBUGW(DISP_TAG, "Wrong operation for main page tab changer");
    }
    if (act >= 3)
        act = 0;
    else if (act < 0)
        act = 3;

    lv_tabview_set_tab_act(tv, act, LV_ANIM_ON);

#if 0
    switch(act) {
    case 0:
        tab_content_anim_create(t1);
        break;
    case 1:
        tab_content_anim_create(t2);
        break;
    case 2:
        tab_content_anim_create(t3);
        break;
    }
#endif
}

GB_REMOTE_USER_MODE gb_get_user_mode()
{
    if (tv)
        return lv_tabview_get_tab_act(tv);
    else
        return GB_USER_MODE_MAX;
}
