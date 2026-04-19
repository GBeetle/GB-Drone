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

#ifndef _GB_CONTROLLER__
#define _GB_CONTROLLER__

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "gpio_setting.h"
#include "results.h"

typedef enum
{
    ADC_THROTTLE,
    ADC_YAW,
    ADC_PITCH,
    ADC_ROLL,
    ADC_TYPE_MAX,
} ADC_SAMPLE_ITEM;

typedef enum
{
    L_DOWN_R_UP,
    L_DOWN_R_DOWN,
    L_UP_R_UP,
    L_UP_R_DOWN,
    R_LEFT,
    R_RIGHT,
    L_UP,
    L_DOWN,
    L_LEFT,
    L_RIGHT,
    SEND_PID_DONE,
    FLASH_PID_TBL,

    BTN_DPAD_UP,
    BTN_DPAD_DOWN,
    BTN_DPAD_LEFT,
    BTN_DPAD_RIGHT,
    BTN_DPAD_MID,

    BTN_OP_A,
    BTN_OP_B,
    BTN_OP_X,
    BTN_OP_Y,

    BTN_OP_MENU,
    BTN_OP_OPTION,
    BTN_OP_SELECT,
    BTN_OP_START,

    TOGGLE_SWITHCH_CHANGED,

    GB_CONTROL_ID_MAX,
} GB_REMOTE_CONTROL_ID;

typedef enum
{
    GB_FLY_MODE,
    GB_PID_SETTING_MODE,
    GB_USER_MODE_MAX,
} GB_REMOTE_USER_MODE;

typedef enum
{
    TOGGLE_SW_1,
    TOGGLE_SW_2,
    TOGGLE_SW_3,
    TOGGLE_SW_4,
    TOGGLE_SW_MAX
} GB_TOGGLE_SWITCH_ID;

typedef struct
{
    bool sw1_state;
    bool sw2_state;
    bool sw3_state;
    bool sw4_state;
} GB_TOGGLE_SWITCH_STATE;

typedef enum
{
    GB_EVENT_BUTTON,
    GB_EVENT_TOGGLE_SWITCH,
} GB_EVENT_TYPE;

typedef struct
{
    GB_EVENT_TYPE type;
    union
    {
        GB_REMOTE_CONTROL_ID button_id;
        GB_TOGGLE_SWITCH_STATE switch_state;
    } data;

} GB_CONTROL_EVENT;

GB_RESULT adc_wrapper_init(void);
void adc_read_by_item(uint8_t item, uint16_t *adc_val, uint8_t is_constrained);

void controller_input_init(void);
QueueHandle_t controller_get_event_queue(void);
void controller_get_toggle_state(GB_TOGGLE_SWITCH_STATE *state);
void controller_set_toggle_state(const GB_TOGGLE_SWITCH_STATE *state);

GB_REMOTE_USER_MODE gb_get_user_mode();
void gb_remote_single_control(GB_REMOTE_CONTROL_ID button_id);

#endif /* end of include guard: _GB_CONTROLLER__ */
