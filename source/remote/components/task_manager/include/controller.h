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
} GB_REMOTE_CONTROL_ID;

typedef enum
{
    GB_FLY_MODE,
    GB_PID_SETTING_MODE,
    GB_USER_MODE_MAX,
} GB_REMOTE_USER_MODE;

void adc_wrapper_init(void);
void adc_read_by_item(uint8_t item, uint16_t *adc_val, uint8_t is_constrained);

GB_REMOTE_USER_MODE gb_get_user_mode();
void gb_remote_single_control(GB_REMOTE_CONTROL_ID button_id);

#endif /* end of include guard: _GB_CONTROLLER__ */
