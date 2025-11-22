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

#include "pwm.h"
#include "error_handle.h"
#include "driver/ledc.h"
#include "log_sys.h"

#define LEDC_SPEED_MODE  LEDC_LOW_SPEED_MODE

GB_RESULT GB_PWM_Init(GB_PWM_TYPE_T *dev)
{
    GB_RESULT res = GB_OK;

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_SPEED_MODE,
        .duty_resolution  = dev->resolution,
        .timer_num        = dev->timer,
        .freq_hz          = dev->freq,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    CHK_ESP_ERROR(ledc_timer_config(&ledc_timer), GB_PWM_INIT_FAIL);

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_SPEED_MODE,
        .channel        = dev->channel,
        .timer_sel      = dev->timer,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = dev->io,
        .duty           = 0,
        .hpoint         = 0,
    };
    CHK_ESP_ERROR(ledc_channel_config(&ledc_channel), GB_PWM_INIT_FAIL);

error_exit:
    return res;
}

GB_RESULT GB_PWM_Start(GB_PWM_TYPE_T dev)
{
    GB_RESULT res = GB_OK;
    uint32_t max_duty = (1 << dev.resolution) - 1;
    uint32_t duty = max_duty / 2;

    CHK_ESP_ERROR(ledc_set_duty(LEDC_SPEED_MODE, dev.channel, duty), GB_PWM_START_FAIL);
    CHK_ESP_ERROR(ledc_update_duty(LEDC_SPEED_MODE, dev.channel), GB_PWM_START_FAIL);

error_exit:
    return res;
}
