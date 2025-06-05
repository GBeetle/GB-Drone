/*
 * This file is part of welkin project (https://github.com/GBeetle/welkin).
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

#include "esc_system.h"
#include "io_define.h"
#include "error_handle.h"
#include "gb_timer.h"

#define MOTOR_INITIALIZE(idx, u, p, t, i) \
    motor[idx].unit = u;  \
    motor[idx].ports = p; \
    motor[idx].timer = t; \
    motor[idx].pin = i;   \

ESC_MOTOR_T motor[4];

GB_RESULT esc_system_init(int gpio_s0, int gpio_s1, int gpio_s2, int gpio_s3)
{
    GB_RESULT res = GB_OK;

    MOTOR_INITIALIZE(0, MCPWM_UNIT_0, MCPWM0A, MCPWM_TIMER_0, gpio_s0);
    MOTOR_INITIALIZE(1, MCPWM_UNIT_0, MCPWM1A, MCPWM_TIMER_1, gpio_s1);
    MOTOR_INITIALIZE(2, MCPWM_UNIT_1, MCPWM0A, MCPWM_TIMER_0, gpio_s2);
    MOTOR_INITIALIZE(3, MCPWM_UNIT_1, MCPWM1A, MCPWM_TIMER_1, gpio_s3);

    mcpwm_config_t pwm_config = {
        .frequency    = 50,                   // frequency = 50Hz, i.e. for every servo motor time period should be 20ms
        .cmpr_a       = 0,                   // duty cycle of PWMxA = 0
        .cmpr_b       = 0,
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0,
    };

    CHK_ESP_ERROR(mcpwm_init(motor[0].unit, motor[0].timer, &pwm_config), GB_ESC_PWM_INIT_FAIL);
    GB_SleepMs(100); // must delay after timer start !!!
    CHK_ESP_ERROR(mcpwm_init(motor[1].unit, motor[1].timer, &pwm_config), GB_ESC_PWM_INIT_FAIL);
    GB_SleepMs(100);
    CHK_ESP_ERROR(mcpwm_init(motor[2].unit, motor[2].timer, &pwm_config), GB_ESC_PWM_INIT_FAIL);
    GB_SleepMs(100);
    CHK_ESP_ERROR(mcpwm_init(motor[3].unit, motor[3].timer, &pwm_config), GB_ESC_PWM_INIT_FAIL);
    GB_SleepMs(100);

    CHK_ESP_ERROR(mcpwm_gpio_init(motor[0].unit, motor[0].ports, motor[0].pin), GB_ESC_IO_INIT_FAIL);
    CHK_ESP_ERROR(mcpwm_gpio_init(motor[1].unit, motor[1].ports, motor[1].pin), GB_ESC_IO_INIT_FAIL);
    CHK_ESP_ERROR(mcpwm_gpio_init(motor[2].unit, motor[2].ports, motor[2].pin), GB_ESC_IO_INIT_FAIL);
    CHK_ESP_ERROR(mcpwm_gpio_init(motor[3].unit, motor[3].ports, motor[3].pin), GB_ESC_IO_INIT_FAIL);

#if 0
    mcpwm_config_t pwm_config = {
        .frequency = 1000,
        .cmpr_a = 10,
        .cmpr_b = 0,
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0,
    };
    ESP_ERROR_CHECK(mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config));
    ESP_ERROR_CHECK(mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config));
    ESP_ERROR_CHECK(mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config));

    // bind output to GPIO
    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MCPWM_S0));
    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, MCPWM_S1));
    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, MCPWM_S2));
#endif

error_exit:
    return res;
}

GB_RESULT esc_set_duty(int idx, float duty)
{
    GB_RESULT res = GB_OK;

    // duty 1ms/20ms ~ 2ms/20ms : 5%~10%
    CHK_BOOL(duty >= 5 && duty <= 10);

    CHK_ESP_ERROR(mcpwm_set_duty(motor[idx].unit, motor[idx].timer, MCPWM_OPR_A, duty), GB_ESC_SET_DUTY_FAIL);
    CHK_ESP_ERROR(mcpwm_set_duty_type(motor[idx].unit, motor[idx].timer, MCPWM_OPR_A, MCPWM_DUTY_MODE_0), GB_ESC_SET_DUTY_FAIL);

error_exit:
    return res;
}

