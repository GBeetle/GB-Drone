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

#include "gpio_setting.h"
#include "driver/gpio.h"

GB_RESULT GB_GPIO_Init(uint32_t pin, GB_GPIO_MODE mode, GB_GPIO_STATUS status)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << pin),
        .pull_down_en = false,
        .pull_up_en = false,
    };

    switch (mode)
    {
        case GB_GPIO_DISABLE:
            io_conf.mode = GPIO_MODE_DISABLE;
            break;
        case GB_GPIO_INPUT:
            io_conf.mode = GPIO_MODE_INPUT;
            break;
        case GB_GPIO_OUTPUT:
            io_conf.mode = GPIO_MODE_OUTPUT;
            break;
        default:
            return GB_INVALID_ARGUMENT;
    }

    switch (status)
    {
        case GB_GPIO_PULL_DISABLE:
            break;
        case GB_GPIO_PULLUP:
            io_conf.pull_up_en = true;
            break;
        case GB_GPIO_PULLDOWN:
            io_conf.pull_down_en = true;
            break;
        default:
            return GB_INVALID_ARGUMENT;
    }

    gpio_config(&io_conf);
    return GB_OK;
}

GB_RESULT GB_GPIO_Reset(uint32_t pin)
{
    gpio_reset_pin(pin);
    return GB_OK;
}

GB_RESULT GB_GPIO_Set(uint32_t pin, uint32_t level)
{
    gpio_set_level(pin, level);
    return GB_OK;
}

GB_RESULT GB_GPIO_SetDirection(uint32_t pin, GB_GPIO_MODE mode)
{
    switch (mode)
    {
        case GB_GPIO_INPUT:
            gpio_set_direction(pin, GPIO_MODE_INPUT);
            break;
        case GB_GPIO_OUTPUT:
            gpio_set_direction(pin, GPIO_MODE_OUTPUT);
            break;
        default:
            return GB_INVALID_ARGUMENT;
    }
    return GB_OK;
}

GB_RESULT GB_GPIO_Get(uint32_t pin, uint32_t *level)
{
    *level = gpio_get_level(pin);
    return GB_OK;
}