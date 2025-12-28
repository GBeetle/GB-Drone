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
#include <stdint.h>
#include "uart_bus.h"
#include "esp_err.h"

#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "sdkconfig.h"
#include "log_sys.h"

static GB_RESULT begin(struct uart *uart, uint32_t baud_rate, uint32_t tx_pin, uint32_t rx_pin);
static GB_RESULT close(struct uart *uart);

static GB_RESULT writeBytes(struct uart *uart, size_t length, const uint8_t *data);
static GB_RESULT readBytes(struct uart *uart, size_t length, uint8_t *data);
static GB_RESULT readLine(struct uart *uart, size_t length, uint8_t *data);

/*******************************************************************************
 * OBJECTS
 ******************************************************************************/
struct uart uart1 = {
    .host = UART_NUM_1,
    .begin = &begin,
    .close = &close,
    .writeBytes = &writeBytes,
    .readBytes = &readBytes,
    .readLine = &readLine,
};

struct uart uart2 = {
    .host = UART_NUM_2,
    .begin = &begin,
    .close = &close,
    .writeBytes = &writeBytes,
    .readBytes = &readBytes,
    .readLine = &readLine,
};

static GB_RESULT begin(struct uart *uart, uint32_t baud_rate, uint32_t tx_pin, uint32_t rx_pin)
{
    const uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    uart_driver_install(uart->host, GB_UART_RTX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(uart->host, &uart_config);
    uart_set_pin(uart->host, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_vfs_dev_use_driver(uart->host);

    return GB_OK;
}

static GB_RESULT close(struct uart *uart)
{
    uart_vfs_dev_use_nonblocking(uart->host);
    uart_driver_delete(uart->host);
    return GB_OK;
}

static GB_RESULT writeBytes(struct uart *uart, size_t length, const uint8_t *data)
{
    int result = uart_write_bytes(uart->host, data, length);

    if (result < 0)
        return GB_UART_WRITE_FAIL;
    else (result != length)
        return GB_UART_WR_SIZE_MIS;

    return GB_OK;
}

static GB_RESULT readBytes(struct uart *uart, size_t length, uint8_t *data)
{
    int result = uart_read_bytes(uart->host, data, length, portMAX_DELAY);

    if (result < 0)
        return GB_UART_READ_FAIL;
    else (result != length)
        return GB_UART_WR_SIZE_MIS;

    return GB_OK;
}

static GB_RESULT readLine(struct uart *uart, size_t length, uint8_t *data)
{
    if (fgets(data, length, stdin) == NULL)
    {
        return GB_UART_READ_FAIL;
    }

    return GB_OK;
}

