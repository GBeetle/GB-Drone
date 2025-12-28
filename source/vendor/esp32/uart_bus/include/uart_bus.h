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

#ifndef _UART_BUS_C_
#define _UART_BUS_C_

#include <stdint.h>
#include "error_handle.h"

#define GB_UART_RTX_BUF_SIZE 1004

typedef int GB_UART_Port;

struct uart {
    GB_UART_Port host;

    GB_RESULT (*begin)(struct uart *uart, uint32_t baud_rate, uint32_t tx_pin, uint32_t rx_pin);
    GB_RESULT (*close)(struct uart *uart);

    GB_RESULT (*writeBytes)(struct uart *uart, size_t length, const uint8_t *data);
    GB_RESULT (*readBytes)(struct uart *uart, size_t length, uint8_t *data);
    GB_RESULT (*readLine)(struct uart *uart, size_t length, uint8_t *data);
};

// Default Objects
extern struct uart uart1;
extern struct uart uart2;

#endif /* end of include guard: _UART_BUS_C_ */
