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

#ifndef DISP_SPI_H
#define DISP_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include <stdint.h>
#include <stdbool.h>

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/
typedef enum _disp_spi_send_flag_t {
    DISP_SPI_SEND_QUEUED        = 0x00000000,
    DISP_SPI_SEND_POLLING       = 0x00000001,
    DISP_SPI_SEND_SYNCHRONOUS   = 0x00000002,
    DISP_SPI_SIGNAL_FLUSH       = 0x00000004,
    DISP_SPI_RECEIVE            = 0x00000008,
    DISP_SPI_CMD_8              = 0x00000010, /* Reserved */
    DISP_SPI_CMD_16             = 0x00000020, /* Reserved */
    DISP_SPI_ADDRESS_8          = 0x00000040,
    DISP_SPI_ADDRESS_16         = 0x00000080,
    DISP_SPI_ADDRESS_24         = 0x00000100,
    DISP_SPI_ADDRESS_32         = 0x00000200,
    DISP_SPI_MODE_DIO           = 0x00000400,
    DISP_SPI_MODE_QIO           = 0x00000800,
    DISP_SPI_MODE_DIOQIO_ADDR   = 0x00001000,
	DISP_SPI_VARIABLE_DUMMY		= 0x00002000,
} disp_spi_send_flag_t;

void disp_spi_transaction(const uint8_t *data, size_t length,
    disp_spi_send_flag_t flags, uint8_t *out, uint64_t addr, uint8_t dummy_bits);

void disp_wait_for_pending_transactions(void);
void disp_spi_acquire(void);
void disp_spi_release(void);

static inline void disp_spi_send_data(uint8_t *data, size_t length) {
    disp_spi_transaction(data, length, DISP_SPI_SEND_POLLING, NULL, 0, 0);
}

static inline void disp_spi_send_colors(uint8_t *data, size_t length) {
    disp_spi_transaction(data, length,
        DISP_SPI_SEND_QUEUED | DISP_SPI_SIGNAL_FLUSH,
        NULL, 0, 0);
}

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*DISP_SPI_H*/
