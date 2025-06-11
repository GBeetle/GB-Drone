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

/*********************
 *      INCLUDES
 *********************/
#include <string.h>

#include "spi_bus.h"
#include "disp_spi.h"
#include "disp_driver.h"

void disp_spi_transaction(const uint8_t *data, size_t length, disp_spi_send_flag_t flags,
                          uint8_t *out, uint64_t addr, uint8_t dummy_bits)
{
    uint8_t cmd_bits = 0;
    uint8_t addr_bits = 0;

    if (flags & DISP_SPI_ADDRESS_8) {
        addr_bits = 8;
    } else if (flags & DISP_SPI_ADDRESS_16) {
        addr_bits = 16;
    } else if (flags & DISP_SPI_ADDRESS_24) {
        addr_bits = 24;
    } else if (flags & DISP_SPI_ADDRESS_32) {
        addr_bits = 32;
    }

    if (flags & DISP_SPI_CMD_8) {
        cmd_bits = 8;
    } else if (flags & DISP_SPI_CMD_16) {
        cmd_bits = 16;
    }

    // fspi.writeBytes(&fspi, GB_SPI_DEV_0, addr, length, data);
    fspi.readWriteBytesWithConfig(&fspi, GB_SPI_DEV_0, addr, length, out, data, cmd_bits, addr_bits, dummy_bits);
#if 0
    /* Poll/Complete/Queue transaction */
    if (flags & DISP_SPI_SEND_POLLING) {
        disp_wait_for_pending_transactions();    /* before polling, all previous pending transactions need to be serviced */
        spi_device_polling_transmit(spi, (spi_transaction_t *) &t);
    } else if (flags & DISP_SPI_SEND_SYNCHRONOUS) {
        disp_wait_for_pending_transactions();    /* before synchronous queueing, all previous pending transactions need to be serviced */
        spi_device_transmit(spi, (spi_transaction_t *) &t);
    } else {
        // TODO: use queue to manage spi trans
    }
#endif
}

// TODO: don't support DISP_SPI_SEND_QUEUED know
void disp_wait_for_pending_transactions(void)
{

}

// do nothing, when you need to acquire bus for multi-spi devices, you should use DISP_SPI_SEND_SYNCHRONOUS to avoid bus conflict
void disp_spi_acquire(void)
{

}

void disp_spi_release(void)
{

}
