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

#include "anotic_debug.h"

#define ANOTIC_DEBUG_V2 0

uint8_t __bswap_8(uint8_t value)
{
    return (value>>4) | (value<<4);
}

uint16_t float2int16(float value, int scale)
{
    uint32_t num = value * scale;
    return num & 0xffffffff;
}

int anotc_init_data(uint8_t *send_buffer, uint32_t *realLen, uint8_t command_id, uint32_t arg_nums, ...)
{
    va_list args;
    uint32_t idx = 0;
    uint8_t checksum = 0;
    uint8_t addcheck = 0;
#if ANOTIC_DEBUG_V2
    uint8_t extraFrame = 6;
    uint16_t *dataLen = &send_buffer[4];
#else
    uint8_t extraFrame = 4;
    uint8_t *dataLen = &send_buffer[3];
#endif

    va_start(args, arg_nums);
#if ANOTIC_DEBUG_V2
    send_buffer[idx++] = 0xAB;
    send_buffer[idx++] = 0xFF;
#else
    send_buffer[idx++] = 0xAA;
#endif
    send_buffer[idx++] = 0xFF;
    send_buffer[idx++] = command_id;
    send_buffer[idx++] = 0x00;
#if ANOTIC_DEBUG_V2
    send_buffer[idx++] = 0x00;
#endif

    for (uint32_t i = 0; i < arg_nums; i++) {
        uint32_t size = va_arg(args, uint32_t);

        *dataLen += size;
        switch (size) {
            case 4: {
                uint32_t num = va_arg(args, uint32_t);
                memcpy(send_buffer + idx, &num, 4);
                break;
            }
            case 2: {
                uint16_t num = va_arg(args, uint32_t);
                memcpy(send_buffer + idx, &num, 2);
                break;
            }
            case 1: {
                uint8_t num = va_arg(args, uint32_t);
                memcpy(send_buffer + idx, &num, 1);
                break;
            }
            default: {
                GB_DEBUGE(ERROR_TAG, "Wrong format for size: %d\n", size);
                return -1;
            }
        }
        idx += size;
    }

    for (int i = 0; i < *dataLen + extraFrame; i++) {
        checksum += send_buffer[i];
        addcheck += checksum;
    }
    send_buffer[idx++] = checksum;
    send_buffer[idx++] = addcheck;

    va_end(args);

    *realLen = *dataLen + extraFrame + 2;
    return 0;
}

