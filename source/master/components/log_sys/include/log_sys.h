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

#ifndef _LOG_SYS__
#define _LOG_SYS__

#include "results.h"

typedef enum
{
    GB_LOG_LEVEL_ERROR,
    GB_LOG_LEVEL_WARNING,
    GB_LOG_LEVEL_INFO,
    GB_LOG_LEVEL_DEBUG,
    GB_LOG_LEVEL_VERBOSE
} GB_LOG_LEVEL;

#define GB_DEBUGE(tag, format, ...) gb_print_log(GB_LOG_LEVEL_ERROR, #tag, #format, #__VA_ARGS__)
#define GB_DEBUGW(tag, format, ...) gb_print_log(GB_LOG_LEVEL_WARNING, #tag, #format, #__VA_ARGS__)
#define GB_DEBUGI(tag, format, ...) gb_print_log(GB_LOG_LEVEL_INFO, #tag, #format, #__VA_ARGS__)
#define GB_DEBUGD(tag, format, ...) gb_print_log(GB_LOG_LEVEL_DEBUG, #tag, #format, #__VA_ARGS__)
#define GB_DEBUGV(tag, format, ...) gb_print_log(GB_LOG_LEVEL_VERBOSE, #tag, #format, #__VA_ARGS__)

extern const char* SENSOR_TAG;
extern const char* ERROR_TAG;
extern const char* ST_TAG;
extern const char* CHK_TAG;
extern const char* BMP_TAG;
extern const char* RF24_TAG;
extern const char* GB_INFO;
extern const char* LORA_TAG;
extern const char* FS_TAG;

void gb_log_system_init(void);
void gb_print_log(GB_LOG_LEVEL level, const char *tag, const char* format, ...);
GB_RESULT gb_usb_read_bytes(int cdc_intf, uint8_t *buf, size_t *rx_size);
GB_RESULT gb_usb_write_bytes(int cdc_intf, const uint8_t *buf, size_t tx_size);

#endif /* end of include guard: _LOG_SYS__ */