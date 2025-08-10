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

#ifndef _FILE_SYSTEM__
#define _FILE_SYSTEM__

#include <stdbool.h>
#include <stdlib.h>
#include "results.h"

typedef enum {
    GB_FILE_SPI_FLASH = 0,
    GB_FILE_SD_CARD,
    GB_FILE_MAX
} GB_FILE_PARTITION;

GB_RESULT GB_FileSystem_Init();
GB_RESULT GB_SDCardFileSystem_Init();
GB_RESULT GB_FileSystem_Write(GB_FILE_PARTITION partition, const char *file, const uint8_t *data, uint32_t len);
GB_RESULT GB_FileSystem_Read(GB_FILE_PARTITION partition, const char *file, uint8_t *data, uint32_t len);
GB_RESULT GB_FileSystem_ListDir(GB_FILE_PARTITION partition);

GB_RESULT GB_Log2fileEnable();
int GB_WriteLog2file(const char *format, va_list args);

#endif /* end of include guard: _FILE_SYSTEM__ */
