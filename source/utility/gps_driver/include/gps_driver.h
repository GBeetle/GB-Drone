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
#ifndef _GPS_DIRVER_H__
#define _GPS_DIRVER_H__

#include "results.h"

typedef struct gps_info
{
    float latitude;
    float longitude;
    float altitude;
    uint8_t fix_type;
    uint8_t num_sv;
} GB_GPS_INFO_T;

GB_RESULT GB_GPS_Init();
GB_RESULT GB_GPS_ReadNMEA(GB_GPS_INFO_T *gpsInfo);
GB_RESULT GB_GPS_ReadUBX(GB_GPS_INFO_T *gpsInfo);

#endif /* end of include guard: _GPS_DIRVER_H__ */
