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

#include "log_sys.h"
#include "gps_driver.h"

GB_RESULT GB_GPS_Init()
{
    return uart1.begin(&uart1, 115200, GPS_TX_PIN, GPS_RX_PIN);
}

GB_RESULT GB_GPS_ReadData(GB_GPS_INFO_T *gpsInfo)
{
    return GB_OK;
}
