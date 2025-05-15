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

#ifndef __RF24_CONFIG_H__
#define __RF24_CONFIG_H__

#define RF_LOW  0
#define RF_HIGH 1

/*** USER DEFINES:    ***/

/**********************/
#define rf24_max(a, b) (a>b?a:b)
#define rf24_min(a, b) (a<b?a:b)

/** @brief The default SPI speed (in Hz) */
#ifndef RF24_SPI_SPEED
#define RF24_SPI_SPEED 10000000
#endif

#define RF24_POWERUP_DELAY    5000

#ifndef sprintf_P
    #define sprintf_P sprintf
#endif // sprintf_P

#define _BV(x) (1<<(x))
#define PSTR(x) x
#define PRIPSTR "%s"
#define pgm_read_byte(p) (*(p))
#define pgm_read_ptr(p) (*(p))

#endif // __RF24_CONFIG_H__