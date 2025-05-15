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

#ifndef _LORA_STATE_MACHINE__
#define _LORA_STATE_MACHINE__

#include <inttypes.h>
#include "nrf24_interface.h"

typedef enum {
    LORA_IDLE,
    LORA_RECEIVE,
    LORA_CHK_ERROR,
    LORA_SEND,
    LORA_TIMEOUT,
} GB_LORA_STATE;

typedef enum {
    LORA_SEND_NA,
    LORA_SEND_SKY_WAL_CONFIG,
    LORA_SEND_CONTROL_COMMAND,
    LORA_SEND_PID_SET_INFO,
    LORA_SEND_PID_GET_INFO,
} GB_SEND_CONFIG;


typedef uint8_t GB_LORA_PACKAGE_TYPE;
#define GB_INIT_DATA 0x01
#define GB_SET_CONFIG 0x02
#define GB_GET_REQUEST 0x03

typedef uint8_t GB_SYSTEM_STATE;
#define GB_SYSTEM_LOCK 0x01
#define GB_SYSTEM_NOT_INITIALIZED 0x02
#define GB_SYSTEM_INITIALIZE_PASS 0x03
#define GB_SYSTEM_INITIALIZE_FAIL 0x04
#define GB_SYSTEM_UNLOCK 0x05
#define GB_SYSTEM_REMOTE_TIMEOUT 0x06

typedef uint8_t GB_SET_TYPE;
#define GB_SET_THROTTLE 0x01
#define GB_SET_PID_0_7 0x02
#define GB_SET_PID_8_15 0x03
#define GB_SET_CONTROL_ARG 0x04

typedef uint8_t GB_GET_TYPE;
#define GB_GET_BATTERY_INFO 0x01
#define GB_GET_PID_INFO_0_7 0x02
#define GB_GET_PID_INFO_8_15 0x03

typedef struct
{
    GB_SYSTEM_STATE system_state;
    uint8_t battery_capacity;
    uint8_t sensor_state; // MPU | COMPASS | BMP280 | CAMERA
} GB_INIT_DATA_T;

typedef struct
{ // 0 ~ 100 for every value
    uint16_t throttle;
    uint16_t yaw;
    uint16_t roll;
    uint16_t pitch;
} GB_SET_CONTROL_ARG_T;

typedef uint16_t GB_SET_THROTTLE_T;
typedef struct
{
    GB_SET_TYPE set_type;
    union
    {
        GB_SET_THROTTLE_T throttle;
        //LORA_GB_PID_INIT_T pid;  TODO
        GB_SET_CONTROL_ARG_T control_arg;
    };
} GB_SET_CONFIG_T;

typedef uint8_t GB_GET_BATTERY_INFO_T;
typedef uint32_t GB_GET_PID_INFO_T;
typedef struct
{
    GB_GET_TYPE get_type;
    union
    {
        GB_GET_BATTERY_INFO_T battery;
        GB_GET_PID_INFO_T pid;
    };
} GB_GET_REQUEST_T;

typedef struct
{
    GB_LORA_PACKAGE_TYPE type;
    uint8_t sync; // ack = sync + 1

    union
    {
        GB_INIT_DATA_T init;
        GB_SET_CONFIG_T config;
        GB_GET_REQUEST_T request;
    };
} GB_LORA_PACKAGE_T;

GB_RESULT GB_LoraSystemInit(GB_LORA_STATE state, bool radioNumber);

#endif /* end of include guard: _LORA_STATE_MACHINE__ */
