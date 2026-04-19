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

#ifdef __cplusplus
extern "C"
{
#endif

#include <inttypes.h>
#include "nrf24_interface.h"

#define GB_THROTTLE_MAX 10
#define GB_THROTTLE_MIN 5

#define GB_YAW_LIMIT 180
#define GB_PITCH_LIMIT 30
#define GB_ROLL_LIMIT 30

#define GB_SPEED_LIMIT 500 // speed °/s

#define ADC_CONSTRAIN_MAX 1000
#define ADC_CONSTRAIN_MIDDLE (ADC_CONSTRAIN_MAX / 2)
#define NRF24_CONTROL_WAITTING_TIME (3 * 1000 * 1000) // 3s
#define NRF24_RECEIVE_WAITTING_TIME	 (1000 * 1000)  // 1s

#define GB_ERLER_SCALE_RATE 100

typedef enum {
    LORA_IDLE,
    LORA_RECEIVE,
    LORA_SEND,
} GB_LORA_STATE;

typedef enum
{
    GB_THROTTLE,
    GB_YAW,
    GB_PITCH,
    GB_ROLL,
    GB_TYPE_MAX,
} GB_SAMPLE_ITEM;

typedef enum {
    LORA_SEND_NA,
    LORA_SEND_SKY_WAL_CONFIG,
    LORA_SEND_CONTROL_COMMAND,
    LORA_SEND_PID_SET_INFO,
    LORA_SEND_PID_GET_INFO,
    LORA_GET_MOTION_STATE,
} GB_SEND_CONFIG;


typedef uint8_t GB_LORA_PACKAGE_TYPE;
#define GB_INIT_DATA 0x01
#define GB_SET_CONFIG 0x02
#define GB_GET_REQUEST 0x03
#define GB_PID_FRAGMENT 0x04

typedef uint8_t GB_SYSTEM_STATE;
#define GB_SYSTEM_LOCK 0x01
#define GB_SYSTEM_NOT_INITIALIZED 0x02
#define GB_SYSTEM_INITIALIZE_PASS 0x03
#define GB_SYSTEM_INITIALIZE_FAIL 0x04
#define GB_SYSTEM_UNLOCK 0x05
#define GB_SYSTEM_REMOTE_TIMEOUT 0x06

typedef uint8_t GB_SET_TYPE;
#define GB_SET_THROTTLE 0x01
#define GB_SET_PID_TABLE 0x05
#define GB_SET_CONTROL_ARG 0x04

typedef uint8_t GB_GET_TYPE;
#define GB_GET_BATTERY_INFO 0x01
#define GB_GET_PID_INFO_0_7 0x02
#define GB_GET_PID_INFO_8_15 0x03
#define GB_GET_MOTION_STATE 0x04
#define GB_GET_PID_TABLE 0x05

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

typedef struct
{ // 0 ~ 100 for every value
    int16_t throttle;
    int16_t yaw;
    int16_t roll;
    int16_t pitch;
} GB_CONTROL_ARG_T;

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
        GB_CONTROL_ARG_T quad_status;
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

// ===========================================================
// PID Table Structure Definitions
// ===========================================================

typedef enum
{
    PID_ROLL_ANGLE = 0,
    PID_PITCH_ANGLE,
    PID_YAW_ANGLE,
    PID_ROLL_RATE,
    PID_PITCH_RATE,
    PID_YAW_RATE,
    PID_ALTITUDE,
    PID_CLIMB_RATE,
    PID_MAX = 8
} GB_PID_TYPE;

typedef struct
{
    uint16_t kp; // Proportional gain x 100 (e.g., 150 = 1.50)
    uint16_t ki; // Integral gain x 100
    uint16_t kd; // Derivative gain x 100
} GB_PID_PARAM_T; // 6 bytes per PID

typedef struct
{
    GB_PID_PARAM_T params[PID_MAX]; // 8 PIDs x 6 bytes = 48 bytes
    uint16_t crc16; // 2 bytes checksum
} GB_PID_TABLE_T; // Total: 50 bytes

// ===========================================================
// Fragment Protocol Definitions
// ===========================================================

#define GB_FRAGMENT_PAYLOAD_SIZE 27 // 32 - 2(type+sync) - 3(fragment control)
#define GB_MAX_FRAGMENTS 4 // Support up to 108 bytes (4 x 27)
#define GB_REASSEMBLY_TIMEOUT_MS 2000 // 2 second timeout

typedef struct
{
    GB_LORA_PACKAGE_TYPE type; // 1 byte: GB_PID_FRAGMENT
    uint8_t sync; // 1 byte: ack = sync + 1

    // Fragment control
    uint8_t msg_id; // 1 byte: unique message ID (0-255)
    uint8_t frag_index; // 1 byte: 0-based fragment index
    uint8_t frag_total; // 1 byte: total fragments in message

    uint8_t payload[GB_FRAGMENT_PAYLOAD_SIZE]; // 27 bytes: actual data
} GB_LORA_FRAGMENT_T; // Total: 32 bytes

typedef struct
{
    bool active; // Reassembly in progress
    uint8_t msg_id; // Current message ID
    uint8_t frag_total; // Expected total fragments
    uint8_t frag_received; // Fragments received count
    uint32_t timestamp_ms; // Start time for timeout
    uint8_t buffer[GB_MAX_FRAGMENTS * GB_FRAGMENT_PAYLOAD_SIZE]; // 108 bytes max
    bool fragments_received[GB_MAX_FRAGMENTS]; // Track which fragments arrived
} GB_REASSEMBLY_CTX_T;

typedef enum
{
    GB_FRAG_OK = 0,      // Fragment processed successfully
    GB_FRAG_DUPLICATE,   // Duplicate fragment received
    GB_FRAG_COMPLETE,    // All fragments received, message complete
    GB_FRAG_TIMEOUT,     // Reassembly timeout
    GB_FRAG_ERROR        // Error in fragment processing
} GB_FRAG_RESULT;

// ===========================================================
// Function Prototypes
// ===========================================================

// CRC16 checksum calculation
uint16_t GB_CRC16(const uint8_t *data, size_t len);

// Initialize reassembly context
void GB_ReassemblyInit(GB_REASSEMBLY_CTX_T *ctx);

// Check and handle reassembly timeout
GB_RESULT GB_ReassemblyCheckTimeout(GB_REASSEMBLY_CTX_T *ctx);

// Fragment sender: splits data into fragments and sends them
GB_RESULT GB_LoraFragmentSend(
    const uint8_t *data,
    size_t len,
    uint8_t msg_id,
    GB_LORA_STATE *state
);

// Fragment receiver: processes incoming fragment and reassembles message
GB_FRAG_RESULT GB_LoraFragmentReceive(
    const GB_LORA_FRAGMENT_T *frag,
    GB_REASSEMBLY_CTX_T *ctx,
    uint8_t *complete_msg,   // Output buffer for complete message
    size_t *msg_len          // Output: actual message length
);

// PID table validation
GB_RESULT GB_PidTableValidate(const GB_PID_TABLE_T *table);

// PID table CRC calculation
uint16_t GB_PidTableCalculateCRC(const GB_PID_TABLE_T *table);

GB_RESULT GB_LoraSystemInit(GB_LORA_STATE init_state, bool radioNumber, GB_LORA_STATE *state);

/*
 * G = ((S - Smin) * (Gmax - Gmin)) / (Smax - Smin) + Gmin
 */
static float inline _scale_to(float val, float src_min, float src_max, float dst_min, float dst_max)
{
    return ((val - src_min) * (dst_max - dst_min)) / (src_max - src_min) + dst_min;
}

// right: 0   left: ADC_CONSTRAIN_MAX
// down:  0   up:   ADC_CONSTRAIN_MAX
// +-: direction(align with imu)   |x|: size
static inline float _control_commander_to_range(uint8_t item, uint16_t control_info, uint8_t is_speed)
{
    // 降低摇杆精度
    control_info &= 0xfffe;

    if (GB_THROTTLE == item)
        return _scale_to(control_info, 0, ADC_CONSTRAIN_MAX, GB_THROTTLE_MIN, GB_THROTTLE_MAX);
    // 去死区 ±10
    if (control_info >= ADC_CONSTRAIN_MAX / 2 - 10 && control_info <= ADC_CONSTRAIN_MAX / 2 + 10)
        return 0.0f;
    if (is_speed)
        return -_scale_to(control_info, 0, ADC_CONSTRAIN_MAX, -GB_SPEED_LIMIT, GB_SPEED_LIMIT);
    if (GB_YAW == item)
        return -_scale_to(control_info, 0, ADC_CONSTRAIN_MAX, -GB_YAW_LIMIT, GB_YAW_LIMIT);
    else if (GB_PITCH == item)
        return -_scale_to(control_info, 0, ADC_CONSTRAIN_MAX, -GB_PITCH_LIMIT, GB_PITCH_LIMIT);
    else if (GB_ROLL == item)
        return -_scale_to(control_info, 0, ADC_CONSTRAIN_MAX, -GB_ROLL_LIMIT, GB_ROLL_LIMIT);
    return 0.0f;
}

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* end of include guard: _LORA_STATE_MACHINE__ */
