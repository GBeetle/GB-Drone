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

#ifndef _FLIGHT_CONTROL_H_
#define _FLIGHT_CONTROL_H_

#include <stdint.h>
#include <stdbool.h>
#include "lora_state.h"

typedef struct {
    float integral;
    float prev_error;
    float integral_limit;
    float output_limit;
} pid_controller_t;

typedef struct {
    GB_PID_TABLE_T pid_table;
    pid_controller_t roll_angle, roll_rate;
    pid_controller_t pitch_angle, pitch_rate;
    pid_controller_t yaw_angle, yaw_rate;
} flight_pid_set_t;

typedef struct {
    float motor[4]; // duty 5.0 - 10.0
} motor_output_t;

typedef enum {
    FLIGHT_DISARMED = 0,
    FLIGHT_ARMED,
    FLIGHT_FAILSAFE,
} flight_state_t;

//PID functions
float pid_update(pid_controller_t *pid, const GB_PID_PARAM_T *param, float error, float dt);
void pid_reset(pid_controller_t *pid);

//Motor mixer (X-configuration)
//throttle:0.0-1.0 normalized from raw ADC (0-1000)/1000
//roll/pitch/yaw_cmd: PID output (-1.0 to +1.0)
void motor_mix(float throttle, float roll_cmd, float pitch_cmd, float yaw_cmd, motor_output_t *out);

//Initialize with conservative default gains
void flight_control_init(flight_pid_set_t *pids);

#endif /*FLIGHT_CONTROL_H*/
