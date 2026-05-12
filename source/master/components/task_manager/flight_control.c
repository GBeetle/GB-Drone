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

#include <string.h>
#include "flight_control.h"

// Maximum PID correction as percentage of ESC duty range (5%-10% = 5% total)
#define MAX_CORRECTION_DUTY 2.0f

static void pid_init(pid_controller_t *pid, float integral_limit, float output_limit)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
}

void pid_reset(pid_controller_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

float pid_update(pid_controller_t *pid, const GB_PID_PARAM_T *param, float error, float dt)
{
    if (dt <= 0.0f) return 0.0f;

    float kp = param->kp / 100.0f;
    float ki = param->ki / 100.0f;
    float kd = param->kd / 100.0f;

    // Proportional
    float p_term = kp * error;

    // Integral with anti-windup (clamping)
    pid->integral += ki * error * dt;
    if (pid->integral > pid->integral_limit)
        pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit)
        pid->integral = -pid->integral_limit;

    // Derivative on error
    float d_term = kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;

    // Total output with saturation
    float output = p_term + pid->integral + d_term;
    if (output > pid->output_limit)
        output = pid->output_limit;
    else if (output < -pid->output_limit)
        output = -pid->output_limit;

    return output;
}

void motor_mix(float throttle, float roll_cmd, float pitch_cmd, float yaw_cmd,
    motor_output_t *out)
{
    // Convert normalized throttle (0.0-1.0) directly to ESC duty:
    // ADC 0 -> 5.0% duty (1ms pulse, idle)
    // ADC 1000 -> 10.0% duty (2ms pulse, full)
    float base = 5.0f + throttle * 5.0f;

    // Scale PID outputs (-1.0 to +1.0) to duty correction
    float r = roll_cmd * MAX_CORRECTION_DUTY;
    float p = pitch_cmd * MAX_CORRECTION_DUTY;
    float y = yaw_cmd * MAX_CORRECTION_DUTY;

    // X-configuration mixing:
    // M0 (Front-Right, CCW): -roll +pitch -yaw
    // M1 (Front-Left, CW): +roll +pitch +yaw
    // M2 (Rear-Left, CCW): +roll -pitch -yaw
    // M3 (Rear-Right, CW): -roll -pitch +yaw

    out->motor[0] = base - r + p - y;
    out->motor[1] = base + r + p + y;
    out->motor[2] = base + r - p - y;
    out->motor[3] = base - r - p + y;

    // Clamp to ESC valid range
    for (int i = 0; i < 4; i++) {
        if (out->motor[i] < 5.0f) out->motor[i] = 5.0f;
        if (out->motor[i] > 10.0f) out->motor[i] = 10.0f;
    }
}

void flight_control_init(flight_pid_set_t *pids)
{
    // Default PID gains (uint16_t x100 format) stored in the shared table
    // Angle PIDs: output is desired rate in deg/s
    pids->pid_table.params[PID_ROLL_ANGLE] = (GB_PID_PARAM_T){400, 2, 0}; // kp=4.0, ki=0.02, kd=0.0
    pids->pid_table.params[PID_PITCH_ANGLE] = (GB_PID_PARAM_T){400, 2, 0}; // kp=4.0, ki=0.02, kd=0.0
    pids->pid_table.params[PID_YAW_ANGLE] = (GB_PID_PARAM_T){300, 1, 0}; // kp=3.0, ki=0.01, kd=0.0

    // Rate PIDs: output is normalized correction (-1.0 to +1.0)
    pids->pid_table.params[PID_ROLL_RATE] = (GB_PID_PARAM_T){60, 30, 2}; // kp=0.6, ki=0.3, kd=0.02
    pids->pid_table.params[PID_PITCH_RATE] = (GB_PID_PARAM_T){60, 30, 2}; // kp=0.6, ki=0.3, kd=0.02
    pids->pid_table.params[PID_YAW_RATE] = (GB_PID_PARAM_T){100, 50, 0}; // kp=1.0, ki=0.5, kd=0.0

    // Reserved
    memset(&pids->pid_table.params[PID_ALTITUDE], 0, sizeof(GB_PID_PARAM_T));
    memset(&pids->pid_table.params[PID_CLIMB_RATE], 0, sizeof(GB_PID_PARAM_T));

    pids->pid_table.crc16 = GB_PidTableCalculateCRC(&pids->pid_table);

    // Conservative defaults for initial testing
    // Angle PIDs: output is desired rate in deg/s
    pid_init(&pids->roll_angle, 75.0f, 300.0f);
    pid_init(&pids->pitch_angle, 75.0f, 300.0f);
    pid_init(&pids->yaw_angle, 75.0f, 300.0f);

    // Rate PIDs: output is normalized correction (-1.0 to +1.0)
    pid_init(&pids->roll_rate, 0.25f, 1.0f);
    pid_init(&pids->pitch_rate, 0.25f, 1.0f);
    pid_init(&pids->yaw_rate, 0.25f, 1.0f);
}
