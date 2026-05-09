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

#include "flight_control.h"

// Maximum PID correction as percentage of ESC duty range (5%-10% = 5% total)
#define MAX_CORRECTION_DUTY 2.0f

void pid_init(pid_controller_t *pid, float kp, float ki, float kd,
    float integral_limit, float output_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
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

float pid_update(pid_controller_t *pid, float error, float dt)
{
    if (dt <= 0.0f) return 0.0f;

    // Proportional
    float p_term = pid->kp * error;

    // Integral with anti-windup (clamping)
    pid->integral += pid->ki * error * dt;
    if (pid->integral > pid->integral_limit)
        pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit)
        pid->integral = -pid->integral_limit;

    // Derivative on error
    float d_term = pid->kd * (error - pid->prev_error) / dt;
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

void flight_control_apply_pid_table(const GB_PID_TABLE_T *table,flight_pid_set_t*pids)
{
    pids->roll_angle.kp = table->params[PID_ROLL_ANGLE].kp / 100.0f;
    pids->roll_angle.ki = table->params[PID_ROLL_ANGLE].ki / 100.0f;
    pids->roll_angle.kd = table->params[PID_ROLL_ANGLE].kd / 100.0f;

    pids->roll_rate.kp = table->params[PID_ROLL_RATE].kp / 100.0f;
    pids->roll_rate.ki = table->params[PID_ROLL_RATE].ki / 100.0f;
    pids->roll_rate.kd = table->params[PID_ROLL_RATE].kd / 100.0f;

    pids->pitch_angle.kp = table->params[PID_PITCH_ANGLE].kp / 100.0f;
    pids->pitch_angle.ki = table->params[PID_PITCH_ANGLE].ki / 100.0f;
    pids->pitch_angle.kd = table->params[PID_PITCH_ANGLE].kd / 100.0f;

    pids->pitch_rate.kp = table->params[PID_PITCH_RATE].kp / 100.0f;
    pids->pitch_rate.ki = table->params[PID_PITCH_RATE].ki / 100.0f;
    pids->pitch_rate.kd = table->params[PID_PITCH_RATE].kd / 100.0f;

    pids->yaw_angle.kp = table->params[PID_YAW_ANGLE].kp / 100.0f;
    pids->yaw_angle.ki = table->params[PID_YAW_ANGLE].ki / 100.0f;
    pids->yaw_angle.kd = table->params[PID_YAW_ANGLE].kd / 100.0f;

    pids->yaw_rate.kp = table->params[PID_YAW_RATE].kp / 100.0f;
    pids->yaw_rate.ki = table->params[PID_YAW_RATE].ki / 100.0f;
    pids->yaw_rate.kd = table->params[PID_YAW_RATE].kd / 100.0f;
}

void flight_control_init(flight_pid_set_t *pids)
{
    // Conservative defaults for initial testing
    // Angle PIDs: output is desired rate in deg/s
    pid_init(&pids->roll_angle, 4.0f, 0.02f, 0.0f, 75.0f, 300.0f);
    pid_init(&pids->pitch_angle, 4.0f, 0.02f, 0.0f, 75.0f, 300.0f);
    pid_init(&pids->yaw_angle, 3.0f, 0.01f, 0.0f, 75.0f, 300.0f);

    // Rate PIDs: output is normalized correction (-1.0 to +1.0)
    pid_init(&pids->roll_rate, 0.6f, 0.3f, 0.02f, 0.25f, 1.0f);
    pid_init(&pids->pitch_rate, 0.6f, 0.3f, 0.02f, 0.25f, 1.0f);
    pid_init(&pids->yaw_rate, 1.0f, 0.5f, 0.0f, 0.25f, 1.0f);
}
