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

#include "driver/uart.h"
#include "soc/uart_struct.h"
#include "hal/uart_ll.h"
#include "sdkconfig.h"
#include "Fusion.h"
#include "gb_timer.h"
#include "lora_state.h"
#include "esc_system.h"
#include "task_manager.h"
#include "max1704x.h"

typedef struct
{
    float roll;
    float pitch;
    float yaw;
    float height;
} GB_Motion_State;

struct imu imu;  // create a default MPU object
GB_Motion_State motionState;

static uint8_t anotic_debug_id = 0x00;
QueueHandle_t gyroQueue, accelQueue, magQueue, baroQueue;
SemaphoreHandle_t mpuDataQueueReady;
SemaphoreHandle_t mpuSensorReady;
SemaphoreHandle_t motionStateMutex;

static inline GB_RESULT get_sensor_data(raw_axes_t *accelRaw, raw_axes_t *gyroRaw, raw_axes_t *magRaw,
    float_axes_t *accelG, float_axes_t *gyroDPS, float_axes_t *magDPS, baro_t *baro_data)
{
    GB_RESULT res = GB_OK;

    if (uxQueueMessagesWaiting(gyroQueue) > 0)
    {
        xQueueReceive(gyroQueue, gyroRaw, portMAX_DELAY);
    }
    if (uxQueueMessagesWaiting(accelQueue) > 0)
    {
        xQueueReceive(accelQueue, accelRaw, portMAX_DELAY);
    }
    if (uxQueueMessagesWaiting(magQueue) > 0)
    {
        xQueueReceive(magQueue, magRaw, portMAX_DELAY);
    }
    if (uxQueueMessagesWaiting(baroQueue) > 0)
    {
        xQueueReceive(baroQueue, baro_data, portMAX_DELAY);
    }

    GB_DEBUGD(SENSOR_TAG, "Rev gyroRaw.x %d, gyroRaw.y %d, gyroRaw.z %d\n", gyroRaw->data.x, gyroRaw->data.y, gyroRaw->data.z);
    GB_DEBUGD(SENSOR_TAG, "Rev accelRaw.x %d, accelRaw.y %d, accelRaw.z %d\n", accelRaw->data.x, accelRaw->data.y, accelRaw->data.z);
    GB_DEBUGD(SENSOR_TAG, "Rev magRaw.x %d, magRaw.y %d, magRaw.z %d\n", magRaw->data.x, magRaw->data.y, magRaw->data.z);
    GB_DEBUGD(SENSOR_TAG, "Rev baro_data: [%+6.2fPa %+6.2fC %+6.2fcm ] \n", baro_data->pressure, baro_data->temperature, baro_data->altitude);

    // Convert
    *accelG  = accelGravity_raw(accelRaw, g_accel_fs);
    *gyroDPS = gyroDegPerSec_raw(gyroRaw, g_gyro_fs);
    if (magRaw->data.x != 0 || magRaw->data.y != 0 || magRaw->data.z != 0)
    {
        *magDPS = magGauss_raw(magRaw, g_lis3mdl_fs);
    }

//error_exit:
    return res;
}

static inline FusionVector FusionMagneticApplyOffset(const FusionVector calibrated)
{
    static uint8_t originMagInitailized = 0;
    static uint32_t magCounter = 0;
    const uint32_t magMaxCounter = 500;
    static FusionVector originMag = {{0.0f, 0.0f, 0.0f}};

    if (0 == originMagInitailized)
    {
        originMag.axis.x += calibrated.axis.x;
        originMag.axis.y += calibrated.axis.y;
        originMag.axis.z += calibrated.axis.z;
        magCounter++;
        if (magCounter >= magMaxCounter)
        {
            originMagInitailized = 1;
            originMag.axis.x /= magCounter;
            originMag.axis.y /= magCounter;
            originMag.axis.z /= magCounter;
            GB_DEBUGI(SENSOR_TAG, "originMag.x %0.1f, originMag.y %0.1f, originMag.z %0.1f\n", originMag.axis.x, originMag.axis.y, originMag.axis.z);
        }
        return FUSION_VECTOR_ZERO;
    }
    else
    {
        return FusionVectorSubtract(calibrated, originMag);
    }
}

void gb_read_sensor_data(void* arg)
{
    GB_TickType ticks = 0;
    GB_RESULT res = GB_OK;
    //selftest_t st_result;

    (void) res;
    // imu initialization
    GB_IMU_Init(&imu);
    CHK_FUNC_EXIT(imu.testConnection(&imu));
    CHK_FUNC_EXIT(imu.initialize(&imu));

    // test for sensor is good & horizontal
    //selftest_t st_result;
    //CHK_FUNC_EXIT(imu.selfTest(&imu, &st_result));
    CHK_FUNC_EXIT(imu.setOffsets(&imu, true, false)); // keep static when power up

    GB_DEBUGI(SENSOR_TAG, "imu status: %02x", imu.mpu_status);

    GB_MsToTick(2, &ticks);
    while (1) {
        if (xSemaphoreTake(mpuSensorReady, portMAX_DELAY) == pdTRUE) {}

        //GB_GPIO_Set( TEST_IMU_IO, 1 );
        raw_axes_t gyroRaw  = GB_RAW_DATA_ZERO;
        raw_axes_t accelRaw  = GB_RAW_DATA_ZERO;
        raw_axes_t magRaw    = GB_RAW_DATA_ZERO;
        baro_t     baro_data = GB_BARO_DATA_ZERO;

        CHK_FUNC_EXIT(imu.rotation(&imu, &gyroRaw));
        CHK_FUNC_EXIT(imu.acceleration(&imu, &accelRaw));
        CHK_FUNC_EXIT(imu.heading(&imu, &magRaw));
        CHK_FUNC_EXIT(imu.baroGetData(&imu, &baro_data));

        GB_DEBUGD(SENSOR_TAG, "Queue gyroRaw.x %d, gyroRaw.y %d, gyroRaw.z %d\n", gyroRaw.data.x, gyroRaw.data.y, gyroRaw.data.z);
        GB_DEBUGD(SENSOR_TAG, "Queue accelRaw.x %d, accelRaw.y %d, accelRaw.z %d\n", accelRaw.data.x, accelRaw.data.y, accelRaw.data.z);
        GB_DEBUGD(SENSOR_TAG, "Queue magRaw.x %d, magRaw.y %d, magRaw.z %d\n", magRaw.data.x, magRaw.data.y, magRaw.data.z);
        GB_DEBUGD(SENSOR_TAG, "Queue baro_data: [%+6.2fPa %+6.2fC %+6.2fcm ] \n", baro_data.pressure, baro_data.temperature, baro_data.altitude);

        if (gyroRaw.data.x != 0 && gyroRaw.data.y != 0 && gyroRaw.data.z != 0)
            xQueueSend(gyroQueue, &gyroRaw, portMAX_DELAY);
        if (accelRaw.data.x != 0 && accelRaw.data.y != 0 && accelRaw.data.z != 0)
            xQueueSend(accelQueue, &accelRaw, portMAX_DELAY);
        if (magRaw.data.x != 0 && magRaw.data.y != 0 && magRaw.data.z != 0)
            xQueueSend(magQueue, &magRaw, portMAX_DELAY);
        if (baro_data.altitude != 0)
            xQueueSend(baroQueue, &baro_data, portMAX_DELAY);
        xSemaphoreGive(mpuDataQueueReady);
        //GB_GPIO_Set( TEST_IMU_IO, 0 );
    }

error_exit:
    GB_DEBUGE(ERROR_TAG, "Sensor data read error");
    return;
}

void GB_MutexInitialize()
{
    mpuDataQueueReady = xSemaphoreCreateBinary();
    mpuSensorReady = xSemaphoreCreateBinary();
    motionStateMutex = xSemaphoreCreateMutex();
    gyroQueue = xQueueCreate(MPU_DATA_QUEUE_SIZE, sizeof(raw_axes_t));
    accelQueue = xQueueCreate(MPU_DATA_QUEUE_SIZE, sizeof(raw_axes_t));
    magQueue = xQueueCreate(MPU_DATA_QUEUE_SIZE, sizeof(raw_axes_t));
    baroQueue = xQueueCreate(MPU_DATA_QUEUE_SIZE, sizeof(baro_t));
}

void gb_sensor_fusion(void* arg)
{
    raw_axes_t   accelRaw         = {0};                   // x, y, z axes as int16
    raw_axes_t   gyroRaw          = {0};                   // x, y, z axes as int16
    raw_axes_t   magRaw           = {0};                   // x, y, z axes as int16
    float_axes_t accelG           = {{0.0f, 0.0f, 0.0f}};  // accel axes in (g) gravity format
    float_axes_t gyroDPS          = {{0.0f, 0.0f, 0.0f}};  // gyro axes in (DPS) º/s format
    float_axes_t magDPS           = {{0.0f, 0.0f, 0.0f}};  // gyro axes in (Gauss) format
    uint8_t      send_buffer[64]  = {0};
    uint32_t     realDataLen      = 0;
    baro_t       baro_data        = {0};

    // Fusion initialization
    const FusionMatrix gyroscopeMisalignment = {{{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}};
    const FusionVector gyroscopeSensitivity = {{1.0f, 1.0f, 1.0f}};
    const FusionVector gyroscopeOffset = {{0.0f, 0.0f, 0.0f}}; // For MPU6050 using setOffsets to remove bias error

    const FusionMatrix accelerometerMisalignment = {{{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}};
    const FusionVector accelerometerSensitivity = {
        .axis.x = (float)(INT16_MAX) / accelFSRvalue(g_accel_fs) / (2056.73),
        .axis.y = (float)(INT16_MAX) / accelFSRvalue(g_accel_fs) / (2036.68),
        .axis.z = (float)(INT16_MAX) / accelFSRvalue(g_accel_fs) / (2071.70)};
    const FusionVector accelerometerOffset = {
        .axis.x = 60.50f * accelResolution(g_accel_fs),
        .axis.y = -34.09f * accelResolution(g_accel_fs),
        .axis.z = 181.67f * accelResolution(g_accel_fs)};

    const FusionMatrix softIronMatrix = {{{2.38f, 0.04f, -0.04f}, {0.04f, 2.49f, -0.09f}, {-0.04f, -0.09f, 2.70f}}};
    const FusionVector hardIronOffset = {{0.33f, -0.77f, 1.14f}};

    FusionOffset offset;
    FusionAhrs ahrs;

    memset(&ahrs, 0x00, sizeof(ahrs));
    FusionOffsetInitialise(&offset, IMU_SAMPLE_RATE);
    FusionAhrsInitialise(&ahrs);

    // Set AHRS algorithm settings
    const FusionAhrsSettings settings = {
            .convention = FusionConventionEnu,
            .gain = 0.5f,
            .gyroscopeRange = 2000.0f,
            .accelerationRejection = 10.0f,
            .magneticRejection = 10.0f,
            .recoveryTriggerPeriod = 5 * IMU_SAMPLE_RATE, /* 5 seconds */
    };
    FusionAhrsSetSettings(&ahrs, &settings);

    while (1)
    {
        if (xSemaphoreTake(mpuDataQueueReady, portMAX_DELAY) == pdTRUE) {}

        //GB_GPIO_Set( TEST_IMU_IO, 1 );

        // 0.3ms start
        CHK_FUNC_EXIT(get_sensor_data(&accelRaw, &gyroRaw, &magRaw, &accelG, &gyroDPS, &magDPS, &baro_data));

        // Acquire latest sensor data
        int64_t timestamp = 0;

        GB_GetTimerMs((uint64_t*)&timestamp); // replace this with actual gyroscope timestamp
        FusionVector gyroscope = {{gyroDPS.data.x, gyroDPS.data.y, gyroDPS.data.z}};  // degrees/s
        FusionVector accelerometer = {{accelG.data.x, accelG.data.y, accelG.data.z}}; // g
        FusionVector magnetometer = {{magDPS.data.x, magDPS.data.y, magDPS.data.z}};  // arbitrary units

        // Apply calibration
        gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
        accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
        if (magnetometer.axis.x != 0 || magnetometer.axis.y != 0 || magnetometer.axis.z != 0)
        {
            magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);
            magnetometer = FusionMagneticApplyOffset(magnetometer);
        }

        // Update gyroscope offset correction algorithm
        gyroscope = FusionOffsetUpdate(&offset, gyroscope);

        // Calculate delta time (in seconds) to account for gyroscope sample clock error
        static int64_t previousTimestamp = 0;
        const float deltaTime = (previousTimestamp == 0) ? 0 : (float) (timestamp - previousTimestamp) / (float) (1000);  // ms to seconds
        previousTimestamp = timestamp;

        // Update gyroscope AHRS algorithm
        FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, deltaTime);

        // Print algorithm outputs
        const FusionQuaternion quat = FusionAhrsGetQuaternion(&ahrs);
        const FusionEuler euler = FusionQuaternionToEuler(quat);
        const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);
        // 0.3ms end

        if (xSemaphoreTake(motionStateMutex, portMAX_DELAY) == pdTRUE)
        {
            motionState.roll = euler.angle.roll;
            motionState.pitch = euler.angle.pitch;
            motionState.yaw = euler.angle.yaw;

            xSemaphoreGive(motionStateMutex);
        }

        GB_DEBUGD(SENSOR_TAG, "Gyro.x %f, Gyro.y %f, Gyro.z %f\n", gyroscope.axis.x, gyroscope.axis.y, gyroscope.axis.z);
        GB_DEBUGD(SENSOR_TAG, "Accel.x %f, Accel.y %f, Accel.z %f\n", accelerometer.axis.x, accelerometer.axis.y, accelerometer.axis.z);
        GB_DEBUGD(SENSOR_TAG, "Mag.x %f, Mag.y %f, Mag.z %f\n", magnetometer.axis.x, magnetometer.axis.y, magnetometer.axis.z);
        GB_DEBUGD(SENSOR_TAG, "Roll %0.1f, Pitch %0.1f, Yaw %0.1f, X %0.1f, Y %0.1f, Z %0.1f, deltaTime: %f\n",
                  euler.angle.roll, euler.angle.pitch, euler.angle.yaw,
                  earth.axis.x, earth.axis.y, earth.axis.z, deltaTime);
        GB_DEBUGD(SENSOR_TAG, "baro_data: [%+6.2fPa %+6.2fC %+6.2fcm ] \n", baro_data.pressure, baro_data.temperature, baro_data.altitude);

        // note: communication_V7-20210528.pdf
        switch (anotic_debug_id) {
            case 0x00:
                break;
            case 0x01:
                anotc_init_data(send_buffer, &realDataLen, 0x01, 6,
                                sizeof(uint16_t), float2int16(accelG.data.x / accelResolution(g_accel_fs), 1),
                                sizeof(uint16_t), float2int16(accelG.data.y / accelResolution(g_accel_fs), 1),
                                sizeof(uint16_t), float2int16(accelG.data.z / accelResolution(g_accel_fs), 1),
                                sizeof(uint16_t), float2int16(gyroDPS.data.x / gyroResolution(g_gyro_fs), 1),
                                sizeof(uint16_t), float2int16(gyroDPS.data.y / gyroResolution(g_gyro_fs), 1),
                                sizeof(uint16_t), float2int16(gyroDPS.data.z / gyroResolution(g_gyro_fs), 1), sizeof(uint8_t), 0x00);
                break;
            case 0x02:
                anotc_init_data(send_buffer, &realDataLen, 0x02, 6, sizeof(uint16_t), magRaw.data.x, sizeof(uint16_t), magRaw.data.y, sizeof(uint16_t), magRaw.data.z,
                    sizeof(uint32_t), (uint32_t)baro_data.altitude, sizeof(uint16_t), float2int16(baro_data.temperature, 100), sizeof(uint8_t), 0x00, sizeof(uint8_t), 0x00);
                break;
            case 0x03:
                // East(X)Nouth(Y)Up(Z) [MPU R.P.Y] to Anotic UI [Quadrotor R.P.Y]
                // roll -> pitch
                // pitch -> roll
                // yaw -> -yaw
                anotc_init_data(send_buffer, &realDataLen, 0x03, 4, sizeof(uint16_t), float2int16(euler.angle.pitch, 100), sizeof(uint16_t), float2int16(euler.angle.roll, 100),
                    sizeof(uint16_t), float2int16(-euler.angle.yaw, 100), sizeof(uint8_t), 0x01);
                break;
            case 0x04:
                // Quaternion
                anotc_init_data(send_buffer, &realDataLen, 0x04, 5, sizeof(uint16_t), float2int16(quat.element.w, 10000), sizeof(uint16_t), float2int16(quat.element.x, 10000),
                    sizeof(uint16_t), float2int16(quat.element.y, 10000), sizeof(uint16_t), float2int16(quat.element.z, 10000), sizeof(uint8_t), 0x01);
                break;
            default:
                GB_DEBUGE(ERROR_TAG, "wrong command id from uart: %02x\n", anotic_debug_id);
                anotic_debug_id = 0x00;
        }
        if (anotic_debug_id >= 0x01 && anotic_debug_id <= 0x04)
        {
            GB_WriteBytes((const uint8_t *)send_buffer, realDataLen);
        }

        // Debug
        GB_DEBUGD(SENSOR_TAG, "gyro: [%+7.2f %+7.2f %+7.2f ] (º/s) \t", gyroDPS.xyz[0], gyroDPS.xyz[1], gyroDPS.xyz[2]);
        GB_DEBUGD(SENSOR_TAG, "accel: [%+6.2f %+6.2f %+6.2f ] (G) \t", accelG.data.x, accelG.data.y, accelG.data.z);
        GB_DEBUGD(SENSOR_TAG, "mag: [%+6.2f %+6.2f %+6.2f ] (Gauss) \n", magDPS.data.x, magDPS.data.y, magDPS.data.z);
        //GB_GPIO_Set( TEST_IMU_IO, 0 );
    }

error_exit:
    GB_DEBUGE(ERROR_TAG, "Sensor fusion error");
    return;
}

static GB_RESULT gb_lora_request_dispatch(GB_MAX1704X_DEV_T *dev, GB_LORA_PACKAGE_T *in, GB_LORA_PACKAGE_T *out, GB_LORA_STATE *state)
{
    GB_RESULT res = GB_OK;
    float voltage = 0;
    //static LORA_GB_PID_INIT_T pid_first = {0}, pid_last = {0};

    switch (in->type)
    {
    case GB_INIT_DATA:
        GB_Max1704xGetVoltage(dev, &voltage);
        out->type = GB_INIT_DATA;
        out->init.system_state = GB_SYSTEM_INITIALIZE_PASS;
        out->init.battery_capacity = (voltage < MAX1704X_VOL_MIN && voltage > MAX1704X_VOL_MAX) ?
                                     (voltage - MAX1704X_VOL_MIN) / (MAX1704X_VOL_MAX - MAX1704X_VOL_MIN) : 0xff;
        out->init.sensor_state = 0xf0;
        *state = LORA_SEND;
        GB_DEBUGI(LORA_TAG, "Receive GB_INIT_DATA, sync: %d, Voltage: %.2f.V, battery_capacity: %d", in->sync, voltage, out->init.battery_capacity);
        break;
    case GB_SET_CONFIG:
        switch (in->config.set_type)
        {
        case GB_SET_THROTTLE:
            // don't need ack for esc setting
            for (int i = 0; i < 4; i++)
                // esc_set_duty(i, 5.0f + 0.05f * (float)(in->config.throttle));
                esc_set_duty(i, _control_commander_to_range(GB_THROTTLE, in->config.throttle, 0));
            // GB_DEBUGI(LORA_TAG, "Receive GB_SET_THROTTLE, throttle: %d", in->config.throttle);
            break;
#if 0
        case GB_SET_PID_0_7:
        {
            lora_pidInit_t *tmp_pid_tbl = NULL;
            for (int i = 0; i < PID_TYPE_MAX; i++)
            {
                tmp_pid_tbl = &(in->config.pid.data[i]);
                GB_DEBUGI(LORA_TAG, "PID[%d]: %02x, %02x, %02x", i, tmp_pid_tbl->half_kp, tmp_pid_tbl->half_ki, tmp_pid_tbl->half_kd);
            }
            memcpy(&pid_first, in->config.pid.data, sizeof(LORA_GB_PID_INIT_T));
            break;
        }
        case GB_SET_PID_8_15:
        {
            memcpy(&pid_last, in->config.pid.data, sizeof(LORA_GB_PID_INIT_T));
            lora_pidInit_t *tmp_pid_tbl = NULL;
            for (int i = 0; i < PID_TYPE_MAX; i++)
            {
                tmp_pid_tbl = &(in->config.pid.data[i]);
                GB_DEBUGI(LORA_TAG, "PID[%d]: %02x, %02x, %02x", i, tmp_pid_tbl->half_kp, tmp_pid_tbl->half_ki, tmp_pid_tbl->half_kd);
                pidParam.data[i].kp = pid_first.data[i].half_kp << 8 | pid_last.data[i].half_kp;
                pidParam.data[i].ki = pid_first.data[i].half_ki << 8 | pid_last.data[i].half_ki;
                pidParam.data[i].kd = pid_first.data[i].half_kd << 8 | pid_last.data[i].half_kd;
            }
            setPidParam();
            *state = LORA_SEND;
            break;
        }
        case GB_SET_CONTROL_ARG:
            // throttle from 0 ~ 1000 for [pid to motor control]
            rev_throttle = in->config.control_arg.throttle;
            portENTER_CRITICAL(&receive_package_mutex);
            rev_set_info.rev_state.roll = _control_commander_to_range(GB_ROLL, in->config.control_arg.roll, 0);
            rev_set_info.rev_state.pitch = _control_commander_to_range(GB_PITCH, in->config.control_arg.pitch, 0);
            rev_set_info.rev_state.yaw = _control_commander_to_range(GB_YAW, in->config.control_arg.yaw, 0);

            // constrain roll/pitch/yaw speed from -500 ~ 500 for [pid to motor control]
            // rev_set_info.rev_speed.roll   = _control_commander_to_range(GB_ROLL, in->config.control_arg.roll, 1);
            // rev_set_info.rev_speed.pitch  = _control_commander_to_range(GB_PITCH, in->config.control_arg.pitch, 1);
            rev_set_info.rev_speed.yaw = _control_commander_to_range(GB_YAW, in->config.control_arg.yaw, 1);
            portEXIT_CRITICAL(&receive_package_mutex);
            portENTER_CRITICAL(&system_state_mutex);
            system_state = GB_SYSTEM_UNLOCK;
            _resetRemoteState();
            portEXIT_CRITICAL(&system_state_mutex);
            break;
#endif
        default:
            GB_DEBUGI(LORA_TAG, "Unknown setting type: %d", in->config.set_type);
        }
        break;
    case GB_GET_REQUEST:
        switch (in->request.get_type)
        {
        case GB_GET_BATTERY_INFO:
            break;
#if 0
        case GB_GET_PID_INFO_0_7:
        {
            out->type = GB_GET_REQUEST;
            pidInit_t *tmp_pid_tbl = NULL;
            for (int i = 0; i < PID_TYPE_MAX; i++)
            {
                tmp_pid_tbl = &(pidParam.data[i]);
                GB_DEBUGI(LORA_TAG, "PID[%d]: %04x, %04x, %04x", i, tmp_pid_tbl->kp, tmp_pid_tbl->ki, tmp_pid_tbl->kd);
                pid_first.data[i].half_kp = pidParam.data[i].kp >> 8;
                pid_first.data[i].half_ki = pidParam.data[i].ki >> 8;
                pid_first.data[i].half_kd = pidParam.data[i].kd >> 8;
            }
            memcpy(out->config.pid.data, pid_first.data, sizeof(LORA_GB_PID_INIT_T));
            break;
        }
        case GB_GET_PID_INFO_8_15:
        {
            out->type = GB_GET_REQUEST;
            pidInit_t *tmp_pid_tbl = NULL;
            for (int i = 0; i < PID_TYPE_MAX; i++)
            {
                tmp_pid_tbl = &(pidParam.data[i]);
                GB_DEBUGI(LORA_TAG, "PID[%d]: %04x, %04x, %04x", i, tmp_pid_tbl->kp, tmp_pid_tbl->ki, tmp_pid_tbl->kd);
                pid_last.data[i].half_kp = pidParam.data[i].kp & 0x00ff;
                pid_last.data[i].half_ki = pidParam.data[i].ki & 0x00ff;
                pid_last.data[i].half_kd = pidParam.data[i].kd & 0x00ff;
            }
            memcpy(out->config.pid.data, pid_last.data, sizeof(LORA_GB_PID_INIT_T));
            break;
        }
#endif
        case GB_GET_MOTION_STATE:

            if (xSemaphoreTake(motionStateMutex, portMAX_DELAY) == pdTRUE)
            {
                GB_DEBUGD(RF24_TAG, "motionState roll: %f, pitch: %f, yaw: %f", motionState.roll, motionState.pitch, motionState.yaw);
                // roll -> pitch
                // pitch -> roll
                // yaw -> yaw
                out->request.quad_status.roll = -motionState.pitch * GB_ERLER_SCALE_RATE;
                out->request.quad_status.pitch = motionState.roll * GB_ERLER_SCALE_RATE;
                out->request.quad_status.yaw = motionState.yaw * GB_ERLER_SCALE_RATE;
                GB_DEBUGD(RF24_TAG, "Send Quad status roll: %d, pitch: %d, yaw: %d",
                          out->request.quad_status.roll,
                          out->request.quad_status.pitch,
                          out->request.quad_status.yaw);

                xSemaphoreGive(motionStateMutex);
            }

            break;
        default:
            GB_DEBUGI(LORA_TAG, "Unknown setting type: %d", in->request.get_type);
        }
        *state = LORA_SEND;
        break;
    default:
        GB_DEBUGI(LORA_TAG, "Unknown PACKAGE type: %d", in->type);
    }
    out->sync = in->sync + 1; // check for remote

    return res;
}

void nrf24_interrupt_func(void *arg)
{
    GB_LORA_PACKAGE_T in_package;
    GB_LORA_PACKAGE_T out_package;
    uint8_t send_retry = 0;
    GB_LORA_STATE lora_state = LORA_IDLE;
    GB_MAX1704X_DEV_T dev = { 0 };

    GB_LoraSystemInit(LORA_RECEIVE, 0, &lora_state);
    GB_Max1704xInit(&dev);
    GB_Max1704xStart(&dev);
    nrf24_isr_register();

    while (1)
    {
        // portENTER_CRITICAL(&lora_mutex);
        // gpio_set_level( TEST_NRF24_IO, 1 );
        uint8_t rf_status = radio.get_status(&radio);
        if ((rf_status & _BV(RX_DR)) && lora_state == LORA_RECEIVE)
        {
            // This device is a RX node
            uint8_t pipe;
            if (radio.available(&radio, &pipe))
            {                                                 // is there a payload? get the pipe number that recieved itv
                uint8_t bytes = radio.getPayloadSize(&radio); // get the size of the payload
                radio.read(&radio, &in_package, bytes);       // fetch payload from FIFO
                CHK_LOGE(gb_lora_request_dispatch(&dev, &in_package, &out_package, &lora_state), "Remote info dispatch failed");
            }
            else
                GB_DEBUGI(RF24_TAG, "Received nothing...");
        }
        if (rf_status & _BV(TX_DS))
        {
            GB_DEBUGI(RF24_TAG, "Transmission successful! ");
        }
        if (rf_status & _BV(MAX_RT))
        {
            GB_DEBUGI(RF24_TAG, "Transmission MAX_RT! ");
        }
        // GB_DEBUGI(RF24_TAG, "RF status: %d", rf_status);
        radio.write_register(&radio, NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT), false);

        if (lora_state == LORA_SEND)
        {
            radio.stopListening(&radio);
            GB_RESULT report = radio.write(&radio, &out_package, sizeof(GB_LORA_PACKAGE_T));
            if (report >= 0)
            {
                GB_DEBUGV(RF24_TAG, "Transmission successful!, config: %02x", radio.read_register(&radio, NRF_CONFIG));
                lora_state = LORA_RECEIVE;
                radio.startListening(&radio);
                send_retry = 0;
            }
            else
            {
                GB_DEBUGI(RF24_TAG, "Transmission failed or timed out, retry: %d", send_retry);
                send_retry++;
                if (send_retry >= 5)
                {
                    send_retry = 0;
                    lora_state = LORA_RECEIVE;
                    radio.startListening(&radio);
                }
            }
        }
        // gpio_set_level( TEST_NRF24_IO, 0 );
        // portEXIT_CRITICAL(&lora_mutex);
        /* Wait to be notified that the transmission is complete.  Note
        the first parameter is pdTRUE, which has the effect of clearing
        the task's notification value back to 0, making the notification
        value act like a binary (rather than a counting) semaphore.  */
        uint32_t ul_notification_value;
        const TickType_t max_block_time = pdMS_TO_TICKS(100);
        ul_notification_value = ulTaskNotifyTake(pdTRUE, max_block_time);

        if (ul_notification_value == 1)
        {
            /* The transmission ended as expected. */
        }
        else
        {
            /* The call to ulTaskNotifyTake() timed out. */
        }
    }
}

// receive frame format 0xAA_ID_AA
/***********ANOTIC DEBUG*****************
  0xAA xxxx 0xAA    xxxx:anotic_debug_id


*****************************************/
void uart_rx_task(void *arg)
{
    uint8_t data[128] = {0};
    int rxBytes = 0;

    while (1)
    {

        GB_ReadBytes(data, (size_t *)&rxBytes);

        if (rxBytes <= 0)
            continue;
        if (rxBytes == 3 && data[0] == 0xaa && data[2] == 0xaa)
        {
            GB_DEBUGI(SENSOR_TAG, "Start anotic debug, id: %02x", data[1]);
            // gb_usb_write_bytes(data, rxBytes);
            anotic_debug_id = data[1];
        }
        else if (rxBytes == 16 && data[0] == 0xaa && data[2] == 0xe0) // anotic function debug
        {
            uint8_t ack[9] = {0x00};
            anotc_init_data(ack, 0x00, 3, sizeof(uint8_t), data[2], sizeof(uint8_t), data[15], sizeof(uint8_t), data[16]);

            GB_DUMPI(CHK_TAG, data, rxBytes);
            GB_DUMPI(CHK_TAG, ack, sizeof(ack));
            GB_WriteBytes((const uint8_t *)ack, sizeof(ack));
        }
        else
        {
            GB_DEBUGE(ERROR_TAG, "Receve data format error, length: %d", rxBytes);
            GB_DUMPE(ERROR_TAG, data, rxBytes);
        }
        rxBytes = 0;
        memset(data, 0, 128);
        /*
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            GB_DEBUGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            GB_DUMPI(RX_TASK_TAG, data, rxBytes);
        }
        */
    }
}
