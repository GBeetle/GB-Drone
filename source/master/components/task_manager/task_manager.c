/*
 * This file is part of welkin project (https://github.com/GBeetle/welkin).
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

#include "task_manager.h"

struct mpu mpu;  // create a default MPU object

static uint8_t anotic_debug_id = 0x00;

static void get_sensor_data(raw_axes_t *accelRaw, raw_axes_t *gyroRaw, raw_axes_t *magRaw,
    float_axes_t *accelG, float_axes_t *gyroDPS, float_axes_t *magDPS, baro_t *baro_data)
{
    mpu.acceleration(&mpu, accelRaw);  // fetch raw data from the registers
    mpu.rotation(&mpu, gyroRaw);       // fetch raw data from the registers
    mpu.heading(&mpu, magRaw);
    mpu.baroGetData(&mpu, baro_data);

    // Convert
    *accelG  = accelGravity_raw(accelRaw, g_accel_fs);
    *gyroDPS = gyroDegPerSec_raw(gyroRaw, g_gyro_fs);
    if (magRaw->data.x != 0 || magRaw->data.y != 0 || magRaw->data.z != 0)
    {
        *magDPS  = magGauss_raw(magRaw, lis3mdl_scale_12_Gs);
    }
}

void mpu_get_sensor_data(void* arg)
{
    raw_axes_t   accelRaw         = {0};                   // x, y, z axes as int16
    raw_axes_t   gyroRaw          = {0};                   // x, y, z axes as int16
    raw_axes_t   magRaw           = {0};                   // x, y, z axes as int16
    float_axes_t accelG           = {{0.0f, 0.0f, 0.0f}};  // accel axes in (g) gravity format
    float_axes_t gyroDPS          = {{0.0f, 0.0f, 0.0f}};  // gyro axes in (DPS) º/s format
    float_axes_t magDPS           = {{0.0f, 0.0f, 0.0f}};  // gyro axes in (Gauss) format
    uint8_t      send_buffer[100] = {0};
    uint32_t     realDataLen      = 0;
    baro_t       baro_data        = {0};

    // mpu initialization
    GB_MpuInit(&mpu);
    CHK_EXIT(mpu.testConnection(&mpu));
    CHK_EXIT(mpu.initialize(&mpu));

    // test for sensor is good & horizontal
    //selftest_t st_result;
    //CHK_EXIT(mpu.selfTest(&mpu, &st_result));
    //CHK_EXIT(mpu.setOffsets(&mpu));

    GB_DEBUGI(SENSOR_TAG, "mpu status: %02x", mpu.mpu_status);

    // Fusion initialization
    const FusionMatrix gyroscopeMisalignment = {{{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}};
    const FusionVector gyroscopeSensitivity = {{1.0f, 1.0f, 1.0f}};
    const FusionVector gyroscopeOffset = {
        .axis.x = 597.80f * gyroResolution(g_gyro_fs),
        .axis.y = -33.02f * gyroResolution(g_gyro_fs),
        .axis.z = 4.01f * gyroResolution(g_gyro_fs)
    };

    const FusionMatrix accelerometerMisalignment = {{{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}};
    const FusionVector accelerometerSensitivity = {
        .axis.x = (float)(INT16_MAX) / accelFSRvalue(g_accel_fs) / (2074.63 - 42.91),
        .axis.y = (float)(INT16_MAX) / accelFSRvalue(g_accel_fs) / (2047.97 - 8.0),
        .axis.z = (float)(INT16_MAX) / accelFSRvalue(g_accel_fs) / (1920.39 + 156.37)
    };
    const FusionVector accelerometerOffset = {
        .axis.x = 42.91f * accelResolution(g_accel_fs),
        .axis.y = 8.0f * accelResolution(g_accel_fs),
        .axis.z = -156.37f * accelResolution(g_accel_fs)
    };

    const FusionMatrix softIronMatrix = {{{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}};
    const FusionVector hardIronOffset = {{0.0f, 0.0f, 0.0f}};

    FusionOffset offset;
    FusionAhrs ahrs;

    FusionOffsetInitialise(&offset, MPU_SAMPLE_RATE);
    FusionAhrsInitialise(&ahrs);

    // Set AHRS algorithm settings
    const FusionAhrsSettings settings = {
            .convention = FusionConventionEnu,
            .gain = 0.5f,
            .gyroscopeRange = 2000.0f,
            .accelerationRejection = 10.0f,
            .magneticRejection = 10.0f,
            .recoveryTriggerPeriod = 5 * MPU_SAMPLE_RATE, /* 5 seconds */
    };
    FusionAhrsSetSettings(&ahrs, &settings);

    while (1) {
        //gpio_set_level( TEST_IMU_IO, 1 );
        if(mpu_isr_manager.mpu_isr_status) {

            get_sensor_data(&accelRaw, &gyroRaw, &magRaw, &accelG, &gyroDPS, &magDPS, &baro_data);

            // Acquire latest sensor data
            int64_t timestamp = 0;

            GB_GetTimerMs((uint64_t*)&timestamp); // replace this with actual gyroscope timestamp
            FusionVector gyroscope = {{gyroDPS.data.x, gyroDPS.data.y, gyroDPS.data.z}};  // degrees/s
            FusionVector accelerometer = {{accelG.data.x, accelG.data.y, accelG.data.z}}; // g
            FusionVector magnetometer = {{magDPS.data.x, magDPS.data.y, magDPS.data.z}};  // arbitrary units

            // Apply calibration
            gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
            accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
            magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);

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

            GB_DEBUGD(SENSOR_TAG, "Gyro.x %0.1f, Gyro.y %0.1f, Gyro.z %0.1f\n", gyroscope.axis.x, gyroscope.axis.y, gyroscope.axis.z);
            GB_DEBUGD(SENSOR_TAG, "Accel.x %0.1f, Accel.y %0.1f, Accel.z %0.1f\n", accelerometer.axis.x, accelerometer.axis.y, accelerometer.axis.z);
            GB_DEBUGD(SENSOR_TAG, "Mag.x %0.1f, Mag.y %0.1f, Mag.z %0.1f\n", magnetometer.axis.x, magnetometer.axis.y, magnetometer.axis.z);
            GB_DEBUGD(SENSOR_TAG, "Roll %0.1f, Pitch %0.1f, Yaw %0.1f, X %0.1f, Y %0.1f, Z %0.1f\n",
                                    euler.angle.roll, euler.angle.pitch, euler.angle.yaw,
                                    earth.axis.x, earth.axis.y, earth.axis.z);
            GB_DEBUGD(SENSOR_TAG, "baro_data: [%+6.2fPa %+6.2fC %+6.2fcm ] \n", baro_data.pressure, baro_data.temperature, baro_data.altitude);

            // note: communication_V7-20210528.pdf
            switch (anotic_debug_id) {
                case 0x00:
                    break;
                case 0x01:
                    anotc_init_data(send_buffer, &realDataLen, 0x01, 6,
                                    sizeof(uint16_t), float2int16(accelerometer.axis.x / accelResolution(g_accel_fs), 1),
                                    sizeof(uint16_t), float2int16(accelerometer.axis.y / accelResolution(g_accel_fs), 1),
                                    sizeof(uint16_t), float2int16(accelerometer.axis.z / accelResolution(g_accel_fs), 1),
                                    sizeof(uint16_t), float2int16(gyroscope.axis.x / gyroResolution(g_gyro_fs), 1),
                                    sizeof(uint16_t), float2int16(gyroscope.axis.y / gyroResolution(g_gyro_fs), 1),
                                    sizeof(uint16_t), float2int16(gyroscope.axis.z / gyroResolution(g_gyro_fs), 1), sizeof(uint8_t), 0x00);
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
            mpu_isr_manager.mpu_isr_status = DATA_NOT_READY;
        }
        //gpio_set_level( TEST_IMU_IO, 0 );
        //vTaskSuspend(mpu_isr_handle);
        /* Wait to be notified that the transmission is complete.  Note
        the first parameter is pdTRUE, which has the effect of clearing
        the task's notification value back to 0, making the notification
        value act like a binary (rather than a counting) semaphore.  */
        uint32_t ul_notification_value;
        const TickType_t max_block_time = pdMS_TO_TICKS( 200 );
        ul_notification_value = ulTaskNotifyTake(pdTRUE, max_block_time );

        if( ul_notification_value == 1 ) {
            /* The transmission ended as expected. */
        }
        else {
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

    while (1) {

        GB_ReadBytes(data, (size_t*)&rxBytes);

        if (rxBytes <= 0)
            continue;
        if (rxBytes == 3 && data[0] == 0xaa && data[2] == 0xaa)
        {
            GB_DEBUGI(SENSOR_TAG, "Start anotic debug, id: %02x", data[1]);
            //gb_usb_write_bytes(data, rxBytes);
            anotic_debug_id = data[1];
        }
        else if (rxBytes == 16 && data[0] == 0xaa && data[2] == 0xe0) //anotic function debug
        {
            uint8_t ack[9] = {0x00};
            anotc_init_data(ack, 0x00, 3, sizeof(uint8_t), data[2], sizeof(uint8_t), data[15], sizeof(uint8_t), data[16]);

            GB_DUMMPI(CHK_TAG, data, rxBytes);
            GB_DUMMPI(CHK_TAG, ack, sizeof(ack));
            GB_WriteBytes((const uint8_t *)ack, sizeof(ack));
        }
        else
        {
            GB_DEBUGE(ERROR_TAG, "Receve data format error, length: %d", rxBytes);
            GB_DUMMPE(ERROR_TAG, data, rxBytes);
        }
        rxBytes = 0;
        memset(data, 0, 128);
        /*
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            GB_DEBUGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            GB_DUMMPI(RX_TASK_TAG, data, rxBytes);
        }
        */
    }
}

