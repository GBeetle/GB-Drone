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
#include "driver/gpio.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include "Fusion.h"

#include "task_manager.h"

struct mpu mpu;  // create a default MPU object

static uint8_t anotic_debug_id = 0x00;
static uint64_t mag_calibration_start_time = 0;

static void get_sensor_data(raw_axes_t *accelRaw, raw_axes_t *gyroRaw, raw_axes_t *magRaw,
    float_axes_t *accelG, float_axes_t *gyroDPS, float_axes_t *magDPS, baro_t *baro_data)
{
    static float magZerof[3] = { 255.000000, 1393.000000, 1324.000000};

    mpu.acceleration(&mpu, accelRaw);  // fetch raw data from the registers
    mpu.rotation(&mpu, gyroRaw);       // fetch raw data from the registers
    mpu.heading(&mpu, magRaw);
    mpu.baroGetData(&mpu, baro_data);

    // compass data calibration
    //mag_calibration(magRaw, magZerof);

    // Convert
    *accelG  = accelGravity_raw(accelRaw, accel_fs);
    *gyroDPS = gyroDegPerSec_raw(gyroRaw, gyro_fs);
    if (magRaw->data.x != 0 || magRaw->data.y != 0 || magRaw->data.z != 0)
    {
        magRaw->data.x -= magZerof[0]; magRaw->data.y -= magZerof[1]; magRaw->data.z -= magZerof[2];
        //magRaw->x = magRaw->y = magRaw->z = 0;
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
    baro_t       baro_data        = {0};

    // mpu initialization
    GB_MpuInit(&mpu);
    CHK_EXIT(mpu.testConnection(&mpu));
    CHK_EXIT(mpu.initialize(&mpu));

    // test for sensor is good & horizontal
    selftest_t st_result;
    CHK_EXIT(mpu.selfTest(&mpu, &st_result));
    CHK_EXIT(mpu.setOffsets(&mpu));

    GB_DEBUGI(SENSOR_TAG, "mpu status: %02x", mpu.mpu_status);

    // mpu interrupt configuration
    gpio_config_t mpu_io_conf;
    mpu_io_conf.intr_type    = GPIO_INTR_POSEDGE;
    mpu_io_conf.pin_bit_mask = MPU_GPIO_INPUT_PIN_SEL;
    mpu_io_conf.mode         = GPIO_MODE_INPUT;
    mpu_io_conf.pull_up_en   = 0;
    mpu_io_conf.pull_down_en = 1;
    gpio_config(&mpu_io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(MPU_DMP_INT, mpu_dmp_isr_handler, (void*) MPU_DMP_INT);

    // Fusion initialization
    const FusionMatrix gyroscopeMisalignment = {{{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}};
    const FusionVector gyroscopeSensitivity = {{1.0f, 1.0f, 1.0f}};
    const FusionVector gyroscopeOffset = {{0.0f, 0.0f, 0.0f}};
    const FusionMatrix accelerometerMisalignment = {{{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}};
    const FusionVector accelerometerSensitivity = {{1.0f, 1.0f, 1.0f}};
    const FusionVector accelerometerOffset = {{0.0f, 0.0f, 0.0f}};
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
            const int64_t timestamp = esp_timer_get_time(); // replace this with actual gyroscope timestamp
            FusionVector gyroscope = {{gyroDPS.data.x, gyroDPS.data.y, gyroDPS.data.z}};  // replace this with actual gyroscope data in degrees/s
            FusionVector accelerometer = {{accelG.data.x, accelG.data.y, accelG.data.z}}; // replace this with actual accelerometer data in g
            FusionVector magnetometer = {{magDPS.data.x, magDPS.data.y, magDPS.data.z}};  // replace this with actual magnetometer data in arbitrary units

            // Apply calibration
            gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
            accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
            magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);

            // Update gyroscope offset correction algorithm
            gyroscope = FusionOffsetUpdate(&offset, gyroscope);

            // Calculate delta time (in seconds) to account for gyroscope sample clock error
            static int64_t previousTimestamp;
            const float deltaTime = (float) (timestamp - previousTimestamp) / (float) (1000 * 1000);  // us to seconds
            previousTimestamp = timestamp;

            // Update gyroscope AHRS algorithm
            FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, deltaTime);

            // Print algorithm outputs
            const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
            const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);

            GB_DEBUGD(SENSOR_TAG, "Roll %0.1f, Pitch %0.1f, Yaw %0.1f, X %0.1f, Y %0.1f, Z %0.1f\n",
                                    euler.angle.roll, euler.angle.pitch, euler.angle.yaw,
                                    earth.axis.x, earth.axis.y, earth.axis.z);
            GB_DEBUGD(SENSOR_TAG, "baro_data: [%+6.2fPa %+6.2fC %+6.2fcm ] \n", baro_data.pressure, baro_data.temperature, baro_data.altitude);

            // note: communication_V7-20210528.pdf
            switch (anotic_debug_id) {
                case 0x00:
                    break;
                case 0x01:
                    anotc_init_data(send_buffer, 0x01, 6, sizeof(uint16_t), accelRaw.data.x, sizeof(uint16_t), accelRaw.data.y, sizeof(uint16_t), accelRaw.data.z,
                        sizeof(uint16_t), gyroRaw.data.x, sizeof(uint16_t), gyroRaw.data.y, sizeof(uint16_t), gyroRaw.data.z, sizeof(uint8_t), 0x00);
                    break;
                case 0x02:
                    anotc_init_data(send_buffer, 0x02, 6, sizeof(uint16_t), magRaw.data.x, sizeof(uint16_t), magRaw.data.y, sizeof(uint16_t), magRaw.data.z,
                        sizeof(uint32_t), (uint32_t)baro_data.altitude, sizeof(uint16_t), float2int16(baro_data.temperature), sizeof(uint8_t), 0x00, sizeof(uint8_t), 0x00);
                    break;
                case 0x03:
                    // East(X)Nouth(Y)Up(Z) [MPU R.P.Y] to Anotic UI [Quadrotor R.P.Y]
                    // roll -> pitch
                    // pitch -> roll
                    // yaw -> -yaw
                    anotc_init_data(send_buffer, 0x03, 4, sizeof(uint16_t), float2int16(euler.angle.pitch), sizeof(uint16_t), float2int16(euler.angle.roll),
                        sizeof(uint16_t), float2int16(-euler.angle.yaw), sizeof(uint8_t), 0x01);
                    break;
                default:
                    GB_DEBUGE(ERROR_TAG, "wrong command id from uart: %02x\n", anotic_debug_id);
                    anotic_debug_id = 0x00;
            }
            if (anotic_debug_id >= 0x01 && anotic_debug_id <= 0x03)
            {
                GB_WriteBytes((const uint8_t *)send_buffer, send_buffer[3] + 6);
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

            // CID: 0x01  CMD0: 0x00  CMD1: 0x04 ======= MAG_CALIBRATION
            if (data[4] == 0x01 && data[5] == 0x00 && data[6] == 0x04)
            {
                mag_calibration_start_time = esp_timer_get_time();
            }
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

