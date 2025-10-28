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

/**
 * @file imu.h
 * imu library main file. Declare imu class.
 *
 * @attention
 *  imu library requires I2Cbus or SPIbus library.
 *  Select the communication protocol in `menuconfig`
 *  and include the corresponding library to project components.
 *
 * @note
 *  The following is taken in the code:
 *  - MPU9250 is the same as MPU6500 + AK8963
 *  - MPU9150 is the same as MPU6050 + AK8975
 *  - MPU6000 code equals MPU6050
 *  - MPU6555 code equals MPU6500
 *  - MPU9255 code equals MPU9250
 * */

#ifndef _MPU_DRIVER_H__
#define _MPU_DRIVER_H__

#include <stdint.h>
#include <math.h>
#include <string.h>
#include "imu_types.h"
#include "log_sys.h"
#include "error_handle.h"
#include "bmp280.h"
#include "lis3mdl.h"


#ifdef CONFIG_MPU_I2C
#include "i2c_bus.h"

#if defined CONFIG_MPU6500 || defined CONFIG_MPU6000 || defined CONFIG_MPU6050
typedef enum {  //
    MPU_I2CADDRESS_AD0_LOW  = 0x68,
    MPU_I2CADDRESS_AD0_HIGH = 0x69
} mpu_i2caddr_t;
#elif defined CONFIG_IMC42688
// TODO
#endif
typedef struct i2c mpu_bus_t;
typedef mpu_i2caddr_t mpu_addr_handle_t;
#elif CONFIG_MPU_SPI
#include "spi_bus.h"

typedef struct spi mpu_bus_t;
typedef uint8_t mpu_addr_handle_t;
#endif

#define IMU_GYRO_STATUS_BIT        (1 << 0)
#define IMU_ACCEL_STATUS_BIT       (1 << 1)
#define IMU_BARO_STATUS_BIT        (1 << 2)
#define IMU_MAG_STATUS_BIT         (1 << 3)

/*! Motion Processing Unit */
struct imu
{
    //! \}
    //! \name Basic
    //! \{
    struct imu* (*setBus)(struct imu *imu, mpu_bus_t *bus);
    struct imu* (*setAddr)(struct imu *imu, mpu_addr_handle_t addr);
    mpu_bus_t* (*getBus)(struct imu *imu);
    mpu_addr_handle_t (*getAddr)(struct imu *imu);
    GB_RESULT (*lastError)(struct imu *imu);
    //! \}
    //! \name Setup
    //! \{
    GB_RESULT (*initialize)(struct imu *imu);
    GB_RESULT (*reset)(struct imu *imu);
    GB_RESULT (*setSleep)(struct imu *imu, bool enable);
    GB_RESULT (*testConnection)(struct imu *imu);
    GB_RESULT (*selfTest)(struct imu *imu, selftest_t* result);
    GB_RESULT (*resetSignalPath)(struct imu *imu);
    uint8_t (*whoAmI)(struct imu *imu);
    bool (*getSleep)(struct imu *imu);
    GB_RESULT (*setFilters)(struct imu *imu, bool gyroFilters, bool accFilters);
    //! \}
    //! \name Main configurations
    //! \{
    GB_RESULT (*setSampleRate)(struct imu *imu, uint16_t rate);
    GB_RESULT (*setClockSource)(struct imu *imu, clock_src_t clockSrc);
    GB_RESULT (*setDigitalLowPassFilter)(struct imu *imu, dlpf_t dlpf);
    uint16_t (*getSampleRate)(struct imu *imu);
    clock_src_t (*getClockSource)(struct imu *imu);
    dlpf_t (*getDigitalLowPassFilter)(struct imu *imu);
    //! \}
    //! \name Power management
    //! \{
    GB_RESULT (*setLowPowerAccelMode)(struct imu *imu, bool enable);
    GB_RESULT (*setLowPowerAccelRate)(struct imu *imu, lp_accel_rate_t rate);
    lp_accel_rate_t (*getLowPowerAccelRate)(struct imu *imu);
    bool (*getLowPowerAccelMode)(struct imu *imu);
    GB_RESULT (*setStandbyMode)(struct imu *imu, stby_en_t mask);
    stby_en_t (*getStandbyMode)(struct imu *imu);
    //! \}
    //! \name Full-Scale Range
    //! \{
    GB_RESULT (*setGyroFullScale)(struct imu *imu, gyro_fs_t fsr);
    GB_RESULT (*setAccelFullScale)(struct imu *imu, accel_fs_t fsr);
    gyro_fs_t (*getGyroFullScale)(struct imu *imu);
    accel_fs_t (*getAccelFullScale)(struct imu *imu);
    //! \}
    //! \name Offset / Bias
    //! \{
    GB_RESULT (*setGyroOffset)(struct imu *imu, raw_axes_t bias);
    GB_RESULT (*setAccelOffset)(struct imu *imu, raw_axes_t bias);
    raw_axes_t (*getGyroOffset)(struct imu *imu);
    raw_axes_t (*getAccelOffset)(struct imu *imu);
    GB_RESULT (*computeOffsets)(struct imu *imu, raw_axes_t* accel, raw_axes_t* gyro);
    //! \}
    //! \name Interrupt
    //! \{
    GB_RESULT (*setInterruptConfig)(struct imu *imu, int_config_t config);
    GB_RESULT (*setInterruptEnabled)(struct imu *imu, int_en_t mask);
    int_stat_t (*getInterruptStatus)(struct imu *imu);
    int_config_t (*getInterruptConfig)(struct imu *imu);
    int_en_t (*getInterruptEnabled)(struct imu *imu);
    //! \}
    //! \name FIFO
    //! \{
    GB_RESULT (*setFIFOMode)(struct imu *imu, fifo_mode_t mode);
    GB_RESULT (*setFIFOConfig)(struct imu *imu, fifo_config_t config);
    GB_RESULT (*setFIFOEnabled)(struct imu *imu, bool enable);
    GB_RESULT (*resetFIFO)(struct imu *imu);
    uint16_t (*getFIFOCount)(struct imu *imu);
    GB_RESULT (*readFIFO)(struct imu *imu, size_t length, uint8_t* data);
    GB_RESULT (*writeFIFO)(struct imu *imu, size_t length, const uint8_t* data);
    fifo_mode_t (*getFIFOMode)(struct imu *imu);
    fifo_config_t (*getFIFOConfig)(struct imu *imu);
    bool (*getFIFOEnabled)(struct imu *imu);
    //! \}
    //! \name Auxiliary I2C Master
    //! \{
    GB_RESULT (*setAuxI2CConfig)(struct imu *imu, const auxi2c_config_t *config);
    GB_RESULT (*setAuxI2CEnabled)(struct imu *imu, bool enable);
    GB_RESULT (*setAuxI2CReset)(struct imu *imu);
    GB_RESULT (*setAuxI2CSlaveConfig)(struct imu *imu, const auxi2c_slv_config_t *config);
    GB_RESULT (*setAuxI2CSlaveEnabled)(struct imu *imu, auxi2c_slv_t slave, bool enable);
    GB_RESULT (*setAuxI2CBypass)(struct imu *imu, bool enable);
    GB_RESULT (*readAuxI2CRxData)(struct imu *imu, size_t length, uint8_t* data, size_t skip);
    GB_RESULT (*restartAuxI2C)(struct imu *imu);
    auxi2c_stat_t (*getAuxI2CStatus)(struct imu *imu);
    auxi2c_config_t (*getAuxI2CConfig)(struct imu *imu);
    auxi2c_slv_config_t (*getAuxI2CSlaveConfig)(struct imu *imu, auxi2c_slv_t slave);
    bool (*getAuxI2CEnabled)(struct imu *imu);
    bool (*getAuxI2CSlaveEnabled)(struct imu *imu, auxi2c_slv_t slave);
    bool (*getAuxI2CBypass)(struct imu *imu);
    GB_RESULT (*auxI2CWriteByte)(struct imu *imu, uint8_t devAddr, uint8_t regAddr, uint8_t data);
    GB_RESULT (*auxI2CReadByte)(struct imu *imu, uint8_t devAddr, uint8_t regAddr, uint8_t* data);
    //! \}
    //! \name Motion Detection Interrupt
    //! \{
    GB_RESULT (*setMotionDetectConfig)(struct imu *imu, mot_config_t *config);
    mot_config_t (*getMotionDetectConfig)(struct imu *imu);
    GB_RESULT (*setMotionFeatureEnabled)(struct imu *imu, bool enable);
    bool (*getMotionFeatureEnabled)(struct imu *imu);
#if defined CONFIG_MPU6000 || defined CONFIG_MPU6050 || defined CONFIG_MPU9150
    GB_RESULT (*setZeroMotionConfig)(struct imu *imu, zrmot_config_t *config);
    zrmot_config_t (*getZeroMotionConfig)(struct imu *imu);
    GB_RESULT (*setFreeFallConfig)(struct imu *imu, ff_config_t *config);
    ff_config_t (*getFreeFallConfig)(struct imu *imu);
    mot_stat_t (*getMotionDetectStatus)(struct imu *imu);
#endif
    GB_RESULT (*compassReadBytes)(struct imu *imu, uint8_t device_addr, uint8_t regAddr, uint8_t* data, uint32_t size);
    GB_RESULT (*compassWriteByte)(struct imu *imu, uint8_t device_addr, uint8_t regAddr, uint8_t data);
    //! \}
    //! \name Compass | Magnetometer
    //! \{
    GB_RESULT (*compassInit)(struct imu *imu);
    GB_RESULT (*compassWhoAmI)(struct imu *imu);
    GB_RESULT (*compassReset)(struct imu *imu);
    GB_RESULT (*compassSetSampleMode)(struct imu *imu, mag_mode_t mode);
    GB_RESULT (*compassSetMeasurementMode)(struct imu *imu, lis3mdl_measurement_mode_t mode);
    GB_RESULT (*setMagfullScale)(struct imu *imu, lis3mdl_scale_t scale);
    GB_RESULT (*heading)(struct imu *imu, raw_axes_t* mag);
    GB_RESULT (*motion_mag)(struct imu *imu, raw_axes_t* accel, raw_axes_t* gyro, raw_axes_t* mag);

    GB_RESULT (*bmp280Init)(struct imu *imu, bmp280_params_t *params);
    GB_RESULT (*baroGetData)(struct imu *imu, baro_t *baro);
    //! \}
    //! \name Miscellaneous
    //! \{
    GB_RESULT (*setFsyncConfig)(struct imu *imu, int_lvl_t level);
    GB_RESULT (*setFsyncEnabled)(struct imu *imu, bool enable);
    int_lvl_t (*getFsyncConfig)(struct imu *imu);
    bool (*getFsyncEnabled)(struct imu *imu);
#if defined CONFIG_MPU6500
    GB_RESULT (*setFchoice)(struct imu *imu, fchoice_t fchoice);
    fchoice_t (*getFchoice)(struct imu *imu);
#endif
#if defined CONFIG_MPU9150 || (defined CONFIG_MPU6050 && !defined CONFIG_MPU6000)
    GB_RESULT (*setAuxVDDIOLevel)(struct imu *imu, auxvddio_lvl_t level);
    auxvddio_lvl_t (*getAuxVDDIOLevel)(struct imu *imu);
#endif
    //! \}
    //! \name Read / Write
    //! Functions to perform direct read or write operation(s) to registers.
    //! \{
    GB_RESULT (*readBit)(struct imu *imu, uint8_t regAddr, uint8_t bitNum, uint8_t* data);
    GB_RESULT (*readBits)(struct imu *imu, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t* data);
    GB_RESULT (*readByte)(struct imu *imu, uint8_t regAddr, uint8_t* data);
    GB_RESULT (*readBytes)(struct imu *imu, uint8_t regAddr, size_t length, uint8_t* data);
    GB_RESULT (*writeBit)(struct imu *imu, uint8_t regAddr, uint8_t bitNum, uint8_t data);
    GB_RESULT (*writeBits)(struct imu *imu, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
    GB_RESULT (*writeByte)(struct imu *imu, uint8_t regAddr, uint8_t data);
    GB_RESULT (*writeBytes)(struct imu *imu, uint8_t regAddr, size_t length, const uint8_t* data);
    GB_RESULT (*registerDump)(struct imu *imu, uint8_t start, uint8_t end);
    //! \}
    //! \name Sensor readings
    //! \{
    GB_RESULT (*acceleration)(struct imu *imu, raw_axes_t* accel);
    GB_RESULT (*rotation)(struct imu *imu, raw_axes_t* gyro);
    GB_RESULT (*temperature)(struct imu *imu, int16_t* temp);
    GB_RESULT (*motion)(struct imu *imu, raw_axes_t* accel, raw_axes_t* gyro);
    GB_RESULT (*sensors)(struct imu *imu, raw_axes_t* accel, raw_axes_t* gyro, int16_t* temp);
    GB_RESULT (*sensors_sen)(struct imu *imu, sensors_t* sensors, size_t extsens_len);
    //! \}

    GB_RESULT (*accelSelfTest)(struct imu *imu, raw_axes_t *regularBias, raw_axes_t *selfTestBias, uint8_t* result);
    GB_RESULT (*gyroSelfTest)(struct imu *imu, raw_axes_t *regularBias, raw_axes_t *selfTestBias, uint8_t* result);
    GB_RESULT (*getBiases)(struct imu *imu, accel_fs_t accelFS, gyro_fs_t gyroFS, raw_axes_t* accelBias, raw_axes_t* gyroBias,
                        bool selftest);
    GB_RESULT (*setOffsets)(struct imu *imu, bool gyro, bool accel);

    mpu_bus_t* bus;         /*!< Communication bus pointer, I2C / SPI */
    mpu_addr_handle_t addr; /*!< I2C address / SPI device handle */
    uint8_t buffer[16];     /*!< Commom buffer for temporary data */
    GB_RESULT err;          /*!< Holds last error code */
    bmp280_t bmp280_dev;
    uint8_t mpu_status;
};

void GB_IMU_Init(struct imu *imu);

extern const accel_fs_t g_accel_fs;
extern const gyro_fs_t g_gyro_fs;
extern const lis3mdl_scale_t g_lis3mdl_fs;

#endif /* end of include guard: _MPU_DRIVER_H__ */
