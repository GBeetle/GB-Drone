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
 * @file mpu.h
 * mpu library main file. Declare mpu class.
 *
 * @attention
 *  mpu library requires I2Cbus or SPIbus library.
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
#include "mpu_math.h"
#include "log_sys.h"
#include "error_handle.h"
#include "bmp280.h"
#include "lis3mdl.h"

#ifdef CONFIG_MPU_I2C
#include "i2c_bus.h"
#elif CONFIG_MPU_SPI
#include "spi_bus.h"
#endif

#define MPU_SAMPLE_RATE 500

#define MPU_GYRO_STATUS_BIT        (1 << 0)
#define MPU_ACCEL_STATUS_BIT       (1 << 1)
#define MPU_AUX_BMP20_STATUS_BIT   (1 << 2)
#define MPU_AUX_LIS3MDL_STATUS_BIT (1 << 3)

/*! Motion Processing Unit */
struct mpu
{
    //! \}
    //! \name Basic
    //! \{
    struct mpu* (*setBus)(struct mpu *mpu, mpu_bus_t *bus);
    struct mpu* (*setAddr)(struct mpu *mpu, mpu_addr_handle_t addr);
    mpu_bus_t* (*getBus)(struct mpu *mpu);
    mpu_addr_handle_t (*getAddr)(struct mpu *mpu);
    GB_RESULT (*lastError)(struct mpu *mpu);
    //! \}
    //! \name Setup
    //! \{
    GB_RESULT (*initialize)(struct mpu *mpu);
    GB_RESULT (*reset)(struct mpu *mpu);
    GB_RESULT (*setSleep)(struct mpu *mpu, bool enable);
    GB_RESULT (*testConnection)(struct mpu *mpu);
    GB_RESULT (*selfTest)(struct mpu *mpu, selftest_t* result);
    GB_RESULT (*resetSignalPath)(struct mpu *mpu);
    uint8_t (*whoAmI)(struct mpu *mpu);
    bool (*getSleep)(struct mpu *mpu);
    //! \}
    //! \name Main configurations
    //! \{
    GB_RESULT (*setSampleRate)(struct mpu *mpu, uint16_t rate);
    GB_RESULT (*setClockSource)(struct mpu *mpu, clock_src_t clockSrc);
    GB_RESULT (*setDigitalLowPassFilter)(struct mpu *mpu, dlpf_t dlpf);
    uint16_t (*getSampleRate)(struct mpu *mpu);
    clock_src_t (*getClockSource)(struct mpu *mpu);
    dlpf_t (*getDigitalLowPassFilter)(struct mpu *mpu);
    //! \}
    //! \name Power management
    //! \{
    GB_RESULT (*setLowPowerAccelMode)(struct mpu *mpu, bool enable);
    GB_RESULT (*setLowPowerAccelRate)(struct mpu *mpu, lp_accel_rate_t rate);
    lp_accel_rate_t (*getLowPowerAccelRate)(struct mpu *mpu);
    bool (*getLowPowerAccelMode)(struct mpu *mpu);
    GB_RESULT (*setStandbyMode)(struct mpu *mpu, stby_en_t mask);
    stby_en_t (*getStandbyMode)(struct mpu *mpu);
    //! \}
    //! \name Full-Scale Range
    //! \{
    GB_RESULT (*setGyroFullScale)(struct mpu *mpu, gyro_fs_t fsr);
    GB_RESULT (*setAccelFullScale)(struct mpu *mpu, accel_fs_t fsr);
    gyro_fs_t (*getGyroFullScale)(struct mpu *mpu);
    accel_fs_t (*getAccelFullScale)(struct mpu *mpu);
    //! \}
    //! \name Offset / Bias
    //! \{
    GB_RESULT (*setGyroOffset)(struct mpu *mpu, raw_axes_t bias);
    GB_RESULT (*setAccelOffset)(struct mpu *mpu, raw_axes_t bias);
    raw_axes_t (*getGyroOffset)(struct mpu *mpu);
    raw_axes_t (*getAccelOffset)(struct mpu *mpu);
    GB_RESULT (*computeOffsets)(struct mpu *mpu, raw_axes_t* accel, raw_axes_t* gyro);
    //! \}
    //! \name Interrupt
    //! \{
    GB_RESULT (*setInterruptConfig)(struct mpu *mpu, int_config_t config);
    GB_RESULT (*setInterruptEnabled)(struct mpu *mpu, int_en_t mask);
    int_stat_t (*getInterruptStatus)(struct mpu *mpu);
    int_config_t (*getInterruptConfig)(struct mpu *mpu);
    int_en_t (*getInterruptEnabled)(struct mpu *mpu);
    //! \}
    //! \name FIFO
    //! \{
    GB_RESULT (*setFIFOMode)(struct mpu *mpu, fifo_mode_t mode);
    GB_RESULT (*setFIFOConfig)(struct mpu *mpu, fifo_config_t config);
    GB_RESULT (*setFIFOEnabled)(struct mpu *mpu, bool enable);
    GB_RESULT (*resetFIFO)(struct mpu *mpu);
    uint16_t (*getFIFOCount)(struct mpu *mpu);
    GB_RESULT (*readFIFO)(struct mpu *mpu, size_t length, uint8_t* data);
    GB_RESULT (*writeFIFO)(struct mpu *mpu, size_t length, const uint8_t* data);
    fifo_mode_t (*getFIFOMode)(struct mpu *mpu);
    fifo_config_t (*getFIFOConfig)(struct mpu *mpu);
    bool (*getFIFOEnabled)(struct mpu *mpu);
    //! \}
    //! \name Auxiliary I2C Master
    //! \{
    GB_RESULT (*setAuxI2CConfig)(struct mpu *mpu, const auxi2c_config_t *config);
    GB_RESULT (*setAuxI2CEnabled)(struct mpu *mpu, bool enable);
    GB_RESULT (*setAuxI2CReset)(struct mpu *mpu);
    GB_RESULT (*setAuxI2CSlaveConfig)(struct mpu *mpu, const auxi2c_slv_config_t *config);
    GB_RESULT (*setAuxI2CSlaveEnabled)(struct mpu *mpu, auxi2c_slv_t slave, bool enable);
    GB_RESULT (*setAuxI2CBypass)(struct mpu *mpu, bool enable);
    GB_RESULT (*readAuxI2CRxData)(struct mpu *mpu, size_t length, uint8_t* data, size_t skip);
    GB_RESULT (*restartAuxI2C)(struct mpu *mpu);
    auxi2c_stat_t (*getAuxI2CStatus)(struct mpu *mpu);
    auxi2c_config_t (*getAuxI2CConfig)(struct mpu *mpu);
    auxi2c_slv_config_t (*getAuxI2CSlaveConfig)(struct mpu *mpu, auxi2c_slv_t slave);
    bool (*getAuxI2CEnabled)(struct mpu *mpu);
    bool (*getAuxI2CSlaveEnabled)(struct mpu *mpu, auxi2c_slv_t slave);
    bool (*getAuxI2CBypass)(struct mpu *mpu);
    GB_RESULT (*auxI2CWriteByte)(struct mpu *mpu, uint8_t devAddr, uint8_t regAddr, uint8_t data);
    GB_RESULT (*auxI2CReadByte)(struct mpu *mpu, uint8_t devAddr, uint8_t regAddr, uint8_t* data);
    //! \}
    //! \name Motion Detection Interrupt
    //! \{
    GB_RESULT (*setMotionDetectConfig)(struct mpu *mpu, mot_config_t *config);
    mot_config_t (*getMotionDetectConfig)(struct mpu *mpu);
    GB_RESULT (*setMotionFeatureEnabled)(struct mpu *mpu, bool enable);
    bool (*getMotionFeatureEnabled)(struct mpu *mpu);
#if defined CONFIG_MPU6000 || defined CONFIG_MPU6050 || defined CONFIG_MPU9150
    GB_RESULT (*setZeroMotionConfig)(struct mpu *mpu, zrmot_config_t *config);
    zrmot_config_t (*getZeroMotionConfig)(struct mpu *mpu);
    GB_RESULT (*setFreeFallConfig)(struct mpu *mpu, ff_config_t *config);
    ff_config_t (*getFreeFallConfig)(struct mpu *mpu);
    mot_stat_t (*getMotionDetectStatus)(struct mpu *mpu);
#endif
    GB_RESULT (*compassReadBytes)(struct mpu *mpu, uint8_t device_addr, uint8_t regAddr, uint8_t* data, uint32_t size);
    GB_RESULT (*compassWriteByte)(struct mpu *mpu, uint8_t device_addr, uint8_t regAddr, uint8_t data);
    //! \}
    //! \name Compass | Magnetometer
    //! \{
    GB_RESULT (*compassInit)(struct mpu *mpu);
    GB_RESULT (*compassWhoAmI)(struct mpu *mpu);
    GB_RESULT (*compassReset)(struct mpu *mpu);
    GB_RESULT (*compassSetSampleMode)(struct mpu *mpu, mag_mode_t mode);
    GB_RESULT (*compassSetMeasurementMode)(struct mpu *mpu, lis3mdl_measurement_mode_t mode);
    GB_RESULT (*setMagfullScale)(struct mpu *mpu, lis3mdl_scale_t scale);
    GB_RESULT (*heading)(struct mpu *mpu, raw_axes_t* mag);
    GB_RESULT (*motion_mag)(struct mpu *mpu, raw_axes_t* accel, raw_axes_t* gyro, raw_axes_t* mag);

    GB_RESULT (*bmp280Init)(struct mpu *mpu, bmp280_params_t *params);
    GB_RESULT (*baroGetData)(struct mpu *mpu, baro_t *baro);
    //! \}
    //! \name Miscellaneous
    //! \{
    GB_RESULT (*setFsyncConfig)(struct mpu *mpu, int_lvl_t level);
    GB_RESULT (*setFsyncEnabled)(struct mpu *mpu, bool enable);
    int_lvl_t (*getFsyncConfig)(struct mpu *mpu);
    bool (*getFsyncEnabled)(struct mpu *mpu);
#if defined CONFIG_MPU6500
    GB_RESULT (*setFchoice)(struct mpu *mpu, fchoice_t fchoice);
    fchoice_t (*getFchoice)(struct mpu *mpu);
#endif
#if defined CONFIG_MPU9150 || (defined CONFIG_MPU6050 && !defined CONFIG_MPU6000)
    GB_RESULT (*setAuxVDDIOLevel)(struct mpu *mpu, auxvddio_lvl_t level);
    auxvddio_lvl_t (*getAuxVDDIOLevel)(struct mpu *mpu);
#endif
    //! \}
    //! \name Read / Write
    //! Functions to perform direct read or write operation(s) to registers.
    //! \{
    GB_RESULT (*readBit)(struct mpu *mpu, uint8_t regAddr, uint8_t bitNum, uint8_t* data);
    GB_RESULT (*readBits)(struct mpu *mpu, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t* data);
    GB_RESULT (*readByte)(struct mpu *mpu, uint8_t regAddr, uint8_t* data);
    GB_RESULT (*readBytes)(struct mpu *mpu, uint8_t regAddr, size_t length, uint8_t* data);
    GB_RESULT (*writeBit)(struct mpu *mpu, uint8_t regAddr, uint8_t bitNum, uint8_t data);
    GB_RESULT (*writeBits)(struct mpu *mpu, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
    GB_RESULT (*writeByte)(struct mpu *mpu, uint8_t regAddr, uint8_t data);
    GB_RESULT (*writeBytes)(struct mpu *mpu, uint8_t regAddr, size_t length, const uint8_t* data);
    GB_RESULT (*registerDump)(struct mpu *mpu, uint8_t start, uint8_t end);
    //! \}
    //! \name Sensor readings
    //! \{
    GB_RESULT (*acceleration)(struct mpu *mpu, raw_axes_t* accel);
    GB_RESULT (*acceleration_xyz)(struct mpu *mpu, int16_t* x, int16_t* y, int16_t* z);
    GB_RESULT (*rotation)(struct mpu *mpu, raw_axes_t* gyro);
    GB_RESULT (*rotation_xyz)(struct mpu *mpu, int16_t* x, int16_t* y, int16_t* z);
    GB_RESULT (*temperature)(struct mpu *mpu, int16_t* temp);
    GB_RESULT (*motion)(struct mpu *mpu, raw_axes_t* accel, raw_axes_t* gyro);
    GB_RESULT (*sensors)(struct mpu *mpu, raw_axes_t* accel, raw_axes_t* gyro, int16_t* temp);
    GB_RESULT (*sensors_sen)(struct mpu *mpu, sensors_t* sensors, size_t extsens_len);
    //! \}

    GB_RESULT (*accelSelfTest)(struct mpu *mpu, raw_axes_t *regularBias, raw_axes_t *selfTestBias, uint8_t* result);
    GB_RESULT (*gyroSelfTest)(struct mpu *mpu, raw_axes_t *regularBias, raw_axes_t *selfTestBias, uint8_t* result);
    GB_RESULT (*getBiases)(struct mpu *mpu, accel_fs_t accelFS, gyro_fs_t gyroFS, raw_axes_t* accelBias, raw_axes_t* gyroBias,
                        bool selftest);
    GB_RESULT (*setOffsets)(struct mpu *mpu);

    mpu_bus_t* bus;         /*!< Communication bus pointer, I2C / SPI */
    mpu_addr_handle_t addr; /*!< I2C address / SPI device handle */
    uint8_t buffer[16];     /*!< Commom buffer for temporary data */
    GB_RESULT err;          /*!< Holds last error code */
    bmp280_t bmp280_dev;
    uint8_t mpu_status;
};

void GB_MPU_Init(struct mpu *mpu);

extern const accel_fs_t g_accel_fs;
extern const gyro_fs_t g_gyro_fs;

#endif /* end of include guard: _MPU_DRIVER_H__ */
