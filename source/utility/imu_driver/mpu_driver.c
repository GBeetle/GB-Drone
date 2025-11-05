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

#include "imu_driver.h"
#include "io_define.h"
#include "gb_timer.h"
#include "isr_manager.h"
#include "imu_types.h"

static GB_RESULT initialize(struct imu *imu);
static GB_RESULT reset(struct imu *imu);
static GB_RESULT setSleep(struct imu *imu, bool enable);
static bool getSleep(struct imu *imu);
static GB_RESULT testConnection(struct imu *imu);
static uint8_t whoAmI(struct imu *imu);
static GB_RESULT setSampleRate(struct imu *imu, uint16_t rate);
static uint16_t getSampleRate(struct imu *imu);
static GB_RESULT setClockSource(struct imu *imu, clock_src_t clockSrc);
static clock_src_t getClockSource(struct imu *imu);
static GB_RESULT setDigitalLowPassFilter(struct imu *imu, dlpf_t dlpf);
static dlpf_t getDigitalLowPassFilter(struct imu *imu);
static GB_RESULT resetSignalPath(struct imu *imu);
static GB_RESULT setLowPowerAccelMode(struct imu *imu, bool enable);
static bool getLowPowerAccelMode(struct imu *imu);
static GB_RESULT setLowPowerAccelRate(struct imu *imu, lp_accel_rate_t rate);
static lp_accel_rate_t getLowPowerAccelRate(struct imu *imu);
static GB_RESULT setMotionFeatureEnabled(struct imu *imu, bool enable);
static bool getMotionFeatureEnabled(struct imu *imu);
static GB_RESULT setMotionDetectConfig(struct imu *imu, mot_config_t* config);
static mot_config_t getMotionDetectConfig(struct imu *imu);
#if defined CONFIG_MPU6050
static GB_RESULT setFreeFallConfig(struct imu *imu, ff_config_t* config);
static ff_config_t getFreeFallConfig(struct imu *imu);
static mot_stat_t getMotionDetectStatus(struct imu *imu);
#endif  // MPU6050's stuff
static GB_RESULT setStandbyMode(struct imu *imu, stby_en_t mask);
static stby_en_t getStandbyMode(struct imu *imu);

#if defined CONFIG_MPU6500
static GB_RESULT setFchoice(struct imu *imu, fchoice_t fchoice);
static fchoice_t getFchoice(struct imu *imu);
#endif

static GB_RESULT setGyroFullScale(struct imu *imu, gyro_fs_t fsr);
static gyro_fs_t getGyroFullScale(struct imu *imu);
static GB_RESULT setAccelFullScale(struct imu *imu, accel_fs_t fsr);
static accel_fs_t getAccelFullScale(struct imu *imu);
static GB_RESULT setGyroOffset(struct imu *imu, raw_axes_t bias);
static raw_axes_t getGyroOffset(struct imu *imu);
static GB_RESULT setAccelOffset(struct imu *imu, raw_axes_t bias);
static raw_axes_t getAccelOffset(struct imu *imu);
static GB_RESULT computeOffsets(struct imu *imu, raw_axes_t* accel, raw_axes_t* gyro);
static GB_RESULT acceleration(struct imu *imu, raw_axes_t* accel);
static GB_RESULT rotation(struct imu *imu, raw_axes_t* gyro);
static GB_RESULT temperature(struct imu *imu, int16_t* temp);
static GB_RESULT motion(struct imu *imu, raw_axes_t* accel, raw_axes_t* gyro);
static GB_RESULT sensors(struct imu *imu, raw_axes_t* accel, raw_axes_t* gyro, int16_t* temp);
static GB_RESULT sensors_sen(struct imu *imu, sensors_t* sensors, size_t extsens_len);

#if defined CONFIG_MPU9150 || (defined CONFIG_MPU6050 && !defined CONFIG_MPU6000)
static GB_RESULT setAuxVDDIOLevel(struct imu *imu, auxvddio_lvl_t level);
static auxvddio_lvl_t getAuxVDDIOLevel(struct imu *imu);
#endif

static GB_RESULT setInterruptConfig(struct imu *imu, int_config_t config);
static int_config_t getInterruptConfig(struct imu *imu);
static GB_RESULT setInterruptEnabled(struct imu *imu, int_en_t mask);
static int_en_t getInterruptEnabled(struct imu *imu);
static int_stat_t getInterruptStatus(struct imu *imu);
static GB_RESULT setFIFOMode(struct imu *imu, fifo_mode_t mode);
static fifo_mode_t getFIFOMode(struct imu *imu);
static GB_RESULT setFIFOConfig(struct imu *imu, fifo_config_t config);
static fifo_config_t getFIFOConfig(struct imu *imu);
static GB_RESULT setFIFOEnabled(struct imu *imu, bool enable);
static bool getFIFOEnabled(struct imu *imu);
static GB_RESULT resetFIFO(struct imu *imu);
static uint16_t getFIFOCount(struct imu *imu);
static GB_RESULT readFIFO(struct imu *imu, size_t length, uint8_t* data);
static GB_RESULT writeFIFO(struct imu *imu, size_t length, const uint8_t* data);
static GB_RESULT setAuxI2CConfig(struct imu *imu, const auxi2c_config_t* config);
static auxi2c_config_t getAuxI2CConfig(struct imu *imu);
static GB_RESULT setAuxI2CEnabled(struct imu *imu, bool enable);
static GB_RESULT setAuxI2CReset(struct imu *imu);
static bool getAuxI2CEnabled(struct imu *imu);
static GB_RESULT setAuxI2CSlaveConfig(struct imu *imu, const auxi2c_slv_config_t* config);
static auxi2c_slv_config_t getAuxI2CSlaveConfig(struct imu *imu, auxi2c_slv_t slave);
static GB_RESULT setAuxI2CSlaveEnabled(struct imu *imu, auxi2c_slv_t slave, bool enable);
static bool getAuxI2CSlaveEnabled(struct imu *imu, auxi2c_slv_t slave);
static GB_RESULT setAuxI2CBypass(struct imu *imu, bool enable);
static bool getAuxI2CBypass(struct imu *imu);
static GB_RESULT readAuxI2CRxData(struct imu *imu, size_t length, uint8_t* data, size_t skip);
static GB_RESULT restartAuxI2C(struct imu *imu);
static auxi2c_stat_t getAuxI2CStatus(struct imu *imu);
static GB_RESULT auxI2CWriteByte(struct imu *imu, uint8_t devAddr, uint8_t regAddr, const uint8_t data);
static GB_RESULT auxI2CReadByte(struct imu *imu, uint8_t devAddr, uint8_t regAddr, uint8_t* data);
static GB_RESULT setFsyncConfig(struct imu *imu, int_lvl_t level);
static int_lvl_t getFsyncConfig(struct imu *imu);
static GB_RESULT setFsyncEnabled(struct imu *imu, bool enable);
static bool getFsyncEnabled(struct imu *imu);
static GB_RESULT registerDump(struct imu *imu, uint8_t start, uint8_t end);
static GB_RESULT compassReadBytes(struct imu *imu, uint8_t device_addr, uint8_t regAddr, uint8_t* data, uint32_t size);
static GB_RESULT compassWriteByte(struct imu *imu, uint8_t device_addr, uint8_t regAddr, uint8_t data);

static GB_RESULT compassInit(struct imu *imu);
static GB_RESULT compassWhoAmI(struct imu *imu);
static GB_RESULT compassReset(struct imu *imu);
static GB_RESULT compassSetMeasurementMode(struct imu *imu, compass_measurement_mode_t mode);
static GB_RESULT compassSetSampleMode(struct imu *imu, mag_mode_t mode);
static GB_RESULT setMagfullScale(struct imu *imu, compass_scale_t scale);

static GB_RESULT heading(struct imu *imu, raw_axes_t* mag);
static GB_RESULT motion_mag(struct imu *imu, raw_axes_t* accel, raw_axes_t* gyro, raw_axes_t* mag);

static GB_RESULT baroInit(struct imu *imu);
static GB_RESULT baroGetData(struct imu *imu, baro_t *baro);

static GB_RESULT selfTest(struct imu *imu, selftest_t* result);
static GB_RESULT accelSelfTest(struct imu *imu, raw_axes_t* regularBias, raw_axes_t* selfTestBias, uint8_t* result);
static GB_RESULT gyroSelfTest(struct imu *imu, raw_axes_t* regularBias, raw_axes_t* selfTestBias, uint8_t* result);
static GB_RESULT getBiases(struct imu *imu, accel_fs_t accelFS, gyro_fs_t gyroFS, raw_axes_t* accelBias, raw_axes_t* gyroBias,
                         bool selftest);
static GB_RESULT setOffsets(struct imu *imu, bool gyro, bool accel);

const accel_fs_t g_accel_fs = ACCEL_FS_16G;
const gyro_fs_t g_gyro_fs = GYRO_FS_2000DPS;
const compass_scale_t g_compass_fs = lis3mdl_scale_16_Gs;

/**
 * @brief Set communication bus.
 * @param bus Bus protocol object of type `I2Cbus` or `SPIbus`.
 */
static struct imu* setBus(struct imu *imu, mpu_bus_t *bus);
/**
 * @brief Return communication bus object.
 */
static mpu_bus_t* getBus(struct imu *imu);
/**
 * @brief Set I2C address or SPI device handle.
 * @param addr I2C address (`mpu_i2caddr_t`) or SPI device handle (`spi_device_handle_t`).
 */
static struct imu* setAddr(struct imu *imu, mpu_addr_handle_t addr);
/**
 * @brief Return I2C address or SPI device handle.
 */
static mpu_addr_handle_t getAddr(struct imu *imu);
/*! Return last error code. */
static GB_RESULT lastError(struct imu *imu);
/*! Read a single bit from a register*/
static GB_RESULT readBit(struct imu *imu, uint8_t regAddr, uint8_t bitNum, uint8_t* data);
/*! Read a range of bits from a register */
static GB_RESULT readBits(struct imu *imu, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t* data);
/*! Read a single register */
static GB_RESULT readByte(struct imu *imu, uint8_t regAddr, uint8_t* data);
/*! Read data from sequence of registers */
static GB_RESULT readBytes(struct imu *imu, uint8_t regAddr, size_t length, uint8_t* data);
/*! Write a single bit to a register */
static GB_RESULT writeBit(struct imu *imu, uint8_t regAddr, uint8_t bitNum, uint8_t data);
/*! Write a range of bits to a register */
static GB_RESULT writeBits(struct imu *imu, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
/*! Write a value to a register */
static GB_RESULT writeByte(struct imu *imu, uint8_t regAddr, uint8_t data);
/*! Write a sequence to data to a sequence of registers */
static GB_RESULT writeBytes(struct imu *imu, uint8_t regAddr, size_t length, const uint8_t* data);

// ==============
// Inline methods
// ==============

/**
 * @brief Construct a imu in the given communication bus and address.
 * @param bus Bus protocol object of type `I2Cbus` or `SPIbus`.
 * @param addr I2C address (`mpu_i2caddr_t`) or SPI device handle (`spi_device_handle_t`).
 */
void GB_IMU_Init(struct imu *imu) {

#if defined CONFIG_MPU_SPI
    // disable SPI DMA in begin
    CHK_EXIT(fspi.begin(&fspi, MPU_FSPI_MOSI, MPU_FSPI_MISO, MPU_FSPI_SCLK, SPI_MAX_DMA_LEN));
    CHK_EXIT(fspi.addDevice(&fspi, 8, 0, 0, MPU_SPI_CLOCK_SPEED, MPU_FSPI_CS));

    imu->bus  = &fspi;
    imu->addr = 0x00;
#endif

#if defined CONFIG_MPU_I2C
    mpu_addr_handle_t  MPU_DEFAULT_I2CADDRESS = MPU_I2CADDRESS_AD0_LOW;

    CHK_EXIT(i2c0.begin(&i2c0, MPU_SDA, MPU_SCL, MPU_I2C_CLOCK_SPEED));
    CHK_EXIT(i2c0.addDevice(&i2c0, MPU_DEFAULT_I2CADDRESS, MPU_I2C_CLOCK_SPEED));
    CHK_EXIT(i2c0.addDevice(&i2c0, BMP280_I2C_ADDRESS_1, MPU_I2C_CLOCK_SPEED));
    CHK_EXIT(i2c0.addDevice(&i2c0, COMPASS_I2CADDRESS, MPU_I2C_CLOCK_SPEED));

    imu->bus  = &i2c0;
    imu->addr = MPU_DEFAULT_I2CADDRESS;
#endif

    CHK_EXIT(mpu_isr_register());

    memset(imu->buffer, 0xff, 16);
    imu->err = GB_OK;
    imu->mpu_status = 0x00;

    imu->initialize              = &initialize;
    imu->reset                   = &reset;
    imu->setSleep                = &setSleep;
    imu->getSleep                = &getSleep;
    imu->testConnection          = &testConnection;
    imu->whoAmI                  = &whoAmI;
    imu->setSampleRate           = &setSampleRate;
    imu->getSampleRate           = &getSampleRate;
    imu->setClockSource          = &setClockSource;
    imu->getClockSource          = &getClockSource;
    imu->setDigitalLowPassFilter = &setDigitalLowPassFilter;
    imu->getDigitalLowPassFilter = &getDigitalLowPassFilter;
    imu->resetSignalPath         = &resetSignalPath;
    imu->setLowPowerAccelMode    = &setLowPowerAccelMode;
    imu->getLowPowerAccelMode    = &getLowPowerAccelMode;
    imu->setLowPowerAccelRate    = &setLowPowerAccelRate;
    imu->getLowPowerAccelRate    = &getLowPowerAccelRate;
    imu->setMotionFeatureEnabled = &setMotionFeatureEnabled;
    imu->getMotionFeatureEnabled = &getMotionFeatureEnabled;
    imu->setMotionDetectConfig   = &setMotionDetectConfig;
    imu->getMotionDetectConfig   = &getMotionDetectConfig;
#if defined CONFIG_MPU6050
    imu->setFreeFallConfig     = &setFreeFallConfig;
    imu->getFreeFallConfig     = &getFreeFallConfig;
    imu->getMotionDetectStatus = &getMotionDetectStatus;
#endif  // MPU6050's stuff
    imu->setStandbyMode = &setStandbyMode;
    imu->getStandbyMode = &getStandbyMode;

#if defined CONFIG_MPU6500
    imu->setFchoice = &setFchoice;
    imu->getFchoice = &getFchoice;
#endif

    imu->setGyroFullScale  = &setGyroFullScale;
    imu->getGyroFullScale  = &getGyroFullScale;
    imu->setAccelFullScale = &setAccelFullScale;
    imu->getAccelFullScale = &getAccelFullScale;
    imu->setGyroOffset     = &setGyroOffset;
    imu->getGyroOffset     = &getGyroOffset;
    imu->setAccelOffset    = &setAccelOffset;
    imu->getAccelOffset    = &getAccelOffset;
    imu->computeOffsets    = &computeOffsets;
    imu->acceleration      = &acceleration;
    imu->rotation          = &rotation;
    imu->temperature       = &temperature;
    imu->motion            = &motion;
    imu->sensors           = &sensors;
    imu->sensors_sen       = &sensors_sen;

#if defined CONFIG_MPU9150 || (defined CONFIG_MPU6050 && !defined CONFIG_MPU6000)
    imu->setAuxVDDIOLevel = &setAuxVDDIOLevel;
    imu->getAuxVDDIOLevel = &getAuxVDDIOLevel;
#endif
    imu->setInterruptConfig    = &setInterruptConfig;
    imu->getInterruptConfig    = &getInterruptConfig;
    imu->setInterruptEnabled   = &setInterruptEnabled;
    imu->getInterruptEnabled   = &getInterruptEnabled;
    imu->getInterruptStatus    = &getInterruptStatus;
    imu->setFIFOMode           = &setFIFOMode;
    imu->getFIFOMode           = &getFIFOMode;
    imu->setFIFOConfig         = &setFIFOConfig;
    imu->getFIFOConfig         = &getFIFOConfig;
    imu->setFIFOEnabled        = &setFIFOEnabled;
    imu->getFIFOEnabled        = &getFIFOEnabled;
    imu->resetFIFO             = &resetFIFO;
    imu->getFIFOCount          = &getFIFOCount;
    imu->readFIFO              = &readFIFO;
    imu->writeFIFO             = &writeFIFO;
    imu->setAuxI2CConfig       = &setAuxI2CConfig;
    imu->getAuxI2CConfig       = &getAuxI2CConfig;
    imu->setAuxI2CEnabled      = &setAuxI2CEnabled;
    imu->setAuxI2CReset        = &setAuxI2CReset;
    imu->getAuxI2CEnabled      = &getAuxI2CEnabled;
    imu->setAuxI2CSlaveConfig  = &setAuxI2CSlaveConfig;
    imu->getAuxI2CSlaveConfig  = &getAuxI2CSlaveConfig;
    imu->setAuxI2CSlaveEnabled = &setAuxI2CSlaveEnabled;
    imu->getAuxI2CSlaveEnabled = &getAuxI2CSlaveEnabled;
    imu->setAuxI2CBypass       = &setAuxI2CBypass;
    imu->getAuxI2CBypass       = &getAuxI2CBypass;
    imu->readAuxI2CRxData      = &readAuxI2CRxData;
    imu->restartAuxI2C         = &restartAuxI2C;
    imu->getAuxI2CStatus       = &getAuxI2CStatus;
    imu->auxI2CWriteByte       = &auxI2CWriteByte;
    imu->auxI2CReadByte        = &auxI2CReadByte;
    imu->setFsyncConfig        = &setFsyncConfig;
    imu->getFsyncConfig        = &getFsyncConfig;
    imu->setFsyncEnabled       = &setFsyncEnabled;
    imu->getFsyncEnabled       = &getFsyncEnabled;
    imu->registerDump          = &registerDump;
    imu->compassReadBytes      = &compassReadBytes;
    imu->compassWriteByte      = &compassWriteByte;

    imu->compassInit               = &compassInit;
    imu->compassSetSampleMode      = &compassSetSampleMode;
    imu->compassWhoAmI             = &compassWhoAmI;
    imu->compassReset              = &compassReset;
    imu->compassSetMeasurementMode = &compassSetMeasurementMode;
    imu->setMagfullScale           = &setMagfullScale;
    imu->heading                   = &heading;
    imu->motion_mag                = &motion_mag;

    // bmp280
    imu->baroInit  = &baroInit;
    imu->baroGetData = &baroGetData;

    imu->selfTest      = &selfTest;
    imu->accelSelfTest = &accelSelfTest;
    imu->gyroSelfTest  = &gyroSelfTest;
    imu->getBiases     = &getBiases;
    imu->setOffsets    = &setOffsets;

    imu->setBus     = &setBus;
    imu->getBus     = &getBus;
    imu->setAddr    = &setAddr;
    imu->getAddr    = &getAddr;
    imu->lastError  = &lastError;
    imu->readBit    = &readBit;
    imu->readBits   = &readBits;
    imu->readByte   = &readByte;
    imu->readBytes  = &readBytes;
    imu->writeBit   = &writeBit;
    imu->writeBits  = &writeBits;
    imu->writeByte  = &writeByte;
    imu->writeBytes = &writeBytes;
}

/**
 * @brief Set communication bus.
 * @param bus Bus protocol object of type `I2Cbus` or `SPIbus`.
 */
static struct imu* setBus(struct imu *imu, mpu_bus_t *bus)
{
    imu->bus = bus;
    return imu;
}
/**
 * @brief Return communication bus object.
 */
static mpu_bus_t* getBus(struct imu *imu)
{
    return imu->bus;
}
/**
 * @brief Set I2C address or SPI device handle.
 * @param addr I2C address (`mpu_i2caddr_t`) or SPI device handle (`spi_device_handle_t`).
 */
static struct imu* setAddr(struct imu *imu, mpu_addr_handle_t addr)
{
    imu->addr = addr;
    return imu;
}
/**
 * @brief Return I2C address or SPI device handle.
 */
static mpu_addr_handle_t getAddr(struct imu *imu)
{
    return imu->addr;
}
/*! Return last error code. */
static GB_RESULT lastError(struct imu *imu)
{
    return imu->err;
}
/*! Read a single bit from a register*/
static GB_RESULT readBit(struct imu *imu, uint8_t regAddr, uint8_t bitNum, uint8_t* data)
{
    return imu->bus->readBit(imu->bus, imu->addr, regAddr, bitNum, data);
}
/*! Read a range of bits from a register */
static GB_RESULT readBits(struct imu *imu, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t* data)
{
    return imu->bus->readBits(imu->bus, imu->addr, regAddr, bitStart, length, data);
}
/*! Read a single register */
static GB_RESULT readByte(struct imu *imu, uint8_t regAddr, uint8_t* data)
{
    return imu->bus->readByte(imu->bus, imu->addr, regAddr, data);
}
/*! Read data from sequence of registers */
static GB_RESULT readBytes(struct imu *imu, uint8_t regAddr, size_t length, uint8_t* data)
{
    return imu->bus->readBytes(imu->bus, imu->addr, regAddr, length, data);
}
/*! Write a single bit to a register */
static GB_RESULT writeBit(struct imu *imu, uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
    return imu->bus->writeBit(imu->bus, imu->addr, regAddr, bitNum, data);
}
/*! Write a range of bits to a register */
static GB_RESULT writeBits(struct imu *imu, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
{
    return imu->bus->writeBits(imu->bus, imu->addr, regAddr, bitStart, length, data);
}
/*! Write a value to a register */
static GB_RESULT writeByte(struct imu *imu, uint8_t regAddr, uint8_t data)
{
    return imu->bus->writeByte(imu->bus, imu->addr, regAddr, data);
}
/*! Write a sequence to data to a sequence of registers */
static GB_RESULT writeBytes(struct imu *imu, uint8_t regAddr, size_t length, const uint8_t* data)
{
    return imu->bus->writeBytes(imu->bus, imu->addr, regAddr, length, data);
}

/**
 * @brief Initialize imu device and set basic configurations.
 * @details
 *  Init configuration:
 *  - Accel FSR: 4G
 *  - Gyro FSR: 500DPS
 *  - Sample rate: 100Hz
 *  - DLPF: 42Hz
 *  - INT pin: disabled
 *  - FIFO: disabled
 *  - Clock source: gyro PLL \n
 *  For MPU9150 and MPU9250:
 *  - Aux I2C Master: enabled, clock: 400KHz
 *  - Compass: enabled on Aux I2C's Slave 0 and Slave 1
 *
 * @note
 *  - A soft reset is performed first, which takes 100-200ms.
 *  - When using SPI, the primary I2C Slave module is disabled right away.
 * */
static GB_RESULT initialize(struct imu *imu)
{
    GB_RESULT res = GB_OK;
    // reset device (wait a little to clear all registers)
    CHK_RES(imu->reset(imu));
    GB_SleepMs(100);
    // wake-up the device (power on-reset state is asleep for some models)
    CHK_RES(imu->setSleep(imu, false));
    // disable imu's I2C slave module when using SPI
#ifdef CONFIG_MPU_SPI
    CHK_RES(imu->writeBit(imu, USER_CTRL, USERCTRL_I2C_IF_DIS_BIT, 1));
#endif
    // set clock source to gyro PLL which is better than internal clock
    CHK_RES(imu->setClockSource(imu, CLOCK_PLL));

#if defined CONFIG_MPU6500
    // MPU6500 / MPU9250 share 4kB of memory between the DMP and the FIFO. Since the
    // first 3kB are needed by the DMP, we'll use the last 1kB for the FIFO.
    CHK_RES(imu->writeBits(imu, ACCEL_CONFIG2, ACONFIG2_FIFO_SIZE_BIT, ACONFIG2_FIFO_SIZE_LENGTH, FIFO_SIZE_1K));
#endif

    // set Full Scale range
    CHK_RES(imu->setGyroFullScale(imu, g_gyro_fs));
    CHK_RES(imu->setAccelFullScale(imu, g_accel_fs));

    GB_SleepMs(5000);   // DON'T REMOVE IT !!!

    CHK_RES(imu->compassInit(imu));

#if defined CONFIG_AUX_BAROMETER
    CHK_RES(imu->baroInit(imu));

    // waitting for bmp280 initialized done
    GB_SleepMs(1000);
    imu->mpu_status |= IMU_BARO_STATUS_BIT;
#endif

    // set gyro to 32kHz fchoice 0 | 1 & set accel to 4KHz fchoice 0
    //CHK_RES(imu->setFchoice(imu, FCHOICE_0));

    // set gyro to 8kHz & set accel to 4kHz
    //CHK_RES(imu->setFchoice(imu, FCHOICE_3));
    //CHK_RES(imu->setDigitalLowPassFilter(imu, DLPF_3600HZ_NOLPF));

    // set Digital Low Pass Filter to get smoother data
    CHK_RES(imu->setDigitalLowPassFilter(imu, DLPF_98HZ));
    // set sample rate to 1000Hz  from 4Hz - 1kHz
    CHK_RES(imu->setSampleRate(imu, IMU_SAMPLE_RATE));

    int_config_t config = {
        .level = INT_LVL_ACTIVE_HIGH,
        .drive = INT_DRV_PUSHPULL,
        .mode  = INT_MODE_PULSE50US,
        .clear = INT_CLEAR_STATUS_REG
    };
    CHK_RES(imu->setInterruptConfig(imu, config));

    int_en_t mask = INT_EN_RAWDATA_READY;
    CHK_RES(imu->setInterruptEnabled(imu, mask));

    GB_SleepMs(3000);
    //GB_DEBUGI(SENSOR_TAG, "MPU Init Done");
    imu->mpu_status |= IMU_GYRO_STATUS_BIT;
    imu->mpu_status |= IMU_ACCEL_STATUS_BIT;

error_exit:
    return res;
}

/**
 * @brief Reset internal registers and restore to default start-up state.
 * @note
 *  - This function delays 100ms when using I2C and 200ms when using SPI.
 *  - It does not initialize the imu again, just call initialize() instead.
 * */
static GB_RESULT reset(struct imu *imu)
{
    GB_RESULT res = GB_OK;

    CHK_RES(imu->writeBit(imu, PWR_MGMT1, PWR1_DEVICE_RESET_BIT, 1));
    GB_SleepMs(100);
#ifdef CONFIG_MPU_SPI
    CHK_RES(imu->resetSignalPath(imu));
#endif
error_exit:
    return res;
}

/**
 * @brief Enable / disable sleep mode
 * @param enable enable value
 * */
static GB_RESULT setSleep(struct imu *imu, bool enable)
{
    return imu->writeBit(imu, PWR_MGMT1, PWR1_SLEEP_BIT, (uint8_t) enable);
}

/**
 * @brief Get current sleep state.
 * @return
 *  - `true`: sleep enabled.
 *  - `false`: sleep disabled.
 */
bool getSleep(struct imu *imu)
{
    imu->err = imu->readBit(imu, PWR_MGMT1, PWR1_SLEEP_BIT, imu->buffer);
    return imu->buffer[0];
}

/**
 * @brief Test connection with imu.
 * @details It reads the WHO_AM_IM register and check its value against the correct chip model.
 * @return
 *  - `GB_OK`: The imu is connected and matchs the model.
 *  - `GB_MPU_NOT_FOUND`: A device is connect, but does not match the chip selected in _menuconfig_.
 * */
static GB_RESULT testConnection(struct imu *imu)
{
    GB_RESULT res = GB_OK;

    GB_SleepMs(1000);
    const uint8_t wai = imu->whoAmI(imu);

    CHK_RES(imu->lastError(imu));
#if defined CONFIG_MPU6000 || defined CONFIG_MPU6050 || defined CONFIG_MPU9150
    return (wai == 0x68) ? GB_OK : GB_MPU_NOT_FOUND;
#elif defined CONFIG_MPU9255
    return (wai == 0x73) ? GB_OK : GB_MPU_NOT_FOUND;
#elif defined CONFIG_MPU9250
    return (wai == 0x71) ? GB_OK : GB_MPU_NOT_FOUND;
#elif defined CONFIG_MPU6555
    return (wai == 0x7C) ? GB_OK : GB_MPU_NOT_FOUND;
#elif defined CONFIG_MPU6500
    return (wai == 0x70) ? GB_OK : GB_MPU_NOT_FOUND;
#endif
error_exit:
    return res;
}

/**
 * @brief Returns the value from WHO_AM_I register.
 */
uint8_t whoAmI(struct imu *imu)
{
    imu->err = imu->readByte(imu, WHO_AM_I, imu->buffer);
    return imu->buffer[0];
}

/**
 * @brief Set sample rate of data output.
 *
 * Sample rate controls sensor data output rate and FIFO sample rate.
 * This is the update rate of sensor register. \n
 * Formula: Sample Rate = Internal Output Rate / (1 + SMPLRT_DIV)
 *
 * @param rate 4Hz ~ 1KHz
 *  - For sample rate 8KHz: set digital low pass filter to DLPF_256HZ_NOLPF.
 *  - For sample rate 32KHZ [MPU6500 / MPU9250]: set fchoice to FCHOICE_0, see setFchoice().
 *
 * @note
 *  For MPU9150 & MPU9250:
 *   - When using compass, this function alters Aux I2C Master `sample_delay` property
 *     to adjust the compass sample rate. (also, `wait_for_es` property to adjust interrupt).
 *   - If sample rate lesser than 100 Hz, data-ready interrupt will wait for compass data.
 *   - If sample rate greater than 100 Hz, data-ready interrupt will not be delayed by the compass.
 * */
static GB_RESULT setSampleRate(struct imu *imu, uint16_t rate)
{
    GB_RESULT res = GB_OK;
    // Check value range
    if (rate < 4) {
        GB_DEBUGW(ERROR_TAG, "INVALID_SAMPLE_RATE %d, minimum rate is 4", rate);
        rate = 4;
    }
    else if (rate > 1000) {
        GB_DEBUGW(ERROR_TAG, "INVALID_SAMPLE_RATE %d, maximum rate is 1000", rate);
        rate = 1000;
    }

#if defined CONFIG_MPU6500
    fchoice_t fchoice = imu->getFchoice(imu);
    CHK_RES(imu->lastError(imu));
    if (fchoice != FCHOICE_3) {
        GB_DEBUGW(ERROR_TAG, "INVALID_STATE, sample rate divider is not effective when Fchoice != 3");
    }
#endif
    // Check dlpf configuration
    dlpf_t dlpf = imu->getDigitalLowPassFilter(imu);
    CHK_RES(imu->lastError(imu));
    if (dlpf == 0 || dlpf == 7)
        GB_DEBUGW(ERROR_TAG, "INVALID_STATE, sample rate divider is not effective when DLPF is (0 or 7)");

    const uint16_t internalSampleRate = 1000;
    uint16_t divider                      = internalSampleRate / rate - 1;
    // Check for rate match
    uint16_t finalRate = (internalSampleRate / (1 + divider));
    if (finalRate != rate) {
    }
    else {
    }
    // Write divider to register
    CHK_RES(imu->writeByte(imu, SMPLRT_DIV, (uint8_t) divider));

error_exit:
    return res;
}

/**
 * @brief Retrieve sample rate divider and calculate the actual rate.
 * TODO: ADD error handle
 */
uint16_t getSampleRate(struct imu *imu)
{
#if defined CONFIG_MPU6500
    fchoice_t fchoice = imu->getFchoice(imu);
    CHK_VAL(imu->lastError(imu));
    if (fchoice != FCHOICE_3) return SAMPLE_RATE_MAX;
#endif

    const uint16_t sampleRateMax_nolpf = 8000;
    dlpf_t dlpf = imu->getDigitalLowPassFilter(imu);
    CHK_VAL(imu->lastError(imu));
    if (dlpf == 0 || dlpf == 7) return sampleRateMax_nolpf;

    const uint16_t internalSampleRate = 1000;
    CHK_VAL(imu->readByte(imu, SMPLRT_DIV, imu->buffer));
    uint16_t rate = internalSampleRate / (1 + imu->buffer[0]);
    return rate;
}

/**
 * @brief Select clock source.
 * @note The gyro PLL is better than internal clock.
 * @param clockSrc clock source
 */
static GB_RESULT setClockSource(struct imu *imu, clock_src_t clockSrc)
{
    return imu->writeBits(imu, PWR_MGMT1, PWR1_CLKSEL_BIT, PWR1_CLKSEL_LENGTH, clockSrc);
}

/**
 * @brief Return clock source.
 */
clock_src_t getClockSource(struct imu *imu)
{
    imu->err = imu->readBits(imu, PWR_MGMT1, PWR1_CLKSEL_BIT, PWR1_CLKSEL_LENGTH, imu->buffer);
    return (clock_src_t) imu->buffer[0];
}

/**
 * @brief Configures Digital Low Pass Filter (DLPF) setting for both the gyroscope and accelerometer.
 * @param dlpf digital low-pass filter value
 */
static GB_RESULT setDigitalLowPassFilter(struct imu *imu, dlpf_t dlpf)
{
    GB_RESULT res = GB_OK;

    CHK_RES(imu->writeBits(imu, CONFIG, CONFIG_DLPF_CFG_BIT, CONFIG_DLPF_CFG_LENGTH, dlpf));
#if defined CONFIG_MPU6500
    CHK_RES(imu->writeBits(imu, ACCEL_CONFIG2, ACONFIG2_A_DLPF_CFG_BIT, ACONFIG2_A_DLPF_CFG_LENGTH, dlpf));
#endif
error_exit:
    return res;
}

/**
 * @brief Return Digital Low Pass Filter configuration
 */
dlpf_t getDigitalLowPassFilter(struct imu *imu)
{
    imu->err = imu->readBits(imu, CONFIG, CONFIG_DLPF_CFG_BIT, CONFIG_DLPF_CFG_LENGTH, imu->buffer);
    return (dlpf_t) imu->buffer[0];
}

/**
 * @brief Reset sensors signal path.
 *
 * Reset all gyro digital signal path, accel digital signal path, and temp
 * digital signal path. This also clears all the sensor registers.
 *
 * @note This function delays 100 ms, needed for reset to complete.
 * */
static GB_RESULT resetSignalPath(struct imu *imu)
{
    GB_RESULT res = GB_OK;

    CHK_RES(imu->writeBit(imu, USER_CTRL, USERCTRL_SIG_COND_RESET_BIT, 1));
    GB_SleepMs(100);
error_exit:
    return res;
}

/**
 * @brief Enter Low Power Accelerometer mode.
 *
 * In low-power accel mode, the chip goes to sleep and only wakes up to sample
 * the accelerometer at a certain frequency.
 * See setLowPowerAccelRate() to set the frequency.
 *
 * @param enable value
 *  + This function does the following to enable:
 *   - Set CYCLE bit to 1
 *   - Set SLEEP bit to 0
 *   - Set TEMP_DIS bit to 1
 *   - Set STBY_XG, STBY_YG, STBY_ZG bits to 1
 *   - Set STBY_XA, STBY_YA, STBY_ZA bits to 0
 *   - Set FCHOICE to 0 (ACCEL_FCHOICE_B bit to 1) [MPU6500 / MPU9250 only]
 *   - Disable Auxiliary I2C Master I/F
 *
 *  + This function does the following to disable:
 *   - Set CYCLE bit to 0
 *   - Set TEMP_DIS bit to 0
 *   - Set STBY_XG, STBY_YG, STBY_ZG bits to 0
 *   - Set STBY_XA, STBY_YA, STBY_ZA bits to 0
 *   - Set FCHOICE to 3 (ACCEL_FCHOICE_B bit to 0) [MPU6500 / MPU9250 only]
 *   - Enable Auxiliary I2C Master I/F
 * */
static GB_RESULT setLowPowerAccelMode(struct imu *imu, bool enable)
{
    GB_RESULT res = GB_OK;
// set FCHOICE
#if defined CONFIG_MPU6500
    fchoice_t fchoice = enable ? FCHOICE_0 : FCHOICE_3;
    CHK_RES(imu->setFchoice(imu, fchoice));
#endif
    // read PWR_MGMT1 and PWR_MGMT2 at once
    CHK_RES(imu->readBytes(imu, PWR_MGMT1, 2, imu->buffer));
    if (enable) {
        // set CYCLE bit to 1 and SLEEP bit to 0 and TEMP_DIS bit to 1
        imu->buffer[0] |= 1 << PWR1_CYCLE_BIT;
        imu->buffer[0] &= ~(1 << PWR1_SLEEP_BIT);
        imu->buffer[0] |= 1 << PWR1_TEMP_DIS_BIT;
        // set STBY_XG, STBY_YG, STBY_ZG bits to 1
        imu->buffer[1] |= PWR2_STBY_XYZG_BITS;
    }
    else {  // disable
        // set CYCLE bit to 0 and TEMP_DIS bit to 0
        imu->buffer[0] &= ~(1 << PWR1_CYCLE_BIT);
        imu->buffer[0] &= ~(1 << PWR1_TEMP_DIS_BIT);
        // set STBY_XG, STBY_YG, STBY_ZG bits to 0
        imu->buffer[1] &= ~(PWR2_STBY_XYZG_BITS);
    }
    // set STBY_XA, STBY_YA, STBY_ZA bits to 0
    imu->buffer[1] &= ~(PWR2_STBY_XYZA_BITS);
    // write back PWR_MGMT1 and PWR_MGMT2 at once
    CHK_RES(imu->writeBytes(imu, PWR_MGMT1, 2, imu->buffer));
    // disable Auxiliary I2C Master I/F in case it was active
    CHK_RES(imu->setAuxI2CEnabled(imu, !enable));
error_exit:
    return res;
}

/**
 * @brief Return Low Power Accelerometer state.
 *
 * Condition to return true:
 *  - CYCLE bit is 1
 *  - SLEEP bit is 0
 *  - TEMP_DIS bit is 1
 *  - STBY_XG, STBY_YG, STBY_ZG bits are 1
 *  - STBY_XA, STBY_YA, STBY_ZA bits are 0
 *  - FCHOICE is 0 (ACCEL_FCHOICE_B bit is 1) [MPU6500 / MPU9250 only]
 *
 * */
bool getLowPowerAccelMode(struct imu *imu)
{
// check FCHOICE
#if defined CONFIG_MPU6500
    fchoice_t fchoice = imu->getFchoice(imu);
    CHK_VAL(imu->lastError(imu));
    if (fchoice != FCHOICE_0) {
        return false;
    }
#endif
    // read PWR_MGMT1 and PWR_MGMT2 at once
    CHK_VAL(imu->readBytes(imu, PWR_MGMT1, 2, imu->buffer));
    // define configuration bits
    const uint8_t LPACCEL_CONFIG_BITMASK[2] = {
        (1 << PWR1_SLEEP_BIT) | (1 << PWR1_CYCLE_BIT) | (1 << PWR1_TEMP_DIS_BIT),
        PWR2_STBY_XYZA_BITS | PWR2_STBY_XYZG_BITS};
    const uint8_t LPACCEL_ENABLED_VALUE[2] = {(1 << PWR1_CYCLE_BIT) | (1 << PWR1_TEMP_DIS_BIT),
                                                  PWR2_STBY_XYZG_BITS};
    // get just the configuration bits
    imu->buffer[0] &= LPACCEL_CONFIG_BITMASK[0];
    imu->buffer[1] &= LPACCEL_CONFIG_BITMASK[1];
    // check pattern
    return imu->buffer[0] == LPACCEL_ENABLED_VALUE[0] && imu->buffer[1] == LPACCEL_ENABLED_VALUE[1];
}

/**
 * @brief Set Low Power Accelerometer frequency of wake-up.
 * */
static GB_RESULT setLowPowerAccelRate(struct imu *imu, lp_accel_rate_t rate)
{
    GB_RESULT res = GB_OK;
#if defined CONFIG_MPU6050
    CHK_RES(imu->writeBits(imu, PWR_MGMT2, PWR2_LP_WAKE_CTRL_BIT, PWR2_LP_WAKE_CTRL_LENGTH, rate));
#elif defined CONFIG_MPU6500
    CHK_RES(imu->writeBits(imu, LP_ACCEL_ODR, LPA_ODR_CLKSEL_BIT, LPA_ODR_CLKSEL_LENGTH, rate));
#endif
error_exit:
    return res;
}

/**
 * @brief Get Low Power Accelerometer frequency of wake-up.
 */
lp_accel_rate_t getLowPowerAccelRate(struct imu *imu)
{
#if defined CONFIG_MPU6050
    CHK_VAL(imu->readBits(imu, PWR_MGMT2, PWR2_LP_WAKE_CTRL_BIT, PWR2_LP_WAKE_CTRL_LENGTH, imu->buffer));
#elif defined CONFIG_MPU6500
    CHK_VAL(imu->readBits(imu, LP_ACCEL_ODR, LPA_ODR_CLKSEL_BIT, LPA_ODR_CLKSEL_LENGTH, imu->buffer));
#endif
    return (lp_accel_rate_t) imu->buffer[0];
}

/**
 * @brief Enable/disable Motion modules (Motion detect, Zero-motion, Free-Fall).
 *
 * @attention
 *  The configurations must've already been set with setMotionDetectConfig() before
 *  enabling the module!
 * @note
 *  - Call getMotionDetectStatus() to find out which axis generated motion interrupt. [MPU6000, MPU6050, MPU9150].
 *  - It is recommended to set the Motion Interrupt to propagate to the INT pin. To do that, use setInterruptEnabled().
 * @param enable
 *  - On _true_, this function modifies the DLPF, put gyro and temperature in standby,
 *    and disable Auxiliary I2C Master I/F.
 *  - On _false_, this function sets DLPF to 42Hz and enables Auxiliary I2C master I/F.
 * */
static GB_RESULT setMotionFeatureEnabled(struct imu *imu, bool enable)
{
    GB_RESULT res = GB_OK;
#if defined CONFIG_MPU6050
    CHK_RES(imu->writeBits(imu, ACCEL_CONFIG, ACONFIG_HPF_BIT, ACONFIG_HPF_LENGTH, ACCEL_DHPF_RESET));
#endif
    /* enabling */
    if (enable) {
#if defined CONFIG_MPU6050
        const dlpf_t kDLPF = DLPF_256HZ_NOLPF;
#elif defined CONFIG_MPU6500
        const dlpf_t kDLPF = DLPF_188HZ;
#endif
        CHK_RES(imu->setDigitalLowPassFilter(imu, kDLPF));
#if defined CONFIG_MPU6050
        // give a time for accumulation of samples
        GB_SleepMs(10);
        CHK_RES(imu->writeBits(imu, ACCEL_CONFIG, ACONFIG_HPF_BIT, ACONFIG_HPF_LENGTH, ACCEL_DHPF_HOLD));
#elif defined CONFIG_MPU6500
        CHK_RES(imu->writeByte(imu, ACCEL_INTEL_CTRL, (1 << ACCEL_INTEL_EN_BIT) | (1 << ACCEL_INTEL_MODE_BIT)));
#endif
        /* disabling */
    }
    else {
#if defined CONFIG_MPU6500
        CHK_RES(imu->writeBits(imu, ACCEL_INTEL_CTRL, ACCEL_INTEL_EN_BIT, 2, 0x0));
#endif
        const dlpf_t kDLPF = DLPF_42HZ;
        CHK_RES(imu->setDigitalLowPassFilter(imu, kDLPF));
    }
    // disable Auxiliary I2C Master I/F in case it was active
    CHK_RES(imu->setAuxI2CEnabled(imu, !enable));
error_exit:
    return res;
}

/**
 * @brief Return true if a Motion Dectection module is enabled.
 */
bool getMotionFeatureEnabled(struct imu *imu)
{
    uint8_t data;
#if defined CONFIG_MPU6050
    CHK_VAL(imu->readBits(imu, ACCEL_CONFIG, ACONFIG_HPF_BIT, ACONFIG_HPF_LENGTH, &data));
    if (data != ACCEL_DHPF_HOLD) return false;
    const dlpf_t kDLPF = DLPF_256HZ_NOLPF;
#elif defined CONFIG_MPU6500
    CHK_VAL(imu->readByte(imu, ACCEL_INTEL_CTRL, &data));
    const uint8_t kAccelIntel = (1 << ACCEL_INTEL_EN_BIT) | (1 << ACCEL_INTEL_MODE_BIT);
    if ((data & kAccelIntel) != kAccelIntel) return false;
    const dlpf_t kDLPF = DLPF_188HZ;
#endif
    dlpf_t dlpf = imu->getDigitalLowPassFilter(imu);
    CHK_VAL(imu->lastError(imu));
    if (dlpf != kDLPF) return false;
    return true;
}

/**
 * @brief Configure Motion-Detect or Wake-on-motion feature.
 *
 * The behaviour of this feature is very different between the MPU6050 (MPU9150) and the
 * MPU6500 (MPU9250). Each chip's version of this feature is explained below.
 *
 * [MPU6050, MPU6000, MPU9150]:
 * Accelerometer measurements are passed through a configurable digital high pass filter (DHPF)
 * in order to eliminate bias due to gravity. A qualifying motion sample is one where the high
 * passed sample from any axis has an absolute value exceeding a user-programmable threshold. A
 * counter increments for each qualifying sample, and decrements for each non-qualifying sample.
 * Once the counter reaches a user-programmable counter threshold, a motion interrupt is triggered.
 * The axis and polarity which caused the interrupt to be triggered is flagged in the
 * MOT_DETECT_STATUS register.
 *
 * [MPU6500, MPU9250]:
 * Unlike the MPU6050 version, the hardware does not "lock in" a reference sample.
 * The hardware monitors the accel data and detects any large change over a short period of time.
 * A qualifying motion sample is one where the high passed sample from any axis has
 * an absolute value exceeding the threshold.
 * The hardware motion threshold can be between 4mg and 1020mg in 4mg increments.
 *
 * @note
 * It is possible to enable **wake-on-motion** mode by doing the following:
 *  1. Enter Low Power Accelerometer mode with setLowPowerAccelMode();
 *  2. Select the wake-up rate with setLowPowerAccelRate();
 *  3. Configure motion-detect interrupt with setMotionDetectConfig();
 *  4. Enable the motion detection module with setMotionFeatureEnabled();
 * */
static GB_RESULT setMotionDetectConfig(struct imu *imu, mot_config_t* config)
{
    GB_RESULT res = GB_OK;
#if defined CONFIG_MPU6050
    CHK_RES(imu->writeByte(imu, MOTION_DUR, config->time));
    CHK_RES(imu->writeBits(imu, MOTION_DETECT_CTRL, MOTCTRL_ACCEL_ON_DELAY_BIT,
                                MOTCTRL_ACCEL_ON_DELAY_LENGTH, config->accel_on_delay));
    CHK_RES(imu->writeBits(imu, MOTION_DETECT_CTRL, MOTCTRL_MOT_COUNT_BIT, MOTCTRL_MOT_COUNT_LENGTH,
                                config->counter));
#endif
    CHK_RES(imu->writeByte(imu, MOTION_THR, config->threshold));
error_exit:
    return res;
}

/**
 * @brief Return Motion Detection Configuration.
 */
mot_config_t getMotionDetectConfig(struct imu *imu)
{
    mot_config_t config;
#if defined CONFIG_MPU6050
    CHK_VAL(imu->readByte(imu, MOTION_DUR, &config.time));
    CHK_VAL(imu->readByte(imu, MOTION_DETECT_CTRL, imu->buffer));
    config.accel_on_delay =
        (imu->buffer[0] >> (MOTCTRL_ACCEL_ON_DELAY_BIT - MOTCTRL_ACCEL_ON_DELAY_LENGTH + 1)) & 0x3;
    config.counter =
        (mot_counter_t)((imu->buffer[0] >> (MOTCTRL_MOT_COUNT_BIT - MOTCTRL_MOT_COUNT_LENGTH + 1)) & 0x3);
#endif
    CHK_VAL(imu->readByte(imu, MOTION_THR, &config.threshold));
    return config;
}

#if defined CONFIG_MPU6050
/**
 * @brief Configure Zero-Motion.
 *
 * The Zero Motion detection capability uses the digital high pass filter (DHPF) and a similar
 * threshold scheme to that of Free Fall detection. Each axis of the high passed accelerometer
 * measurement must have an absolute value less than a threshold specified in the ZRMOT_THR
 * register, which can be increased in 1 mg increments. Each time a motion sample meets this
 * condition, a counter increments. When this counter reaches a threshold specified in ZRMOT_DUR, an
 * interrupt is generated.
 *
 * Unlike Free Fall or Motion detection, Zero Motion detection triggers an interrupt both when Zero
 * Motion is first detected and when Zero Motion is no longer detected. While Free Fall and Motion
 * are indicated with a flag which clears after being read, reading the state of the Zero Motion
 * detected from the MOT_DETECT_STATUS register does not clear its status.
 *
 * @note Enable by calling setMotionFeatureEnabled();
 * */
GB_RESULT setZeroMotionConfig(struct imu *imu, zrmot_config_t* config)
{
    GB_RESULT res = GB_OK;
    imu->buffer[0] = config->threshold;
    imu->buffer[1] = config->time;
    CHK_RES(imu->writeBytes(imu, ZRMOTION_THR, 2, imu->buffer));
error_exit:
    return res;
}

/**
 * @brief Return Zero-Motion configuration.
 */
zrmot_config_t getZeroMotionConfig(struct imu *imu)
{
    CHK_VAL(imu->readBytes(imu, ZRMOTION_THR, 2, imu->buffer));
    zrmot_config_t config;
    config.threshold = imu->buffer[0];
    config.time      = imu->buffer[1];
    return config;
}

/**
 * @brief Configure Free-Fall.
 *
 * Free fall is detected by checking if the accelerometer measurements from all 3 axes have an
 * absolute value below a user-programmable threshold (acceleration threshold). For each sample
 * where this condition is true (a qualifying sample), a counter is incremented. For each sample
 * where this condition is false (a non- qualifying sample), the counter is decremented. Once the
 * counter reaches a user-programmable threshold (the counter threshold), the Free Fall interrupt is
 * triggered and a flag is set. The flag is cleared once the counter has decremented to zero. The
 * counter does not increment above the counter threshold or decrement below zero.
 *
 * @note Enable by calling setMotionFeatureEnabled().
 * */
static GB_RESULT setFreeFallConfig(struct imu *imu, ff_config_t* config)
{
    GB_RESULT res = GB_OK;
    imu->buffer[0] = config->threshold;
    imu->buffer[1] = config->time;
    CHK_RES(imu->writeBytes(imu, FF_THR, 2, imu->buffer));
    CHK_RES(imu->writeBits(imu, MOTION_DETECT_CTRL, MOTCTRL_ACCEL_ON_DELAY_BIT,
                                MOTCTRL_ACCEL_ON_DELAY_LENGTH, config->accel_on_delay));
    CHK_RES(imu->writeBits(imu, MOTION_DETECT_CTRL, MOTCTRL_MOT_COUNT_BIT, MOTCTRL_MOT_COUNT_LENGTH,
                                config->counter));
error_exit:
    return res;
}

/**
 * @brief Return Free-Fall Configuration.
 */
ff_config_t getFreeFallConfig(struct imu *imu)
{
    ff_config_t config;
    CHK_VAL(imu->readBytes(imu, FF_THR, 2, imu->buffer));
    config.threshold = imu->buffer[0];
    config.time      = imu->buffer[1];
    CHK_VAL(imu->readByte(imu, MOTION_DETECT_CTRL, imu->buffer));
    config.accel_on_delay =
        (imu->buffer[0] >> (MOTCTRL_ACCEL_ON_DELAY_BIT - MOTCTRL_ACCEL_ON_DELAY_LENGTH + 1)) & 0x3;
    config.counter =
        (mot_counter_t)((imu->buffer[0] >> (MOTCTRL_MOT_COUNT_BIT - MOTCTRL_MOT_COUNT_LENGTH + 1)) & 0x3);
    return config;
}

/**
 * @brief Return Motion Detection Status.
 * @note Reading this register clears all motion detection status bits.
 * */
mot_stat_t getMotionDetectStatus(struct imu *imu)
{
    CHK_VAL(imu->readByte(imu, MOTION_DETECT_STATUS, imu->buffer));
    return (mot_stat_t) imu->buffer[0];
}
#endif  // MPU6050's stuff

/**
 * @brief Configure sensors' standby mode.
 * */
static GB_RESULT setStandbyMode(struct imu *imu, stby_en_t mask)
{
    GB_RESULT res = GB_OK;
    const uint8_t kPwr1StbyBits = mask >> 6;
    CHK_RES(imu->writeBits(imu, PWR_MGMT1, PWR1_GYRO_STANDBY_BIT, 2, kPwr1StbyBits));
    CHK_RES(imu->writeBits(imu, PWR_MGMT2, PWR2_STBY_XA_BIT, 6, mask));
error_exit:
    return res;
}

/**
 * @brief Return Standby configuration.
 * */
stby_en_t getStandbyMode(struct imu *imu)
{
    CHK_VAL(imu->readBytes(imu, PWR_MGMT1, 2, imu->buffer));
    const uint8_t kStbyTempAndGyroPLLBits = STBY_EN_TEMP | STBY_EN_LOWPWR_GYRO_PLL_ON;
    stby_en_t mask = imu->buffer[0] << 3 & kStbyTempAndGyroPLLBits;
    const uint8_t kStbyAccelAndGyroBits   = STBY_EN_ACCEL | STBY_EN_GYRO;
    mask |= imu->buffer[1] & kStbyAccelAndGyroBits;
    return mask;
}

#if defined CONFIG_MPU6500
/**
 * @brief Select FCHOICE.
 *
 * Dev note: FCHOICE is the inverted value of FCHOICE_B (e.g. FCHOICE=2b’00 is same as FCHOICE_B=2b’11).
 * Reset value is FCHOICE_3
 * */
static GB_RESULT setFchoice(struct imu *imu, fchoice_t fchoice)
{
    GB_RESULT res = GB_OK;
    imu->buffer[0] = (~(fchoice) & 0x3);  // invert to fchoice_b
    CHK_RES(imu->writeBits(imu, GYRO_CONFIG, GCONFIG_FCHOICE_B, GCONFIG_FCHOICE_B_LENGTH, imu->buffer[0]));
    CHK_RES(imu->writeBit(imu, ACCEL_CONFIG2, ACONFIG2_ACCEL_FCHOICE_B_BIT, (imu->buffer[0] == 0) ? 0 : 1));
error_exit:
    return res;
}

/**
 * @brief Return FCHOICE.
 */
fchoice_t getFchoice(struct imu *imu)
{
    imu->err = imu->readBits(imu, GYRO_CONFIG, GCONFIG_FCHOICE_B, GCONFIG_FCHOICE_B_LENGTH, imu->buffer);
    return (fchoice_t)(~(imu->buffer[0]) & 0x3);
}
#endif

/**
 * @brief Select Gyroscope Full-scale range.
 * */
static GB_RESULT setGyroFullScale(struct imu *imu, gyro_fs_t fsr)
{
    return imu->writeBits(imu, GYRO_CONFIG, GCONFIG_FS_SEL_BIT, GCONFIG_FS_SEL_LENGTH, fsr);
}

/**
 * @brief Return Gyroscope Full-scale range.
 */
gyro_fs_t getGyroFullScale(struct imu *imu)
{
    imu->err = imu->readBits(imu, GYRO_CONFIG, GCONFIG_FS_SEL_BIT, GCONFIG_FS_SEL_LENGTH, imu->buffer);
    return (gyro_fs_t) imu->buffer[0];
}

/**
 * @brief Select Accelerometer Full-scale range.
 * */
static GB_RESULT setAccelFullScale(struct imu *imu, accel_fs_t fsr)
{
    return imu->writeBits(imu, ACCEL_CONFIG, ACONFIG_FS_SEL_BIT, ACONFIG_FS_SEL_LENGTH, fsr);
}

/**
 * @brief Return Accelerometer Full-scale range.
 */
accel_fs_t getAccelFullScale(struct imu *imu)
{
    imu->err = imu->readBits(imu, ACCEL_CONFIG, ACONFIG_FS_SEL_BIT, ACONFIG_FS_SEL_LENGTH, imu->buffer);
    return (accel_fs_t) imu->buffer[0];
}

/**
 * @brief Push biases to the gyro offset registers.
 *
 * This function expects biases relative to the current sensor output, and
 * these biases will be added to the factory-supplied values.
 *
 * Note: Bias inputs are LSB in +-1000dps format.
 * */
static GB_RESULT setGyroOffset(struct imu *imu, raw_axes_t bias)
{
    GB_RESULT res = GB_OK;
    imu->buffer[0] = (uint8_t)(bias.data.x >> 8);
    imu->buffer[1] = (uint8_t)(bias.data.x);
    imu->buffer[2] = (uint8_t)(bias.data.y >> 8);
    imu->buffer[3] = (uint8_t)(bias.data.y);
    imu->buffer[4] = (uint8_t)(bias.data.z >> 8);
    imu->buffer[5] = (uint8_t)(bias.data.z);
    CHK_RES(imu->writeBytes(imu, XG_OFFSET_H, 6, imu->buffer));
error_exit:
    return res;
}

/**
 * @brief Return biases from the gyro offset registers.
 *
 * Note: Bias output are LSB in +-1000dps format.
 * */
raw_axes_t getGyroOffset(struct imu *imu)
{
    CHK_VAL(imu->readBytes(imu, XG_OFFSET_H, 6, imu->buffer));
    raw_axes_t bias;
    bias.data.x = (imu->buffer[0] << 8) | imu->buffer[1];
    bias.data.y = (imu->buffer[2] << 8) | imu->buffer[3];
    bias.data.z = (imu->buffer[4] << 8) | imu->buffer[5];
    return bias;
}

/**
 * @brief Push biases to the accel offset registers.
 *
 * This function expects biases relative to the current sensor output, and
 * these biases will be added to the factory-supplied values.
 *
 * Note: Bias inputs are LSB in +-16G format.
 * */
static GB_RESULT setAccelOffset(struct imu *imu, raw_axes_t bias)
{
    raw_axes_t facBias;
    GB_RESULT res = GB_OK;
    // first, read OTP values of Accel factory trim

#if defined CONFIG_MPU6050
    CHK_RES(imu->readBytes(imu, XA_OFFSET_H, 6, imu->buffer));
    facBias.data.x = (imu->buffer[0] << 8) | imu->buffer[1];
    facBias.data.y = (imu->buffer[2] << 8) | imu->buffer[3];
    facBias.data.z = (imu->buffer[4] << 8) | imu->buffer[5];

#elif defined CONFIG_MPU6500
    CHK_RES(imu->readBytes(imu, XA_OFFSET_H, 8, imu->buffer));
    // note: imu->buffer[2] and imu->buffer[5], stay the same,
    //  they are read just to keep the burst reading
    facBias.data.x = (imu->buffer[0] << 8) | imu->buffer[1];
    facBias.data.y = (imu->buffer[3] << 8) | imu->buffer[4];
    facBias.data.z = (imu->buffer[6] << 8) | imu->buffer[7];
#endif

    // note: preserve bit 0 of factory value (for temperature compensation)
    facBias.data.x += (bias.data.x & ~1);
    facBias.data.y += (bias.data.y & ~1);
    facBias.data.z += (bias.data.z & ~1);

#if defined CONFIG_MPU6050
    imu->buffer[0] = (uint8_t)(facBias.data.x >> 8);
    imu->buffer[1] = (uint8_t)(facBias.data.x);
    imu->buffer[2] = (uint8_t)(facBias.data.y >> 8);
    imu->buffer[3] = (uint8_t)(facBias.data.y);
    imu->buffer[4] = (uint8_t)(facBias.data.z >> 8);
    imu->buffer[5] = (uint8_t)(facBias.data.z);
    CHK_RES(imu->writeBytes(imu, XA_OFFSET_H, 6, imu->buffer));

#elif defined CONFIG_MPU6500
    imu->buffer[0] = (uint8_t)(facBias.data.x >> 8);
    imu->buffer[1] = (uint8_t)(facBias.data.x);
    imu->buffer[3] = (uint8_t)(facBias.data.y >> 8);
    imu->buffer[4] = (uint8_t)(facBias.data.y);
    imu->buffer[6] = (uint8_t)(facBias.data.z >> 8);
    imu->buffer[7] = (uint8_t)(facBias.data.z);
    CHK_RES(imu->writeBytes(imu, XA_OFFSET_H, 8, imu->buffer));
#endif
error_exit:
    return res;
}

/**
 * @brief Return biases from accel offset registers.
 * This returns the biases with OTP values from factory trim added,
 * so returned values will be different than that ones set with setAccelOffset().
 *
 * Note: Bias output are LSB in +-16G format.
 * */
raw_axes_t getAccelOffset(struct imu *imu)
{
    raw_axes_t bias;

#if defined CONFIG_MPU6050
    CHK_VAL(imu->readBytes(imu, XA_OFFSET_H, 6, imu->buffer));
    bias.data.x = (imu->buffer[0] << 8) | imu->buffer[1];
    bias.data.y = (imu->buffer[2] << 8) | imu->buffer[3];
    bias.data.z = (imu->buffer[4] << 8) | imu->buffer[5];

#elif defined CONFIG_MPU6500
    CHK_VAL(imu->readBytes(imu, XA_OFFSET_H, 8, imu->buffer));
    bias.data.x                        = (imu->buffer[0] << 8) | imu->buffer[1];
    bias.data.y                        = (imu->buffer[3] << 8) | imu->buffer[4];
    bias.data.z                        = (imu->buffer[6] << 8) | imu->buffer[7];
#endif

    return bias;
}

/**
 * @brief Compute Accelerometer and Gyroscope offsets.
 *
 * This takes about ~400ms to compute offsets.
 * When calculating the offsets the imu must remain as horizontal as possible (0 degrees), facing
 * up. It is better to call computeOffsets() before any configuration is done (better right after
 * initialize()).
 *
 * Note: Gyro offset output are LSB in 1000DPS format.
 * Note: Accel offset output are LSB in 16G format.
 * */
static GB_RESULT computeOffsets(struct imu *imu, raw_axes_t* accel, raw_axes_t* gyro)
{
    GB_RESULT res = GB_OK;
    const accel_fs_t kAccelFS = ACCEL_FS_2G;     // most sensitive
    const gyro_fs_t kGyroFS   = GYRO_FS_250DPS;  // most sensitive
    CHK_RES(imu->getBiases(imu, kAccelFS, kGyroFS, accel, gyro, false));
    // convert offsets to 16G and 1000DPS format and invert values
    for (int i = 0; i < 3; i++) {
        //acel bias / 8 (16 / 2)
        (*accel).xyz[i] = -((*accel).xyz[i] >> (ACCEL_FS_16G - kAccelFS));
        //gyro bias / 4 (1000 / 250)
        (*gyro).xyz[i]  = -((*gyro).xyz[i] >> (GYRO_FS_1000DPS - kGyroFS));
    }
error_exit:
    return res;
}

/**
 * @brief Read accelerometer raw data.
 * */
static GB_RESULT acceleration(struct imu *imu, raw_axes_t* accel)
{
    GB_RESULT res = GB_OK;
    CHK_RES(imu->readBytes(imu, ACCEL_XOUT_H, 6, imu->buffer));
    accel->data.x = imu->buffer[0] << 8 | imu->buffer[1];
    accel->data.y = imu->buffer[2] << 8 | imu->buffer[3];
    accel->data.z = imu->buffer[4] << 8 | imu->buffer[5];
error_exit:
    return res;
}

/**
 * @brief Read gyroscope raw data.
 * */
static GB_RESULT rotation(struct imu *imu, raw_axes_t* gyro)
{
    GB_RESULT res = GB_OK;
    CHK_RES(imu->readBytes(imu, GYRO_XOUT_H, 6, imu->buffer));
    gyro->data.x = imu->buffer[0] << 8 | imu->buffer[1];
    gyro->data.y = imu->buffer[2] << 8 | imu->buffer[3];
    gyro->data.z = imu->buffer[4] << 8 | imu->buffer[5];
error_exit:
    return res;
}

/**
 * Read temperature raw data.
 * */
static GB_RESULT temperature(struct imu *imu, int16_t* temp)
{
    GB_RESULT res = GB_OK;
    CHK_RES(imu->readBytes(imu, TEMP_OUT_H, 2, imu->buffer));
    *temp = imu->buffer[0] << 8 | imu->buffer[1];
error_exit:
    return res;
}

/**
 * @brief Read accelerometer and gyroscope data at once.
 * */
static GB_RESULT motion(struct imu *imu, raw_axes_t* accel, raw_axes_t* gyro)
{
    GB_RESULT res = GB_OK;
    CHK_RES(imu->readBytes(imu, ACCEL_XOUT_H, 14, imu->buffer));
    accel->data.x = imu->buffer[0] << 8 | imu->buffer[1];
    accel->data.y = imu->buffer[2] << 8 | imu->buffer[3];
    accel->data.z = imu->buffer[4] << 8 | imu->buffer[5];
    gyro->data.x  = imu->buffer[8] << 8 | imu->buffer[9];
    gyro->data.y  = imu->buffer[10] << 8 | imu->buffer[11];
    gyro->data.z  = imu->buffer[12] << 8 | imu->buffer[13];
error_exit:
    return res;
}

/**
 * @brief Read data from all internal sensors.
 * */
static GB_RESULT sensors(struct imu *imu, raw_axes_t* accel, raw_axes_t* gyro, int16_t* temp)
{
    GB_RESULT res = GB_OK;
    CHK_RES(imu->readBytes(imu, ACCEL_XOUT_H, 14, imu->buffer));
    accel->data.x = imu->buffer[0] << 8 | imu->buffer[1];
    accel->data.y = imu->buffer[2] << 8 | imu->buffer[3];
    accel->data.z = imu->buffer[4] << 8 | imu->buffer[5];
    *temp    = imu->buffer[6] << 8 | imu->buffer[7];
    gyro->data.x  = imu->buffer[8] << 8 | imu->buffer[9];
    gyro->data.y  = imu->buffer[10] << 8 | imu->buffer[11];
    gyro->data.z  = imu->buffer[12] << 8 | imu->buffer[13];
error_exit:
    return res;
}

/**
 * @brief Read data from all sensors, including external sensors in Aux I2C.
 * */
static GB_RESULT sensors_sen(struct imu *imu, sensors_t* sensors, size_t extsens_len)
{
    GB_RESULT res = GB_OK;
    const size_t kIntSensLenMax = 14;  // internal sensors data length max
    const size_t kExtSensLenMax = 24;  // external sensors data length max
    uint8_t buffer[kIntSensLenMax + kExtSensLenMax];
#if defined AK89xx
    const size_t kMagLen = 8;  // magnetometer data length
    const size_t length      = kIntSensLenMax + extsens_len + kMagLen;
#else
    const size_t length           = kIntSensLenMax + extsens_len;
#endif
    CHK_RES(imu->readBytes(imu, ACCEL_XOUT_H, length, buffer));
    sensors->accel.data.x = buffer[0] << 8 | buffer[1];
    sensors->accel.data.y = buffer[2] << 8 | buffer[3];
    sensors->accel.data.z = buffer[4] << 8 | buffer[5];
    sensors->temp    = buffer[6] << 8 | buffer[7];
    sensors->gyro.data.x  = buffer[8] << 8 | buffer[9];
    sensors->gyro.data.y  = buffer[10] << 8 | buffer[11];
    sensors->gyro.data.z  = buffer[12] << 8 | buffer[13];
#if defined AK89xx
    sensors->mag.data.x = buffer[16] << 8 | buffer[15];
    sensors->mag.data.y = buffer[18] << 8 | buffer[17];
    sensors->mag.data.z = buffer[20] << 8 | buffer[19];
#endif
    memcpy(sensors->extsens, buffer + (length - extsens_len), extsens_len);
error_exit:
    return res;
}

#if defined CONFIG_MPU9150 || (defined CONFIG_MPU6050 && !defined CONFIG_MPU6000)
/**
 * @brief The imu-6050’s I/O logic levels are set to be either VDD or VLOGIC.
 *
 * VLOGIC may be set to be equal to VDD or to another voltage. However, VLOGIC must be ≤ VDD at all
 * times. When AUX_VDDIO is set to 0 (its power-on-reset value), VLOGIC is the power supply voltage
 * for both the microprocessor system bus and the auxiliary I C bus. When AUX_VDDIO is set to 1,
 * VLOGIC is the power supply voltage for the microprocessor system bus and VDD is the supply for
 * the auxiliary I2C bus
 * */
static GB_RESULT setAuxVDDIOLevel(struct imu *imu, auxvddio_lvl_t level)
{
    return imu->writeBit(imu, YG_OTP_OFFSET_TC, TC_PWR_MODE_BIT, level);
}

/**
 * Return imu-6050’s I/O logic levels.
 */
auxvddio_lvl_t getAuxVDDIOLevel(struct imu *imu)
{
    imu->err = imu->readBit(imu, YG_OTP_OFFSET_TC, TC_PWR_MODE_BIT, imu->buffer);
    return (auxvddio_lvl_t) imu->buffer[0];
}
#endif

/**
 * @brief Configure the Interrupt pin (INT).
 * @param config configuration desired.
 */
static GB_RESULT setInterruptConfig(struct imu *imu, int_config_t config)
{
    GB_RESULT res = GB_OK;
    CHK_RES(imu->readByte(imu, INT_PIN_CONFIG, imu->buffer));
    // zero the bits we're setting, but keep the others we're not setting as they are;
    const uint8_t INT_PIN_CONFIG_BITMASK = (1 << INT_CFG_LEVEL_BIT) | (1 << INT_CFG_OPEN_BIT) |
                                               (1 << INT_CFG_LATCH_EN_BIT) |
                                               (1 << INT_CFG_ANYRD_2CLEAR_BIT);
    imu->buffer[0] &= ~INT_PIN_CONFIG_BITMASK;
    // set the configurations
    imu->buffer[0] |= config.level << INT_CFG_LEVEL_BIT;
    imu->buffer[0] |= config.drive << INT_CFG_OPEN_BIT;
    imu->buffer[0] |= config.mode << INT_CFG_LATCH_EN_BIT;
    imu->buffer[0] |= config.clear << INT_CFG_ANYRD_2CLEAR_BIT;
    CHK_RES(imu->writeByte(imu, INT_PIN_CONFIG, imu->buffer[0]));
error_exit:
    return res;
}

/**
 * @brief Return Interrupt pin (INT) configuration.
 */
int_config_t getInterruptConfig(struct imu *imu)
{
    CHK_VAL(imu->readByte(imu, INT_PIN_CONFIG, imu->buffer));
    int_config_t config;
    config.level = (int_lvl_t)((imu->buffer[0] >> INT_CFG_LEVEL_BIT) & 0x1);
    config.drive = (int_drive_t)((imu->buffer[0] >> INT_CFG_OPEN_BIT) & 0x1);
    config.mode  = (int_mode_t)((imu->buffer[0] >> INT_CFG_LATCH_EN_BIT) & 0x1);
    config.clear = (int_clear_t)((imu->buffer[0] >> INT_CFG_ANYRD_2CLEAR_BIT) & 0x1);
    return config;
}

/**
 * @brief Enable features to generate signal at Interrupt pin
 * @param mask ORed features.
 */
static GB_RESULT setInterruptEnabled(struct imu *imu, int_en_t mask)
{
    return imu->writeByte(imu, INT_ENABLE, mask);
}

/**
 * @brief Return enabled features configured to generate signal at Interrupt pin.
 */
int_en_t getInterruptEnabled(struct imu *imu)
{
    imu->err = imu->readByte(imu, INT_ENABLE, imu->buffer);
    return (int_en_t) imu->buffer[0];
}

/**
 * @brief Return the Interrupt status from INT_STATUS register.
 *
 * Note: Reading this register, clear all bits.
 */
int_stat_t getInterruptStatus(struct imu *imu)
{
    imu->err = imu->readByte(imu, INT_STATUS, imu->buffer);
    return (int_stat_t) imu->buffer[0];
}

/**
 * @brief Change FIFO mode.
 *
 * Options:
 * `FIFO_MODE_OVERWRITE`: When the fifo is full, additional writes will be
 *  written to the fifo,replacing the oldest data.
 * `FIFO_MODE_STOP_FULL`: When the fifo is full, additional writes will not be written to fifo.
 * */
static GB_RESULT setFIFOMode(struct imu *imu, fifo_mode_t mode)
{
    return imu->writeBit(imu, CONFIG, CONFIG_FIFO_MODE_BIT, mode);
}

/**
 * @brief Return FIFO mode.
 */
fifo_mode_t getFIFOMode(struct imu *imu)
{
    imu->err = imu->readBit(imu, CONFIG, CONFIG_FIFO_MODE_BIT, imu->buffer);
    return (fifo_mode_t) imu->buffer[0];
}

/**
 * @brief Configure the sensors that will be written to the FIFO.
 * */
static GB_RESULT setFIFOConfig(struct imu *imu, fifo_config_t config)
{
    GB_RESULT res = GB_OK;
    CHK_RES(imu->writeByte(imu, FIFO_EN, (uint8_t) config));
    CHK_RES(imu->writeBit(imu, I2C_MST_CTRL, I2CMST_CTRL_SLV_3_FIFO_EN_BIT, config >> 8));
error_exit:
    return res;
}

/**
 * @brief Return FIFO configuration.
 */
fifo_config_t getFIFOConfig(struct imu *imu)
{
    CHK_VAL(imu->readBytes(imu, FIFO_EN, 2, imu->buffer));
    fifo_config_t config = imu->buffer[0];
    config |= (imu->buffer[1] & (1 << I2CMST_CTRL_SLV_3_FIFO_EN_BIT)) << 3;
    return config;
}

/**
 * @brief Enabled / disable FIFO module.
 * */
static GB_RESULT setFIFOEnabled(struct imu *imu, bool enable)
{
    return imu->writeBit(imu, USER_CTRL, USERCTRL_FIFO_EN_BIT, (uint8_t) enable);
}

/**
 * @brief Return FIFO module state.
 */
bool getFIFOEnabled(struct imu *imu)
{
    imu->err = imu->readBit(imu, USER_CTRL, USERCTRL_FIFO_EN_BIT, imu->buffer);
    return imu->buffer[0];
}

/**
 * @brief Reset FIFO module.
 *
 * Zero FIFO count, reset is asynchronous. \n
 * The bit auto clears after one clock cycle.
 * */
static GB_RESULT resetFIFO(struct imu *imu)
{
    return imu->writeBit(imu, USER_CTRL, USERCTRL_FIFO_RESET_BIT, 1);
}

/**
 * @brief Return number of written bytes in the FIFO.
 * @note FIFO overflow generates an interrupt which can be check with getInterruptStatus().
 * */
uint16_t getFIFOCount(struct imu *imu)
{
    CHK_VAL(imu->readBytes(imu, FIFO_COUNT_H, 2, imu->buffer));
    uint16_t count = imu->buffer[0] << 8 | imu->buffer[1];
    return count;
}

/**
 * @brief Read data contained in FIFO imu->buffer.
 * */
static GB_RESULT readFIFO(struct imu *imu, size_t length, uint8_t* data)
{
    return imu->readBytes(imu, FIFO_R_W, length, data);
}

/**
 * @brief Write data to FIFO imu->buffer.
 * */
static GB_RESULT writeFIFO(struct imu *imu, size_t length, const uint8_t* data)
{
    return imu->writeBytes(imu, FIFO_R_W, length, data);
}

/**
 * @brief Configure the Auxiliary I2C Master.
 * @note For [MPU9150, MPU9250]: The Auxiliary I2C is configured in the initialization stage
 *  to connect with the compass in Slave 0 and Slave 1.
 * */
static GB_RESULT setAuxI2CConfig(struct imu *imu, const auxi2c_config_t* config)
{
    GB_RESULT res = GB_OK;
    // TODO: check compass enabled, to constrain sample_delay which defines the compass read sample
    // rate
    CHK_RES(imu->readBit(imu, I2C_MST_CTRL, I2CMST_CTRL_SLV_3_FIFO_EN_BIT, imu->buffer));
    imu->buffer[0] <<= I2CMST_CTRL_SLV_3_FIFO_EN_BIT;
    imu->buffer[0] |= config->multi_master_en << I2CMST_CTRL_MULT_EN_BIT;
    imu->buffer[0] |= config->wait_for_es << I2CMST_CTRL_WAIT_FOR_ES_BIT;
    imu->buffer[0] |= config->transition << I2CMST_CTRL_P_NSR_BIT;
    imu->buffer[0] |= config->clock;
    CHK_RES(imu->writeByte(imu, I2C_MST_CTRL, imu->buffer[0]));
    CHK_RES(imu->writeBits(imu, I2C_SLV4_CTRL, I2C_SLV4_MST_DELAY_BIT, I2C_SLV4_MST_DELAY_LENGTH,
                                config->sample_delay));
    CHK_RES(imu->writeBit(imu, I2C_MST_DELAY_CRTL, I2CMST_DLY_ES_SHADOW_BIT, config->shadow_delay_en));
    /*
    GB_DEBUGE(ERROR_TAG, "EMPTY, Master:: multi_master_en: %d, wait_for_es: %d,"
                "transition: %d, clock: %d, sample_delay: %d, shadow_delay_en: %d\n",
                config->multi_master_en, config->wait_for_es, config->transition, config->clock, config->sample_delay,
                config->shadow_delay_en);
    */
error_exit:
    return res;
}

/**
 * @brief Get Auxiliary I2C Master configuration.
 */
auxi2c_config_t getAuxI2CConfig(struct imu *imu)
{
    CHK_VAL(imu->readByte(imu, I2C_MST_CTRL, imu->buffer));
    auxi2c_config_t config;
    config.multi_master_en = imu->buffer[0] >> I2CMST_CTRL_MULT_EN_BIT;
    config.wait_for_es     = (imu->buffer[0] >> I2CMST_CTRL_WAIT_FOR_ES_BIT) & 0x1;
    config.transition      = (auxi2c_trans_t)((imu->buffer[0] >> I2CMST_CTRL_P_NSR_BIT) & 0x1);
    config.clock           = (auxi2c_clock_t)(imu->buffer[0] & ((1 << I2CMST_CTRL_CLOCK_LENGTH) - 1));
    CHK_VAL(imu->readBits(imu, I2C_SLV4_CTRL, I2C_SLV4_MST_DELAY_BIT, I2C_SLV4_MST_DELAY_LENGTH, imu->buffer + 1));
    config.sample_delay = imu->buffer[1];
    CHK_VAL(imu->readBit(imu, I2C_MST_DELAY_CRTL, I2CMST_DLY_ES_SHADOW_BIT, imu->buffer + 2));
    config.shadow_delay_en = imu->buffer[2];
    return config;
}

/**
 * @brief Enable / disable Auxiliary I2C Master module.
 * */
static GB_RESULT setAuxI2CEnabled(struct imu *imu, bool enable)
{
    GB_RESULT res = GB_OK;
    CHK_RES(imu->writeBit(imu, USER_CTRL, USERCTRL_I2C_MST_EN_BIT, (uint8_t) enable));
    if (enable) {
        CHK_RES(imu->writeBit(imu, INT_PIN_CONFIG, INT_CFG_I2C_BYPASS_EN_BIT, 0));
    }
error_exit:
    return res;
}

/**
 * @brief Enable / disable Auxiliary I2C Master module.
 * */
static GB_RESULT setAuxI2CReset(struct imu *imu)
{
    return imu->writeBit(imu, USER_CTRL, USERCTRL_I2C_MST_RESET_BIT, 1);
}

/**
 * @brief Return Auxiliary I2C Master state.
 */
bool getAuxI2CEnabled(struct imu *imu)
{
    CHK_VAL(imu->readBit(imu, USER_CTRL, USERCTRL_I2C_MST_EN_BIT, imu->buffer));
    CHK_VAL(imu->readBit(imu, INT_PIN_CONFIG, INT_CFG_I2C_BYPASS_EN_BIT, imu->buffer + 1));
    return imu->buffer[0] && (!imu->buffer[1]);
}

/**
 * @brief Configure communication with a Slave connected to Auxiliary I2C bus.
 * */
static GB_RESULT setAuxI2CSlaveConfig(struct imu *imu, const auxi2c_slv_config_t* config)
{
    GB_RESULT res = GB_OK;
    // slaves' config registers are grouped as 3 regs in a row
    const uint8_t regAddr = config->slave * 3 + I2C_SLV0_ADDR;
    // data for I2C_SLVx_ADDR
    imu->buffer[0] = config->rw << I2C_SLV_RNW_BIT;
    imu->buffer[0] |= config->addr;
    // data for I2C_SLVx_REG
    imu->buffer[1] = config->reg_addr;
    // data for I2C_SLVx_CTRL
    CHK_RES(imu->readByte(imu, regAddr + 2, imu->buffer + 2));
    if (config->rw == AUXI2C_READ) {
        imu->buffer[2] &= 1 << I2C_SLV_EN_BIT;  // keep enable bit, clear the rest
        imu->buffer[2] |= config->reg_dis << I2C_SLV_REG_DIS_BIT;
        imu->buffer[2] |= config->swap_en << I2C_SLV_BYTE_SW_BIT;
        imu->buffer[2] |= config->end_of_word << I2C_SLV_GRP_BIT;
        imu->buffer[2] |= config->rxlength & 0xF;
    }
    else {                                                     // AUXI2C_WRITE
        imu->buffer[2] &= ~(1 << I2C_SLV_REG_DIS_BIT | 0xF);  // clear length bits and register disable bit
        imu->buffer[2] |= config->reg_dis << I2C_SLV_REG_DIS_BIT;
        imu->buffer[2] |= 0x1;  // set length to write 1 byte
        CHK_RES(imu->writeByte(imu, I2C_SLV0_DO + config->slave, config->txdata));
    }
    CHK_RES(imu->writeBytes(imu, regAddr, 3, imu->buffer));
    // sample_delay enable/disable
    CHK_RES(imu->writeBit(imu, I2C_MST_DELAY_CRTL, config->slave, config->sample_delay_en));
    /*
    GBDEBUGE(ERROR_TAG, "EMPTY, Slave%d:: r/w: %s, addr: 0x%X, reg_addr: 0x%X, reg_dis: %d, %s: 0x%X, sample_delay_en: %d\n",
        config->slave, (config->rw == AUXI2C_READ ? "read" : "write"), config->addr, config->reg_addr, config->reg_dis,
        (config->rw == AUXI2C_READ ? "rxlength" : "txdata"), config->txdata, config->sample_delay_en);
    */
error_exit:
    return res;
}

/**
 * @brief Return configuration of a Aux I2C Slave.
 * @param slave slave number.
 */
auxi2c_slv_config_t getAuxI2CSlaveConfig(struct imu *imu, auxi2c_slv_t slave)
{
    auxi2c_slv_config_t config;
    const uint8_t regAddr = slave * 3 + I2C_SLV0_ADDR;
    config.slave          = slave;
    CHK_VAL(imu->readBytes(imu, regAddr, 3, imu->buffer));
    config.rw       = (auxi2c_rw_t)((imu->buffer[0] >> I2C_SLV_RNW_BIT) & 0x1);
    config.addr     = imu->buffer[0] & 0x7F;
    config.reg_addr = imu->buffer[1];
    config.reg_dis  = (imu->buffer[2] >> I2C_SLV_REG_DIS_BIT) & 0x1;
    if (config.rw == AUXI2C_READ) {
        config.swap_en     = (imu->buffer[2] >> I2C_SLV_BYTE_SW_BIT) & 0x1;
        config.end_of_word = (auxi2c_eow_t)((imu->buffer[2] >> I2C_SLV_GRP_BIT) & 0x1);
        config.rxlength    = imu->buffer[2] & 0xF;
    }
    else {
        CHK_VAL(imu->readByte(imu, I2C_SLV0_DO + slave, imu->buffer + 3));
        config.txdata = imu->buffer[3];
    }
    CHK_VAL(imu->readByte(imu, I2C_MST_DELAY_CRTL, imu->buffer + 4));
    config.sample_delay_en = (imu->buffer[4] >> slave) & 0x1;
    return config;
}

/**
 * @brief Enable the Auxiliary I2C module to transfer data with a slave at sample rate.
 * */
static GB_RESULT setAuxI2CSlaveEnabled(struct imu *imu, auxi2c_slv_t slave, bool enable)
{
    const uint8_t regAddr = slave * 3 + I2C_SLV0_CTRL;
    return imu->writeBit(imu, regAddr, I2C_SLV_EN_BIT, enable);
}

/**
 * @brief Return enable state of a Aux I2C's Slave.
 */
bool getAuxI2CSlaveEnabled(struct imu *imu, auxi2c_slv_t slave)
{
    const uint8_t regAddr = slave * 3 + I2C_SLV0_CTRL;
    CHK_VAL(imu->readBit(imu, regAddr, I2C_SLV_EN_BIT, imu->buffer));
    return imu->buffer[0];
}

/**
 * @brief Enable / disable Auxiliary I2C bypass mode.
 * @param enable
 *  - `true`: Auxiliar I2C Master I/F is disabled, and Bypass enabled.
 *  - `false`: Bypass is disabled, but the Auxiliar I2C Master I/F is not enabled back,
 *             if needed, enabled it again with setAuxI2CmasterEnabled().
 * */
static GB_RESULT setAuxI2CBypass(struct imu *imu, bool enable)
{
    GB_RESULT res = GB_OK;
#ifdef CONFIG_MPU_SPI
    if (enable) {
        GB_DEBUGE(ERROR_TAG, "EMPTY, Setting Aux I2C to bypass mode while imu is connected via SPI");
    }
#endif
    if (enable) {
        CHK_RES(imu->writeBit(imu, USER_CTRL, USERCTRL_I2C_MST_EN_BIT, 0));
    }
    CHK_RES(imu->writeBit(imu, INT_PIN_CONFIG, INT_CFG_I2C_BYPASS_EN_BIT, enable));
error_exit:
    return res;
}

/**
 * @brief Return Auxiliary I2C Master bypass mode state.
 */
bool getAuxI2CBypass(struct imu *imu)
{
    CHK_VAL(imu->readBit(imu, USER_CTRL, USERCTRL_I2C_MST_EN_BIT, imu->buffer));
    CHK_VAL(imu->readBit(imu, INT_PIN_CONFIG, INT_CFG_I2C_BYPASS_EN_BIT, imu->buffer + 1));
    return (!imu->buffer[0]) && imu->buffer[1];
}

/**
 * @brief Read data from slaves connected to Auxiliar I2C bus.
 *
 * Data is placed in these external sensor data registers according to I2C_SLV0_CTRL,
 * I2C_SLV1_CTRL, I2C_SLV2_CTRL, and I2C_SLV3_CTRL (Registers 39, 42, 45, and 48). When
 * more than zero bytes are read (I2C_SLVx_LEN > 0) from an enabled slave (I2C_SLVx_EN = 1), the
 * slave is read at the Sample Rate (as defined in Register 25) or delayed rate (if specified in
 * Register 52 and 103). During each sample cycle, slave reads are performed in order of Slave
 * number. If all slaves are enabled with more than zero bytes to be read, the order will be Slave
 * 0, followed by Slave 1, Slave 2, and Slave 3.
 *
 * If the sum of the read lengths of all SLVx transactions exceed the number of available
 * EXT_SENS_DATA registers, the excess bytes will be dropped. There are 24 EXT_SENS_DATA
 * registers and hence the total read lengths between all the slaves cannot be greater than 24 or
 * some bytes will be lost.
 *
 * @attention Set `skip` to `8` when using compass, because compass data takes up the first `8` bytes.
 * */
static GB_RESULT readAuxI2CRxData(struct imu *imu, size_t length, uint8_t* data, size_t skip)
{
    GB_RESULT res = GB_OK;
    if (length + skip > 24) {
        GB_DEBUGE(ERROR_TAG, "INVALID_LENGTH,  %d, imu has only 24 external sensor data registers!", length);
        return imu->err = GB_MPU_AUX_RW_FAIL;
    }
// check if I2C Master is enabled, just for warning and debug
    const bool kAuxI2CEnabled = getAuxI2CEnabled(imu);
    CHK_RES(imu->lastError(imu));
    if (!kAuxI2CEnabled) GB_DEBUGE(ERROR_TAG, "AUX_I2C_DISABLED, , better turn on.\n");
    // read the specified amount of registers
    CHK_RES(imu->readBytes(imu, EXT_SENS_DATA_00 + skip, length, data));
error_exit:
    return res;
}

/**
 * @brief Restart Auxiliary I2C Master module, reset is asynchronous.
 *
 * This bit (I2C_MST_RST) should only be set when the I2C master has hung. If this bit
 * is set during an active I2C master transaction, the I2C slave will hang, which
 * will require the host to reset the slave.
 * */
static GB_RESULT restartAuxI2C(struct imu *imu)
{
    return imu->writeBit(imu, USER_CTRL, USERCTRL_I2C_MST_RESET_BIT, 1);
}

/**
 * @brief Return Auxiliary I2C Master status from register I2C_MST_STATUS.
 * Reading this register clear all its bits.
 * */
auxi2c_stat_t getAuxI2CStatus(struct imu *imu)
{
    imu->err = imu->readByte(imu, I2C_MST_STATUS, imu->buffer);
    return (auxi2c_stat_t) imu->buffer[0];
}

/**
 * @brief Write to a slave a single byte just once (use for configuring a slave at initialization).
 *
 * This function uses Slave 4 to perform single transfers to the slave device on Aux I2C. \n
 * The byte will be transfered at first sample take, so when sample rate is at minimum (4 Hz)
 * it may take up to a quarter of a second to start the transfer.
 * @attention Auxiliary I2C Master must have already been configured before calling this function.
 *
 * @return
 *  - `GB_MPU_AUX_NOT_ENABLE`: Auxiliary I2C Master not enabled;
 *  - `GB_MPU_AUX_NOT_FOUND`:  Slave doesn't ACK the transfer;
 *  - `GB_MPU_AUX_LOST_ARB`:   Auxiliary I2C Master lost arbitration of the bus;
 *  - or other standard I2C driver error codes.
 * */
static GB_RESULT auxI2CWriteByte(struct imu *imu, uint8_t devAddr, uint8_t regAddr, const uint8_t data)
{
    GB_RESULT res = GB_OK;
    // check for Aux I2C master enabled first
    const bool kAuxI2CEnabled = imu->getAuxI2CEnabled(imu);
    CHK_RES(imu->lastError(imu));
    if (!kAuxI2CEnabled) {
        GB_DEBUGE(ERROR_TAG, "AUX_I2C_DISABLED, , must enable first\n");
        return imu->err = GB_MPU_AUX_NOT_ENABLE;
    }
    // data for I2C_SLV4_ADDR
    imu->buffer[0] = AUXI2C_WRITE << I2C_SLV_RNW_BIT;
    imu->buffer[0] |= devAddr & (0x7F);
    // data for I2C_SLV4_REG
    imu->buffer[1] = regAddr;
    // data for I2C_SLV4_DO
    imu->buffer[2] = data;
    // write configuration above to slave 4 registers
    CHK_RES(imu->writeBytes(imu, I2C_SLV4_ADDR, 3, imu->buffer));
    // clear status register before enable this transfer
    CHK_RES(imu->readByte(imu, I2C_MST_STATUS, imu->buffer + 15));
    // enable transfer in slave 4
    CHK_RES(imu->writeBit(imu, I2C_SLV4_CTRL, I2C_SLV4_EN_BIT, 1));
    // check status until transfer is done
    GB_TickType startTick = 0, offsetTick = 0;
    CHK_RES(GB_GetTicks(&startTick));
    CHK_RES(GB_MsToTick(1000, &offsetTick));

    GB_TickType endTick  = startTick + offsetTick;
    auxi2c_stat_t status = 0x00;

    do {
        CHK_RES(imu->readByte(imu, I2C_MST_STATUS, &status));
        if (status & (1 << I2CMST_STAT_SLV4_NACK_BIT)) {
            GB_DEBUGE(ERROR_TAG, "AUX_I2C_SLAVE_NACK, %02x\n", (uint8_t)status);
            return imu->err = GB_MPU_AUX_NOT_FOUND;
        }
        if (status & (1 << I2CMST_STAT_LOST_ARB_BIT)) {
            GB_DEBUGE(ERROR_TAG, "AUX_I2C_LOST_ARB, ");
            return imu->err = GB_MPU_AUX_LOST_ARB;
        }
        if (startTick >= endTick) {
            GB_DEBUGE(ERROR_TAG, "TIMEOUT, . Aux I2C might've hung. Restart it.");
            return imu->err = GB_MPU_AUX_RW_TIMEOUT;
        }
    } while (!(status & (1 << I2C_SLV4_DONE_INT_BIT)));
error_exit:
    return res;
}

/**
 * @brief Read a single byte frome slave just once (use for configuring a slave at initialization).
 *
 * This function uses Slave 4 to perform single transfers to the slave device on Aux I2C. \n
 * The byte will be transfered at first sample take, so when sample rate is at minimum (4 Hz)
 * it may take up to a quarter of a second to start the transfer.
 * @attention Auxiliary I2C Master must have already been configured before calling this function.
 *
 * @return
 *  - GB_MPU_AUX_NOT_ENABLE  Auxiliary I2C Master not enabled;
 *  - GB_MPU_AUX_NOT_FOUND      Slave doesn't ACK the transfer;
 *  - GB_MPU_AUX_LOST_ARB               Auxiliary I2C Master lost arbitration of the bus;
 *  - or other standard I2C driver error codes.
 * */
static GB_RESULT auxI2CReadByte(struct imu *imu, uint8_t devAddr, uint8_t regAddr, uint8_t* data)
{
    GB_RESULT res = GB_OK;
    // check for Aux I2C master enabled first
    const bool kAuxI2CEnabled = imu->getAuxI2CEnabled(imu);
    CHK_RES(imu->lastError(imu));
    if (!kAuxI2CEnabled) {
        GB_DEBUGE(ERROR_TAG, "AUX_I2C_DISABLED, , must enable first\n");
        return imu->err = GB_MPU_AUX_NOT_ENABLE;
    }
    // data for I2C_SLV4_ADDR
    imu->buffer[0] = AUXI2C_READ << I2C_SLV_RNW_BIT;
    imu->buffer[0] |= devAddr & (0x7F);
    // data for I2C_SLV4_REG
    imu->buffer[1] = regAddr;
    // write configuration above to slave 4 registers
    CHK_RES(imu->writeBytes(imu, I2C_SLV4_ADDR, 2, imu->buffer));
    // clear status register before enable this transfer
    CHK_RES(imu->readByte(imu, I2C_MST_STATUS, imu->buffer + 15));
    // enable transfer in slave 4
    CHK_RES(imu->writeBit(imu, I2C_SLV4_CTRL, I2C_SLV4_EN_BIT, 1));
    // check status until transfer is done
    GB_TickType startTick = 0, offsetTick = 0;
    CHK_RES(GB_GetTicks(&startTick));
    CHK_RES(GB_MsToTick(1000, &offsetTick));

    GB_TickType endTick  = startTick + offsetTick;
    auxi2c_stat_t status = 0x00;

    do {
        CHK_RES(imu->readByte(imu, I2C_MST_STATUS, &status));
        if (status & (1 << I2CMST_STAT_SLV4_NACK_BIT)) {
            GB_DEBUGE(ERROR_TAG, "AUX_I2C_SLAVE_NACK, %02x\n", (uint8_t)status);
            return imu->err = GB_MPU_AUX_NOT_FOUND;
        }
        if (status & (1 << I2CMST_STAT_LOST_ARB_BIT)) {
            GB_DEBUGE(ERROR_TAG, "AUX_I2C_LOST_ARB, ");
            return imu->err = GB_MPU_AUX_LOST_ARB;
        }
        if (startTick >= endTick) {
            GB_DEBUGE(ERROR_TAG, "TIMEOUT, . Aux I2C might've hung. Restart it.");
            return imu->err = GB_MPU_AUX_RW_TIMEOUT;
        }
    } while (!(status & (1 << I2C_SLV4_DONE_INT_BIT)));
    // get read value
    CHK_RES(imu->readByte(imu, I2C_SLV4_DI, data));
error_exit:
    return res;
}

/**
 * @brief Configure the active level of FSYNC pin that will cause an interrupt.
 * @details Use setFsyncEnabled() to enable / disable this interrupt.
 * */
static GB_RESULT setFsyncConfig(struct imu *imu, int_lvl_t level)
{
    return imu->writeBit(imu, INT_PIN_CONFIG, INT_CFG_FSYNC_LEVEL_BIT, level);
}

/**
 * @brief Return FSYNC pin active level configuration.
 */
int_lvl_t getFsyncConfig(struct imu *imu)
{
    imu->err = imu->readBit(imu, INT_PIN_CONFIG, INT_CFG_FSYNC_LEVEL_BIT, imu->buffer);
    return (int_lvl_t) imu->buffer[0];
}

/**
 * @brief Enable / disable FSYNC pin to cause an interrupt.
 * @note
 * - The interrupt status is located in I2C_MST_STATUS register, so use
 *   the method getAuxI2CStatus() which reads this register to get FSYNC status.
 *   Keep in mind that a read from I2C_MST_STATUS register clears all its status bits,
 *   so take care to miss status bits when using Auxiliary I2C bus too.
 *
 * - It is possible to enable the FSYNC interrupt propagate to INT pin
 *   with setInterruptEnabled(), then the status can also be read with getInterruptStatus().
 *
 * @see setFsyncConfig().
 * */
static GB_RESULT setFsyncEnabled(struct imu *imu, bool enable)
{
    return imu->writeBit(imu, INT_PIN_CONFIG, INT_CFG_FSYNC_INT_MODE_EN_BIT, enable);
}

/**
 * @brief Return FSYNC enable state.
 */
bool getFsyncEnabled(struct imu *imu)
{
    imu->err = imu->readBit(imu, INT_PIN_CONFIG, INT_CFG_FSYNC_INT_MODE_EN_BIT, imu->buffer);
    return imu->buffer[0];
}

/**
 * @brief Print out register values for debugging purposes.
 * @param start first register number.
 * @param end last register number.
 */
static GB_RESULT registerDump(struct imu *imu, uint8_t start, uint8_t end)
{
    const uint8_t kNumOfRegs = 128;
    if (end - start < 0 || start >= kNumOfRegs || end >= kNumOfRegs) return GB_MPU_DUMP_REG_FAIL;
    GB_DEBUGE(ERROR_TAG, "LOG_COLOR_W >>  CONFIG_MPU_CHIP_MODEL  register dump: LOG_RESET_COLOR \n");
    uint8_t data;
    for (int i = start; i <= end; i++) {
        CHK_VAL(imu->readByte(imu, i, &data));
        GB_DEBUGE(ERROR_TAG, "imu: reg[ 0x%s%X ]  data( 0x%s%X )\n", i < 0x10 ? "0" : "", i, data < 0x10 ? "0" : "", data);
    }
    return GB_OK;
}

/**
 * @brief Read a single byte from aux device.
 *
 * How it's done: \n
 * It will check the communication protocol which the imu is connected by.
 *  - I2C, Auxiliary I2C bus will set to bypass mode and the reading will be performed directly (faster).
 *  - SPI, the function will use Slave 4 of Auxiliary I2C bus to read the byte (slower).
 * TODO: JUST support read multi bytes for I2C bypass.
 * */
static GB_RESULT compassReadBytes(struct imu *imu, uint8_t device_addr, uint8_t regAddr, uint8_t* data, uint32_t size)
{
    GB_RESULT res = GB_OK;
// in case of I2C
#if defined CONFIG_MPU_I2C
    const bool kPrevAuxI2CBypassState = imu->getAuxI2CBypass(imu);
    CHK_RES(imu->lastError(imu));
    if (kPrevAuxI2CBypassState == false) {
        CHK_RES(imu->setAuxI2CBypass(imu, true));
    }
    CHK_RES(imu->bus->readBytes(imu->bus, device_addr, regAddr, size, data));
    if (kPrevAuxI2CBypassState == false) {
        CHK_RES(imu->setAuxI2CBypass(imu, false));
    }
        // in case of SPI
#elif defined CONFIG_MPU_SPI
    CHK_RES(imu->auxI2CReadByte(imu, device_addr, regAddr, data));
#endif
error_exit:
    return res;
}

/**
 * @brief Write a single byte to aux device.
 *
 * How it's done: \n
 * It will check the communication protocol which the imu is connected by.
 *  - I2C, Auxiliary I2C bus will set to bypass mode and the reading will be performed directly (faster).
 *  - SPI, the function will use Slave 4 of Auxiliary I2C bus to read the byte (slower).
 * */
static GB_RESULT compassWriteByte(struct imu *imu, uint8_t device_addr, uint8_t regAddr, const uint8_t data)
{
    GB_RESULT res = GB_OK;
// in case of I2C
#if defined CONFIG_MPU_I2C
    const bool kPrevAuxI2CBypassState = imu->getAuxI2CBypass(imu);
    CHK_RES(imu->lastError(imu));
    if (kPrevAuxI2CBypassState == false) {
        CHK_RES(imu->setAuxI2CBypass(imu, true));
    }
    CHK_RES(imu->bus->writeByte(imu->bus, device_addr, regAddr, data));
    if (kPrevAuxI2CBypassState == false) {
        CHK_RES(imu->setAuxI2CBypass(imu, false));
    }
        // in case of SPI
#elif defined CONFIG_MPU_SPI
    CHK_RES(imu->auxI2CWriteByte(imu, device_addr, regAddr, data));
#endif
error_exit:
    return res;
}

static GB_RESULT mpu_read_calibration_data(struct imu *imu)
{
    GB_RESULT res = GB_OK;

    CHK_RES(imu->compassReadBytes(imu, BMP280_I2C_ADDRESS_1, 0x88, (uint8_t *)&imu->baro_dev.dig_T1, 2));
    CHK_RES(imu->compassReadBytes(imu, BMP280_I2C_ADDRESS_1, 0x8a, (uint8_t *)&imu->baro_dev.dig_T2, 2));
    CHK_RES(imu->compassReadBytes(imu, BMP280_I2C_ADDRESS_1, 0x8c, (uint8_t *)&imu->baro_dev.dig_T3, 2));
    CHK_RES(imu->compassReadBytes(imu, BMP280_I2C_ADDRESS_1, 0x8e, (uint8_t *)&imu->baro_dev.dig_P1, 2));
    CHK_RES(imu->compassReadBytes(imu, BMP280_I2C_ADDRESS_1, 0x90, (uint8_t *)&imu->baro_dev.dig_P2, 2));
    CHK_RES(imu->compassReadBytes(imu, BMP280_I2C_ADDRESS_1, 0x92, (uint8_t *)&imu->baro_dev.dig_P3, 2));
    CHK_RES(imu->compassReadBytes(imu, BMP280_I2C_ADDRESS_1, 0x94, (uint8_t *)&imu->baro_dev.dig_P4, 2));
    CHK_RES(imu->compassReadBytes(imu, BMP280_I2C_ADDRESS_1, 0x96, (uint8_t *)&imu->baro_dev.dig_P5, 2));
    CHK_RES(imu->compassReadBytes(imu, BMP280_I2C_ADDRESS_1, 0x98, (uint8_t *)&imu->baro_dev.dig_P6, 2));
    CHK_RES(imu->compassReadBytes(imu, BMP280_I2C_ADDRESS_1, 0x9a, (uint8_t *)&imu->baro_dev.dig_P7, 2));
    CHK_RES(imu->compassReadBytes(imu, BMP280_I2C_ADDRESS_1, 0x9c, (uint8_t *)&imu->baro_dev.dig_P8, 2));
    CHK_RES(imu->compassReadBytes(imu, BMP280_I2C_ADDRESS_1, 0x9e, (uint8_t *)&imu->baro_dev.dig_P9, 2));

    GB_DEBUGD(BMP_TAG, "Calibration data received:");
    GB_DEBUGD(BMP_TAG, "dig_T1=%d", imu->baro_dev.dig_T1);
    GB_DEBUGD(BMP_TAG, "dig_T2=%d", imu->baro_dev.dig_T2);
    GB_DEBUGD(BMP_TAG, "dig_T3=%d", imu->baro_dev.dig_T3);
    GB_DEBUGD(BMP_TAG, "dig_P1=%d", imu->baro_dev.dig_P1);
    GB_DEBUGD(BMP_TAG, "dig_P2=%d", imu->baro_dev.dig_P2);
    GB_DEBUGD(BMP_TAG, "dig_P3=%d", imu->baro_dev.dig_P3);
    GB_DEBUGD(BMP_TAG, "dig_P4=%d", imu->baro_dev.dig_P4);
    GB_DEBUGD(BMP_TAG, "dig_P5=%d", imu->baro_dev.dig_P5);
    GB_DEBUGD(BMP_TAG, "dig_P6=%d", imu->baro_dev.dig_P6);
    GB_DEBUGD(BMP_TAG, "dig_P7=%d", imu->baro_dev.dig_P7);
    GB_DEBUGD(BMP_TAG, "dig_P8=%d", imu->baro_dev.dig_P8);
    GB_DEBUGD(BMP_TAG, "dig_P9=%d", imu->baro_dev.dig_P9);
error_exit:
    return res;
}

static GB_RESULT mpu_read_hum_calibration_data(struct imu *imu)
{
    uint16_t h4, h5;
    GB_RESULT res = GB_OK;

    CHK_RES(imu->compassReadBytes(imu, BMP280_I2C_ADDRESS_1, 0xa1, &imu->baro_dev.dig_H1, 1));
    CHK_RES(imu->compassReadBytes(imu, BMP280_I2C_ADDRESS_1, 0xe1, (uint8_t *)&imu->baro_dev.dig_H2, 2));
    CHK_RES(imu->compassReadBytes(imu, BMP280_I2C_ADDRESS_1, 0xe3, &imu->baro_dev.dig_H3, 1));
    CHK_RES(imu->compassReadBytes(imu, BMP280_I2C_ADDRESS_1, 0xe4, (uint8_t *)&h4, 2));
    CHK_RES(imu->compassReadBytes(imu, BMP280_I2C_ADDRESS_1, 0xe5, (uint8_t *)&h5, 2));
    CHK_RES(imu->compassReadBytes(imu, BMP280_I2C_ADDRESS_1, 0xe7, (uint8_t *)&imu->baro_dev.dig_H6, 1));

    imu->baro_dev.dig_H4 = (h4 & 0x00ff) << 4 | (h4 & 0x0f00) >> 8;
    imu->baro_dev.dig_H5 = h5 >> 4;
    GB_DEBUGD(BMP_TAG, "Calibration data received:");
    GB_DEBUGD(BMP_TAG, "dig_H1=%d", imu->baro_dev.dig_H1);
    GB_DEBUGD(BMP_TAG, "dig_H2=%d", imu->baro_dev.dig_H2);
    GB_DEBUGD(BMP_TAG, "dig_H3=%d", imu->baro_dev.dig_H3);
    GB_DEBUGD(BMP_TAG, "dig_H4=%d", imu->baro_dev.dig_H4);
    GB_DEBUGD(BMP_TAG, "dig_H5=%d", imu->baro_dev.dig_H5);
    GB_DEBUGD(BMP_TAG, "dig_H6=%d", imu->baro_dev.dig_H6);
error_exit:
    return res;
}

static GB_RESULT baroInit(struct imu *imu)
{
    GB_RESULT res = GB_OK;
    bmp280_params_t bmp280_params;

    CHK_RES(bmp280_init_default_params(&bmp280_params));
    CHK_RES(imu->setAuxI2CReset(imu));
    // must delay, or compass may not be initialized
    GB_SleepMs(50);
    // I2C => bypass mode
#ifdef CONFIG_MPU_I2C
    CHK_RES(imu->setAuxI2CBypass(imu, true));
#elif CONFIG_MPU_SPI
    // SPI => master mode
    const auxi2c_config_t kAuxI2CConfig = {
        .clock           = AUXI2C_CLOCK_400KHZ,
        .multi_master_en = 1,
        .sample_delay    = 0,
        .shadow_delay_en = 0,
        .wait_for_es     = 0,
        .transition      = AUXI2C_TRANS_RESTART
    };
    CHK_RES(imu->setAuxI2CConfig(imu, &kAuxI2CConfig));
    CHK_RES(imu->setAuxI2CEnabled(imu, true));
#endif

    CHK_RES(imu->compassReadBytes(imu, BMP280_I2C_ADDRESS_1, BMP280_REG_ID, &imu->baro_dev.id, 1));

    if (imu->baro_dev.id != BMP280_CHIP_ID && imu->baro_dev.id != BME280_CHIP_ID)
    {
        CHK_LOGE(GB_BARO_DEV_ID_ERROR,
                "Invalid chip ID: expected: 0x%x (BME280) or 0x%x (BMP280) got: 0x%x",
                BME280_CHIP_ID, BMP280_CHIP_ID, imu->baro_dev.id);
    }
    // Soft reset.
    CHK_RES(imu->compassWriteByte(imu, BMP280_I2C_ADDRESS_1, BMP280_REG_RESET, BMP280_RESET_VALUE));
    // Wait until finished copying over the NVM data.
    while (1)
    {
        uint8_t status;
        if (!imu->compassReadBytes(imu, BMP280_I2C_ADDRESS_1, BMP280_REG_STATUS, &status, 1) && (status & 1) == 0)
            break;
    }
    CHK_RES(mpu_read_calibration_data(imu));
    if (imu->baro_dev.id == BME280_CHIP_ID)
    {
        CHK_RES(mpu_read_hum_calibration_data(imu));
    }

    uint8_t config = (bmp280_params.standby << 5) | (bmp280_params.filter << 2);

    GB_DEBUGD(BMP_TAG, "Writing config reg=%x", config);
    CHK_RES(imu->compassWriteByte(imu, BMP280_I2C_ADDRESS_1, BMP280_REG_CONFIG, config));

    if (bmp280_params.mode == BMP280_MODE_FORCED)
    {
        bmp280_params.mode = BMP280_MODE_SLEEP;  // initial mode for forced is sleep
    }

    uint8_t ctrl = (bmp280_params.oversampling_temperature << 5) | (bmp280_params.oversampling_pressure << 2) | (bmp280_params.mode);

    if (imu->baro_dev.id == BME280_CHIP_ID)
    {
        // Write crtl hum reg first, only active after write to BMP280_REG_CTRL.
        uint8_t ctrl_hum = bmp280_params.oversampling_humidity;
        GB_DEBUGD(BMP_TAG, "Writing ctrl hum reg=%x", ctrl_hum);
        CHK_RES(imu->compassWriteByte(imu, BMP280_I2C_ADDRESS_1, BMP280_REG_CTRL_HUM, ctrl_hum));
    }
    GB_DEBUGD(BMP_TAG, "Writing ctrl reg=%x", ctrl);
    CHK_RES(imu->compassWriteByte(imu, BMP280_I2C_ADDRESS_1, BMP280_REG_CTRL, ctrl));


#ifdef CONFIG_MPU_I2C
    // finished configs, disable bypass mode
    CHK_RES(imu->setAuxI2CBypass(imu, false));

    // enable master mode to read bmp280
    const auxi2c_config_t kAuxI2CConfig = {
        .clock           = AUXI2C_CLOCK_400KHZ,
        .multi_master_en = 1,
        .sample_delay    = 0,
        .shadow_delay_en = 0,
        .wait_for_es     = 0,
        .transition      = AUXI2C_TRANS_RESTART
    };
    CHK_RES(imu->setAuxI2CConfig(imu, &kAuxI2CConfig));
    CHK_RES(imu->setAuxI2CEnabled(imu, true));
#endif

    // slave 1 reads from bmp280 data register
    const auxi2c_slv_config_t kSlaveReadDataConfig = {
        .slave           = BMP_SLAVE_READ_DATA,
        .addr            = BMP280_I2C_ADDRESS_1,
        .rw              = AUXI2C_READ,
        .reg_addr        = BMP280_REG_PRESS_MSB,
        .reg_dis         = 0,
        .sample_delay_en = 0,
        {{
            .swap_en     = 0,
            .end_of_word = (auxi2c_eow_t) 0,
            .rxlength    = 6
        }}
    };
    CHK_RES(imu->setAuxI2CSlaveConfig(imu, &kSlaveReadDataConfig));
    CHK_RES(imu->setAuxI2CSlaveEnabled(imu, BMP_SLAVE_READ_DATA, true));

    GB_DEBUGI(SENSOR_TAG, "Aux BMP280 Init done, chip id: 0x%x", imu->baro_dev.id);
error_exit:
    return res;
}

static GB_RESULT baroGetData(struct imu *imu, baro_t *baro)
{
    GB_RESULT res = GB_OK;
    int32_t adc_pressure, adc_temp;
    float temperature, pressure;
    int32_t fine_temp;

    if (!(imu->mpu_status & IMU_BARO_STATUS_BIT))
    {
        // BMP280 not available
        goto error_exit;
    }
    CHK_RES(imu->readBytes(imu, EXT_SENS_DATA_06, 6, imu->buffer));
    //GB_DUMPI(SENSOR_TAG, imu->buffer, 6);
    adc_pressure = imu->buffer[0] << 12 | imu->buffer[1] << 4 | imu->buffer[2] >> 4;
    adc_temp     = imu->buffer[3] << 12 | imu->buffer[4] << 4 | imu->buffer[5] >> 4;

    temperature = compensate_temperature(&imu->baro_dev, adc_temp, &fine_temp) / 100.0f;  //°
    pressure    = compensate_pressure(&imu->baro_dev, adc_pressure, fine_temp) / 256.0f;  //Pa

    //中位值滤波
    pressure = applyBarometerMedianFilter(pressure * 10) / 10.0f;
    if (isBaroCalibrationFinished())
    {
        //计算去除地面高度后相对高度
        baro->altitude = pressureToAltitude(pressure) - getBaroGroundAltitude();
    }
    else
    {
        performBaroCalibrationCycle(pressure);
        baro->altitude = 0.0f;
    }
    baro->temperature = temperature;
    baro->pressure    = pressure;

    //GB_DEBUGI(SENSOR_TAG, "DEBUG baro_data: [%+6.2fPa %+6.2fC %+6.2fcm ] [%d, %d] \n", baro->pressure, baro->temperature, baro->altitude, adc_pressure, adc_temp);
error_exit:
    return res;
}

/**
 * @brief Initialize Magnetometer sensor.
 *
 * Initial configuration:
 *  - Mode: single measurement (permits variable sample rate).
 *  - Sensitivity: 0.15 uT/LSB  =  16-bit output.
 *
 * To disable the compass, call compassSetMode(MAG_MODE_POWER_DOWN).
 * */
static GB_RESULT compassInit(struct imu *imu)
{
    GB_RESULT res = GB_OK;
    CHK_RES(imu->setAuxI2CReset(imu));
    // must delay, or compass may not be initialized
    GB_SleepMs(50);
    // I2C => bypass mode
#ifdef CONFIG_MPU_I2C
    CHK_RES(imu->setAuxI2CBypass(imu, true));
#elif CONFIG_MPU_SPI
    // SPI => master mode
    const auxi2c_config_t kAuxI2CConfig = {
        .clock           = AUXI2C_CLOCK_400KHZ,
        .multi_master_en = 1,
        .sample_delay    = 0,
        .shadow_delay_en = 0,
        .wait_for_es     = 0,
        .transition      = AUXI2C_TRANS_RESTART
    };
    CHK_RES(imu->setAuxI2CConfig(imu, &kAuxI2CConfig));
    //GB_SleepMs(50);
    CHK_RES(imu->setAuxI2CEnabled(imu, true));
    //GB_SleepMs(50);
#endif

    // who am i
    res = imu->compassWhoAmI(imu);
    if (GB_OK == res && LIS3MDL_CHIP_ID == imu->buffer[0])
    {
#ifdef CONFIG_AUX_COMPASS
        imu->mpu_status |= IMU_MAG_STATUS_BIT;
        GB_DEBUGI(SENSOR_TAG, "LIS3MDL Compass Chip Selected");

        GB_SleepMs(50);
        // SPI MODE: In write mode, the contents of I2C_SLV0_DO (Register 99) will be written to the slave device.
        // I2C MODE: Auxiliary Pass-Through Mode
        /* configure the magnetometer */
        CHK_RES(imu->setMagfullScale(imu, g_compass_fs));
        //GB_SleepMs(50);
        CHK_RES(imu->compassSetSampleMode(imu, lis3mdl_hpm_300));
        //GB_SleepMs(50);
        CHK_RES(imu->compassSetMeasurementMode(imu, lis3mdl_continuous_measurement));
        GB_SleepMs(50);
#else
        GB_DEBUGD(SENSOR_TAG, "CONFIG_AUX_COMPASS not enabled");
        CHK_RES(GB_MPU_AUXDEV_NOT_ENABLE);
#endif
    }

#ifdef CONFIG_MPU_I2C
    // finished configs, disable bypass mode
    CHK_RES(imu->setAuxI2CBypass(imu, false));

    // enable master mode to read mag
    const auxi2c_config_t kAuxI2CConfig = {
        .clock           = AUXI2C_CLOCK_400KHZ,
        .multi_master_en = 1,
        .sample_delay    = 0,
        .shadow_delay_en = 0,
        .wait_for_es     = 0,
        .transition      = AUXI2C_TRANS_RESTART
    };
    CHK_RES(imu->setAuxI2CConfig(imu, &kAuxI2CConfig));
    CHK_RES(imu->setAuxI2CEnabled(imu, true));
#endif


    // slave 0 reads from magnetometer data register
    auxi2c_slv_config_t kSlaveReadDataConfig = {
        .slave           = MAG_SLAVE_READ_DATA,
        .addr            = COMPASS_I2CADDRESS,
        .rw              = AUXI2C_READ,
        .reg_addr        = 0x00,
        .reg_dis         = 0,
        .sample_delay_en = 0,
        {{
            .swap_en     = 0,
            .end_of_word = (auxi2c_eow_t) 0,
            .rxlength    = 6
        }}
    };

    if (GB_OK == res && (imu->mpu_status & IMU_MAG_STATUS_BIT))
    {
        kSlaveReadDataConfig.reg_addr = LIS3MDL_REG_OUT_X_L;
    }
    CHK_RES(imu->setAuxI2CSlaveConfig(imu, &kSlaveReadDataConfig));
    CHK_RES(imu->setAuxI2CSlaveEnabled(imu, MAG_SLAVE_READ_DATA, true));

    res = GB_OK;
    GB_DEBUGI(SENSOR_TAG, "Aux Compass Init done");
error_exit:
    return res;
}
/**
 * @brief Soft reset LIS3MDL.
 * */
static GB_RESULT compassReset(struct imu *imu)
{
    return imu->compassWriteByte(imu, COMPASS_I2CADDRESS, LIS3MDL_REG_CTRL2, 0x04);
}

/**
 * @brief Return value from WHO_I_AM register.
 * @details Should be `0x48` for AK8963 and AK8975.
 * */
static GB_RESULT compassWhoAmI(struct imu *imu)
{
    return imu->compassReadBytes(imu, COMPASS_I2CADDRESS, LIS3MDL_REG_WHO_AM_I, imu->buffer, 1);
}

/**
 * @brief Change magnetometer's measurement mode.
 * @note
 *  - When user wants to change operation mode, transit to power-down mode first and then transit to other modes.
 *    After power-down mode is set, at least 100µs(Twat) is needed before setting another mode.
 *  - Setting to MAG_MODE_POWER_DOWN will disable readings from compass and disable (free) Aux I2C slaves 0 and 1.
 *    It will not disable Aux I2C Master I/F though! To enable back, use compassInit().
 * */
static GB_RESULT compassSetSampleMode(struct imu *imu, mag_mode_t mode)
{
    GB_RESULT res = GB_OK;
    uint8_t ctrl_reg1 = 0x00;
    uint8_t ctrl_reg4 = 0x00;

    switch (mode) {
    case lis3mdl_lpm_0_625:  // low power mode at 0.625 Hz
        ctrl_reg1 = 0x00;
        break;
    case lis3mdl_lpm_1_25:       // low power mode at 1.25 Hz
        ctrl_reg1 = 0x04;
        break;
    case lis3mdl_lpm_2_5:        // low power mode at 2.5 Hz
        ctrl_reg1 = 0x08;
        break;
    case lis3mdl_lpm_5:          // low power mode at 5 Hz
        ctrl_reg1 = 0x0c;
        break;
    case lis3mdl_lpm_10:         // low power mode at 10 Hz
        ctrl_reg1 = 0x10;
        break;
    case lis3mdl_lpm_20:         // low power mode at 20 Hz
        ctrl_reg1 = 0x14;
        break;
    case lis3mdl_lpm_40:         // low power mode at 40 Hz
        ctrl_reg1 = 0x18;
        break;
    case lis3mdl_lpm_80:         // low power mode at 80 Hz
        ctrl_reg1 = 0x1c;
        break;
    case lis3mdl_lpm_1000:       // low power mode at 1000 Hz
        ctrl_reg1 = 0x02;
        break;
    case lis3mdl_mpm_560:        // medium performance mode at 560 Hz
        ctrl_reg1 = 0x22;
        ctrl_reg4 = 0x04;
        break;
    case lis3mdl_hpm_300:        // high performance mode at 300 Hz
        ctrl_reg1 = 0x42;
        ctrl_reg4 = 0x08;
        break;
    case lis3mdl_uhpm_155:       // ultra high performance mode at 155 Hz
        ctrl_reg1 = 0x62;
        ctrl_reg4 = 0x0c;
        break;
    case lis3mdl_low_power:      // low power mode at 0.625 Hz
        break;
    default:
        GB_DEBUGE(ERROR_TAG, "[compassSetSampleMode] should never get to here\n");
    }
    CHK_RES(compassWriteByte(imu, COMPASS_I2CADDRESS, LIS3MDL_REG_CTRL1, ctrl_reg1));
    CHK_RES(compassWriteByte(imu, COMPASS_I2CADDRESS, LIS3MDL_REG_CTRL4, ctrl_reg4));
error_exit:
    return res;
}

/**
 * @brief set LIS3MDL FULL-scale range
 * @details
 * */
static GB_RESULT setMagfullScale(struct imu *imu, compass_scale_t scale)
{
    GB_RESULT res = GB_OK;
    uint8_t ctrl_reg2;
    CHK_RES(compassReadBytes(imu, COMPASS_I2CADDRESS, LIS3MDL_REG_CTRL2, &ctrl_reg2, 1));
    switch (scale) {
    case lis3mdl_scale_4_Gs:
        ctrl_reg2 &= 0x9f;
        break;
    case lis3mdl_scale_8_Gs:
        ctrl_reg2 &= 0xbf;
        ctrl_reg2 |= 0x20;
        break;
    case lis3mdl_scale_12_Gs:
        ctrl_reg2 |= 0x40;
        ctrl_reg2 &= 0xdf;
        break;
    case lis3mdl_scale_16_Gs:
        ctrl_reg2 |= 0x60;
        break;
    default:
        GB_DEBUGE(ERROR_TAG, "[setMagfullScale] should never get to here\n");
        CHK_RES(GB_COMPASS_W_SCALE);
    }
    CHK_RES(imu->compassWriteByte(imu, COMPASS_I2CADDRESS, LIS3MDL_REG_CTRL2, ctrl_reg2));
error_exit:
    return res;
}

/**
 * @brief Change magnetometer's measurement mode.
 * @note
 *  - Setting to MAG_MODE_POWER_DOWN will disable readings from compass and disable (free) Aux I2C slaves 0 and 1.
 *    It will not disable Aux I2C Master I/F though! To enable back, use compassInit().
 * */
static GB_RESULT compassSetMeasurementMode(struct imu *imu, compass_measurement_mode_t mode)
{
    uint8_t ctrl_reg3 = 0x00;
    GB_RESULT res = GB_OK;

    //MPU_ERR_CHECK(compassReadBytes(imu, COMPASS_I2CADDRESS, LIS3MDL_REG_CTRL3, &ctrl_reg3, 1));
    switch (mode) {
    case lis3mdl_power_down:
        ctrl_reg3 |= 0x02;
        break;
    case lis3mdl_single_measurement:
        ctrl_reg3 &= 0xfd;
        ctrl_reg3 |= 0x01;
        break;
    case lis3mdl_continuous_measurement:
        ctrl_reg3 &= 0xfc;
        break;
    default:
        GB_DEBUGE(ERROR_TAG, "[compassSetMeasurementMode] should never get to here\n");
        CHK_RES(GB_COMPASS_W_MODE);
    }
    CHK_RES(imu->compassWriteByte(imu, COMPASS_I2CADDRESS, LIS3MDL_REG_CTRL3, ctrl_reg3));
error_exit:
    return res;
}

/**
 * @brief Read compass data.
 * */
static GB_RESULT heading(struct imu *imu, raw_axes_t* mag)
{
    GB_RESULT res = GB_OK;

    if (!(imu->mpu_status & IMU_MAG_STATUS_BIT)) {
        // compass not available
        goto error_exit;
    }
    CHK_RES(imu->readBytes(imu, EXT_SENS_DATA_00, 6, imu->buffer));
    mag->data.x = imu->buffer[1] << 8 | imu->buffer[0];
    mag->data.y = imu->buffer[3] << 8 | imu->buffer[2];
    mag->data.z = imu->buffer[5] << 8 | imu->buffer[4];
error_exit:
    return res;
}

/**
 * @brief Read accelerometer, gyroscope, compass raw data.
 * */
static GB_RESULT motion_mag(struct imu *imu, raw_axes_t* accel, raw_axes_t* gyro, raw_axes_t* mag)
{
    GB_RESULT res = GB_OK;
    uint8_t buffer[22];
    CHK_RES(imu->readBytes(imu, ACCEL_XOUT_H, 22, buffer));
    accel->data.x = buffer[0] << 8 | buffer[1];
    accel->data.y = buffer[2] << 8 | buffer[3];
    accel->data.z = buffer[4] << 8 | buffer[5];
    gyro->data.x  = buffer[8] << 8 | buffer[9];
    gyro->data.y  = buffer[10] << 8 | buffer[11];
    gyro->data.z  = buffer[12] << 8 | buffer[13];
    mag->data.x   = buffer[16] << 8 | buffer[15];
    mag->data.y   = buffer[18] << 8 | buffer[17];
    mag->data.z   = buffer[20] << 8 | buffer[19];
error_exit:
    return res;
}

/**
 * @brief Trigger gyro and accel hardware self-test.
 * @attention when calling this function, the imu must remain as horizontal as possible (0 degrees), facing up.
 * @param result Should be ZERO if gyro and accel passed.
 * @todo Elaborate doc.
 * */
static GB_RESULT selfTest(struct imu *imu, selftest_t* result)
{
    GB_RESULT res = GB_OK;
#ifdef CONFIG_MPU6050
    const accel_fs_t kAccelFS = ACCEL_FS_16G;
    const gyro_fs_t kGyroFS   = GYRO_FS_250DPS;
#elif defined CONFIG_MPU6500
    const accel_fs_t kAccelFS = ACCEL_FS_2G;
    const gyro_fs_t kGyroFS   = GYRO_FS_250DPS;
#endif
    raw_axes_t gyroRegBias, accelRegBias;
    raw_axes_t gyroSTBias, accelSTBias;
    // get regular biases
    CHK_RES(imu->getBiases(imu, kAccelFS, kGyroFS, &accelRegBias, &gyroRegBias, false));
    // get self-test biases
    CHK_RES(imu->getBiases(imu, kAccelFS, kGyroFS, &accelSTBias, &gyroSTBias, true));
    // perform self-tests
    uint8_t accelST, gyroST;
    CHK_RES(imu->accelSelfTest(imu, &accelRegBias, &accelSTBias, &accelST));
    CHK_RES(imu->gyroSelfTest(imu, &gyroRegBias, &gyroSTBias, &gyroST));
    // check results
    *result = 0;
    if (accelST != 0) *result |= SELF_TEST_ACCEL_FAIL;
    if (gyroST != 0) *result |= SELF_TEST_GYRO_FAIL;

    if (*result != SELF_TEST_PASS) {
        if (*result == SELF_TEST_GYRO_FAIL)
            GB_DEBUGE(ERROR_TAG, "SELF_TEST_GYRO_FAIL\n");
        else if (*result == SELF_TEST_ACCEL_FAIL)
            GB_DEBUGE(ERROR_TAG, "SELF_TEST_ACCEL_FAIL\n");
        else
            GB_DEBUGE(ERROR_TAG, "SELT_TEST_FAIL 0x%x\n", (uint8_t)*result);
        res = GB_MPU_SELF_TEST_FAIL;
    }

error_exit:
    return res;
}

#if defined CONFIG_MPU6500
// Production Self-Test table for MPU6500 based models,
// used in accel and gyro self-test code below.
static const uint16_t kSelfTestTable[256] = {
    2620,  2646,  2672,  2699,  2726,  2753,  2781,  2808,   // 7
    2837,  2865,  2894,  2923,  2952,  2981,  3011,  3041,   // 15
    3072,  3102,  3133,  3165,  3196,  3228,  3261,  3293,   // 23
    3326,  3359,  3393,  3427,  3461,  3496,  3531,  3566,   // 31
    3602,  3638,  3674,  3711,  3748,  3786,  3823,  3862,   // 39
    3900,  3939,  3979,  4019,  4059,  4099,  4140,  4182,   // 47
    4224,  4266,  4308,  4352,  4395,  4439,  4483,  4528,   // 55
    4574,  4619,  4665,  4712,  4759,  4807,  4855,  4903,   // 63
    4953,  5002,  5052,  5103,  5154,  5205,  5257,  5310,   // 71
    5363,  5417,  5471,  5525,  5581,  5636,  5693,  5750,   // 79
    5807,  5865,  5924,  5983,  6043,  6104,  6165,  6226,   // 87
    6289,  6351,  6415,  6479,  6544,  6609,  6675,  6742,   // 95
    6810,  6878,  6946,  7016,  7086,  7157,  7229,  7301,   // 103
    7374,  7448,  7522,  7597,  7673,  7750,  7828,  7906,   // 111
    7985,  8065,  8145,  8227,  8309,  8392,  8476,  8561,   // 119
    8647,  8733,  8820,  8909,  8998,  9088,  9178,  9270,   //
    9363,  9457,  9551,  9647,  9743,  9841,  9939,  10038,  //
    10139, 10240, 10343, 10446, 10550, 10656, 10763, 10870,  //
    10979, 11089, 11200, 11312, 11425, 11539, 11654, 11771,  //
    11889, 12008, 12128, 12249, 12371, 12495, 12620, 12746,  //
    12874, 13002, 13132, 13264, 13396, 13530, 13666, 13802,  //
    13940, 14080, 14221, 14363, 14506, 14652, 14798, 14946,  //
    15096, 15247, 15399, 15553, 15709, 15866, 16024, 16184,  //
    16346, 16510, 16675, 16842, 17010, 17180, 17352, 17526,  //
    17701, 17878, 18057, 18237, 18420, 18604, 18790, 18978,  //
    19167, 19359, 19553, 19748, 19946, 20145, 20347, 20550,  //
    20756, 20963, 21173, 21385, 21598, 21814, 22033, 22253,  //
    22475, 22700, 22927, 23156, 23388, 23622, 23858, 24097,  //
    24338, 24581, 24827, 25075, 25326, 25579, 25835, 26093,  //
    26354, 26618, 26884, 27153, 27424, 27699, 27976, 28255,  //
    28538, 28823, 29112, 29403, 29697, 29994, 30294, 30597,  //
    30903, 31212, 31524, 31839, 32157, 32479, 32804, 33132   //
};
#endif

/**
 * @brief Accel Self-test.
 * @param result self-test error for each axis (X=bit0, Y=bit1, Z=bit2). Zero is a pass.
 * @note Bias should be in 16G format for MPU6050 and 2G for MPU6500 based models.
 * */
static GB_RESULT accelSelfTest(struct imu *imu, raw_axes_t* regularBias, raw_axes_t* selfTestBias, uint8_t* result)
{
    GB_RESULT res = GB_OK;
#if defined CONFIG_MPU6050
    const accel_fs_t kAccelFS = ACCEL_FS_16G;
    // Criteria A: must be within 30% variation
    const float kMaxVariation = .3f;
    // Criteria B: must be between 300 mg and 950 mg
    const float kMinGravity = .3f, kMaxGravity = .95f;

#elif defined CONFIG_MPU6500
    const accel_fs_t kAccelFS = ACCEL_FS_2G;
    // Criteria A: must be within 50% variation
    const float kMaxVariation = .5f;
    // Criteria B: must be between 255 mg and 675 mg
    const float kMinGravity = .225f, kMaxGravity = .675f;
    // Criteria C: 500 mg for accel
    const float kMaxGravityOffset = .5f;
#endif

    /* Convert biases */
    float_axes_t regularBiasGravity  = accelGravity_raw(regularBias, kAccelFS);
    float_axes_t selfTestBiasGravity = accelGravity_raw(selfTestBias, kAccelFS);
    GB_DEBUGI(ST_TAG, "EMPTY, regularBias: %+d %+d %+d | regularBiasGravity: %+.2f %+.2f %+.2f\n", regularBias->data.x,
                regularBias->data.y, regularBias->data.z, regularBiasGravity.data.x, regularBiasGravity.data.y, regularBiasGravity.data.z);
    GB_DEBUGI(ST_TAG, "EMPTY, selfTestBias: %+d %+d %+d | selfTestBiasGravity: %+.2f %+.2f %+.2f\n", selfTestBias->data.x,
                selfTestBias->data.y, selfTestBias->data.z, selfTestBiasGravity.data.x, selfTestBiasGravity.data.y, selfTestBiasGravity.data.z);

    /* Get OTP production shift code */
    uint8_t shiftCode[3];
#if defined CONFIG_MPU6050
    CHK_RES(imu->readBytes(imu, SELF_TEST_X, 4, imu->buffer));
    shiftCode[0] = ((imu->buffer[0] & 0xE0) >> 3) | ((imu->buffer[3] & 0x30) >> 4);
    shiftCode[1] = ((imu->buffer[1] & 0xE0) >> 3) | ((imu->buffer[3] & 0x0C) >> 2);
    shiftCode[2] = ((imu->buffer[2] & 0xE0) >> 3) | (imu->buffer[3] & 0x03);

#elif defined CONFIG_MPU6500
    CHK_RES(imu->readBytes(imu, SELF_TEST_X_ACCEL, 3, shiftCode));
#endif
    GB_DEBUGI(ST_TAG, "EMPTY, shiftCode: %+d %+d %+d\n", shiftCode[0], shiftCode[1], shiftCode[2]);

    /* Calulate production shift value */
    float shiftProduction[3] = {0};
    for (int i = 0; i < 3; i++) {
        if (shiftCode[i] != 0) {
#if defined CONFIG_MPU6050
            // Equivalent to.. shiftProduction[i] = 0.34f * powf(0.92f/0.34f, (shiftCode[i]-1)
            // / 30.f)
            shiftProduction[i] = 0.34f;
            while (--shiftCode[i]) shiftProduction[i] *= 1.034f;

#elif defined CONFIG_MPU6500
            shiftProduction[i] = kSelfTestTable[shiftCode[i] - 1];
            shiftProduction[i] /= accelSensitivity(ACCEL_FS_2G);
#endif
        }
    }
    GB_DEBUGI(ST_TAG, "EMPTY, shiftProduction: %+.2f %+.2f %+.2f\n", shiftProduction[0], shiftProduction[1],
                shiftProduction[2]);

    /* Evaluate criterias */
    *result                 = 0;
    float shiftResponse[3]  = {0};
    float shiftVariation[3] = {0};
    for (int i = 0; i < 3; i++) {
        shiftResponse[i] = fabs(selfTestBiasGravity.xyz[i] - regularBiasGravity.xyz[i]);
        // Criteria A
        if (shiftProduction[i] != 0) {
            shiftVariation[i] = shiftResponse[i] / shiftProduction[i] - 1;
            if (fabs(shiftVariation[i]) > kMaxVariation) *result |= 1 << i;
            // Criteria B
        }
        else if (shiftResponse[i] < kMinGravity || shiftResponse[i] > kMaxGravity) {
            *result |= 1 << i;
        }
// Criteria C
#if defined CONFIG_MPU6050
            // no criteria C
#elif defined CONFIG_MPU6500
        if (fabs(regularBiasGravity.xyz[i] > kMaxGravityOffset)) *result |= 1 << i;
#endif
    }
    GB_DEBUGI(ST_TAG, "EMPTY, shiftResponse: %+.2f %+.2f %+.2f\n", shiftResponse[0], shiftResponse[1], shiftResponse[2]);
    GB_DEBUGI(ST_TAG, "EMPTY, shiftVariation: %+.2f %+.2f %+.2f\n", shiftVariation[0], shiftVariation[1],
                shiftVariation[2]);

    GB_DEBUGI(ST_TAG, "Accel self-test: [X=%s] [Y=%s] [Z=%s]\n", ((*result & 0x1) ? "FAIL" : "OK"),
             ((*result & 0x2) ? "FAIL" : "OK"), ((*result & 0x4) ? "FAIL" : "OK"));
error_exit:
    return res;
}

/**
 * @brief Gyro Self-test.
 * @param result Self-test error for each axis (X=bit0, Y=bit1, Z=bit2). Zero is a pass.
 * @note Bias should be in 250DPS format for both MPU6050 and MPU6500 based models.
 * */
static GB_RESULT gyroSelfTest(struct imu *imu, raw_axes_t* regularBias, raw_axes_t* selfTestBias, uint8_t* result)
{
    GB_RESULT res = GB_OK;
    const gyro_fs_t kGyroFS = GYRO_FS_250DPS;

#if defined CONFIG_MPU6050
    // Criteria A: must not exceed +14% variation
    const float kMaxVariation = .14f;
    // Criteria B: must be between 10 dps and 105 dps
    const float kMinDPS = 10.f, kMaxDPS = 105.f;

#elif defined CONFIG_MPU6500
    // Criteria A: must be within 50% variation
    const float kMaxVariation = .5f;
    // Criteria B: must be between 20 dps and 60 dps
    const float kMinDPS = 20.f, kMaxDPS = 60.f;
#endif

    /* Convert biases */
    float_axes_t regularBiasDPS  = gyroDegPerSec_raw(regularBias, kGyroFS);
    float_axes_t selfTestBiasDPS = gyroDegPerSec_raw(selfTestBias, kGyroFS);
    GB_DEBUGI(ST_TAG, "EMPTY, regularBias: %+d %+d %+d | regularBiasDPS: %+.2f %+.2f %+.2f\n", regularBias->data.x,
                regularBias->data.y, regularBias->data.z, regularBiasDPS.data.x, regularBiasDPS.data.y, regularBiasDPS.data.z);
    GB_DEBUGI(ST_TAG, "EMPTY, selfTestBias: %+d %+d %+d | selfTestBiasDPS: %+.2f %+.2f %+.2f\n", selfTestBias->data.x,
                selfTestBias->data.y, selfTestBias->data.z, selfTestBiasDPS.data.x, selfTestBiasDPS.data.y, selfTestBiasDPS.data.z);

    /* Get OTP production shift code */
    uint8_t shiftCode[3];
#if defined CONFIG_MPU6050
    CHK_RES(imu->readBytes(imu, SELF_TEST_X, 3, imu->buffer));
    shiftCode[0] = imu->buffer[0] & 0x1F;
    shiftCode[1] = imu->buffer[1] & 0x1F;
    shiftCode[2] = imu->buffer[2] & 0x1F;

#elif defined CONFIG_MPU6500
    CHK_RES(imu->readBytes(imu, SELF_TEST_X_GYRO, 3, shiftCode));
#endif
    GB_DEBUGI(ST_TAG, "EMPTY, shiftCode: %+d %+d %+d\n", shiftCode[0], shiftCode[1], shiftCode[2]);

    /* Calulate production shift value */
    float shiftProduction[3] = {0};
    for (int i = 0; i < 3; i++) {
        if (shiftCode[i] != 0) {
#if defined CONFIG_MPU6050
            shiftProduction[i] = 3275.f / gyroSensitivity(kGyroFS);  // should yield 25
            while (--shiftCode[i]) shiftProduction[i] *= 1.046f;

#elif defined CONFIG_MPU6500
            shiftProduction[i] = kSelfTestTable[shiftCode[i] - 1];
            shiftProduction[i] /= gyroSensitivity(kGyroFS);
#endif
        }
    }
    GB_DEBUGI(ST_TAG, "EMPTY, shiftProduction: %+.2f %+.2f %+.2f\n", shiftProduction[0], shiftProduction[1],
                shiftProduction[2]);

    /* Evaluate criterias */
    *result                 = 0;
    float shiftResponse[3]  = {0};
    float shiftVariation[3] = {0};
    for (int i = 0; i < 3; i++) {
        shiftResponse[i] = fabs(selfTestBiasDPS.xyz[i] - regularBiasDPS.xyz[i]);
        // Criteria A
        if (shiftProduction[i] != 0) {
            shiftVariation[i] = shiftResponse[i] / shiftProduction[i] - 1;
            if (fabs(shiftVariation[i]) > kMaxVariation) *result |= 1 << i;
            // Criteria B
        }
        else if (shiftResponse[i] < kMinDPS || shiftResponse[i] > kMaxDPS) {
            *result |= 1 << i;
        }
    }
    GB_DEBUGI(ST_TAG, "EMPTY, shiftResponse: %+.2f %+.2f %+.2f\n", shiftResponse[0], shiftResponse[1], shiftResponse[2]);
    GB_DEBUGI(ST_TAG, "EMPTY, shiftVariation: %+.2f %+.2f %+.2f\n", shiftVariation[0], shiftVariation[1],
                shiftVariation[2]);

    GB_DEBUGI(ST_TAG, "Gyro self-test: [X=%s] [Y=%s] [Z=%s]\n", ((*result & 0x1) ? "FAIL" : "OK"),
             ((*result & 0x2) ? "FAIL" : "OK"), ((*result & 0x4) ? "FAIL" : "OK"));
error_exit:
    return res;
}

/**
 * @brief Compute the Biases in regular mode and self-test mode.
 * @attention When calculating the biases the imu must remain as horizontal as possible (0 degrees), facing up.
 * This algorithm takes about ~400ms to compute offsets.
 * */
static GB_RESULT getBiases(struct imu *imu, accel_fs_t accelFS, gyro_fs_t gyroFS, raw_axes_t* accelBias, raw_axes_t* gyroBias,
                         bool selftest)
{
    GB_RESULT res = GB_OK;
    // configurations to compute biases
    const uint16_t kSampleRate      = 1000;
    const dlpf_t kDLPF              = DLPF_188HZ;
    const fifo_config_t kFIFOConfig = FIFO_CFG_ACCEL | FIFO_CFG_GYRO;
    int32_t accelAvgx = 0, accelAvgy = 0, accelAvgz = 0;
    int32_t gyroAvgx  = 0, gyroAvgy  = 0, gyroAvgz  = 0;
    float_axes_t accelG;   // accel axes in (g) gravity format
    float_axes_t gyroDPS;  // gyro axes in (DPS) º/s format
    // backup previous configuration
    const uint16_t prevSampleRate      = imu->getSampleRate(imu);
    const dlpf_t prevDLPF              = imu->getDigitalLowPassFilter(imu);
    const accel_fs_t prevAccelFS       = imu->getAccelFullScale(imu);
    const gyro_fs_t prevGyroFS         = imu->getGyroFullScale(imu);
    const fifo_config_t prevFIFOConfig = imu->getFIFOConfig(imu);
    const bool prevFIFOState           = imu->getFIFOEnabled(imu);

    // setup
    CHK_RES(imu->setSampleRate(imu, kSampleRate));
    CHK_RES(imu->setDigitalLowPassFilter(imu, kDLPF));
    CHK_RES(imu->setAccelFullScale(imu, accelFS));
    CHK_RES(imu->setGyroFullScale(imu, gyroFS));
    CHK_RES(imu->setFIFOConfig(imu, kFIFOConfig));
    CHK_RES(imu->setFIFOEnabled(imu, true));
    if (selftest) {
        CHK_RES(imu->writeBits(imu, ACCEL_CONFIG, ACONFIG_XA_ST_BIT, 3, 0x7));
        CHK_RES(imu->writeBits(imu, GYRO_CONFIG, GCONFIG_XG_ST_BIT, 3, 0x7));
    }
    // wait for 200ms for sensors to stabilize
    GB_SleepMs(1000);

    raw_axes_t accelRaw;   // x, y, z axes as int16
    raw_axes_t gyroRaw;    // x, y, z axes as int16
    const int packetCount = 500;

    for (int i = 0; i < packetCount; i++) {
        GB_SleepMs(4);
        imu->acceleration(imu, &accelRaw);  // fetch raw data from the registers
        imu->rotation(imu, &gyroRaw);       // fetch raw data from the registers
        // add up
        accelAvgx += accelRaw.data.x;
        accelAvgy += accelRaw.data.y;
        accelAvgz += accelRaw.data.z;
        gyroAvgx += gyroRaw.data.x;
        gyroAvgy += gyroRaw.data.y;
        gyroAvgz += gyroRaw.data.z;

        float_axes_t accelG1  = accelGravity_raw(&accelRaw, accelFS);
        float_axes_t gyroDPS1 = gyroDegPerSec_raw(&gyroRaw, gyroFS);
        GB_DEBUGD(ST_TAG, "[sample]gyro: [%+7.2f %+7.2f %+7.2f ] (º/s) \t", gyroDPS1.xyz[0], gyroDPS1.xyz[1], gyroDPS1.xyz[2]);
        GB_DEBUGD(ST_TAG, "[sample]accel: [%+6.2f %+6.2f %+6.2f ] (G) \t\n", accelG1.data.x, accelG1.data.y, accelG1.data.z);
    }
    raw_axes_t accelAvg;   // x, y, z axes as int16
    raw_axes_t gyroAvg;    // x, y, z axes as int16
    // calculate average
    accelAvg.data.x = accelAvgx / packetCount;
    accelAvg.data.y = accelAvgy / packetCount;
    accelAvg.data.z = accelAvgz / packetCount;
    gyroAvg.data.x = gyroAvgx / packetCount;
    gyroAvg.data.y = gyroAvgy / packetCount;
    gyroAvg.data.z = gyroAvgz / packetCount;

    accelG  = accelGravity_raw(&accelAvg, accelFS);
    gyroDPS = gyroDegPerSec_raw(&gyroAvg, gyroFS);
    GB_DEBUGD(ST_TAG, "[Bias]gyro: [%+7.2f %+7.2f %+7.2f ] (º/s) \t", gyroDPS.xyz[0], gyroDPS.xyz[1], gyroDPS.xyz[2]);
    GB_DEBUGD(ST_TAG, "[Bias]accel: [%+6.2f %+6.2f %+6.2f ] (G) \t\n", accelG.data.x, accelG.data.y, accelG.data.z);

    // remove gravity from Accel Z axis
    const uint16_t gravityLSB = INT16_MAX >> (accelFS + 1);
    accelAvg.data.z -= gravityLSB;

    accelG  = accelGravity_raw(&accelAvg, accelFS);
    gyroDPS = gyroDegPerSec_raw(&gyroAvg, gyroFS);
    GB_DEBUGD(ST_TAG, "[Bias]gyro: [%+7.2f %+7.2f %+7.2f ] (º/s) \t", gyroDPS.xyz[0], gyroDPS.xyz[1], gyroDPS.xyz[2]);
    GB_DEBUGD(ST_TAG, "[Bias]accel: [%+6.2f %+6.2f %+6.2f ] (G) \t\n", accelG.data.x, accelG.data.y, accelG.data.z);

    // save biases
    for (int i = 0; i < 3; i++) {
        (*accelBias).xyz[i] = (int16_t) accelAvg.xyz[i];
        (*gyroBias).xyz[i]  = (int16_t) gyroAvg.xyz[i];
    }
    // set back previous configs
    CHK_RES(imu->setSampleRate(imu, prevSampleRate));
    CHK_RES(imu->setDigitalLowPassFilter(imu, prevDLPF));
    CHK_RES(imu->setAccelFullScale(imu, prevAccelFS));
    CHK_RES(imu->setGyroFullScale(imu, prevGyroFS));
    CHK_RES(imu->setFIFOConfig(imu, prevFIFOConfig));
    CHK_RES(imu->setFIFOEnabled(imu, prevFIFOState));

    // reset self test mode
    if (selftest) {
        CHK_RES(imu->writeBits(imu, ACCEL_CONFIG, ACONFIG_XA_ST_BIT, 3, 0x0));
        CHK_RES(imu->writeBits(imu, GYRO_CONFIG, GCONFIG_XG_ST_BIT, 3, 0x0));
    }
error_exit:
    return res;
}

/**
 * @brief Set Accelerometer and Gyroscope offsets.
 *
 * This takes about ~400ms to compute offsets.
 * When calculating the offsets the imu must remain as horizontal as possible (0 degrees), facing
 * up. It is better to call computeOffsets() before any configuration is done (better right after
 * initialize()).
 *
 * Note: Gyro offset output are LSB in 1000DPS format.
 * Note: Accel offset output are LSB in 16G format.
 * */
static GB_RESULT setOffsets(struct imu *imu, bool gyro_offset_enable, bool accel_offset_enable)
{
    GB_RESULT res = GB_OK;
    raw_axes_t accel;   // x, y, z axes as int16
    raw_axes_t gyro;    // x, y, z axes as int16

    const accel_fs_t prevAccelFS = imu->getAccelFullScale(imu);
    const gyro_fs_t prevGyroFS   = imu->getGyroFullScale(imu);

    CHK_RES(imu->setAccelFullScale(imu, ACCEL_FS_16G));
    CHK_RES(imu->setGyroFullScale(imu, GYRO_FS_1000DPS));

    CHK_RES(imu->computeOffsets(imu, &accel, &gyro));

    //GB_DEBUGE(ERROR_TAG, "set Bias gyro: %x, %x, %x\n", gyro.data.x, gyro.data.y, gyro.data.z);
    //GB_DEBUGE(ERROR_TAG, "set Bias accel: %x, %x, %x\n", accel.data.x, accel.data.y, accel.data.z);
    if (gyro_offset_enable)
        CHK_RES(imu->setGyroOffset(imu, gyro));
    if (accel_offset_enable)
        CHK_RES(imu->setAccelOffset(imu, accel));

    CHK_RES(imu->setAccelFullScale(imu, prevAccelFS));
    CHK_RES(imu->setGyroFullScale(imu, prevGyroFS));

    //gyro = imu->getGyroOffset(imu);
    //accel = imu->getAccelOffset(imu);
    //GB_DEBUGE(ERROR_TAG, "get Bias gyro: %x, %x, %x\n", gyro.data.x, gyro.data.y, gyro.data.z);
    //GB_DEBUGE(ERROR_TAG, "get Bias accel: %x, %x, %x\n", accel.data.x, accel.data.y, accel.data.z);
    GB_SleepMs(500);
error_exit:
    return res;
}

/*
 * MPU Math Function
 */
inline uint8_t accelFSRvalue(const accel_fs_t fs)
{
    return 2 << fs;
}

inline uint16_t gyroFSRvalue(const gyro_fs_t fs)
{
    return 250 << fs;
}

inline uint16_t accelSensitivity(const accel_fs_t fs)
{
    return 16384 >> fs;
}

inline float gyroSensitivity(const gyro_fs_t fs)
{
    return 131.f / (1 << fs);
}

inline float accelResolution(const accel_fs_t fs)
{
    return (float)(accelFSRvalue(fs)) / INT16_MAX;
}

inline float gyroResolution(const gyro_fs_t fs)
{
    return (float)(gyroFSRvalue(fs)) / INT16_MAX;
}

inline float accelGravity(const int16_t axis, const accel_fs_t fs)
{
    return axis * accelResolution(fs);
}

inline float_axes_t accelGravity_raw(const raw_axes_t *raw_axes, const accel_fs_t fs)
{
    float_axes_t axes;
    axes.data.x = raw_axes->data.x * accelResolution(fs);
    axes.data.y = raw_axes->data.y * accelResolution(fs);
    axes.data.z = raw_axes->data.z * accelResolution(fs);
    return axes;
}

inline float gyroDegPerSec(const int16_t axis, const gyro_fs_t fs)
{
    return axis * gyroResolution(fs);
}

inline float_axes_t gyroDegPerSec_raw(const raw_axes_t *raw_axes, const gyro_fs_t fs)
{
    float_axes_t axes;
    axes.data.x = raw_axes->data.x * gyroResolution(fs);
    axes.data.y = raw_axes->data.y * gyroResolution(fs);
    axes.data.z = raw_axes->data.z * gyroResolution(fs);
    return axes;
}

inline float gyroRadPerSec(const int16_t axis, const gyro_fs_t fs)
{
    return (M_PI / 180) * gyroDegPerSec(axis, fs);
}

inline float_axes_t gyroRadPerSec_raw(const raw_axes_t *raw_axes, const gyro_fs_t fs)
{
    float_axes_t axes;
    axes.data.x = (M_PI / 180) * gyroDegPerSec(raw_axes->data.x, fs);
    axes.data.y = (M_PI / 180) * gyroDegPerSec(raw_axes->data.y, fs);
    axes.data.z = (M_PI / 180) * gyroDegPerSec(raw_axes->data.z, fs);
    return axes;
}

inline float magResolution(const compass_scale_t fs)
{
    switch (fs) {
        case lis3mdl_scale_4_Gs:
            return LIS3MDL_MAG_SENSITIVITY_FOR_FS_4GA;
        case lis3mdl_scale_8_Gs:
            return LIS3MDL_MAG_SENSITIVITY_FOR_FS_8GA;
        case lis3mdl_scale_12_Gs:
            return LIS3MDL_MAG_SENSITIVITY_FOR_FS_12GA;
        case lis3mdl_scale_16_Gs:
            return LIS3MDL_MAG_SENSITIVITY_FOR_FS_16GA;
    }
    return 0;
}

inline float_axes_t magGauss_raw(const raw_axes_t *raw_axes, const compass_scale_t fs)
{
    float_axes_t axes;
    axes.data.x = raw_axes->data.x * magResolution(fs);
    axes.data.y = raw_axes->data.y * magResolution(fs);
    axes.data.z = raw_axes->data.z * magResolution(fs);
    return axes;
}

inline float tempCelsius(const int16_t temp)
{
    // TEMP_degC = ((TEMP_OUT – RoomTemp_Offset)/Temp_Sensitivity) + DegreesCelsius_Offset
    return (temp - kRoomTempOffset) * kTempResolution + kCelsiusOffset;
}

inline float tempFahrenheit(const int16_t temp)
{
    return (temp - kRoomTempOffset) * kTempResolution * 1.8f + kFahrenheitOffset;
}

