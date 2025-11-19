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
#include "gpio_setting.h"

static GB_RESULT initialize(struct imu *imu);
static GB_RESULT reset(struct imu *imu);
static GB_RESULT testConnection(struct imu *imu);
static uint8_t whoAmI(struct imu *imu);
static GB_RESULT setSampleRate(struct imu *imu, uint16_t rate);
static uint16_t getSampleRate(struct imu *imu);

static GB_RESULT setFilters(struct imu *imu, bool gyroFilters, bool accFilters);
static GB_RESULT setDigitalLowPassFilter(struct imu *imu, dlpf_t dlpf);
static dlpf_t getDigitalLowPassFilter(struct imu *imu);

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

static GB_RESULT setInterruptConfig(struct imu *imu, int_config_t config);
static int_config_t getInterruptConfig(struct imu *imu);
static GB_RESULT setInterruptEnabled(struct imu *imu, int_en_t mask);
static int_en_t getInterruptEnabled(struct imu *imu);
static GB_RESULT setFIFOMode(struct imu *imu, fifo_mode_t mode);
static fifo_mode_t getFIFOMode(struct imu *imu);
static GB_RESULT setFIFOConfig(struct imu *imu, fifo_config_t config);
static fifo_config_t getFIFOConfig(struct imu *imu);
static uint16_t getFIFOCount(struct imu *imu);
static GB_RESULT readFIFO(struct imu *imu, size_t length, uint8_t* data);

static GB_RESULT getBiases(struct imu *imu, accel_fs_t accelFS, gyro_fs_t gyroFS, raw_axes_t* accelBias, raw_axes_t* gyroBias,
                         bool selftest);
static GB_RESULT setOffsets(struct imu *imu, bool gyro, bool accel);

static GB_RESULT compassInit(struct imu *imu);
static GB_RESULT heading(struct imu *imu, raw_axes_t* mag);

static GB_RESULT baroInit(struct imu *imu);
static GB_RESULT baroGetData(struct imu *imu, baro_t *baro);

const accel_fs_t g_accel_fs = ACCEL_FS_16G;
const gyro_fs_t g_gyro_fs = GYRO_FS_2000DPS;
const compass_scale_t g_compass_fs = QMC5883L_RNG_8;

// Possible gyro Anti-Alias Filter (AAF) cutoffs for ICM-42688P
// actual cutoff differs slightly from those of the 42688P
static aafConfig_t imuAAFConfig[DLPF_MAX] = { // see table in section 5.3
    [DLPF_42HZ]  = {1, 1, 15},      // actually 42 Hz
    [DLPF_258HZ] = {21, 440, 6},    // actually 249 Hz
    [DLPF_536HZ] = {39, 1536, 4},   // actually 524 Hz
    [DLPF_997HZ] = {63, 3968, 3},   // actually 995 Hz
    [DLPF_1962HZ] = {63, 3968, 3}, // 995 Hz is the max cut off on the 42605
};

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
    uint8_t mode = 3;
    imu->bus  = &fspi;
    imu->addr = GB_SPI_DEV_0;

    CHK_EXIT(fspi.begin(&fspi, MPU_FSPI_MOSI, MPU_FSPI_MISO, MPU_FSPI_SCLK, SPI_MAX_DMA_LEN));
    CHK_EXIT(fspi.addDevice(&fspi, imu->addr, 8, mode, SPI_DEVICE_NO_DUMMY, MPU_SPI_CLOCK_SPEED, MPU_FSPI_CS));

#if defined CONFIG_AUX_BAROMETER
    CHK_EXIT(fspi.addDevice(&fspi, GB_SPI_DEV_1, 8, mode, SPI_DEVICE_NO_DUMMY, BARO_SPI_CLOCK_SPEED, BARO_SPI_CS));
#endif
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
    imu->setSleep                = NULL;
    imu->getSleep                = NULL;
    imu->testConnection          = &testConnection;
    imu->whoAmI                  = &whoAmI;
    imu->setSampleRate           = &setSampleRate;
    imu->getSampleRate           = &getSampleRate;
    imu->setClockSource          = NULL;
    imu->getClockSource          = NULL;
    imu->setDigitalLowPassFilter = &setDigitalLowPassFilter;
    imu->getDigitalLowPassFilter = &getDigitalLowPassFilter;
    imu->resetSignalPath         = NULL;
    imu->setLowPowerAccelMode    = NULL;
    imu->getLowPowerAccelMode    = NULL;
    imu->setLowPowerAccelRate    = NULL;
    imu->getLowPowerAccelRate    = NULL;
    imu->setFilters              = &setFilters;

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
    imu->sensors_sen       = NULL;

    imu->setInterruptConfig    = &setInterruptConfig;
    imu->getInterruptConfig    = &getInterruptConfig;
    imu->setInterruptEnabled   = &setInterruptEnabled;
    imu->getInterruptEnabled   = &getInterruptEnabled;
    imu->getInterruptStatus    = NULL;
    imu->setFIFOMode           = &setFIFOMode;
    imu->getFIFOMode           = &getFIFOMode;
    imu->setFIFOConfig         = &setFIFOConfig;
    imu->getFIFOConfig         = &getFIFOConfig;
    imu->setFIFOEnabled        = NULL;
    imu->getFIFOEnabled        = NULL;
    imu->resetFIFO             = NULL;
    imu->getFIFOCount          = &getFIFOCount;
    imu->readFIFO              = &readFIFO;
    imu->writeFIFO             = NULL;
    imu->registerDump          = NULL;

    imu->selfTest      = NULL;
    imu->accelSelfTest = NULL;
    imu->gyroSelfTest  = NULL;
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

    imu->baroInit    = &baroInit;
    imu->baroGetData = &baroGetData;

    imu->compassInit = &compassInit;
    imu->heading     = &heading;
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

static GB_RESULT setBank(struct imu *imu, uint8_t bank) {
    static uint8_t _bank = 0xff;

    // if we are already on this bank, bail
    if (_bank == bank) {
        return GB_OK;
    }

    _bank = bank;
    return imu->writeByte(imu, REG_BANK_SEL, bank);
}

/**
 * @brief Initialize imu device and set basic configurations.
 * */
static GB_RESULT initialize(struct imu *imu)
{
    GB_RESULT res = GB_OK;
    int_config_t int_config = {
        .int2_mode = 0,
        .int2_drive = 1,
        .int2_level = 1,
        .int1_mode = 0,
        .int1_drive = 1,
        .int1_level = 1,
    }; // push-pull, pulsed, active HIGH interrupts

    // reset device (wait a little to clear all registers)
    CHK_RES(imu->reset(imu));
    CHK_RES(imu->testConnection(imu));

    CHK_RES(setBank(imu, 0));
    // Turn off ACC and GYRO so they can be configured
    // See section 12.9 in ICM-42688-P datasheet v1.7
    CHK_RES(imu->writeByte(imu, UB0_REG_PWR_MGMT0, UB0_REG_PWR_MGMT0_GYRO_ACCEL_MODE_OFF));
    // set filter
    CHK_RES(imu->setDigitalLowPassFilter(imu, DLPF_258HZ));
    // disable inner filters. default enable
    // CHK_RES(imu->setFilters(imu, false, false));

    CHK_RES(imu->setInterruptConfig(imu, int_config));
    CHK_RES(imu->setInterruptEnabled(imu, 0));

    // Disable AFSR to prevent stalls in gyro output. ref: https://github.com/ArduPilot/ardupilot/pu11/25332
    CHK_RES(imu->readByte(imu, UB0_REG_INTF_CONFIG1, imu->buffer));
    imu->buffer[0] &= ~UB0_REG_INTF_CONFIG1_AFSR_MASK;
    imu->buffer[0] |= UB0_REG_INTF_CONFIG1_AFSR_DISABLE;
    CHK_RES(imu->writeByte(imu, UB0_REG_INTF_CONFIG1, imu->buffer[0]));

    // Turn on ACC and GYRO
    CHK_RES(setBank(imu, 0));
    CHK_RES(imu->writeByte(imu, UB0_REG_PWR_MGMT0, UB0_REG_PWR_MGMT0_TEMP_DISABLE_OFF | UB0_REG_PWR_MGMT0_ACCEL_MODE_LN | UB0_REG_PWR_MGMT0_GYRO_MODE_LN));
    GB_SleepMs (100); // ICM-42688-P datasheet PWR_MGMTO note

    CHK_RES(imu->setGyroFullScale(imu, g_gyro_fs));
    CHK_RES(imu->setAccelFullScale(imu, g_accel_fs));
    CHK_RES(imu->setSampleRate(imu, odr1k << 8 | odr1k));

#if defined CONFIG_AUX_BAROMETER
    //CHK_RES(imu->baroInit(imu));
#endif

#if defined CONFIG_AUX_COMPASS
    imu->compassInit(imu);
#endif

    CHK_RES(imu->testConnection(imu));

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

    CHK_RES(setBank(imu, 0));
    CHK_RES(imu->writeByte(imu, UB0_REG_DEVICE_CONFIG, 0x01));
    GB_SleepMs(100);

error_exit:
    return res;
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

    const uint8_t wai = imu->whoAmI(imu);
    CHK_RES(imu->lastError(imu));

    GB_DEBUGI(SENSOR_TAG, "Who Am I: %02x", wai);
    res = (wai == ICM_WHO_AM_I) ? GB_OK : GB_MPU_NOT_FOUND;
error_exit:
    return res;
}

/**
 * @brief Returns the value from WHO_AM_I register.
 */
uint8_t whoAmI(struct imu *imu)
{
    CHK_VAL(setBank(imu, 0));

    CHK_VAL(imu->readByte(imu, UB0_REG_WHO_AM_I, imu->buffer));
    return imu->buffer[0];
}

static GB_RESULT setFilters(struct imu *imu, bool gyroFilters, bool accFilters)
{
    GB_RESULT res = GB_OK;

    CHK_RES(setBank(imu, 1));

    if (gyroFilters == true) {
        CHK_RES(imu->writeByte(imu, UB1_REG_GYRO_CONFIG_STATIC2, GYRO_NF_ENABLE | GYRO_AAF_ENABLE));
    } else {
        CHK_RES(imu->writeByte(imu, UB1_REG_GYRO_CONFIG_STATIC2, GYRO_NF_DISABLE | GYRO_AAF_DISABLE));
    }

    CHK_RES(setBank(imu, 2));

    if (accFilters == true) {
        CHK_RES(imu->writeByte(imu, UB2_REG_ACCEL_CONFIG_STATIC2, ACCEL_AAF_ENABLE));
    } else {
        CHK_RES(imu->writeByte(imu, UB2_REG_ACCEL_CONFIG_STATIC2, ACCEL_AAF_DISABLE));
    }

    CHK_RES(setBank(imu, 0));

error_exit:
    return res;
}

/**
 * @brief Set sample rate of data output.
 *
 * Sample rate controls sensor data output rate and FIFO sample rate.
 * */
static GB_RESULT setSampleRate(struct imu *imu, uint16_t rate)
{
    GB_RESULT res = GB_OK;
    uint8_t gyro_reg, accel_reg;

    CHK_RES(setBank(imu, 0));
    CHK_RES(imu->readByte(imu, UB0_REG_GYRO_CONFIG0, &gyro_reg));
    gyro_reg = (rate >> 8) | (gyro_reg & 0xF0);
    CHK_RES(imu->writeByte(imu, UB0_REG_GYRO_CONFIG0, gyro_reg));

    CHK_RES(imu->readByte(imu, UB0_REG_ACCEL_CONFIG0, &accel_reg));
    accel_reg = rate | (accel_reg & 0xF0);
    CHK_RES(imu->writeByte(imu, UB0_REG_ACCEL_CONFIG0, accel_reg));

error_exit:
    return res;
}

/**
 * @brief Retrieve sample rate divider and calculate the actual rate.
 */
uint16_t getSampleRate(struct imu *imu)
{
    uint8_t gyro_reg, accel_reg;

    CHK_VAL(setBank(imu, 0));
    CHK_VAL(imu->readByte(imu, UB0_REG_GYRO_CONFIG0, &gyro_reg));
    CHK_VAL(imu->readByte(imu, UB0_REG_ACCEL_CONFIG0, &accel_reg));

    return (gyro_reg << 8) | accel_reg;
}

/**
 * @brief Configures Digital Low Pass Filter (DLPF) setting for both the gyroscope and accelerometer.
 * @param dlpf digital low-pass filter value
 */
static GB_RESULT setDigitalLowPassFilter(struct imu *imu, dlpf_t dlpf)
{
    GB_RESULT res = GB_OK;

    CHK_RES(setBank(imu, 1));
    // Configure gyro and accel Anti-Alias Filter (see section 5.3 "ANTI-ALIAS FILTER")
    CHK_RES(imu->writeByte(imu, UB1_REG_GYRO_CONFIG_STATIC3, imuAAFConfig[dlpf].delt));
    CHK_RES(imu->writeByte(imu, UB1_REG_GYRO_CONFIG_STATIC4, imuAAFConfig[dlpf].deltSqr & 0xFF));
    CHK_RES(imu->writeByte(imu, UB1_REG_GYRO_CONFIG_STATIC5, (imuAAFConfig[dlpf].deltSqr >> 8) | (imuAAFConfig[dlpf].bitshift << 4)));

    CHK_RES(setBank(imu, 2));
    CHK_RES(imu->writeByte(imu, UB2_REG_ACCEL_CONFIG_STATIC2, imuAAFConfig[dlpf].delt << 1)) ;
    CHK_RES(imu->writeByte(imu, UB2_REG_ACCEL_CONFIG_STATIC3, imuAAFConfig[dlpf].deltSqr & 0xFF)) ;
    CHK_RES(imu->writeByte(imu, UB2_REG_ACCEL_CONFIG_STATIC4, (imuAAFConfig[dlpf].deltSqr >> 8) | (imuAAFConfig[dlpf].bitshift << 4)));

    // Configure gyro and acc UI Filters
    CHK_RES(setBank(imu, 0));
    CHK_RES(imu->writeByte(imu, UB0_REG_GYRO_ACCEL_CONFIG0, UB0_REG_ACCEL_UI_FILT_BW_LOW_LATENCY | UB0_REG_GYRO_UI_FILT_BW_LOW_LATENCY));

error_exit:
    return res;
}

/**
 * @brief Return Digital Low Pass Filter configuration
 */
dlpf_t getDigitalLowPassFilter(struct imu *imu)
{
    return (dlpf_t) 0;
}

/**
 * @brief Select Gyroscope Full-scale range.
 * */
static GB_RESULT setGyroFullScale(struct imu *imu, gyro_fs_t fsr)
{
    GB_RESULT res = GB_OK;
    uint8_t reg;

    CHK_RES(setBank(imu, 0));
    CHK_RES(imu->readByte(imu, UB0_REG_GYRO_CONFIG0, &reg));
    // only change FS_SEL in reg
    reg = (fsr << 5) | (reg & 0x1F);
    CHK_RES(imu->writeByte(imu, UB0_REG_GYRO_CONFIG0, reg));

error_exit:
    return res;
}

/**
 * @brief Return Gyroscope Full-scale range.
 */
gyro_fs_t getGyroFullScale(struct imu *imu)
{
    uint8_t reg;

    CHK_VAL(setBank(imu, 0));
    CHK_VAL(imu->readByte(imu, UB0_REG_GYRO_CONFIG0, &reg));
    return (reg & 0xE0) >> 5;
}

/**
 * @brief Select Accelerometer Full-scale range.
 * */
static GB_RESULT setAccelFullScale(struct imu *imu, accel_fs_t fsr)
{
    GB_RESULT res = GB_OK;
    uint8_t reg;

    CHK_RES(setBank(imu, 0));
    CHK_RES(imu->readByte(imu, UB0_REG_ACCEL_CONFIG0, &reg));
    // only change FS_SEL in reg
    reg = (fsr << 5) | (reg & 0x1F);
    CHK_RES(imu->writeByte(imu, UB0_REG_ACCEL_CONFIG0, reg));

error_exit:
    return res;
}

/**
 * @brief Return Accelerometer Full-scale range.
 */
accel_fs_t getAccelFullScale(struct imu *imu)
{
    uint8_t reg;

    CHK_VAL(setBank(imu, 0));
    CHK_VAL(imu->readByte(imu, UB0_REG_ACCEL_CONFIG0, &reg));

    return (reg & 0xE0) >> 5;
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
    uint8_t reg1, reg2;

    CHK_RES(setBank(imu, 4));

    // Gyro X
    reg1 = bias.data.x & 0x00FF;
    CHK_RES(imu->readByte(imu, UB4_REG_OFFSET_USER1, &reg2));
    reg2 = (reg2 & 0xF0) | ((bias.data.x & 0x0F00) >> 8);
    CHK_RES(imu->writeByte(imu, UB4_REG_OFFSET_USER0, reg1));
    CHK_RES(imu->writeByte(imu, UB4_REG_OFFSET_USER1, reg2));

    // Gyro Y
    reg1 = bias.data.y & 0x00FF;
    CHK_RES(imu->readByte(imu, UB4_REG_OFFSET_USER1, &reg2));
    reg2 = (reg2 & 0x0F) | ((bias.data.y & 0x0F00) >> 4);
    reg2 = (bias.data.y & 0x0F00) >> 4 | (reg2 & 0x0F00) >> 4;
    CHK_RES(imu->writeByte(imu, UB4_REG_OFFSET_USER2, reg1));
    CHK_RES(imu->writeByte(imu, UB4_REG_OFFSET_USER1, reg2));

    // Gyro Z
    reg1 = bias.data.z & 0x00FF;
    CHK_RES(imu->readByte(imu, UB4_REG_OFFSET_USER4, &reg2));
    reg2 = (reg2 & 0xF0) | ((bias.data.z & 0x0F00) >> 8);
    CHK_RES(imu->writeByte(imu, UB4_REG_OFFSET_USER3, reg1));
    CHK_RES(imu->writeByte(imu, UB4_REG_OFFSET_USER4, reg2));

    CHK_RES(setBank(imu, 0));

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
    uint8_t reg1, reg2;
    raw_axes_t bias;

    CHK_VAL(setBank(imu, 4));

    // Gyro X
    CHK_VAL(imu->readByte(imu, UB4_REG_OFFSET_USER0, &reg1));
    CHK_VAL(imu->readByte(imu, UB4_REG_OFFSET_USER1, &reg2));
    bias.data.x = (reg2 & 0x0F) << 4 | reg1;

    // Gyro Y
    CHK_VAL(imu->readByte(imu, UB4_REG_OFFSET_USER2, &reg1));
    CHK_VAL(imu->readByte(imu, UB4_REG_OFFSET_USER1, &reg2));
    bias.data.y = (reg2 & 0xF0) << 4 | reg1;

    // Gyro Z
    CHK_VAL(imu->readByte(imu, UB4_REG_OFFSET_USER3, &reg1));
    CHK_VAL(imu->readByte(imu, UB4_REG_OFFSET_USER4, &reg2));
    bias.data.z = (reg2 & 0x0F) << 4 | reg1;

    CHK_VAL(setBank(imu, 0));

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
    GB_RESULT res = GB_OK;
    uint8_t reg1, reg2;

    CHK_RES(setBank(imu, 4));

    // Accel X
    reg1 = bias.data.x & 0x00FF;
    CHK_RES(imu->readByte(imu, UB4_REG_OFFSET_USER4, &reg2));
    reg2 = (reg2 & 0x0F) | ((bias.data.x & 0x0F00) >> 4);
    CHK_RES(imu->writeByte(imu, UB4_REG_OFFSET_USER5, reg1));
    CHK_RES(imu->writeByte(imu, UB4_REG_OFFSET_USER4, reg2));

    // Accel Y
    reg1 = bias.data.y & 0x00FF;
    CHK_RES(imu->readByte(imu, UB4_REG_OFFSET_USER7, &reg2));
    reg2 = (reg2 & 0xF0) | ((bias.data.y  & 0x0F00) >> 8);
    CHK_RES(imu->writeByte(imu, UB4_REG_OFFSET_USER6, reg1));
    CHK_RES(imu->writeByte(imu, UB4_REG_OFFSET_USER7, reg2));

    // Accel Z
    reg1 = bias.data.z & 0x00FF;
    CHK_RES(imu->readByte(imu, UB4_REG_OFFSET_USER7, &reg2));
    reg2 = (reg2 & 0x0F) | ((bias.data.z & 0x0F00) >> 4);
    CHK_RES(imu->writeByte(imu, UB4_REG_OFFSET_USER8, reg1));
    CHK_RES(imu->writeByte(imu, UB4_REG_OFFSET_USER7, reg2));

    CHK_RES(setBank(imu, 0));

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
    uint8_t reg1, reg2;
    raw_axes_t bias;

    CHK_VAL(setBank(imu, 4));

    // Accel X
    CHK_VAL(imu->readByte(imu, UB4_REG_OFFSET_USER0, &reg1));
    CHK_VAL(imu->readByte(imu, UB4_REG_OFFSET_USER1, &reg2));
    bias.data.x = (reg2 & 0x0F) << 4 | reg1;

    // Accel Y
    CHK_VAL(imu->readByte(imu, UB4_REG_OFFSET_USER2, &reg1));
    CHK_VAL(imu->readByte(imu, UB4_REG_OFFSET_USER1, &reg2));
    bias.data.y = (reg2 & 0xF0) << 4 | reg1;

    // Accel Z
    CHK_VAL(imu->readByte(imu, UB4_REG_OFFSET_USER3, &reg1));
    CHK_VAL(imu->readByte(imu, UB4_REG_OFFSET_USER4, &reg2));
    bias.data.z = (reg2 & 0x0F) << 4 | reg1;

    CHK_VAL(setBank(imu, 0));

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
        (*accel).xyz[i] = -((*accel).xyz[i] >> (3));
        //gyro bias / 4 (1000 / 250)
        (*gyro).xyz[i]  = -((*gyro).xyz[i] >> (2));
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
    CHK_RES(imu->readBytes(imu, UB0_REG_ACCEL_DATA_X1, 6, imu->buffer));
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
    CHK_RES(imu->readBytes(imu, UB0_REG_GYRO_DATA_X1, 6, imu->buffer));
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
    CHK_RES(imu->readBytes(imu, UB0_REG_TEMP_DATA1, 2, imu->buffer));
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
    CHK_RES(imu->readBytes(imu, UB0_REG_ACCEL_DATA_X1, 12, imu->buffer));
    accel->data.x = imu->buffer[0] << 8 | imu->buffer[1];
    accel->data.y = imu->buffer[2] << 8 | imu->buffer[3];
    accel->data.z = imu->buffer[4] << 8 | imu->buffer[5];
    gyro->data.x  = imu->buffer[6] << 8 | imu->buffer[7];
    gyro->data.y  = imu->buffer[8] << 8 | imu->buffer[9];
    gyro->data.z  = imu->buffer[10] << 8 | imu->buffer[11];
error_exit:
    return res;
}

/**
 * @brief Read data from all internal sensors.
 * */
static GB_RESULT sensors(struct imu *imu, raw_axes_t* accel, raw_axes_t* gyro, int16_t* temp)
{
    GB_RESULT res = GB_OK;
    CHK_RES(imu->readBytes(imu, UB0_REG_TEMP_DATA1, 14, imu->buffer));
    *temp    = imu->buffer[0] << 8 | imu->buffer[1];
    accel->data.x = imu->buffer[2] << 8 | imu->buffer[3];
    accel->data.y = imu->buffer[4] << 8 | imu->buffer[5];
    accel->data.z = imu->buffer[6] << 8 | imu->buffer[7];
    gyro->data.x  = imu->buffer[8] << 8 | imu->buffer[9];
    gyro->data.y  = imu->buffer[10] << 8 | imu->buffer[11];
    gyro->data.z  = imu->buffer[12] << 8 | imu->buffer[13];
error_exit:
    return res;
}

/**
 * @brief Configure the Interrupt pin (INT).
 * @param config configuration desired.
 */
static GB_RESULT setInterruptConfig(struct imu *imu, int_config_t config)
{
    GB_RESULT res = GB_OK;

    CHK_RES(imu->readByte(imu, UB0_REG_INT_CONFIG, imu->buffer));
    // zero the bits we're setting, but keep the others we're not setting as they are;
    const uint8_t INT_PIN_CONFIG_BITMASK = (1 << INT_CFG_INT2_MODE) | (1 << INT_CFG_INT2_DRIVE_CIRCUIT) |
                                            (1 << INT_CFG_INT2_POLARITY) | (1 << INT_CFG_INT1_MODE) |
                                            (1<< INT_CFG_INT1_DRIVE_CIRCUIT) | (1 << INT_CFG_INT1_POLARITY);
    imu->buffer[0] &= ~INT_PIN_CONFIG_BITMASK;
    imu->buffer[0] |= config.int2_mode << INT_CFG_INT2_MODE;
    imu->buffer[0] |= config.int2_drive << INT_CFG_INT2_DRIVE_CIRCUIT;
    imu->buffer[0] |= config.int2_level << INT_CFG_INT2_POLARITY;
    imu->buffer[0] |= config.int1_mode << INT_CFG_INT1_MODE;
    imu->buffer[0] |= config.int1_drive << INT_CFG_INT1_DRIVE_CIRCUIT;
    imu->buffer[0] |= config.int1_level << INT_CFG_INT1_POLARITY;

    CHK_RES(imu->writeByte(imu, UB0_REG_INT_CONFIG, imu->buffer[0]));

error_exit:
    return res;
}

/**
 * @brief Return Interrupt pin (INT) configuration.
 */
int_config_t getInterruptConfig(struct imu *imu)
{
    CHK_VAL(imu->readByte(imu, UB0_REG_INT_CONFIG, imu->buffer));

    int_config_t config;
    config.int2_mode = (int_mode_t)((imu->buffer[0] >> INT_CFG_INT2_MODE) & 0x1);
    config.int2_drive = (int_drive_t)((imu->buffer[0] >> INT_CFG_INT2_DRIVE_CIRCUIT) & 0x1);
    config.int2_level = (int_lvl_t)((imu->buffer[0] >> INT_CFG_INT2_POLARITY) & 0x1);
    config.int1_mode = (int_mode_t)((imu->buffer[0] >> INT_CFG_INT1_MODE) & 0x1);
    config.int1_drive = (int_drive_t)((imu->buffer[0] >> INT_CFG_INT1_DRIVE_CIRCUIT) & 0x1);
    config.int1_level = (int_lvl_t)((imu->buffer[0] >> INT_CFG_INT1_POLARITY) & 0x1);

    return config;
}

/**
 * @brief Enable features to generate signal at Interrupt pin
 * @param mask ORed features.
 */
static GB_RESULT setInterruptEnabled(struct imu *imu, int_en_t mask)
{
    GB_RESULT res = GB_OK;
    uint8_t reg;

    CHK_RES(imu->readByte(imu, UB0_REG_INT_CONFIG1, &reg));
    reg &= ~0x10;  // INT_ASYNC_RESET
    CHK_RES(imu->writeByte(imu, UB0_REG_INT_CONFIG1, reg));
    CHK_RES(imu->writeByte(imu, UB0_REG_INT_SOURCE0, 0x18)); // route UI data ready interrupt to INT1

error_exit:
    return res;
}

/**
 * @brief Return enabled features configured to generate signal at Interrupt pin.
 */
int_en_t getInterruptEnabled(struct imu *imu)
{
    CHK_VAL(imu->readByte(imu, UB0_REG_INT_CONFIG1, imu->buffer));
    return (int_en_t) imu->buffer[0];
}

/**
 * @brief Change FIFO mode.
 * */
static GB_RESULT setFIFOMode(struct imu *imu, fifo_mode_t mode)
{
    GB_RESULT res = GB_OK;

    CHK_RES(imu->writeByte(imu, UB0_REG_FIFO_CONFIG, mode));

error_exit:
    return res;
}

/**
 * @brief Return FIFO mode.
 */
fifo_mode_t getFIFOMode(struct imu *imu)
{
    CHK_VAL(imu->readByte(imu, UB0_REG_FIFO_CONFIG, imu->buffer));
    return (fifo_mode_t) imu->buffer[0];
}

/**
 * @brief Configure the sensors that will be written to the FIFO.
 * */
static GB_RESULT setFIFOConfig(struct imu *imu, fifo_config_t config)
{
    GB_RESULT res = GB_OK;

    CHK_RES(imu->writeByte(imu, UB0_REG_FIFO_CONFIG1, config));

error_exit:
    return res;
}

/**
 * @brief Return FIFO configuration.
 */
fifo_config_t getFIFOConfig(struct imu *imu)
{
    CHK_VAL(imu->readByte(imu, UB0_REG_FIFO_CONFIG1, imu->buffer));
    return (fifo_config_t)imu->buffer[0];
}

/**
 * @brief Return number of written bytes in the FIFO.
 * */
uint16_t getFIFOCount(struct imu *imu)
{
    CHK_VAL(imu->readByte(imu, UB0_REG_FIFO_COUNTH, imu->buffer));
    CHK_VAL(imu->readByte(imu, UB0_REG_FIFO_COUNTL, imu->buffer +1));
    uint16_t count = imu->buffer[0] << 8 | imu->buffer[1];
    return count;
}

/**
 * @brief Read data contained in FIFO imu->buffer.
 * */
static GB_RESULT readFIFO(struct imu *imu, size_t length, uint8_t* data)
{
    return imu->readBytes(imu, UB0_REG_FIFO_DATA, length, data);
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
    int32_t accelAvgx = 0, accelAvgy = 0, accelAvgz = 0;
    int32_t gyroAvgx  = 0, gyroAvgy  = 0, gyroAvgz  = 0;
    float_axes_t accelG;   // accel axes in (g) gravity format
    float_axes_t gyroDPS;  // gyro axes in (DPS) º/s format
    // backup previous configuration
    const uint16_t prevSampleRate      = imu->getSampleRate(imu);
    const accel_fs_t prevAccelFS       = imu->getAccelFullScale(imu);
    const gyro_fs_t prevGyroFS         = imu->getGyroFullScale(imu);

    // setup
    CHK_RES(imu->setSampleRate(imu, kSampleRate));
    CHK_RES(imu->setAccelFullScale(imu, accelFS));
    CHK_RES(imu->setGyroFullScale(imu, gyroFS));
    // wait for 200ms for sensors to stabilize
    GB_SleepMs(1000);

    raw_axes_t accelRaw;   // x, y, z axes as int16
    raw_axes_t gyroRaw;    // x, y, z axes as int16
    const int packetCount = 500;

    for (int i = 0; i < packetCount; i++) {
        GB_SleepMs(4);
        imu->motion(imu, &accelRaw, &gyroRaw);  // fetch raw data from the registers
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
    CHK_RES(imu->setAccelFullScale(imu, prevAccelFS));
    CHK_RES(imu->setGyroFullScale(imu, prevGyroFS));

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

static GB_RESULT baroInit(struct imu *imu)
{
    GB_RESULT res = GB_OK;

    CHK_RES(ms5611_init_desc(&(imu->baro_dev), (baro_bus_t*)&fspi, GB_SPI_DEV_1));
    CHK_RES(ms5611_init(&(imu->baro_dev), MS5611_OSR_4096));

    imu->mpu_status |= IMU_BARO_STATUS_BIT;
    GB_DEBUGI(SENSOR_TAG, "Aux Barometer Init done");
error_exit:
    return res;
}

static GB_RESULT baroGetData(struct imu *imu, baro_t *baro)
{
    GB_RESULT res = GB_OK;
    float temperature, pressure;

    if (!(imu->mpu_status & IMU_BARO_STATUS_BIT))
    {
        // Barometer not available
        goto error_exit;
    }
    CHK_RES(ms5611_get_sensor_data(&(imu->baro_dev), &pressure, &temperature));

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

    GB_DEBUGI(SENSOR_TAG, "DEBUG baro_data: [%+6.2fPa %+6.2fC %+6.2fcm ]\n", baro->pressure, baro->temperature, baro->altitude);
error_exit:
    return res;
}

static GB_RESULT compassInit(struct imu *imu)
{
    GB_RESULT res = GB_OK;
    uint8_t chip_id = 0;

    CHK_RES(i2c0.begin(&i2c0, COMPASS_I2C_SDA, COMPASS_I2C_SCL, COMPASS_I2C_CLOCK_SPEED));
    CHK_RES(i2c0.addDevice(&i2c0, QMC5883L_I2C_ADDR_DEF, COMPASS_I2C_CLOCK_SPEED));

    CHK_RES(qmc5883l_init_desc(&(imu->compass_dev), &i2c0, QMC5883L_I2C_ADDR_DEF));
    CHK_RES(qmc5883l_set_config(&(imu->compass_dev), QMC5883L_DR_200, QMC5883L_OSR_512, g_compass_fs));
    CHK_RES(qmc5883l_get_chip_id(&(imu->compass_dev), &chip_id));

    if (chip_id == QMC5883L_CHIP_ID)
    {
        imu->mpu_status |= IMU_MAG_STATUS_BIT;
        GB_DEBUGI(SENSOR_TAG, "QMC5883L Compass Chip Selected");
    }

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

    CHK_RES(qmc5883l_get_raw_data(&(imu->compass_dev), (qmc5883l_raw_data_t *)mag));

error_exit:
    return res;
}

inline uint8_t accelFSRvalue(const accel_fs_t fs)
{
    uint8_t acc_fs = 0;

    switch (fs) {
    case ACCEL_FS_16G:
        acc_fs = 16;
        break;
    case ACCEL_FS_8G:
        acc_fs = 8;
        break;
    case ACCEL_FS_4G:
        acc_fs = 4;
        break;
    case ACCEL_FS_2G:
        acc_fs = 2;
        break;
    default:
        ;
    }

    return acc_fs;
}

inline uint16_t gyroFSRvalue(const gyro_fs_t fs)
{
    float gyro_fs = 0;

    switch (fs) {
    case GYRO_FS_2000DPS:
        gyro_fs = 2000;
        break;
    case GYRO_FS_1000DPS:
        gyro_fs = 1000;
        break;
    case GYRO_FS_500DPS:
        gyro_fs = 500;
        break;
    case GYRO_FS_250DPS:
        gyro_fs = 250;
        break;
    case GYRO_FS_125DPS:
        gyro_fs = 125;
        break;
    case GYRO_FS_62_5DPS:
        gyro_fs = 62.5;
        break;
    case GYRO_FS_31_25DPS:
        gyro_fs = 31.25;
        break;
    case GYRO_FS_15_625DPS:
        gyro_fs = 15.625;
        break;
    default:
        ;
    }
    return gyro_fs;
}

inline uint16_t accelSensitivity(const accel_fs_t fs)
{
    return 0;
}

inline float gyroSensitivity(const gyro_fs_t fs)
{
    return 0;
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
    return (fs == QMC5883L_RNG_2 ? 2000.0 : 8000.0) / 32768;;
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
    return 0;
}

inline float tempFahrenheit(const int16_t temp)
{
    return 0;
}
