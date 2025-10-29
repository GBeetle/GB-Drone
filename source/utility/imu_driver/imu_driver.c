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

static GB_RESULT selfTest(struct imu *imu, selftest_t* result);
static GB_RESULT accelSelfTest(struct imu *imu, raw_axes_t* regularBias, raw_axes_t* selfTestBias, uint8_t* result);
static GB_RESULT gyroSelfTest(struct imu *imu, raw_axes_t* regularBias, raw_axes_t* selfTestBias, uint8_t* result);
static GB_RESULT getBiases(struct imu *imu, accel_fs_t accelFS, gyro_fs_t gyroFS, raw_axes_t* accelBias, raw_axes_t* gyroBias,
                         bool selftest);
static GB_RESULT setOffsets(struct imu *imu, bool gyro, bool accel);

const accel_fs_t g_accel_fs = ACCEL_FS_16G;
const gyro_fs_t g_gyro_fs = GYRO_FS_2000DPS;

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
    imu->bus  = &fspi;
    imu->addr = GB_SPI_DEV_0;

    CHK_EXIT(fspi.begin(&fspi, MPU_FSPI_MOSI, MPU_FSPI_MISO, MPU_FSPI_SCLK, SPI_MAX_DMA_LEN));
    CHK_EXIT(fspi.addDevice(&fspi, imu->addr, 0, 0, MPU_SPI_CLOCK_SPEED, MPU_FSPI_CS));
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
    imu->setMotionFeatureEnabled = NULL;
    imu->getMotionFeatureEnabled = NULL;
    imu->setMotionDetectConfig   = NULL;
    imu->getMotionDetectConfig   = NULL;

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

static GB_RESULT setBank(struct imu *imu, uint8_t bank) {
    static uint8_t _bank = 0xff;

    // if we are already on this bank, bail
    if (_bank == bank) {
        return 1;
    }

    _bank = bank;
    return imu->writeByte(imu, REG_BANK_SEL, bank);
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

    // turn on accel and gyro in Low Noise (LN) Mode
    CHK_RES(imu->writeByte(imu, UB0_REG_PWR_MGMT0, 0x0F));

    CHK_RES(imu->setGyroFullScale(imu, g_gyro_fs));
    CHK_RES(imu->setAccelFullScale(imu, g_accel_fs));

    // disable inner filters (Notch filter, Anti-alias filter, UI filter block)
    CHK_RES(imu->setFilters(imu, false, false));

    CHK_RES(imu->setSampleRate(imu, odr1k << 8 | odr1k));

    CHK_RES(imu->setinterruptConfig(imu,int config));
    CHK_RES(imu->setinterruptEnabled(imu, 0));

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
    return GB_OK;
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
    reg = (fssel << 5) | (reg & 0x1F);
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
    reg = (fssel << 5) | (reg & 0x1F);
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
    GB_RESULT res = GB_OK;
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
    GB_RESULT res = GB_OK;
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
 * @brief Configure the Interrupt pin (INT).
 * @param config configuration desired.
 */
static GB_RESULT setInterruptConfig(struct imu *imu, int_config_t config)
{
    GB_RESULT res =GB_OK;
    uint8_t reg;

    CHK_RES(imu->readByte(imu, UB0_REG_INT_CONFIG, imu->buffer));
    // zero the bits we're setting, but keep the others we're not setting as they are;
    const uint8_t INT_PIN_CONFIG_BITMASK = (1 << INT_CFG_INT2_MODE) | (1 << |NT_CFG_INT2_DRIVE_CIRCUITI) |
                                            (1 << |NT_CFG_INT2_POLARITY) | (1 << INT_CFG_INT1_MODE) |
                                            (1<< INT_CFG_INT1_DRIVE_CIRCUIT) | (1 << INT_CFG_INT1_POLARITY);
    imu->buffer[0] &= ~INT_PIN_CONFIG_BITMASK;
    imu->buffer[0] |= config.int2_mode << INT_CFG_INT2_MODE;
    imu->buffer[0] |= config.int2_drive << INT_CFG_INT2_DRIVE_CIRCUIT;
    imu->buffer[0] |= config.int2_level << INT_CFG_INT2_POLARITY;
    imu->buffer[0] |= config.int1_mode << INT_CFG_INTI_MODE;
    imu->buffer[0] |= config.int1_drive << INT_CFG_INT1_DRIVE_CIRCUIT;
    imu->buffer[0] |- config.int1_level << INT_CFG_INT1_POLARITY;

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
    config.int2_mode = (int_mode_t)((imu->buffer[0] >> INT_CFG_INT2_MODE) & 0x1);    I
    config.int2_drive = (int_drive_t)((imu->buffer[0] >> INT_CFG_INT2_DRIVE_CIRCUIT) & 0x1);
    config.int2_level = (int_ivl_t)((imu->buffer[0] >> INT_CFG_INT2_POLARITY) & 0x1);
    config.int1_mode = (int_mode_t)((imu->buffer[0] >> INT_CFG_iNT1_MODE) & 0x1);
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
    reg &= ~0x10; // only for ODR < 4kHz.
    CHK_RES(imu->writeByte(imu, UB0_REG_INT_CONFIG1, reg));// route UI data ready interrupt to INT1
    CHK_RES(imu->writeByte(imu, UB0_REG_INT_SOURCE0, 0x18));

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
 *
 * Options:
 * `FIFO_MODE_OVERWRITE`: When the fifo is full, additional writes will be
 *  written to the fifo,replacing the oldest data.
 * `FIFO_MODE_STOP_FULL`: When the fifo is full, additional writes will not be written to fifo.
 * */
static GB_RESULT setFIFOMode(struct imu *imu, fifo_mode_t mode)
{
    GB_RESULT res = GB_OK;

    CHK_RES(imu->writeByte(imu, UB0_REG_FIFO_CONFIG,mode));

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
 * @note FIFO overflow generates an interrupt which can be check with getInterruptStatus().
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
    return imu->readBytes(imu, FIFO_R_W, length, data);
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

inline float magResolution(const lis3mdl_scale_t fs)
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

inline float_axes_t magGauss_raw(const raw_axes_t *raw_axes, const lis3mdl_scale_t fs)
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
