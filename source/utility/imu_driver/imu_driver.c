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

static GB_RESULT setFilters(bool gyroFilters, bool accFilters);
static GB_RESULT setDigitalLowPassFilter(struct imu *imu, dlpf_t dlpf);
static dlpf_t getDigitalLowPassFilter(struct imu *imu);

static GB_RESULT setLowPowerAccelMode(struct imu *imu, bool enable);
static bool getLowPowerAccelMode(struct imu *imu);
static GB_RESULT setLowPowerAccelRate(struct imu *imu, lp_accel_rate_t rate);
static lp_accel_rate_t getLowPowerAccelRate(struct imu *imu);
static GB_RESULT setMotionFeatureEnabled(struct imu *imu, bool enable);
static bool getMotionFeatureEnabled(struct imu *imu);
static GB_RESULT setMotionDetectConfig(struct imu *imu, mot_config_t* config);
static mot_config_t getMotionDetectConfig(struct imu *imu);


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

static GB_RESULT registerDump(struct imu *imu, uint8_t start, uint8_t end);

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
    imu->setLowPowerAccelMode    = &setLowPowerAccelMode;
    imu->getLowPowerAccelMode    = &getLowPowerAccelMode;
    imu->setLowPowerAccelRate    = &setLowPowerAccelRate;
    imu->getLowPowerAccelRate    = &getLowPowerAccelRate;
    imu->setMotionFeatureEnabled = &setMotionFeatureEnabled;
    imu->getMotionFeatureEnabled = &getMotionFeatureEnabled;
    imu->setMotionDetectConfig   = &setMotionDetectConfig;
    imu->getMotionDetectConfig   = &getMotionDetectConfig;

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

    imu->registerDump          = &registerDump;

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

    // reset device (wait a little to clear all registers)
    CHK_RES(imu->reset(imu));
    CHK_RES(imu->testConnection(imu));

    // turn on accel and gyro in Low Noise (LN) Mode
    CHK_RES(imu->writeByte(imu, UB0_REG_PWR_MGMT0, 0x0F));

    CHK_RES(imu->setGyroFullScale(imu, g_gyro_fs));
    CHK_RES(imu->setAccelFullScale(imu, g_accel_fs));

    // disable inner filters (Notch filter, Anti-alias filter, UI filter block)
    CHK_RES(imu->setFilters(false, false));

	// estimate gyro bias
	if (calibrateGyro() < 0) {
		return -8;
	}

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
    setBank(imu, 0);

    imu->err = imu->readByte(imu, UB0_REG_WHO_AM_I, imu->buffer);
    return imu->buffer[0];
}

static GB_RESULT setFilters(bool gyroFilters, bool accFilters)
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

    setBank(imu, 0);
    CHK_RES(imu->readByte(imu, UB0_REG_GYRO_CONFIG0, &reg));
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

    setBank(imu, 0);
    CHK_RES(imu->readByte(imu, UB0_REG_ACCEL_CONFIG0, &reg));

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
