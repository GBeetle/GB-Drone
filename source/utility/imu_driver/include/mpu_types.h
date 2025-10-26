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

#ifndef _MPU_TYPES_H_
#define _MPU_TYPES_H_

#include <stdint.h>
#include <stdbool.h>
#include "mpu_registers.h"
#include "sdkconfig.h"
#include "lis3mdl.h"

#define IMU_SAMPLE_RATE 300

#if defined CONFIG_MPU6500
#define kRoomTempOffset 0        // LSB
#define kCelsiusOffset    21.f    // ºC
#define kTempSensitivity  333.87f  // LSB/ºC
#elif defined CONFIG_MPU6000 || defined CONFIG_MPU6050 || defined CONFIG_MPU9150
#define kRoomTempOffset -521   // LSB
#define kCelsiusOffset    35.f   // ºC
#define kTempSensitivity  340.f  // LSB/ºC
#endif

#define kTempResolution   (98.67f / INT16_MAX)
#define kFahrenheitOffset (kCelsiusOffset * 1.8f + 32)  // ºF

#if defined CONFIG_MPU6050
static const uint16_t SAMPLE_RATE_MAX = 8000;
#elif defined CONFIG_MPU6500
static const uint16_t SAMPLE_RATE_MAX = 32000;
#endif

/*! Gyroscope full-scale range 传感器坐标系上角速度 满量程32800*/
// 16bit 满量程65536LSB +-250(°/s) 500(°/s) / 65536LSB = 3.81mV/(°/s) 取倒数 131LSB/(°/s)
typedef enum {
    GYRO_FS_250DPS  = 0,  //!< +/- 250 º/s  -> 131 LSB/(º/s)
    GYRO_FS_500DPS  = 1,  //!< +/- 500 º/s  -> 65.5 LSB/(º/s)
    GYRO_FS_1000DPS = 2,  //!< +/- 1000 º/s -> 32.8 LSB/(º/s)
    GYRO_FS_2000DPS = 3   //!< +/- 2000 º/s -> 16.4 LSB/(º/s)
} gyro_fs_t;

/*! Accel full-scale range 传感器坐标系中重力场大小和方向 满量程32.768*/
typedef enum {
    ACCEL_FS_2G  = 0,  //!< +/- 2 g  -> 16.384 LSB/g
    ACCEL_FS_4G  = 1,  //!< +/- 4 g  -> 8.192 LSB/g
    ACCEL_FS_8G  = 2,  //!< +/- 8 g  -> 4.096 LSB/g
    ACCEL_FS_16G = 3   //!< +/- 16 g -> 2.048 LSB/g
} accel_fs_t;

/*! Digital low-pass filter (based on gyro bandwidth) */
typedef enum {
    DLPF_256HZ_NOLPF = 0,
    DLPF_188HZ       = 1,
    DLPF_98HZ        = 2,
    DLPF_42HZ        = 3,
    DLPF_20HZ        = 4,
    DLPF_10HZ        = 5,
    DLPF_5HZ         = 6,
#ifdef CONFIG_MPU6050
    DLPF_2100HZ_NOLPF = 7
#elif CONFIG_MPU6500
    DLPF_3600HZ_NOLPF     = 7
#endif
} dlpf_t;

/*! Clock Source */
typedef enum {
    CLOCK_INTERNAL = 0,  //!< Internal oscillator: 20MHz for MPU6500 and 8MHz for MPU6050
    CLOCK_PLL      = 3,  //!< Selects automatically best pll source (recommended)
#if defined CONFIG_MPU6050
    CLOCK_EXT32KHZ = 4,  //!< PLL with external 32.768kHz reference
    CLOCK_EXT19MHZ = 5,  //!< PLL with external 19.2MHz reference
#endif
    CLOCK_KEEP_RESET = 7  //!< Stops the clock and keeps timing generator in reset
} clock_src_t;

/*! Fchoice (Frequency choice maybe ?) [MPU6500 and MPU9250 only] */
typedef enum {  //
    FCHOICE_0 = 0,
    FCHOICE_1 = 1,
    FCHOICE_2 = 2,
    FCHOICE_3 = 3
} fchoice_t;

/*! Low-Power Accelerometer wake-up rates */
typedef enum {
#if defined CONFIG_MPU6000 || defined CONFIG_MPU6050 || defined CONFIG_MPU9150
    LP_ACCEL_RATE_1_25HZ = 0,
    LP_ACCEL_RATE_5HZ    = 1,
    LP_ACCEL_RATE_20HZ   = 2,
    LP_ACCEL_RATE_40HZ   = 3
#elif defined CONFIG_MPU6500
    LP_ACCEL_RATE_0_24HZ  = 0,
    LP_ACCEL_RATE_0_49HZ  = 1,
    LP_ACCEL_RATE_0_98HZ  = 2,
    LP_ACCEL_RATE_1_95HZ  = 3,
    LP_ACCEL_RATE_3_91HZ  = 4,
    LP_ACCEL_RATE_7_81HZ  = 5,
    LP_ACCEL_RATE_15_63HZ = 6,
    LP_ACCEL_RATE_31_25HZ = 7,
    LP_ACCEL_RATE_62_50HZ = 8,
    LP_ACCEL_RATE_125HZ   = 9,
    LP_ACCEL_RATE_250HZ   = 10,
    LP_ACCEL_RATE_500HZ   = 11
#endif
} lp_accel_rate_t;

/*! Accelerometer Digital High Pass Filter (only for motion detection modules) */
typedef enum {
    ACCEL_DHPF_RESET = 0,   /**< This effectively disables the high pass filter. This mode may be toggled to quickly
                             * settle the filter. */
    ACCEL_DHPF_5HZ    = 1,  //!< ON state, the high pass filter will pass signals above the cut off frequency.
    ACCEL_DHPF_2_5HZ  = 2,  //!< ON state, the high pass filter will pass signals above the cut off frequency.
    ACCEL_DHPF_1_25HZ = 3,  //!< ON state, the high pass filter will pass signals above the cut off frequency.
    ACCEL_DHPF_0_63HZ = 4,  //!< ON state, the high pass filter will pass signals above the cut off frequency.
    ACCEL_DHPF_HOLD   = 7,  /**< The filter holds the present sample. The output will be the difference between the
                             * input sample and the held sample. */
} accel_dhpf_t;

#if defined CONFIG_MPU6000 || defined CONFIG_MPU6050 || defined CONFIG_MPU9150
/*! Motion Detection counter decrement rate (Motion and FreeFall) */
typedef enum {
    MOT_COUNTER_RESET = 0,  //!< When set, any non-qualifying sample will reset the corresponding counter to 0
    MOT_COUNTER_DEC_1 = 1,  //!< Decrement counter in 1
    MOT_COUNTER_DEC_2 = 2,  //!< Decrement counter in 2
    MOT_COUNTER_DEC_4 = 3   //!< Decrement counter in 4
} mot_counter_t;
#endif

/*! Motion Detection configuration */
typedef struct
{
    uint8_t threshold; /**< Motion threshold in LSB.
                        * For MPU6000 / MPU6050 / MPU9150: 1LSB = 32mg, 255LSB = 8160mg.
                        * For MPU6500 / MPU9250: 1LSB = 4mg, 255LSB = 1020mg. */
#if defined CONFIG_MPU6000 || defined CONFIG_MPU6050 || defined CONFIG_MPU9150
    uint8_t time;               /**< Duration in milliseconds that the accel data must exceed
                                 * the threshold before motion is reported. MAX = 255ms. */
    uint8_t accel_on_delay : 2; /**< Specifies in milliseconds the additional power-on delay applied to accelerometer
                                 * data path modules. MAX = 3ms.
                                 * More: The signal path contains filters which must be flushed on wake-up with new
                                 * samples before the detection modules begin operations.
                                 * There is already a default built-in 4ms delay. */
    mot_counter_t counter : 2;  //!< Configures the detection counter decrement rate.
#endif
} mot_config_t;

#if defined CONFIG_MPU6000 || defined CONFIG_MPU6050 || defined CONFIG_MPU9150
/*! Zero-motion configuration */
typedef struct
{
    uint8_t threshold;  //!< Motion threshold in LSB. 1LSB = 1mg, 255LSB = 1020mg
    uint8_t time;       /**< Duration in milliseconds that the accel data must exceed
                         * the threshold before motion is reported. MAX = 255ms. */
} zrmot_config_t;
#endif

#if defined CONFIG_MPU6000 || defined CONFIG_MPU6050 || defined CONFIG_MPU9150
/*! Free-fall configuration */
typedef struct
{
    uint8_t threshold;          //!< Motion threshold in LSB. 1LSB = 1mg, 255LSB = 1020mg
    uint8_t time;               /*!< Duration in milliseconds that the accel data must exceed
                                 * the threshold before motion is reported. MAX = 255ms. */
    uint8_t accel_on_delay : 2; /**< Specifies in milliseconds the additional power-on delay applied to accelerometer
                                 * data path modules. MAX = 3ms.
                                 * More: The signal path contains filters which must be flushed on wake-up with new
                                 * samples before the detection modules begin operations.
                                 * There is already a default built-in 4ms delay. */
    mot_counter_t counter : 2;  //!< Configures the detection counter decrement rate.
} ff_config_t;
#endif

#if defined CONFIG_MPU6000 || defined CONFIG_MPU6050 || defined CONFIG_MPU9150
/*! Motion Detection Status (MPU6000, MPU6050, MPU9150) */
typedef uint8_t mot_stat_t;
static const mot_stat_t MOT_STAT_XNEG       = (1 << MOT_STATUS_X_NEG_BIT);
static const mot_stat_t MOT_STAT_XPOS       = (1 << MOT_STATUS_X_POS_BIT);
static const mot_stat_t MOT_STAT_YNEG       = (1 << MOT_STATUS_Y_NEG_BIT);
static const mot_stat_t MOT_STAT_YPOS       = (1 << MOT_STATUS_Y_POS_BIT);
static const mot_stat_t MOT_STAT_ZNEG       = (1 << MOT_STATUS_Z_NEG_BIT);
static const mot_stat_t MOT_STAT_ZPOS       = (1 << MOT_STATUS_Z_POS_BIT);
static const mot_stat_t MOT_STAT_ZEROMOTION = (1 << MOT_STATUS_ZRMOT_BIT);
#endif

/*! Standby Mode */
typedef uint8_t stby_en_t;
static const stby_en_t STBY_EN_NONE    = (0x0);
static const stby_en_t STBY_EN_ACCEL_X = (1 << PWR2_STBY_XA_BIT);
static const stby_en_t STBY_EN_ACCEL_Y = (1 << PWR2_STBY_YA_BIT);
static const stby_en_t STBY_EN_ACCEL_Z = (1 << PWR2_STBY_ZA_BIT);
static const stby_en_t STBY_EN_ACCEL   = (STBY_EN_ACCEL_X | STBY_EN_ACCEL_Y | STBY_EN_ACCEL_Z);
static const stby_en_t STBY_EN_GYRO_X  = (1 << PWR2_STBY_XG_BIT);
static const stby_en_t STBY_EN_GYRO_Y  = (1 << PWR2_STBY_YG_BIT);
static const stby_en_t STBY_EN_GYRO_Z  = (1 << PWR2_STBY_ZG_BIT);
static const stby_en_t STBY_EN_GYRO    = (STBY_EN_GYRO_X | STBY_EN_GYRO_Y | STBY_EN_GYRO_Z);
static const stby_en_t STBY_EN_TEMP    = (1 << 6);
/*! This is a low power mode that allows quick enabling of the gyros.
 * \note: When set, the gyro drive and pll circuitry are enabled, but the sense paths are disabled. */
static const stby_en_t STBY_EN_LOWPWR_GYRO_PLL_ON = (1 << 7);

/*! Auxiliary I2C Master clock speed */
typedef enum {
    AUXI2C_CLOCK_348KHZ = 0,
    AUXI2C_CLOCK_333KHZ = 1,
    AUXI2C_CLOCK_320KHZ = 2,
    AUXI2C_CLOCK_308KHZ = 3,
    AUXI2C_CLOCK_296KHZ = 4,
    AUXI2C_CLOCK_286KHZ = 5,
    AUXI2C_CLOCK_276KHZ = 6,
    AUXI2C_CLOCK_267KHZ = 7,
    AUXI2C_CLOCK_258KHZ = 8,
    AUXI2C_CLOCK_500KHZ = 9,
    AUXI2C_CLOCK_471KHZ = 10,
    AUXI2C_CLOCK_444KHZ = 11,
    AUXI2C_CLOCK_421KHZ = 12,
    AUXI2C_CLOCK_400KHZ = 13,
    AUXI2C_CLOCK_381KHZ = 14,
    AUXI2C_CLOCK_364KHZ = 15
} auxi2c_clock_t;

/*! Auxiliary I2C Master’s transition from one slave read to the next slave read */
typedef enum { AUXI2C_TRANS_RESTART = 0, AUXI2C_TRANS_STOP = 1 } auxi2c_trans_t;

/**
 * @brief Auxiliary I2C Slaves slots
 * @note For MPU9150 & MPU9250: \n
 *  The MPU uses SLAVE0 and SLAVE1 to read Compass data so do not use this slave slots when compass is enabled.
 * */
typedef enum { AUXI2C_SLAVE_0 = 0, AUXI2C_SLAVE_1 = 1, AUXI2C_SLAVE_2 = 2, AUXI2C_SLAVE_3 = 3 } auxi2c_slv_t;

/*! Auxiliary I2C operation */
typedef enum { AUXI2C_WRITE = 0, AUXI2C_READ = 1 } auxi2c_rw_t;

/**
 * @brief Auxiliary I2C, EOW = end of word, use for swap
 * @details This allows byte swapping of registers that are grouped starting at any address.
 * @note External sensor data typically comes in as groups of two bytes.
 *  This bit is used to determine if the groups are from the slave’s
 *  register address 0 and 1, 2 and 3, etc.., or if the groups are address 1 and 2, 3 and 4, etc..
 */
typedef enum {
    AUXI2C_EOW_ODD_NUM = 0, /**< Indicates slave register addresses 0 and 1 are grouped together
                             * (odd numbered register ends the word). */
    AUXI2C_EOW_EVEN_NUM = 1 /**< Indicates slave register addresses 1 and 2 are grouped together
                             * (even numbered register ends the word). */
} auxi2c_eow_t;

/*! Auxiliary I2C Master configuration struct */
typedef struct
{
    auxi2c_clock_t clock : 4;      //!< Clock signal speed
    bool multi_master_en : 1;      //!< Enable if there is an another master driving the bus too.
    uint8_t sample_delay : 5;      /**< Number of samples to delay on Aux i2c trasactions, max = 31,
                                    * formula: rate = (sample_rate / delay + 1).
                                    * (e.g. sample_delay = 4, aux i2c transaction will occour after 4 samples.
                                    * If sample rate is 1KHz, the Aux i2c transaction rate will be 1000 / (4 + 1) = 200 Hz)
                                    * set zero if no delay is needed, so the transaction rate will be the same as
                                    * sample rate. */
    bool shadow_delay_en : 1;      //!< Delays shadowing of external sensor data until all data has been received.
    bool wait_for_es : 1;          /**< Delays the data ready interrupt until external sensor data is loaded.
                                    * (if data ready interrupt is enabled). */
    auxi2c_trans_t transition : 1; /**< Transition condition from one slave read to the next slave read,
                                    * default is `restart`. */
} auxi2c_config_t;

/**
 * @brief Auxiliary I2C Slave configuration struct.
 * Note on SWAP: \n
 * Swap bytes may be needed when external data is in another order. \n
 * The option swap_en, swaps bytes when reading both the low and high byte of a word. \n
 *
 * For example, if `rxlength = 4`, and if `reg_addr = 0x1`, and group = `ODD_NUM`:
 *  1. The first byte read from address 0x1 will be stored at `EXT_SENS_DATA_00`;
 *  2. the second and third bytes will be read and swapped, so the data read from address 0x2
 *     will be stored at `EXT_SENS_DATA_02`, and the data read from address 0x3 will be stored at `EXT_SENS_DATA_03`;
 *  3. The last byte read from address 0x4 will be stored at `EXT_SENS_DATA_04`.
 *
 * Note: there is nothing to swap after reading the first byte if `reg_addr[bit 0] = 1`,
 * or if the last byte read has a register address `[bit 0] = 0`. The opposite is true for 'group' = `EVEN_NUM`.
 * */
typedef struct
{
    auxi2c_slv_t slave;  //!< Slave slot
    uint8_t addr : 7;    //!< Slave device address
    auxi2c_rw_t rw : 1;  //!< Read/write flag
    uint8_t reg_addr;    //!< Register address to read/write to
    bool reg_dis : 1;    /**< When set, the transaction does not write the register address, it will only read data,
                          * or write data. */
    bool sample_delay_en : 1;  //!< enable delay specifided in master config, sample_delay, for this slave in specific.
    union
    {
        // when reading
        struct
        {
            bool swap_en : 1;  //!< Enable swap of bytes when reading both the low and high byte of a word. See the
                               //!< note.
            auxi2c_eow_t end_of_word : 1; /**< Define at which register address a word ends, for swap low and high
                                           * bytes of the word (when swap enabled). */
            uint8_t rxlength : 4;         //!< Number of bytes to read, when set to read, max = 15.
        };
        // when writing
        uint8_t txdata;  //!< Data to transfer when slave is set to write.
    };
} auxi2c_slv_config_t;

/*! Auxiliary I2C master status register data */
typedef uint8_t auxi2c_stat_t;
static const auxi2c_stat_t AUXI2C_STAT_FSYNC     = (1 << I2CMST_STAT_PASS_THROUGH_BIT);
static const auxi2c_stat_t AUXI2C_STAT_LOST_ARB  = (1 << I2CMST_STAT_LOST_ARB_BIT);
static const auxi2c_stat_t AUXI2C_STAT_SLV4_DONE = (1 << I2CMST_STAT_SLV4_DONE_BIT);
static const auxi2c_stat_t AUXI2C_STAT_SLV4_NACK = (1 << I2CMST_STAT_SLV4_NACK_BIT);
static const auxi2c_stat_t AUXI2C_STAT_SLV3_NACK = (1 << I2CMST_STAT_SLV3_NACK_BIT);
static const auxi2c_stat_t AUXI2C_STAT_SLV2_NACK = (1 << I2CMST_STAT_SLV2_NACK_BIT);
static const auxi2c_stat_t AUXI2C_STAT_SLV1_NACK = (1 << I2CMST_STAT_SLV1_NACK_BIT);
static const auxi2c_stat_t AUXI2C_STAT_SLV0_NACK = (1 << I2CMST_STAT_SLV0_NACK_BIT);

#if defined CONFIG_MPU9150 || (defined CONFIG_MPU6050 && !defined CONFIG_MPU6000)
/*! Auxiliary I2C bus VDDIO level [MPU6050 / MPU9150 only] */
typedef enum { AUXVDDIO_LVL_VLOGIC = 0, AUXVDDIO_LVL_VDD = 1 } auxvddio_lvl_t;
#endif

/*! Interrupt active level */
typedef enum { INT_LVL_ACTIVE_HIGH = 0, INT_LVL_ACTIVE_LOW = 1 } int_lvl_t;

/*! Interrupt drive state */
typedef enum { INT_DRV_PUSHPULL = 0, INT_DRV_OPENDRAIN = 1 } int_drive_t;

/*! Interrupt mode */
typedef enum { INT_MODE_PULSE50US = 0, INT_MODE_LATCH = 1 } int_mode_t;

/*! Interrupt clear mode */
typedef enum { INT_CLEAR_STATUS_REG = 0, INT_CLEAR_ANYREAD = 1 } int_clear_t;

/*! Interrupt configuration struct */
typedef struct
{
    int_lvl_t level : 1;
    int_drive_t drive : 1;
    int_mode_t mode : 1;
    int_clear_t clear : 1;
} int_config_t;

/**
 * @brief Enable features to generate signal at Interrupt pin
 * @note Freefall and zero motion only available to MPU6000 / MPU6050 / MPU9150
 * */
typedef uint8_t int_en_t;
static const int_en_t INT_EN_NONE          = (0x0);
static const int_en_t INT_EN_MOTION_DETECT = (1 << INT_ENABLE_MOTION_BIT);
static const int_en_t INT_EN_FIFO_OVERFLOW = (1 << INT_ENABLE_FIFO_OFLOW_BIT);
static const int_en_t INT_EN_I2C_MST_FSYNC = (1 << INT_ENABLE_I2C_MST_FSYNC_BIT);  // int from I2C_MST_STATUS
static const int_en_t INT_EN_PLL_READY     = (1 << INT_ENABLE_PLL_RDY_BIT);
static const int_en_t INT_EN_DMP_READY     = (1 << INT_ENABLE_DMP_RDY_BIT);
static const int_en_t INT_EN_RAWDATA_READY = (1 << INT_ENABLE_RAW_DATA_RDY_BIT);
#if defined CONFIG_MPU6000 || defined CONFIG_MPU6050 || defined CONFIG_MPU9150
static const int_en_t INT_EN_FREE_FALL   = (1 << INT_ENABLE_FREEFALL_BIT);
static const int_en_t INT_EN_ZERO_MOTION = (1 << INT_ENABLE_ZEROMOT_BIT);
#endif

/**
 * @brief Interrupt Status
 * @note Freefall and zero motion only available to MPU6000 / MPU6050 / MPU9150
 * */
typedef uint8_t int_stat_t;
static const int_stat_t INT_STAT_MOTION_DETECT = (1 << INT_STATUS_MOTION_BIT);
static const int_stat_t INT_STAT_FIFO_OVERFLOW = (1 << INT_STATUS_FIFO_OFLOW_BIT);
static const int_stat_t INT_STAT_I2C_MST_FSYNC = (1 << INT_STATUS_I2C_MST_BIT);  // int from I2C_MST_STATUS
static const int_stat_t INT_STAT_PLL_READY     = (1 << INT_STATUS_PLL_RDY_BIT);
static const int_stat_t INT_STAT_DMP_READY     = (1 << INT_STATUS_DMP_RDY_BIT);
static const int_stat_t INT_STAT_RAWDATA_READY = (1 << INT_STATUS_RAW_DATA_RDY_BIT);
#if defined CONFIG_MPU6000 || defined CONFIG_MPU6050 || defined CONFIG_MPU9150
static const int_stat_t INT_STAT_FREE_FALL   = (1 << INT_STATUS_FREEFALL_BIT);
static const int_stat_t INT_STAT_ZERO_MOTION = (1 << INT_STATUS_ZEROMOT_BIT);
#endif

#if defined CONFIG_MPU6500
/*! MPU6500 Fifo size */
typedef enum { FIFO_SIZE_512B = 0, FIFO_SIZE_1K = 1, FIFO_SIZE_2K = 2, FIFO_SIZE_4K = 3 } fifo_size_t;
#endif

/*! DMP Interrupt mode */
typedef enum { DMP_INT_MODE_PACKET = 0, DMP_INT_MODE_GESTURE = 1 } dmp_int_mode_t;

/*! FIFO mode */
typedef enum {
    FIFO_MODE_OVERWRITE = 0,  //!< when fifo full, additional writes will be written to fifo, replacing the oldest data.
    FIFO_MODE_STOP_FULL = 1   //!< when fifo full, additional writes will not be written to fifo.
} fifo_mode_t;

/*! FIFO configuration, enable sensors to be written to FIFO */
typedef uint16_t fifo_config_t;
static const fifo_config_t FIFO_CFG_NONE = (0x0);
static const fifo_config_t FIFO_CFG_GYRO =
    (1 << FIFO_XGYRO_EN_BIT | 1 << FIFO_YGYRO_EN_BIT | 1 << FIFO_ZGYRO_EN_BIT);
static const fifo_config_t FIFO_CFG_ACCEL       = (1 << FIFO_ACCEL_EN_BIT);
static const fifo_config_t FIFO_CFG_TEMPERATURE = (1 << FIFO_TEMP_EN_BIT);
static const fifo_config_t FIFO_CFG_SLAVE0      = (1 << FIFO_SLV_0_EN_BIT);
static const fifo_config_t FIFO_CFG_SLAVE1      = (1 << FIFO_SLV_1_EN_BIT);
static const fifo_config_t FIFO_CFG_SLAVE2      = (1 << FIFO_SLV_2_EN_BIT);
static const fifo_config_t FIFO_CFG_SLAVE3      = (1 << 8);
#if defined AK89xx
static const fifo_config_t FIFO_CFG_COMPASS = (FIFO_CFG_SLAVE0);  // 8 bytes
#endif

// Enable DMP features
/* @note DMP_FEATURE_LP_QUAT and DMP_FEATURE_6X_LP_QUAT are mutually exclusive.
 * @note DMP_FEATURE_SEND_RAW_GYRO and DMP_FEATURE_SEND_CAL_GYRO are also
 * mutually exclusive.
 * @note DMP_FEATURE_PEDOMETER is always enabled.
 */

/* typedef uint16_t dmp_features_t;
static const dmp_features_t DMP_FEATURE_TAP =            {0x001};
static const dmp_features_t DMP_FEATURE_ANDROID_ORIENT = {0x002};
static const dmp_features_t DMP_FEATURE_LP_QUAT =        {0x004};
static const dmp_features_t DMP_FEATURE_PEDOMETER =      {0x008};
static const dmp_features_t DMP_FEATURE_6X_LP_QUAT =     {0x010};
static const dmp_features_t DMP_FEATURE_GYRO_CAL =       {0x020};
static const dmp_features_t DMP_FEATURE_SEND_RAW_ACCEL = {0x040};
static const dmp_features_t DMP_FEATURE_SEND_RAW_GYRO =  {0x080};
static const dmp_features_t DMP_FEATURE_SEND_CAL_GYRO =  {0x100};

// DMP Tap axes
typedef uint8_t dmp_tap_axis_t;
static const dmp_tap_axis_t DMP_TAP_X       {0x30};
static const dmp_tap_axis_t DMP_TAP_Y       {0x0C};
static const dmp_tap_axis_t DMP_TAP_Z       {0x03};
static const dmp_tap_axis_t DMP_TAP_XYZ     {0x3F};
 */

// ============
// MAGNETOMETER
// ============
#if defined AK89xx
static const uint8_t COMPASS_I2CADDRESS      = 0xC;
static const uint8_t COMPASS_SAMPLE_RATE_MAX = 100;  // 100 Hz

/*! Magnetometer operation modes */
typedef enum {
    MAG_MODE_POWER_DOWN     = 0x0,
    MAG_MODE_SINGLE_MEASURE = 0x1,
    MAG_MODE_SELF_TEST      = 0x8,
    MAG_MODE_FUSE_ROM       = 0xF,
#ifdef AK8963
    MAG_MODE_CONTINUOUS_8HZ   = 0x2,  //!< @warning Not yet supported.
    MAG_MODE_CONTINUOUS_100HZ = 0x6,  //!< @warning Not yet supported.
    MAG_MODE_EXTERNAL_TRIGGER = 0x4   //!< @warning Not supported.
#endif
} mag_mode_t;

/*! Magnetometer sensor status 1 */
typedef uint8_t mag_stat1_t;
static const mag_stat1_t MAG_STAT1_DATA_RDY = {1 << STATUS1_DATA_RDY_BIT};
#ifdef AK8963
static const mag_stat1_t MAG_STAT1_DATA_OVERRUN = {1 << STATUS1_DATA_OVERRUN_BIT};
#endif

/*! Magnetometer sensor status 2 */
typedef uint8_t mag_stat2_t;
static const mag_stat2_t MAG_STAT2_SENSOR_OVERFLOW = {1 << STATUS2_OVERFLOW_BIT};
#ifdef AK8963
static const mag_stat2_t MAG_STAT2_BIT_OUTPUT_SETTING = {1 << STATUS2_BIT_OUTPUT_M_BIT};
#elif CONFIG_MPU9150
static const mag_stat2_t MAG_STAT2_DATA_ERROR{1 << STATUS2_DATA_ERROR_BIT};
#endif

#ifdef AK8963
/*! Magnetometer sensitivity */
typedef enum {
    MAG_SENSITIVITY_0_6_uT  = 0,  //!< 0.6  uT/LSB  =  14-bit output
    MAG_SENSITIVITY_0_15_uT = 1,  //!< 0.15 uT/LSB  =  16-bit output
} mag_sensy_t;
#endif

// Auxiliary I2C slaves that operate the Magnetometer (do not change)
static const auxi2c_slv_t MAG_SLAVE_READ_DATA  = AUXI2C_SLAVE_0;  // read measurement data
static const auxi2c_slv_t BMP_SLAVE_READ_DATA  = AUXI2C_SLAVE_1;  // change mode to single measure

static const uint8_t MAG_DATA_LENGTH = 8;  // bytes

#elif defined CONFIG_AUX_LIS3MDL

#define COMPASS_I2CADDRESS       LIS3MDL_I2C_ADDRESS_2
#define COMPASS_SAMPLE_RATE_MAX  1000  // 1000 Hz

#define MAG_DATA_LENGTH  8  // bytes

#else // fake data, fix build error

#define COMPASS_I2CADDRESS       0x00

#endif  // end of Magnetometer stuff

#define MAG_SLAVE_READ_DATA   AUXI2C_SLAVE_0  // read measurement data
#define BMP_SLAVE_READ_DATA   AUXI2C_SLAVE_1  // change mode to single measure

/*! Self-Test results */
typedef uint8_t selftest_t;
static const selftest_t SELF_TEST_PASS = 0x0;
static const selftest_t SELF_TEST_GYRO_FAIL = 1 << 0;   // 0x1
static const selftest_t SELF_TEST_ACCEL_FAIL = 1 << 1;  // 0x2

#endif /* end of include guard: _MPU_TYPES_HPP_ */
