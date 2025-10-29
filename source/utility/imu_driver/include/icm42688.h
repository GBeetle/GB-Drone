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

#ifndef _ICM42688_REGISTERS_H_
#define _ICM42688_REGISTERS_H_

#include <stdint.h>

#define IMU_SAMPLE_RATE 1000

// Accesible from all user banks
#define REG_BANK_SEL   0x76

// User Bank 0
#define UB0_REG_DEVICE_CONFIG   0x11
// break
#define UB0_REG_DRIVE_CONFIG   0x13

#define UB0_REG_INT_CONFIG           0x14
#define INT_CFG_INT2_MODE            (5)
#define INT_CFG_INT2_DRIVE_CIRCUIT   (4)
#define INT_CFG_INT2_POLARITY	     (3)
#define INT_CFG_INT1_MODE            (2)
#define INT_CFG_INT1_DRIVE_CIRCUIT   (1)
#define INT_CFG_INT1_POLARITY        (0)

// break
#define UB0_REG_FIFO_CONFIG   0x16
// break
#define UB0_REG_TEMP_DATA1      0x1D
#define UB0_REG_TEMP_DATA0      0x1E
#define UB0_REG_ACCEL_DATA_X1   0x1F
#define UB0_REG_ACCEL_DATA_X0   0x20
#define UB0_REG_ACCEL_DATA_Y1   0x21
#define UB0_REG_ACCEL_DATA_Y0   0x22
#define UB0_REG_ACCEL_DATA_Z1   0x23
#define UB0_REG_ACCEL_DATA_Z0   0x24
#define UB0_REG_GYRO_DATA_X1    0x25
#define UB0_REG_GYRO_DATA_X0    0x26
#define UB0_REG_GYRO_DATA_Y1    0x27
#define UB0_REG_GYRO_DATA_Y0    0x28
#define UB0_REG_GYRO_DATA_Z1    0x29
#define UB0_REG_GYRO_DATA_Z0    0x2A
#define UB0_REG_TMST_FSYNCH     0x2B
#define UB0_REG_TMST_FSYNCL     0x2C
#define UB0_REG_INT_STATUS      0x2D
#define UB0_REG_FIFO_COUNTH     0x2E
#define UB0_REG_FIFO_COUNTL     0x2F
#define UB0_REG_FIFO_DATA       0x30
#define UB0_REG_APEX_DATA0      0x31
#define UB0_REG_APEX_DATA1      0x32
#define UB0_REG_APEX_DATA2      0x33
#define UB0_REG_APEX_DATA3      0x34
#define UB0_REG_APEX_DATA4      0x35
#define UB0_REG_APEX_DATA5      0x36
#define UB0_REG_INT_STATUS2     0x37
#define UB0_REG_INT_STATUS3     0x38
// break
#define UB0_REG_SIGNAL_PATH_RESET    0x4B
#define UB0_REG_INTF_CONFIG0         0x4C
#define UB0_REG_INTF_CONFIG1         0x4D
#define UB0_REG_PWR_MGMT0            0x4E
#define UB0_REG_GYRO_CONFIG0         0x4F
#define UB0_REG_ACCEL_CONFIG0        0x50
#define UB0_REG_GYRO_CONFIG1         0x51
#define UB0_REG_GYRO_ACCEL_CONFIG0   0x52
#define UB0_REG_ACCEFL_CONFIG1       0x53
#define UB0_REG_TMST_CONFIG          0x54
// break
#define UB0_REG_APEX_CONFIG0   0x56
#define UB0_REG_SMD_CONFIG     0x57

// break
#define UB0_REG_FIFO_CONFIG1        0x5F
#define FIFO_CFG_RESUME_PARTIAL_RD	(6)
#define FIFO_CFG_WM_GT_TH	        (5)
#define FIFO_CFG_HIRES_EN	        (4)
#define FIFO_CFG_TMST_FSYNC_EN	    (3)
#define FIFO_CFG_TEMP_EN            (2)
#define FIFO_CFG_GYRO_EN            (1)
#define FIFO_CFG_ACCEL_EN           (0)

#define UB0_REG_FIFO_CONFIG2   0x60
#define UB0_REG_FIFO_CONFIG3   0x61
#define UB0_REG_FSYNC_CONFIG   0x62
#define UB0_REG_INT_CONFIG0    0x63
#define UB0_REG_INT_CONFIG1    0x64
#define UB0_REG_INT_SOURCE0    0x65
#define UB0_REG_INT_SOURCE1    0x66
// break
#define UB0_REG_INT_SOURCE3   0x68
#define UB0_REG_INT_SOURCE4   0x69
// break
#define UB0_REG_FIFO_LOST_PKT0   0x6C
#define UB0_REG_FIFO_LOST_PKT1   0x6D
// break
#define UB0_REG_SELF_TEST_CONFIG   0x70
// break
#define UB0_REG_WHO_AM_I   0x75

// User Bank 1
#define UB1_REG_SENSOR_CONFIG0   0x03
// break
#define UB1_REG_GYRO_CONFIG_STATIC2    0x0B
#define UB1_REG_GYRO_CONFIG_STATIC3    0x0C
#define UB1_REG_GYRO_CONFIG_STATIC4    0x0D
#define UB1_REG_GYRO_CONFIG_STATIC5    0x0E
#define UB1_REG_GYRO_CONFIG_STATIC6    0x0F
#define UB1_REG_GYRO_CONFIG_STATIC7    0x10
#define UB1_REG_GYRO_CONFIG_STATIC8    0x11
#define UB1_REG_GYRO_CONFIG_STATIC9    0x12
#define UB1_REG_GYRO_CONFIG_STATIC10   0x13
// break
#define UB1_REG_XG_ST_DATA   0x5F
#define UB1_REG_YG_ST_DATA   0x60
#define UB1_REG_ZG_ST_DATA   0x61
#define UB1_REG_TMSTVAL0     0x62
#define UB1_REG_TMSTVAL1     0x63
#define UB1_REG_TMSTVAL2     0x64
// break
#define UB1_REG_INTF_CONFIG4   0x7A
#define UB1_REG_INTF_CONFIG5   0x7B
#define UB1_REG_INTF_CONFIG6   0x7C

// User Bank 2
#define UB2_REG_ACCEL_CONFIG_STATIC2   0x03
#define UB2_REG_ACCEL_CONFIG_STATIC3   0x04
#define UB2_REG_ACCEL_CONFIG_STATIC4   0x05
// break
#define UB2_REG_XA_ST_DATA   0x3B
#define UB2_REG_YA_ST_DATA   0x3C
#define UB2_REG_ZA_ST_DATA   0x3D

// User Bank 4
#define UB4_REG_APEX_CONFIG1   0x40
#define UB4_REG_APEX_CONFIG2   0x41
#define UB4_REG_APEX_CONFIG3   0x42
#define UB4_REG_APEX_CONFIG4   0x43
#define UB4_REG_APEX_CONFIG5   0x44
#define UB4_REG_APEX_CONFIG6   0x45
#define UB4_REG_APEX_CONFIG7   0x46
#define UB4_REG_APEX_CONFIG8   0x47
#define UB4_REG_APEX_CONFIG9   0x48
// break
#define UB4_REG_ACCEL_WOM_X_THR   0x4A
#define UB4_REG_ACCEL_WOM_Y_THR   0x4B
#define UB4_REG_ACCEL_WOM_Z_THR   0x4C
#define UB4_REG_INT_SOURCE6       0x4D
#define UB4_REG_INT_SOURCE7       0x4E
#define UB4_REG_INT_SOURCE8       0x4F
#define UB4_REG_INT_SOURCE9       0x50
#define UB4_REG_INT_SOURCE10      0x51
// break
#define UB4_REG_OFFSET_USER0   0x77
#define UB4_REG_OFFSET_USER1   0x78
#define UB4_REG_OFFSET_USER2   0x79
#define UB4_REG_OFFSET_USER3   0x7A
#define UB4_REG_OFFSET_USER4   0x7B
#define UB4_REG_OFFSET_USER5   0x7C
#define UB4_REG_OFFSET_USER6   0x7D
#define UB4_REG_OFFSET_USER7   0x7E
#define UB4_REG_OFFSET_USER8   0x7F


#define ICM_WHO_AM_I         (0x47)
#define FIFO_EN              (0x5F)
#define FIFO_TEMP_EN         (0x04)
#define FIFO_GYRO            (0x02)
#define FIFO_ACCEL           (0x01)

// BANK 1
#define GYRO_NF_ENABLE       (0x00)
#define GYRO_NF_DISABLE      (0x01)
#define GYRO_AAF_ENABLE      (0x00)
#define GYRO_AAF_DISABLE     (0x02)

// BANK 2
#define ACCEL_AAF_ENABLE     (0x00)
#define ACCEL_AAF_DISABLE    (0x01)

typedef enum {
    first_order  = 0x00,
    second_order = 0x01,
    third_order  = 0x02,
} uifilt_ord;

/*! Gyroscope full-scale range */
typedef enum {
    GYRO_FS_2000DPS,
    GYRO_FS_1000DPS,
    GYRO_FS_500DPS,
    GYRO_FS_250DPS,
    GYRO_FS_125DPS,
    GYRO_FS_62_5DPS,
    GYRO_FS_31_25DPS,
    GYRO_FS_15_625DPS,
} gyro_fs_t;

/*! Accel full-scale range */
typedef enum {
    ACCEL_FS_16G,
    ACCEL_FS_8G,
    ACCEL_FS_4G,
    ACCEL_FS_2G,
} accel_fs_t;

/*! Digital low-pass filter (based on gyro bandwidth) */
typedef enum {
    DLPF_1449Hz  = 0,
    DLPF_680Hz   = 1,
    DLPF_329Hz   = 2,
    DLPF_162Hz   = 3,
    DLPF_80Hz    = 4,
    DLPF_40Hz    = 5,
    DLPF_20HZ    = 6,
    DLPF_10HZ    = 7,
} dlpf_t;

typedef enum {
    odr32k    = 0x01,  // LN mode only
    odr16k    = 0x02,  // LN mode only
    odr8k     = 0x03,  // LN mode only
    odr4k     = 0x04,  // LN mode only
    odr2k     = 0x05,  // LN mode only
    odr1k     = 0x06,  // LN mode only
    odr200    = 0x07,
    odr100    = 0x08,
    odr50     = 0x09,
    odr25     = 0x0A,
    odr12_5   = 0x0B,
    odr6a25   = 0x0C,  // LP mode only (accel only)
    odr3a125  = 0x0D,  // LP mode only (accel only)
    odr1a5625 = 0x0E,  // LP mode only (accel only)
    odr500    = 0x0F,
} odr_t;

/*! Interrupt active level */
typedef enum { INT_LVL_ACTIVE_HIGH = 0, INT_LVL_ACTIVE_LOW = 1} int_lvl_t;/*! Interrupt drive state */
typedef enum { INT_DRV_PUSHPULL = 0, INT_DRV_OPENDRAIN = 1 } int_drive_t;/*! Interrupt mode */
typedef enum { INT_MODE_PULSESOUS =0,INT_MODE_LATCH =1} int_mode_t;/*! Interrupt clear mode */
typedef enum { INT_CLEAR_STATUS_REG = 0, INT_CLEAR_ANYREAD = 1 } int_clear_t;/*! Interrupt configuration struct */

typedef struct {
    int_mode_t int2_mode : 1;
    int_drive_t int2_drive : 1;
    int_lvl_t int2_level : 1;
    int_mode_t int1_mode : 1;
    int_drive_t int1_drive : 1;
    int_lvl_t int1_level : 1;
}int_config_t;

typedef uint8_t int_en_t;
typedef uint8_t int_stat_t;

typedef uint8_t fifo_config_t;
static const fifo_config_t FIFO_CFG_NONE	    = (0x0);
static const fifo_config_t FIFO_CFG_GYRO	    = (1 << FIFO_CFG_GYRO_EN);
static const fifo_config_t FIFO_CFG_ACCEL	    = (1 << FIFO_CFG_ACCEL_EN);
static const fifo_config_t FIFO_CFG_TEMPERATURE = (1 << FIFO_CFG_TEMP_EN);

typedef uint8_t fifo_mode_t;
static const fifo_config_t FIFO_MODE_BYPASS      = (0x00);
static const fifo_config_t FIFO_MODE_STREAM2FIFO = (0x40);
static const fifo_config_t FIFO_MODE_STOPONFULL  = (0xC0);

#endif /* end of include guard: _ICM42688_REGISTERS_H_ */
