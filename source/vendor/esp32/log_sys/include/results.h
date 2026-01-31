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

#ifndef _GB_RESULTS__
#define _GB_RESULTS__

#include <stddef.h>
#include <inttypes.h>

typedef uint32_t GB_RESULT;

#define GB_OK                   0x00000000
#define GB_FAIL                 0x80000000
#define GB_INVALID_ARGUMENT     0x80000001

#define GB_CHK_BOOL_FAIL        0x80000500

#define GB_SPI_RW_FAIL          0x80000001
#define GB_SPI_INI_FAIL         0x80000002
#define GB_SPI_FREE_FAIL        0x80000003
#define GB_SPI_CFG_FAIL         0x80000004
#define GB_SPI_RMV_FAIL         0x80000005
#define GB_SPI_INVALID_SIZE     0x80000006
#define GB_SPI_BUF_TOO_SMALL    0x80000007

#define GB_I2C_RW_FAIL               0x80000010
#define GB_I2C_CFG_FAIL              0x80000011
#define GB_I2C_INS_FAIL              0x80000012
#define GB_I2C_RMV_FAIL              0x80000013
#define GB_I2C_CONNECT_FAIL          0x80000014
#define GB_I2C_MAX_DEVICE_REACHED    0x80000015

#define GB_MPU_NOT_FOUND             0X80000100
#define GB_MPU_DUMP_REG_FAIL         0X80000101
#define GB_MPU_RW_TEST_FAIL          0X80000102
#define GB_MPU_AUX_RW_FAIL           0x80000103
#define GB_MPU_AUX_NOT_ENABLE        0x80000104
#define GB_MPU_AUX_NOT_FOUND         0x80000105
#define GB_MPU_AUX_LOST_ARB          0x80000106
#define GB_MPU_AUX_RW_TIMEOUT        0x80000107
#define GB_MPU_SELF_TEST_FAIL        0x80000108
#define GB_MPU_AUXDEV_NOT_ENABLE     0x80000109

#define GB_COMPASS_W_SCALE      0x80000200
#define GB_COMPASS_W_MODE       0x80000201
#define GB_COMPASS_DEVICE_NULL  0x80000202

#define GB_BARO_DEV_ID_ERROR     0X80000300
#define GB_BARO_DEVICE_NULL      0x80000301
#define GB_BARO_DEVICE_BUS_NULL  0x80000302
#define GB_BARO_DATA_NOT_READY   0x80000303

#define GB_RF24_INVALID_PIN     0x80000401
#define GB_RF24_IO_CFG_FAIL     0x80000402
#define GB_RF24_IO_SET_FAIL     0x80000403
#define GB_RF24_W_DATA_FAIL     0x80000404
#define GB_RF24_W_FAST_FAIL     0x80000405
#define GB_RF24_UNKNOWN_STAT    0x80000406

#define GB_CAM_NVS_INIT_FAIL    0x80000500
#define GB_CAM_INIT_FAIL        0x80000501
#define GB_CAM_FLIP_FAIL        0x80000502
#define GB_CAM_DEINIT_FAIL      0x80000503
#define GB_CAM_HTTP_STR_FAIL    0x80000504
#define GB_CAM_HTTP_REG_FAIL    0x80000505
#define GB_CAM_HTTP_STP_FAIL    0x80000506
#define GB_CAM_WIFI_NET_FAIL    0x80000507
#define GB_CAM_WIFI_EVN_FAIL    0x80000508
#define GB_CAM_WIFI_INI_FAIL    0x80000509
#define GB_CAM_WIFI_SET_FAIL    0x8000050a
#define GB_CAM_WIFI_STR_FAIL    0x8000050b
#define GB_CAM_WIFI_STP_FAIL    0x8000050c

#define GB_LOG_USB_READ_FAIL    0x80000600
#define GB_LOG_USB_WROTE_FAIL   0x80000601

#define GB_ESC_IO_INIT_FAIL     0x80000700
#define GB_ESC_PWM_INIT_FAIL    0x80000701
#define GB_ESC_SET_DUTY_FAIL    0x80000702
#define GB_ESC_TIMER_STR_FAIL   0x80000703

#define GB_FS_INIT_FLASH_FAIL   0x80000800
#define GB_FS_MOUNT_FAIL        0x80000801
#define GB_FS_FOPEN_FAIL        0x80000802
#define GB_FS_RE_FOPEN_FAIL     0x80000803
#define GB_FS_WRITE_FAIL        0x80000804
#define GB_FS_READ_FAIL         0x80000805
#define GB_FS_SD_INIT_FAIL      0x80000805

#define GB_FTP_INIT_FAIL        0x80000900
#define GB_FTP_CONNECT_FAIL     0x80000901
#define GB_FTP_GET_IP_FAIL      0x80000902

#define GB_PWM_INIT_FAIL        0x80000a00
#define GB_PWM_START_FAIL       0x80000a01

#define GB_UART_WRITE_FAIL      0x80000b00
#define GB_UART_READ_FAIL       0x80000b01
#define GB_UART_WR_SIZE_MIS     0x80000b02
#define GB_UART_INIT_FAIL       0x80000b03

#define GB_LASER_DEVICE_NULL    0x80000c00
#define GB_LASER_BUS_NULL       0x80000c01
#define GB_LASER_INIT_FAIL      0x80000c02
#define GB_LASER_CFG_FAIL       0x80000c03
#define GB_LASER_START_FAIL     0x80000c04
#define GB_LASER_STOP_FAIL      0x80000c05
#define GB_LASER_MEASURE_FAIL   0x80000c06
#define GB_LASER_READ_FAIL      0x80000c07
#define GB_LASER_TIMEOUT        0x80000c08
#define GB_LASER_NOT_INIT       0x80000c09
#define GB_LASER_PARAM_NULL     0x80000c0a
#define GB_LASER_CAL_FAIL       0x80000c0b

#define GB_OFLOW_DEVINE_NULL    0x80000d00
#define GB_OFLOW_BUS_NULL       0x80000d01
#define GB_OFLOW_INIT_FAIL      0x80000d02
#define GB_OFLOW_CFG_FAIL       0x80000d03
#define GB_OFLOW_READ_FAIL      0x80000d04
#define GB_OFLOW_WRITE_FAIL     0x80000d05
#define GB_OFLOW_CHIP_ID_FAIL   0x80000d06
#define GB_OFLOW_NOT_INIT       0x80000d07
#define GB_OFLOW_PARAM_NULL     0x80000d08

#endif /* end of include guard: _GB_RESULTS__ */
