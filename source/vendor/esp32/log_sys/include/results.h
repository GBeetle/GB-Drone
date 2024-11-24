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

typedef uint32_t GB_RESULT;

#define GB_OK                   0x00000000
#define GB_FAIL                 0x80000000

#define GB_CHK_BOOL_FAIL        0x80000500

#define GB_SPI_RW_FAIL          0x80000001
#define GB_SPI_INI_FAIL         0x80000002
#define GB_SPI_FREE_FAIL        0x80000003
#define GB_SPI_CFG_FAIL         0x80000004
#define GB_SPI_RMV_FAIL         0x80000005
#define GB_SPI_INVALID_SIZE     0x80000006

#define GB_I2C_RW_FAIL               0x80000010
#define GB_I2C_CFG_FAIL              0x80000011
#define GB_I2C_INS_FAIL              0x80000012
#define GB_I2C_RMV_FAIL              0x80000013
#define GB_I2C_CONNECT_FAIL          0x80000014
#define GB_I2C_MAX_DEVICE_REACHED    0x80000015

#define GB_MPU_NOT_FOUND        0X80000100
#define GB_MPU_DUMP_REG_FAIL    0X80000101
#define GB_MPU_RW_TEST_FAIL     0X80000102
#define GB_MPU_AUX_RW_FAIL      0x80000103
#define GB_MPU_AUX_NOT_ENABLE   0x80000104
#define GB_MPU_AUX_NOT_FOUND    0x80000105
#define GB_MPU_AUX_LOST_ARB     0x80000106
#define GB_MPU_AUX_RW_TIMEOUT   0x80000107
#define GB_MPU_SELF_TEST_FAIL   0x80000108

#define GB_COMPASS_W_SCALE      0x80000200
#define GB_COMPASS_W_MODE       0x80000201

#define GB_BMP_DEV_ID_ERROR     0X80000300
#define GB_BMP_DEVICE_NULL      0x80000301
#define GB_BMP_DEVICE_BUS_NULL  0x80000302

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

#define GB_FTP_INIT_FAIL        0x80000900
#define GB_FTP_CONNECT_FAIL     0x80000901
#define GB_FTP_GET_IP_FAIL      0x80000902

#endif /* end of include guard: _GB_RESULTS__ */
