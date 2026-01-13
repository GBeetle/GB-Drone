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

#ifndef __VL53L1X_DRIVER_H__
#define __VL53L1X_DRIVER_H__

#include "vl53l1_ll_def.h"
#include "vl53l1_platform_user_data.h"
#include "vl53l1_api.h"
#include "i2cdev.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define    VL53L1_BYTES_PER_WORD              2
#define    VL53L1_BYTES_PER_DWORD             4

/* Define polling delays */
#define VL53L1_BOOT_COMPLETION_POLLING_TIMEOUT_MS     500
#define VL53L1_RANGE_COMPLETION_POLLING_TIMEOUT_MS   2000
#define VL53L1_TEST_COMPLETION_POLLING_TIMEOUT_MS   60000

#define VL53L1_POLLING_DELAY_MS                         1

/* Define LLD TuningParms Page Base Address
 * - Part of Patch_AddedTuningParms_11761
 */
#define VL53L1_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS  0x8000
#define VL53L1_TUNINGPARM_PRIVATE_PAGE_BASE_ADDRESS 0xC000

#define VL53L1_GAIN_FACTOR__STANDARD_DEFAULT       0x0800
	/*!<  Default standard ranging gain correction factor
	      1.11 format. 1.0 = 0x0800, 0.980 = 0x07D7 */

#define VL53L1_OFFSET_CAL_MIN_EFFECTIVE_SPADS  0x0500
	/*!< Lower Limit for the  MM1 effective SPAD count during offset
	     calibration Format 8.8 0x0500 -> 5.0 effective SPADs */

#define VL53L1_OFFSET_CAL_MAX_PRE_PEAK_RATE_MCPS   0x1900
	/*!< Max Limit for the pre range peak rate during offset
	     calibration Format 9.7 0x1900 -> 50.0 Mcps.
	     If larger then in pile up */

#define VL53L1_OFFSET_CAL_MAX_SIGMA_MM             0x0040
	/*!< Max sigma estimate limit during offset calibration
	     Check applies to pre-range, mm1 and mm2 ranges
	     Format 14.2 0x0040 -> 16.0mm. */

#define VL53L1_MAX_USER_ZONES                 169
	/*!< Max number of user Zones - maximal limitation from
		 FW stream divide - value of 254 */

#define VL53L1_MAX_RANGE_RESULTS              2
	/*!< Allocates storage for return and reference restults */


#define VL53L1_MAX_STRING_LENGTH 512


#define VL53L1X_DEFAULT_ADDRESS 0b0101001
#define VL53L1X_ID			0xEACC

typedef struct {

	VL53L1_DevData_t   Data;

	uint8_t   I2cDevAddr;
	uint8_t   comms_type;
	uint16_t  comms_speed_khz;
	uint32_t  new_data_ready_poll_duration_ms;
	I2C_Dev *I2Cx;

	// I2C_HandleTypeDef *I2cHandle;

} VL53L1_Dev_t;

typedef VL53L1_Dev_t *VL53L1_DEV;

/**
 * @file   vl53l1_platform.h
 *
 * @brief  All end user OS/platform/application porting
 */

bool vl53l1xInit(VL53L1_Dev_t *pdev, I2C_Dev *I2cHandle);

bool vl53l1xTestConnection(VL53L1_Dev_t* pdev);

VL53L1_Error vl53l1xSetI2CAddress(VL53L1_Dev_t* pdev, uint8_t address);

/**
 * @brief Writes the supplied byte buffer to the device
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[in]   pdata     : pointer to uint8_t (byte) buffer containing the data to be written
 * @param[in]   count     : number of bytes in the supplied byte buffer
 *
 * @return   VL53L1_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53L1_Error
 */

VL53L1_Error VL53L1_WriteMulti(
		VL53L1_Dev_t *pdev,
		uint16_t      index,
		uint8_t      *pdata,
		uint32_t      count);


/**
 * @brief  Reads the requested number of bytes from the device
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[out]  pdata     : pointer to the uint8_t (byte) buffer to store read data
 * @param[in]   count     : number of bytes to read
 *
 * @return   VL53L1_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53L1_Error
 */

VL53L1_Error VL53L1_ReadMulti(
		VL53L1_Dev_t *pdev,
		uint16_t      index,
		uint8_t      *pdata,
		uint32_t      count);


/**
 * @brief  Writes a single byte to the device
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[in]   data      : uint8_t data value to write
 *
 * @return   VL53L1_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53L1_Error
 */

VL53L1_Error VL53L1_WrByte(
		VL53L1_Dev_t *pdev,
		uint16_t      index,
		uint8_t       data);


/**
 * @brief  Writes a single word (16-bit unsigned) to the device
 *
 * Manages the big-endian nature of the device register map
 * (first byte written is the MS byte).
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[in]   data      : uin16_t data value write
 *
 * @return   VL53L1_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53L1_Error
 */

VL53L1_Error VL53L1_WrWord(
		VL53L1_Dev_t *pdev,
		uint16_t      index,
		uint16_t      data);


/**
 * @brief  Writes a single dword (32-bit unsigned) to the device
 *
 * Manages the big-endian nature of the device register map
 * (first byte written is the MS byte).
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[in]   data      : uint32_t data value to write
 *
 * @return   VL53L1_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53L1_Error
 */

VL53L1_Error VL53L1_WrDWord(
		VL53L1_Dev_t *pdev,
		uint16_t      index,
		uint32_t      data);



/**
 * @brief  Reads a single byte from the device
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index
 * @param[out]  pdata     : pointer to uint8_t data value
 *
 * @return   VL53L1_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53L1_Error
 *
 */

VL53L1_Error VL53L1_RdByte(
		VL53L1_Dev_t *pdev,
		uint16_t      index,
		uint8_t      *pdata);


/**
 * @brief  Reads a single word (16-bit unsigned) from the device
 *
 * Manages the big-endian nature of the device (first byte read is the MS byte).
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[out]  pdata     : pointer to uint16_t data value
 *
 * @return   VL53L1_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53L1_Error
 */

VL53L1_Error VL53L1_RdWord(
		VL53L1_Dev_t *pdev,
		uint16_t      index,
		uint16_t     *pdata);


/**
 * @brief  Reads a single dword (32-bit unsigned) from the device
 *
 * Manages the big-endian nature of the device (first byte read is the MS byte).
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[out]  pdata     : pointer to uint32_t data value
 *
 * @return   VL53L1_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53L1_Error
 */

VL53L1_Error VL53L1_RdDWord(
		VL53L1_Dev_t *pdev,
		uint16_t      index,
		uint32_t     *pdata);



/**
 * @brief  Implements a programmable wait in us
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   wait_us   : integer wait in micro seconds
 *
 * @return  VL53L1_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53L1_Error
 */

VL53L1_Error VL53L1_WaitUs(
		VL53L1_Dev_t *pdev,
		int32_t       wait_us);


/**
 * @brief  Implements a programmable wait in ms
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   wait_ms   : integer wait in milliseconds
 *
 * @return  VL53L1_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53L1_Error
 */

VL53L1_Error VL53L1_WaitMs(
		VL53L1_Dev_t *pdev,
		int32_t       wait_ms);

/*
 * @brief Gets current system tick count in [ms]
 *
 * @return  time_ms : current time in [ms]
 *
 * @return  VL53L1_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53L1_Error
 */

VL53L1_Error VL53L1_GetTickCount(
	uint32_t *ptime_ms);


/**
 * @brief Register "wait for value" polling routine
 *
 * Port of the V2WReg Script function  WaitValueMaskEx()
 *
 * @param[in]   pdev          : pointer to device structure (device handle)
 * @param[in]   timeout_ms    : timeout in [ms]
 * @param[in]   index         : uint16_t register index value
 * @param[in]   value         : value to wait for
 * @param[in]   mask          : mask to be applied before comparison with value
 * @param[in]   poll_delay_ms : polling delay been each read transaction in [ms]
 *
 * @return  VL53L1_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53L1_Error
 */

VL53L1_Error VL53L1_WaitValueMaskEx(
		VL53L1_Dev_t *pdev,
		uint32_t      timeout_ms,
		uint16_t      index,
		uint8_t       value,
		uint8_t       mask,
		uint32_t      poll_delay_ms);

#ifdef __cplusplus
}
#endif

#endif  //__VL53L1X_DRIVER_H__
