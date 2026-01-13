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

#include "vl53l1x.h"

bool vl53l1xInit(VL53L1_Dev_t *pdev, I2C_Dev *I2cHandle)
{
  VL53L1_Error status = VL53L1_ERROR_NONE;

  pdev->I2cDevAddr = VL53L1X_DEFAULT_ADDRESS;
  pdev->I2Cx = I2cHandle;
  i2cdevInit(pdev->I2Cx);

  /* Move initialized sensor to a new I2C address */
  //int newAddress;

  //taskENTER_CRITICAL();
  //newAddress = nextI2CAddress++;
  //taskEXIT_CRITICAL();

  //vl53l1xSetI2CAddress(pdev, newAddress);

  uint8_t byteData;
  uint16_t wordData;
  VL53L1_RdByte(pdev, 0x010F, &byteData);
  DEBUG_PRINT( "VL53L1X Model_ID: %02X\n\r", byteData);
  VL53L1_RdByte(pdev, 0x0110, &byteData);
  DEBUG_PRINT( "VL53L1X Module_Type: %02X\n\r", byteData);
  VL53L1_RdWord(pdev, 0x010F, &wordData);
  DEBUG_PRINT( "VL53L1X: %02X\n\r", wordData);

  status = VL53L1_WaitDeviceBooted(pdev);
  if (status == VL53L1_ERROR_NONE)
  {
	status = VL53L1_DataInit(pdev);

	if (status == VL53L1_ERROR_NONE)
	{
		status = VL53L1_StaticInit(pdev);
	}
  }

#ifdef SET_VL53LX_ROI
  VL53L1_UserRoi_t Roi0;
  Roi0.TopLeftX = 0; //set ROI according to requirement
  Roi0.TopLeftY = 15;
  Roi0.BotRightX = 15;
  Roi0.BotRightY = 0;
  status = VL53L1_SetUserROI(pdev, &Roi0); //SET region of interest
#endif
	//Restart sensor remove to zranger task
	//VL53L1_StopMeasurement(pdev);
	//status = VL53L1_SetDistanceMode(pdev,VL53L1_DISTANCEMODE_LONG);
	//status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(pdev, 160000);
	//status = VL53L1_SetInterMeasurementPeriodMilliSeconds(pdev, 200);
	//status = VL53L1_StartMeasurement(pdev);

  return status == VL53L1_ERROR_NONE;
}

bool vl53l1xTestConnection(VL53L1_Dev_t* pdev)
{
  VL53L1_DeviceInfo_t info;
  VL53L1_Error status = VL53L1_ERROR_NONE;

  status = VL53L1_GetDeviceInfo(pdev, &info);

  return status == VL53L1_ERROR_NONE;
}

/** Set I2C address
 * Any subsequent communication will be on the new address
 * The address passed is the 7bit I2C address from LSB (ie. without the
 * read/write bit)
 */
VL53L1_Error vl53l1xSetI2CAddress(VL53L1_Dev_t* pdev, uint8_t address)
{
  VL53L1_Error status = VL53L1_ERROR_NONE;

  status = VL53L1_SetDeviceAddress(pdev, address);
  pdev->I2cDevAddr = address;
  return  status;
}


/*
 * ----------------- COMMS FUNCTIONS -----------------
 */

VL53L1_Error VL53L1_WriteMulti(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint8_t      *pdata,
	uint32_t      count)
{
	VL53L1_Error status         = VL53L1_ERROR_NONE;

  if (!i2cdevWrite16(pdev->I2Cx, pdev->I2cDevAddr, index, count, pdata))
  {
    status = VL53L1_ERROR_CONTROL_INTERFACE;
  }

	return status;
}

VL53L1_Error VL53L1_ReadMulti(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint8_t      *pdata,
	uint32_t      count)
{
	VL53L1_Error status         = VL53L1_ERROR_NONE;

  if (!i2cdevRead16(pdev->I2Cx, pdev->I2cDevAddr, index, count, pdata))
  {
    status = VL53L1_ERROR_CONTROL_INTERFACE;
  }

	return status;
}


VL53L1_Error VL53L1_WrByte(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint8_t       data)
{
	VL53L1_Error status         = VL53L1_ERROR_NONE;

	if (!i2cdevWrite16(pdev->I2Cx, pdev->I2cDevAddr, index, 1, &data))
	{
	  status = VL53L1_ERROR_CONTROL_INTERFACE;
	}

	return status;
}


VL53L1_Error VL53L1_WrWord(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint16_t      data)
{
  VL53L1_Error status         = VL53L1_ERROR_NONE;
  uint8_t _I2CBuffer[2];
  _I2CBuffer[0] = data >> 8;
  _I2CBuffer[1] = data & 0x00FF;
  if (!i2cdevWrite16(pdev->I2Cx, pdev->I2cDevAddr, index, 2, (uint8_t *)_I2CBuffer))
  {
    status = VL53L1_ERROR_CONTROL_INTERFACE;
  }

	return status;
}


VL53L1_Error VL53L1_WrDWord(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint32_t      data)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;
  	uint8_t _I2CBuffer[4];
	_I2CBuffer[0] = (data >> 24) & 0xFF;
	_I2CBuffer[1] = (data >> 16) & 0xFF;
	_I2CBuffer[2] = (data >> 8) & 0xFF;
	_I2CBuffer[3] = (data >> 0) & 0xFF;

	if (!i2cdevWrite16(pdev->I2Cx, pdev->I2cDevAddr, index, 4, (uint8_t *)_I2CBuffer))
  {
    status = VL53L1_ERROR_CONTROL_INTERFACE;
  }

	return status;
}


VL53L1_Error VL53L1_RdByte(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint8_t      *pdata)
{
	VL53L1_Error status         = VL53L1_ERROR_NONE;

	if (!i2cdevRead16(pdev->I2Cx, pdev->I2cDevAddr, index, 1, pdata))
  {
    status = VL53L1_ERROR_CONTROL_INTERFACE;
  }

	return status;
}

VL53L1_Error VL53L1_RdWord(
	VL53L1_Dev_t *pdev,
	uint16_t index,
	uint16_t *pdata)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;
	uint8_t _I2CBuffer[2];

	if (!i2cdevRead16(pdev->I2Cx, pdev->I2cDevAddr, index, 2, (uint8_t *)_I2CBuffer))
	{
		status = VL53L1_ERROR_CONTROL_INTERFACE;
	}
	*pdata = ((uint16_t)_I2CBuffer[0] << 8) + (uint16_t)_I2CBuffer[1];
	return status;
}

VL53L1_Error VL53L1_RdDWord(
	VL53L1_Dev_t *pdev,
	uint16_t index,
	uint32_t *pdata)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;
	uint8_t _I2CBuffer[4];
	if (!i2cdevRead16(pdev->I2Cx, pdev->I2cDevAddr, index, 4, (uint8_t *)_I2CBuffer))
	{
		status = VL53L1_ERROR_CONTROL_INTERFACE;
	}
	*pdata = ((uint32_t)_I2CBuffer[0] << 24) + ((uint32_t)_I2CBuffer[1] << 16) + ((uint32_t)_I2CBuffer[2] << 8) + (uint32_t)_I2CBuffer[3];
	return status;
}

/*
 * ----------------- HOST TIMING FUNCTIONS -----------------
 */

VL53L1_Error VL53L1_WaitUs(
	VL53L1_Dev_t *pdev,
	int32_t       wait_us)
{
	VL53L1_Error status         = VL53L1_ERROR_NONE;
	uint32_t delay_ms = (wait_us + 900) / 1000;

	if(delay_ms == 0)
	{
	  delay_ms = 1;
	}

	vTaskDelay(M2T(delay_ms));

	return status;
}


VL53L1_Error VL53L1_WaitMs(
	VL53L1_Dev_t *pdev,
	int32_t       wait_ms)
{
  vTaskDelay(M2T(wait_ms));

  return VL53L1_ERROR_NONE;
}

/*
 * ----------------- DEVICE TIMING FUNCTIONS -----------------
 */

VL53L1_Error VL53L1_GetTickCount(
	uint32_t *ptick_count_ms)
{
	/* Returns current tick count in [ms] */
	*ptick_count_ms = xTaskGetTickCount();

	return VL53L1_ERROR_NONE;
}


VL53L1_Error VL53L1_WaitValueMaskEx(
	VL53L1_Dev_t *pdev,
	uint32_t      timeout_ms,
	uint16_t      index,
	uint8_t       value,
	uint8_t       mask,
	uint32_t      poll_delay_ms)
{
	/*
	 * Platform implementation of WaitValueMaskEx V2WReg script command
	 *
	 * WaitValueMaskEx(
	 *          duration_ms,
	 *          index,
	 *          value,
	 *          mask,
	 *          poll_delay_ms);
	 */

	VL53L1_Error status         = VL53L1_ERROR_NONE;
	uint32_t     start_time_ms   = 0;
	uint32_t     current_time_ms = 0;
	uint8_t      byte_value      = 0;
	uint8_t      found           = 0;
#ifdef VL53L1_LOG_ENABLE
	uint32_t     trace_functions = 0;
#endif

	SUPPRESS_UNUSED_WARNING(poll_delay_ms);

#ifdef VL53L1_LOG_ENABLE
	/* look up register name */
	VL53L1_get_register_name(
			index,
			register_name);

	/* Output to I2C logger for FMT/DFT  */
	trace_i2c("WaitValueMaskEx(%5d, %s, 0x%02X, 0x%02X, %5d);\n",
		timeout_ms, register_name, value, mask, poll_delay_ms);
#endif // VL53L1_LOG_ENABLE

	/* calculate time limit in absolute time */

	VL53L1_GetTickCount(&start_time_ms);
	pdev->new_data_ready_poll_duration_ms = 0;

	/* remember current trace functions and temporarily disable
	 * function logging
	 */

#ifdef VL53L1_LOG_ENABLE
	trace_functions = _LOG_GET_TRACE_FUNCTIONS();
#endif

	/* wait until value is found, timeout reached on error occurred */

	while ((status == VL53L1_ERROR_NONE) &&
		   (pdev->new_data_ready_poll_duration_ms < timeout_ms) &&
		   (found == 0))
	{
		status = VL53L1_RdByte(
						pdev,
						index,
						&byte_value);

		if ((byte_value & mask) == value)
		{
			found = 1;
		}

		if (status == VL53L1_ERROR_NONE  &&
			found == 0 &&
			poll_delay_ms > 0)
			status = VL53L1_WaitMs(
							pdev,
							poll_delay_ms);

		/* Update polling time (Compare difference rather than absolute to
		negate 32bit wrap around issue) */
		VL53L1_GetTickCount(&current_time_ms);
		pdev->new_data_ready_poll_duration_ms = current_time_ms - start_time_ms;
	}

	if (found == 0 && status == VL53L1_ERROR_NONE)
		status = VL53L1_ERROR_TIME_OUT;

	return status;
}
