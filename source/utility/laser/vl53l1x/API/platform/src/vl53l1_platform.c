/*******************************************************************************
 Copyright (C) 2016, STMicroelectronics International N.V.
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of STMicroelectronics nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
 NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED.
 IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/**
 * @file   vl53l1_platform.c
 * @brief  Code function definitions for EwokPlus25 Platform Layer
 *         RANGING SENSOR VERSION
 *
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "vl53l1_platform.h"
#include "vl53l1_platform_log.h"
#include "vl53l1_api.h"
#include "gb_timer.h"
#include "i2c_bus.h"
#include "results.h"
#include "log_sys.h"

static struct i2c *g_laser_i2c_bus = NULL;

void VL53L1_SetI2CBus(struct i2c *bus)
{
	g_laser_i2c_bus = bus;
}

VL53L1_Error VL53L1_CommsInitialise(
	VL53L1_Dev_t *pdev,
	uint8_t       comms_type,
	uint16_t      comms_speed_khz)
{
	if (pdev == NULL || g_laser_i2c_bus == NULL) {
		return VL53L1_ERROR_CONTROL_INTERFACE;
	}

	return VL53L1_ERROR_NONE;
}


VL53L1_Error VL53L1_CommsClose(
	VL53L1_Dev_t *pdev)
{
	return VL53L1_ERROR_NONE;
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
	if (pdev == NULL || pdata == NULL || g_laser_i2c_bus == NULL) {
		return VL53L1_ERROR_CONTROL_INTERFACE;
	}

	uint8_t *write_buffer = (uint8_t *)malloc(count + 2);
	if (write_buffer == NULL) {
		return VL53L1_ERROR_CONTROL_INTERFACE;
	}

	write_buffer[0] = (index >> 8) & 0xff;  // MSB of register address
	write_buffer[1] = index & 0xff;         // LSB of register address
	memcpy(&write_buffer[2], pdata, count);

	GB_RESULT res = g_laser_i2c_bus->writeBytes(g_laser_i2c_bus, pdev->i2c_slave_address, write_buffer[0], count + 1, &write_buffer[1]);

	free(write_buffer);

	return (res == GB_OK) ? VL53L1_ERROR_NONE: VL53L1_ERROR_CONTROL_INTERFACE;
}


VL53L1_Error VL53L1_ReadMulti(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint8_t      *pdata,
	uint32_t      count)
{
	if (pdev == NULL || pdata == NULL || g_laser_i2c_bus == NULL) {
		return VL53L1_ERROR_CONTROL_INTERFACE;
	}

	uint8_t reg_addr[2];
	reg_addr[0] = (index >> 8) & 0xff;
	reg_addr[1] = index & 0xff;

	GB_RESULT res = g_laser_i2c_bus->writeBytes(g_laser_i2c_bus, pdev->i2c_slave_address, reg_addr[0], 1, &reg_addr[1]);
	if (res != GB_OK) {
		return VL53L1_ERROR_CONTROL_INTERFACE;
	}

	res = g_laser_i2c_bus->readBytes(g_laser_i2c_bus, pdev->i2c_slave_address, reg_addr[0], count, pdata);

	return (res == GB_OK) ? VL53L1_ERROR_NONE: VL53L1_ERROR_CONTROL_INTERFACE;
}


VL53L1_Error VL53L1_WrByte(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint8_t       data)
{
	return VL53L1_WriteMulti(pdev, index, &data, 1);
}


VL53L1_Error VL53L1_WrWord(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint16_t      data)
{
	uint8_t  buffer[2];

	buffer[0] = (data >> 8) & 0xff;
	buffer[1] = data & 0xff;
	return VL53L1_WriteMulti(pdev, index, buffer, 2);
}


VL53L1_Error VL53L1_WrDWord(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint32_t      data)
{
	uint8_t  buffer[2];

	buffer[0] = (data >> 24) & 0xff;
	buffer[1] = (data >> 16) & 0xff;
	buffer[2] = (data >> 8) & 0xff;
	buffer[3] = data & 0xff;
	return VL53L1_WriteMulti(pdev, index, buffer, 4);
}


VL53L1_Error VL53L1_RdByte(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint8_t      *pdata)
{
	return VL53L1_ReadMulti(pdev, index, pdata, 1);
}


VL53L1_Error VL53L1_RdWord(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint16_t     *pdata)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;
	uint8_t  buffer[2];

	status = VL53L1_ReadMulti(
					pdev,
					index,
					buffer,
					VL53L1_BYTES_PER_WORD);

	*pdata = (uint16_t)(((uint16_t)(buffer[0]) << 8) + (uint16_t)buffer[1]);

	return status;
}


VL53L1_Error VL53L1_RdDWord(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint32_t     *pdata)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;
	uint8_t  buffer[4];

	status = VL53L1_ReadMulti(
					pdev,
					index,
					buffer,
					VL53L1_BYTES_PER_DWORD);

	*pdata = ((uint32_t)buffer[0] << 24) + ((uint32_t)buffer[1] << 16) + ((uint32_t)buffer[2] << 8) + (uint32_t)buffer[3];

	return status;
}

/*
 * ----------------- HOST TIMING FUNCTIONS -----------------
 */

VL53L1_Error VL53L1_WaitUs(
	VL53L1_Dev_t *pdev,
	int32_t       wait_us)
{
	GB_SleepUs(wait_us);
	return VL53L1_ERROR_NONE;
}


VL53L1_Error VL53L1_WaitMs(
	VL53L1_Dev_t *pdev,
	int32_t       wait_ms)
{
	GB_SleepMs(wait_ms);
	return VL53L1_ERROR_NONE;
}

/*
 * ----------------- DEVICE TIMING FUNCTIONS -----------------
 */

VL53L1_Error VL53L1_GetTimerFrequency(int32_t *ptimer_freq_hz)
{
	*ptimer_freq_hz = 1000; // 1KHZ

	return VL53L1_ERROR_NONE;
}


VL53L1_Error VL53L1_GetTimerValue(int32_t *ptimer_count)
{
	uint64_t timer_ms;

	GB_GetTimerMs(&timer_ms);
	*ptimer_count = (int32_t)timer_ms;
	return VL53L1_ERROR_NONE;
}


/*
 * ----------------- GPIO FUNCTIONS -----------------
 */

VL53L1_Error VL53L1_GpioSetMode(uint8_t pin, uint8_t mode)
{
	return VL53L1_ERROR_NONE;
}


VL53L1_Error  VL53L1_GpioSetValue(uint8_t pin, uint8_t value)
{
	return VL53L1_ERROR_NONE;
}


VL53L1_Error  VL53L1_GpioGetValue(uint8_t pin, uint8_t *pvalue)
{
	return VL53L1_ERROR_NONE;
}

/*
 * ----------------- HARDWARE STATE FUNCTIONS -----------------
 */

VL53L1_Error  VL53L1_GpioXshutdown(uint8_t value)
{
	return VL53L1_ERROR_NONE;
}


VL53L1_Error  VL53L1_GpioCommsSelect(uint8_t value)
{
	return VL53L1_ERROR_NONE;
}


VL53L1_Error  VL53L1_GpioPowerEnable(uint8_t value)
{
	return VL53L1_ERROR_NONE;
}


VL53L1_Error  VL53L1_GpioInterruptEnable(void (*function)(void), uint8_t edge_type)
{
	return VL53L1_ERROR_NONE;
}


VL53L1_Error  VL53L1_GpioInterruptDisable(void)
{
	return VL53L1_ERROR_NONE;
}


VL53L1_Error VL53L1_GetTickCount(
	uint32_t *ptick_count_ms)
{
	uint64_t timer_ms;

	GB_GetTimerMs(&timer_ms);
	*ptick_count_ms = (int32_t)timer_ms;

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
	uint32_t     start_time_ms   = 0;
	uint32_t     current_time_ms = 0;
	uint32_t     polling_time_ms = 0;
	uint8_t      byte_value      = 0;
	uint8_t      found           = 0;
	VL53L1_Error status          = VL53L1_ERROR_NONE;

	VL53L1_GetTickCount(&start_time_ms);

	while (!found && status == VL53L1_ERROR_NONE &&
			polling_time_ms < timeout_ms)
	{
		status = VL53L1_RdByte(
						pdev,
						index,
						&byte_value);

		if ((byte_value & mask) == value)
		{
			found = 1;
		}

		if (!found) {
			status = VL53L1_WaitMs(pdev, poll_delay_ms);

			VL53L1_GetTickCount(&current_time_ms);
			polling_time_ms = current_time_ms - start_time_ms;
		}


	}

	if (!found && status == VL53L1_ERROR_NONE)
		status = VL53L1_ERROR_TIME_OUT;

	return status;
}

