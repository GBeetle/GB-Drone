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

#include <stddef.h>
#include "ms5611.h"
#include "gb_timer.h"
#include "error_handle.h"

#define CMD_CONVERT_D1 0x40
#define CMD_CONVERT_D2 0x50
#define CMD_ADC_READ   0x00
#define CMD_RESET      0x1E

#define PROM_ADDR_MANU     0xa0
#define PROM_ADDR_SENS     0xa2
#define PROM_ADDR_OFF      0xa4
#define PROM_ADDR_TCS      0xa6
#define PROM_ADDR_TCO      0xa8
#define PROM_ADDR_T_REF    0xaa
#define PROM_ADDR_TEMPSENS 0xac
#define PROM_ADDR_CRC      0xae

static inline GB_RESULT read_bytes(ms5611_t *dev, uint8_t reg, uint8_t *r, size_t length)
{
    GB_RESULT res = GB_OK;
    uint8_t w_data = 0x00;

    if (reg != 0x00)
    {
        CHK_RES(dev->bus->readWriteBytesWithConfig(dev->bus, dev->addr, 0x00, 1, NULL, &reg, 0, 0, 0));
    }

    for (int i = 0; i < length; i++)
    {
        CHK_RES(dev->bus->readWriteBytesWithConfig(dev->bus, dev->addr, 0x00, 1, r + i, &w_data, 0, 0, 0));
    }

error_exit:
    return res;
}

static inline GB_RESULT send_command(ms5611_t *dev, uint8_t cmd)
{
    return dev->bus->readWriteBytesWithConfig(dev->bus, dev->addr, 0x00, 1, NULL, &cmd, 0, 0, 0);
}

static inline uint16_t shuffle(uint16_t val)
{
    return ((val & 0xff00) >> 8) | ((val & 0xff) << 8);
}

static inline GB_RESULT ms5611_crc(uint16_t *prom)
{
    int32_t i, j;
    uint32_t res = 0;
    uint8_t crc = prom[7] & 0xF;
    prom[7] &= 0xFF00;

    bool blankEeprom = true;

    for (i = 0; i < 16; i++) {
        if (prom[i >> 1]) {
            blankEeprom = false;
        }
        if (i & 1)
            res ^= ((prom[i >> 1]) & 0x00FF);
        else
            res ^= (prom[i >> 1] >> 8);
        for (j = 8; j > 0; j--) {
            if (res & 0x8000)
                res ^= 0x1800;
            res <<= 1;
        }
    }
    prom[7] |= crc;
    if (!blankEeprom && crc == ((res >> 12) & 0xF))
        return GB_OK;

    return GB_BARO_DEV_ID_ERROR;
}

static inline GB_RESULT read_prom(ms5611_t *dev)
{
    GB_RESULT res = GB_OK;
    uint16_t tmp;

    CHK_RES(read_bytes(dev, PROM_ADDR_MANU, (uint8_t *)&tmp, 2));
    dev->config_data.menu = shuffle(tmp);
    CHK_RES(read_bytes(dev, PROM_ADDR_SENS, (uint8_t *)&tmp, 2));
    dev->config_data.sens = shuffle(tmp);
    CHK_RES(read_bytes(dev, PROM_ADDR_OFF, (uint8_t *)&tmp, 2));
    dev->config_data.off = shuffle(tmp);
    CHK_RES(read_bytes(dev, PROM_ADDR_TCS, (uint8_t *)&tmp, 2));
    dev->config_data.tcs = shuffle(tmp);
    CHK_RES(read_bytes(dev, PROM_ADDR_TCO, (uint8_t *)&tmp, 2));
    dev->config_data.tco = shuffle(tmp);
    CHK_RES(read_bytes(dev, PROM_ADDR_T_REF, (uint8_t *)&tmp, 2));
    dev->config_data.t_ref = shuffle(tmp);
    CHK_RES(read_bytes(dev, PROM_ADDR_TEMPSENS, (uint8_t *)&tmp, 2));
    dev->config_data.tempsens = shuffle(tmp);
    CHK_RES(read_bytes(dev, PROM_ADDR_CRC, (uint8_t *)&tmp, 2));
    dev->config_data.crc = shuffle(tmp);

    GB_DUMPI(BMP_TAG, (uint8_t*)&(dev->config_data), sizeof(ms5611_config_data_t));

    CHK_RES(ms5611_crc((uint16_t*)&(dev->config_data)));

error_exit:
    return res;
}

static GB_RESULT read_adc(ms5611_t *dev, uint32_t *result)
{
    GB_RESULT res = GB_OK;
    uint8_t tmp[3];

    CHK_RES(read_bytes(dev, 0, tmp, 3));
    *result = (tmp[0] << 16) | (tmp[1] << 8) | tmp[2];

error_exit:
    return res;
}

static void wait_conversion(ms5611_t *dev)
{
    uint32_t us = 8220;
    switch (dev->osr)
    {
        case MS5611_OSR_256: us = 500; break;   // 0.5ms
        case MS5611_OSR_512: us = 1100; break;  // 1.1ms
        case MS5611_OSR_1024: us = 2100; break; // 2.1ms
        case MS5611_OSR_2048: us = 4100; break; // 4.1ms
        case MS5611_OSR_4096: us = 8220; break; // 8.22ms
    }
    GB_SleepMs(us / 1000 + 1);
}

static inline GB_RESULT get_raw_temperature(ms5611_t *dev, uint32_t *result)
{
    GB_RESULT res = GB_OK;

    CHK_RES(send_command(dev, CMD_CONVERT_D2 + dev->osr));
    wait_conversion(dev);
    CHK_RES(read_adc(dev, result));

error_exit:
    return res;
}

static inline GB_RESULT get_raw_pressure(ms5611_t *dev, uint32_t *result)
{
    GB_RESULT res = GB_OK;

    CHK_RES(send_command(dev, CMD_CONVERT_D1 + dev->osr));
    wait_conversion(dev);
    CHK_RES(read_adc(dev, result));

error_exit:
    return res;
}

static GB_RESULT ms5611_reset(ms5611_t *dev)
{
    return send_command(dev, CMD_RESET);
}

/////////////////////////Public//////////////////////////////////////

GB_RESULT ms5611_init_desc(ms5611_t *dev, baro_bus_t* bus, baro_addr_t addr)
{
    GB_RESULT res = GB_OK;

    CHK_NULL(dev, GB_BARO_DEVICE_NULL);

    dev->bus = bus;
    dev->addr = addr;

error_exit:
    return res;
}

GB_RESULT ms5611_free_desc(ms5611_t *dev)
{
    return GB_OK;
}

GB_RESULT ms5611_init(ms5611_t *dev, ms5611_osr_t osr)
{
    GB_RESULT res = GB_OK;

    CHK_NULL(dev, GB_BARO_DEVICE_NULL);
    dev->osr = osr;

    // First of all we need to reset the chip
    CHK_RES(ms5611_reset(dev));
    // Wait a bit for the device to reset
    GB_SleepMs(100);
    // Get the config
    CHK_RES(read_prom(dev));

error_exit:
    return res;
}

GB_RESULT ms5611_get_sensor_data(ms5611_t *dev, float *pressure, float *temperature)
{
    GB_RESULT res = GB_OK;

    CHK_NULL(dev && pressure && temperature, GB_BARO_DEVICE_NULL);

    // Second order temperature compensation see datasheet p8
    uint32_t raw_pressure = 0;
    uint32_t raw_temperature = 0;

    CHK_RES(get_raw_pressure(dev, &raw_pressure));
    CHK_RES(get_raw_temperature(dev, &raw_temperature));

    // dT = D2 - T_ref = D2 - C5 * 2^8
    int32_t dt = raw_temperature - ((int32_t)dev->config_data.t_ref << 8);
    // Actual temperature (-40...85C with 0.01 resolution)
    // TEMP = 20C +dT * TEMPSENSE =2000 + dT * C6 / 2^23
    int64_t temp = (2000 + (int64_t)dt * dev->config_data.tempsens / 8388608);
    // Offset at actual temperature
    // OFF=OFF_t1 + TCO * dT = OFF_t1(C2) * 2^16 + (C4*dT)/2^7
    int64_t off = (int64_t)((int64_t)dev->config_data.off * 65536)
        + (((int64_t)dev->config_data.tco * dt) / 128);
    // Sensitivity at actual temperature
    // SENS=SENS_t1 + TCS *dT = SENS_t1(C1) *2^15 + (TCS(C3) *dT)/2^8
    int64_t sens = (int64_t)(((int64_t)dev->config_data.sens) * 32768)
        + (((int64_t)dev->config_data.tcs * dt) / 256);

    // Set defaults for temp >= 2000
    int64_t t_2 = 0;
    int64_t off_2 = 0;
    int64_t sens_2 = 0;
    int64_t help = 0;
    if (temp < 2000)
    {
        // Low temperature
        t_2 = ((dt * dt) >> 31); // T2 = dT^2/2^31
        help = (temp - 2000);
        help = 5 * help * help;
        off_2 = help >> 1;       // OFF_2  = 5 * (TEMP - 2000)^2/2^1
        sens_2 = help >> 2;      // SENS_2 = 5 * (TEMP - 2000)^2/2^2
        if (temp < -1500)
        {
            // Very low temperature
            help = (temp + 1500);
            help = help * help;
            off_2 = off_2 + 7 * help;             // OFF_2  = OFF_2 + 7 * (TEMP + 1500)^2
            sens_2 = sens_2 + ((11 * help) >> 1); // SENS_2 = SENS_2 + 7 * (TEMP + 1500)^2/2^1
        }
    }

    temp = temp - t_2;
    off = off - off_2;
    sens = sens - sens_2;

    // Temperature compensated pressure (10...1200mbar with 0.01mbar resolution
    // P = digital pressure value  * SENS - OFF = (D1 * SENS/2^21 -OFF)/2^15
    *pressure = (int32_t)(((int64_t)raw_pressure * (sens / 0x200000) - off) / 32768);
    *temperature = (float)temp / 100.0;

    GB_DEBUGI(SENSOR_TAG, "RAW baro_data: [0x%08x, 0x%08x]\n", raw_pressure, raw_temperature);

error_exit:
    return res;
}
