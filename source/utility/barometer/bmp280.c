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

#include "bmp280.h"
#include "gb_timer.h"

static inline GB_RESULT read_register16(bmp_bus_t *dev, bmp_addr_handle_t addr, uint8_t reg, uint16_t *r)
{
    return dev->readBytes(dev, addr, reg, 2, (uint8_t*)r);
}

static inline GB_RESULT read_register8(bmp_bus_t *dev, bmp_addr_handle_t addr, uint8_t reg, uint8_t *r)
{
    return dev->readByte(dev, addr, reg, (uint8_t*)r);
}

inline static GB_RESULT write_register8(bmp_bus_t *dev, bmp_addr_handle_t addr, uint8_t reg, uint8_t value)
{
    return dev->writeByte(dev, addr, reg, value);
}

static GB_RESULT read_calibration_data(bmp280_t *dev)
{
    GB_RESULT res = GB_OK;

    CHK_RES(read_register16(dev->bus, dev->addr, 0x88, &dev->dig_T1));
    CHK_RES(read_register16(dev->bus, dev->addr, 0x8a, (uint16_t *)&dev->dig_T2));
    CHK_RES(read_register16(dev->bus, dev->addr, 0x8c, (uint16_t *)&dev->dig_T3));
    CHK_RES(read_register16(dev->bus, dev->addr, 0x8e, &dev->dig_P1));
    CHK_RES(read_register16(dev->bus, dev->addr, 0x90, (uint16_t *)&dev->dig_P2));
    CHK_RES(read_register16(dev->bus, dev->addr, 0x92, (uint16_t *)&dev->dig_P3));
    CHK_RES(read_register16(dev->bus, dev->addr, 0x94, (uint16_t *)&dev->dig_P4));
    CHK_RES(read_register16(dev->bus, dev->addr, 0x96, (uint16_t *)&dev->dig_P5));
    CHK_RES(read_register16(dev->bus, dev->addr, 0x98, (uint16_t *)&dev->dig_P6));
    CHK_RES(read_register16(dev->bus, dev->addr, 0x9a, (uint16_t *)&dev->dig_P7));
    CHK_RES(read_register16(dev->bus, dev->addr, 0x9c, (uint16_t *)&dev->dig_P8));
    CHK_RES(read_register16(dev->bus, dev->addr, 0x9e, (uint16_t *)&dev->dig_P9));

    GB_DEBUGD(BMP_TAG, "Calibration data received:");
    GB_DEBUGD(BMP_TAG, "dig_T1=%d", dev->dig_T1);
    GB_DEBUGD(BMP_TAG, "dig_T2=%d", dev->dig_T2);
    GB_DEBUGD(BMP_TAG, "dig_T3=%d", dev->dig_T3);
    GB_DEBUGD(BMP_TAG, "dig_P1=%d", dev->dig_P1);
    GB_DEBUGD(BMP_TAG, "dig_P2=%d", dev->dig_P2);
    GB_DEBUGD(BMP_TAG, "dig_P3=%d", dev->dig_P3);
    GB_DEBUGD(BMP_TAG, "dig_P4=%d", dev->dig_P4);
    GB_DEBUGD(BMP_TAG, "dig_P5=%d", dev->dig_P5);
    GB_DEBUGD(BMP_TAG, "dig_P6=%d", dev->dig_P6);
    GB_DEBUGD(BMP_TAG, "dig_P7=%d", dev->dig_P7);
    GB_DEBUGD(BMP_TAG, "dig_P8=%d", dev->dig_P8);
    GB_DEBUGD(BMP_TAG, "dig_P9=%d", dev->dig_P9);
error_exit:
    return res;
}

static GB_RESULT read_hum_calibration_data(bmp280_t *dev)
{
    uint16_t h4, h5;
    GB_RESULT res = GB_OK;

    CHK_RES(read_register8(dev->bus, dev->addr, 0xa1, &dev->dig_H1));
    CHK_RES(read_register16(dev->bus, dev->addr, 0xe1, (uint16_t *)&dev->dig_H2));
    CHK_RES(read_register8(dev->bus, dev->addr, 0xe3, &dev->dig_H3));
    CHK_RES(read_register16(dev->bus, dev->addr, 0xe4, &h4));
    CHK_RES(read_register16(dev->bus, dev->addr, 0xe5, &h5));
    CHK_RES(read_register8(dev->bus, dev->addr, 0xe7, (uint8_t *)&dev->dig_H6));

    dev->dig_H4 = (h4 & 0x00ff) << 4 | (h4 & 0x0f00) >> 8;
    dev->dig_H5 = h5 >> 4;
    GB_DEBUGD(BMP_TAG, "Calibration data received:");
    GB_DEBUGD(BMP_TAG, "dig_H1=%d", dev->dig_H1);
    GB_DEBUGD(BMP_TAG, "dig_H2=%d", dev->dig_H2);
    GB_DEBUGD(BMP_TAG, "dig_H3=%d", dev->dig_H3);
    GB_DEBUGD(BMP_TAG, "dig_H4=%d", dev->dig_H4);
    GB_DEBUGD(BMP_TAG, "dig_H5=%d", dev->dig_H5);
    GB_DEBUGD(BMP_TAG, "dig_H6=%d", dev->dig_H6);
error_exit:
    return res;
}

GB_RESULT bmp280_init_desc(bmp280_t *dev, bmp_bus_t* bus, bmp_addr_handle_t addr)
{
    GB_RESULT res = GB_OK;

    CHK_NULL(dev, GB_BARO_DEVICE_NULL);
    dev->bus = bus;
    dev->addr = addr;
error_exit:
    return res;
}

GB_RESULT bmp280_init_default_params(bmp280_params_t *params)
{
    GB_RESULT res = GB_OK;
    CHK_NULL(params, GB_BARO_DEVICE_NULL);

    params->mode = BMP280_MODE_NORMAL;
    params->filter = BMP280_FILTER_OFF;
    params->oversampling_pressure = BMP280_STANDARD;
    params->oversampling_temperature = BMP280_STANDARD;
    params->oversampling_humidity = BMP280_STANDARD;
    params->standby = BMP280_STANDBY_250;
error_exit:
    return res;
}

GB_RESULT bmp280_init(bmp280_t *dev, bmp280_params_t *params)
{
    GB_RESULT res = GB_OK;

#if defined CONFIG_BARO_SPI
    spi_device_handle_t bmp_spi_handle;
    // Initialize SPI on HSPI host through SPIbus interface:
    init_spi(&hspi, SPI3_HOST);
    // disable SPI DMA in begin
    CHK_LOGE(hspi.begin(&hspi, BMP_HSPI_MOSI, BMP_HSPI_MISO, BMP_HSPI_SCLK, SPI_MAX_DMA_LEN), "hspi begin fail");
    CHK_LOGE(hspi.addDevice(&hspi, 8, 0, 0, BMP_SPI_CLOCK_SPEED, BMP_HSPI_CS, &bmp_spi_handle), "hspi addDevice fail");

    CHK_LOGE(bmp280_init_desc(dev, &hspi, bmp_spi_handle), "bmp280_init_desc fail");
#endif

#if defined CONFIG_BARO_I2C
    init_i2c(&i2c1, I2C_NUM_1);
    CHK_LOGE(i2c1.begin(&i2c1, BMP_SDA, BMP_SCL, BMP_I2C_CLOCK_SPEED), "i2c1 begin fail");
    bmp_i2caddr_t  BMP_DEFAULT_I2CADDRESS = BMP280_I2C_ADDRESS_1;

    CHK_LOGE(bmp280_init_desc(dev, &i2c1, BMP_DEFAULT_I2CADDRESS), "bmp280_init_desc fail");
#endif

    CHK_NULL(dev && params, GB_BARO_DEVICE_NULL);
    CHK_NULL(dev->bus, GB_BARO_DEVICE_BUS_NULL);

    CHK_LOGE(read_register8(dev->bus, dev->addr, BMP280_REG_ID, &dev->id), "bmp280 Sensor not found");

    if (dev->id != BMP280_CHIP_ID && dev->id != BME280_CHIP_ID)
    {
        CHK_LOGE(GB_BARO_DEV_ID_ERROR,
                "Invalid chip ID: expected: 0x%x (BME280) or 0x%x (BMP280) got: 0x%x",
                BME280_CHIP_ID, BMP280_CHIP_ID, dev->id);
    }

    // Soft reset.
    CHK_LOGE(write_register8(dev->bus, dev->addr, BMP280_REG_RESET, BMP280_RESET_VALUE), "Failed to reset sensor");

    // Wait until finished copying over the NVM data.
    while (1)
    {
        uint8_t status;
        if (!read_register8(dev->bus, dev->addr, BMP280_REG_STATUS, &status) && (status & 1) == 0)
            break;
    }

    CHK_LOGE(read_calibration_data(dev), "Failed to read calibration data");

    if (dev->id == BME280_CHIP_ID)
    {
        CHK_LOGE(read_hum_calibration_data(dev), "Failed to read humidity calibration data");
    }

    uint8_t config = (params->standby << 5) | (params->filter << 2);
    GB_DEBUGD(BMP_TAG, "Writing config reg=%x", config);

    CHK_LOGE(write_register8(dev->bus, dev->addr, BMP280_REG_CONFIG, config), "Failed to configure sensor");

    if (params->mode == BMP280_MODE_FORCED)
    {
        params->mode = BMP280_MODE_SLEEP;  // initial mode for forced is sleep
    }

    uint8_t ctrl = (params->oversampling_temperature << 5) | (params->oversampling_pressure << 2) | (params->mode);

    if (dev->id == BME280_CHIP_ID)
    {
        // Write crtl hum reg first, only active after write to BMP280_REG_CTRL.
        uint8_t ctrl_hum = params->oversampling_humidity;
        GB_DEBUGD(BMP_TAG, "Writing ctrl hum reg=%x", ctrl_hum);
        CHK_LOGE(write_register8(dev->bus, dev->addr, BMP280_REG_CTRL_HUM, ctrl_hum), "Failed to control sensor");
    }

    GB_DEBUGD(BMP_TAG, "Writing ctrl reg=%x", ctrl);
    CHK_LOGE(write_register8(dev->bus, dev->addr, BMP280_REG_CTRL, ctrl), "Failed to control sensor");
error_exit:
    return res;
}

GB_RESULT bmp280_force_measurement(bmp280_t *dev)
{
    GB_RESULT res = GB_OK;
    CHK_NULL(dev, GB_BARO_DEVICE_NULL);

    uint8_t ctrl;
    CHK_RES(read_register8(dev->bus, dev->addr, BMP280_REG_CTRL, &ctrl));
    ctrl &= ~0b11;  // clear two lower bits
    ctrl |= BMP280_MODE_FORCED;
    GB_DEBUGD(BMP_TAG, "Writing ctrl reg=%x", ctrl);
    CHK_LOGE(write_register8(dev->bus, dev->addr, BMP280_REG_CTRL, ctrl), "Failed to start forced mode");
error_exit:
    return res;
}

GB_RESULT bmp280_is_measuring(bmp280_t *dev, bool *busy)
{
    GB_RESULT res = GB_OK;
    CHK_NULL(dev && busy, GB_BARO_DEVICE_NULL);

    const uint8_t regs[2] = { BMP280_REG_STATUS, BMP280_REG_CTRL };
    uint8_t status[2];
    CHK_LOGE(read_register8(dev->bus, dev->addr, regs[0], &status[0]), "Failed to read status[0] registers");
    CHK_LOGE(read_register8(dev->bus, dev->addr, regs[1], &status[1]), "Failed to read status[1] registers");

    // CHK_RES mode - FORCED means BM280 is busy (it switches to SLEEP mode when finished)
    // Additionally, CHK_RES 'measuring' bit in status register
    *busy = ((status[1] & 0b11) == BMP280_MODE_FORCED) || (status[0] & (1 << 3));
error_exit:
    return res;
}

/**
 * Compensation algorithm is taken from BMP280 datasheet.
 *
 * Return value is in degrees Celsius.
 */
int32_t compensate_temperature(bmp280_t *dev, int32_t adc_temp, int32_t *fine_temp)
{
    int32_t var1, var2;

    var1 = ((((adc_temp >> 3) - ((int32_t)dev->dig_T1 << 1))) * (int32_t)dev->dig_T2) >> 11;
    var2 = (((((adc_temp >> 4) - (int32_t)dev->dig_T1) * ((adc_temp >> 4) - (int32_t)dev->dig_T1)) >> 12) * (int32_t)dev->dig_T3) >> 14;

    *fine_temp = var1 + var2;
    return (*fine_temp * 5 + 128) >> 8;
}

/**
 * Compensation algorithm is taken from BMP280 datasheet.
 *
 * Return value is in Pa, 24 integer bits and 8 fractional bits.
 */
uint32_t compensate_pressure(bmp280_t *dev, int32_t adc_press, int32_t fine_temp)
{
    int64_t var1, var2, p;

    var1 = (int64_t)fine_temp - 128000;
    var2 = var1 * var1 * (int64_t)dev->dig_P6;
    var2 = var2 + ((var1 * (int64_t)dev->dig_P5) << 17);
    var2 = var2 + (((int64_t)dev->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dev->dig_P3) >> 8) + ((var1 * (int64_t)dev->dig_P2) << 12);
    var1 = (((int64_t)1 << 47) + var1) * ((int64_t)dev->dig_P1) >> 33;

    if (var1 == 0)
    {
        return 0;  // avoid exception caused by division by zero
    }

    p = 1048576 - adc_press;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((int64_t)dev->dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((int64_t)dev->dig_P8 * p) >> 19;

    p = ((p + var1 + var2) >> 8) + ((int64_t)dev->dig_P7 << 4);
    return p;
}

/**
 * Compensation algorithm is taken from BME280 datasheet.
 *
 * Return value is in Pa, 24 integer bits and 8 fractional bits.
 */
static inline uint32_t compensate_humidity(bmp280_t *dev, int32_t adc_hum, int32_t fine_temp)
{
    int32_t v_x1_u32r;

    v_x1_u32r = fine_temp - (int32_t)76800;
    v_x1_u32r = ((((adc_hum << 14) - ((int32_t)dev->dig_H4 << 20) - ((int32_t)dev->dig_H5 * v_x1_u32r)) + (int32_t)16384) >> 15)
            * (((((((v_x1_u32r * (int32_t)dev->dig_H6) >> 10) * (((v_x1_u32r * (int32_t)dev->dig_H3) >> 11) + (int32_t)32768)) >> 10)
                    + (int32_t)2097152) * (int32_t)dev->dig_H2 + 8192) >> 14);
    v_x1_u32r = v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * (int32_t)dev->dig_H1) >> 4);
    v_x1_u32r = v_x1_u32r < 0 ? 0 : v_x1_u32r;
    v_x1_u32r = v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r;
    return v_x1_u32r >> 12;
}

GB_RESULT bmp280_read_fixed(bmp280_t *dev, int32_t *temperature, uint32_t *pressure, uint32_t *humidity)
{
    GB_RESULT res = GB_OK;

    CHK_NULL(dev && temperature && pressure, GB_BARO_DEVICE_NULL);

    int32_t adc_pressure;
    int32_t adc_temp;
    uint8_t data[8];

    // Only the BME280 supports reading the humidity.
    if (dev->id != BME280_CHIP_ID)
    {
        if (humidity)
            *humidity = 0;
        humidity = NULL;
    }

    // Need to read in one sequence to ensure they match.
    size_t size = humidity ? 8 : 6;
    CHK_LOGE(dev->bus->readBytes(dev->bus, dev->addr, 0xf7, size, data), "Failed to read data");

    adc_pressure = data[0] << 12 | data[1] << 4 | data[2] >> 4;
    adc_temp = data[3] << 12 | data[4] << 4 | data[5] >> 4;
    GB_DEBUGD(BMP_TAG, "ADC temperature: %d", adc_temp);
    GB_DEBUGD(BMP_TAG, "ADC pressure: %d", adc_pressure);

    int32_t fine_temp;
    *temperature = compensate_temperature(dev, adc_temp, &fine_temp);
    *pressure = compensate_pressure(dev, adc_pressure, fine_temp);

    if (humidity)
    {
        int32_t adc_humidity = data[6] << 8 | data[7];
        GB_DEBUGD(BMP_TAG, "ADC humidity: %d", adc_humidity);
        *humidity = compensate_humidity(dev, adc_humidity, fine_temp);
    }
error_exit:
    return res;
}

GB_RESULT bmp280_read_float(bmp280_t *dev, float *temperature, float *pressure, float *humidity)
{
    GB_RESULT res = GB_OK;
    int32_t fixed_temperature;
    uint32_t fixed_pressure;
    uint32_t fixed_humidity;
    CHK_RES(bmp280_read_fixed(dev, &fixed_temperature, &fixed_pressure, humidity ? &fixed_humidity : NULL));
    *temperature = (float)fixed_temperature / 100;
    *pressure = (float)fixed_pressure / 256;
    if (humidity)
        *humidity = (float)fixed_humidity / 1024;
error_exit:
    return res;
}
