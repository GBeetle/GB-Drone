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

bmp280_t bmp280_device;

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

    CHK_NULL(dev, GB_BMP_DEVICE_NULL);
    dev->bus = bus;
    dev->addr = addr;
error_exit:
    return res;
}

GB_RESULT bmp280_init_default_params(bmp280_params_t *params)
{
    GB_RESULT res = GB_OK;
    CHK_NULL(params, GB_BMP_DEVICE_NULL);

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

#if defined CONFIG_BMP_SPI
    spi_device_handle_t bmp_spi_handle;
    // Initialize SPI on HSPI host through SPIbus interface:
    init_spi(&hspi, SPI3_HOST);
    // disable SPI DMA in begin
    CHK_LOGE(hspi.begin(&hspi, BMP_HSPI_MOSI, BMP_HSPI_MISO, BMP_HSPI_SCLK, SPI_MAX_DMA_LEN), "hspi begin fail");
    CHK_LOGE(hspi.addDevice(&hspi, 8, 0, 0, BMP_SPI_CLOCK_SPEED, BMP_HSPI_CS, &bmp_spi_handle), "hspi addDevice fail");

    CHK_LOGE(bmp280_init_desc(&bmp280_device, &hspi, bmp_spi_handle), "bmp280_init_desc fail");
#endif

#if defined CONFIG_BMP_I2C
    init_i2c(&i2c1, I2C_NUM_1);
    CHK_LOGE(i2c1.begin(&i2c1, BMP_SDA, BMP_SCL, BMP_I2C_CLOCK_SPEED), "i2c1 begin fail");
    bmp_i2caddr_t  BMP_DEFAULT_I2CADDRESS = BMP280_I2C_ADDRESS_1;

    CHK_LOGE(bmp280_init_desc(&bmp280_device, &i2c1, BMP_DEFAULT_I2CADDRESS), "bmp280_init_desc fail");
#endif

    CHK_NULL(dev && params, GB_BMP_DEVICE_NULL);
    CHK_NULL(dev->bus, GB_BMP_DEVICE_BUS_NULL);

    CHK_LOGE(read_register8(dev->bus, dev->addr, BMP280_REG_ID, &dev->id), "bmp280 Sensor not found");

    if (dev->id != BMP280_CHIP_ID && dev->id != BME280_CHIP_ID)
    {
        CHK_LOGE(GB_BMP_DEV_ID_ERROR,
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
    CHK_NULL(dev, GB_BMP_DEVICE_NULL);

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
    CHK_NULL(dev && busy, GB_BMP_DEVICE_NULL);

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

    CHK_NULL(dev && temperature && pressure, GB_BMP_DEVICE_NULL);

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

// Quick median filter implementation
// (c) N. Devillard - 1998
// http://ndevilla.free.fr/median/median.pdf
#define QMF_SORT(type,a,b) { if ((a)>(b)) QMF_SWAP(type, (a),(b)); }
#define QMF_SWAP(type,a,b) { type temp=(a);(a)=(b);(b)=temp; }

int32_t quickMedianFilter3(int32_t * v)
{
    int32_t p[3];
    memcpy(p, v, sizeof(p));

    QMF_SORT(int32_t, p[0], p[1]); QMF_SORT(int32_t, p[1], p[2]); QMF_SORT(int32_t, p[0], p[1]) ;
    return p[1];
}

int16_t quickMedianFilter3_16(int16_t * v)
{
    int16_t p[3];
    memcpy(p, v, sizeof(p));

    QMF_SORT(int16_t, p[0], p[1]); QMF_SORT(int16_t, p[1], p[2]); QMF_SORT(int16_t, p[0], p[1]) ;
    return p[1];
}

int32_t quickMedianFilter5(int32_t * v)
{
    int32_t p[5];
    memcpy(p, v, sizeof(p));

    QMF_SORT(int32_t, p[0], p[1]); QMF_SORT(int32_t, p[3], p[4]); QMF_SORT(int32_t, p[0], p[3]);
    QMF_SORT(int32_t, p[1], p[4]); QMF_SORT(int32_t, p[1], p[2]); QMF_SORT(int32_t, p[2], p[3]);
    QMF_SORT(int32_t, p[1], p[2]);
    return p[2];
}

int16_t quickMedianFilter5_16(int16_t * v)
{
    int16_t p[5];
    memcpy(p, v, sizeof(p));

    QMF_SORT(int16_t, p[0], p[1]); QMF_SORT(int16_t, p[3], p[4]); QMF_SORT(int16_t, p[0], p[3]);
    QMF_SORT(int16_t, p[1], p[4]); QMF_SORT(int16_t, p[1], p[2]); QMF_SORT(int16_t, p[2], p[3]);
    QMF_SORT(int16_t, p[1], p[2]);
    return p[2];
}

int32_t quickMedianFilter7(int32_t * v)
{
    int32_t p[7];
    memcpy(p, v, sizeof(p));

    QMF_SORT(int32_t, p[0], p[5]); QMF_SORT(int32_t, p[0], p[3]); QMF_SORT(int32_t, p[1], p[6]);
    QMF_SORT(int32_t, p[2], p[4]); QMF_SORT(int32_t, p[0], p[1]); QMF_SORT(int32_t, p[3], p[5]);
    QMF_SORT(int32_t, p[2], p[6]); QMF_SORT(int32_t, p[2], p[3]); QMF_SORT(int32_t, p[3], p[6]);
    QMF_SORT(int32_t, p[4], p[5]); QMF_SORT(int32_t, p[1], p[4]); QMF_SORT(int32_t, p[1], p[3]);
    QMF_SORT(int32_t, p[3], p[4]);
    return p[3];
}

int32_t quickMedianFilter9(int32_t * v)
{
    int32_t p[9];
    memcpy(p, v, sizeof(p));

    QMF_SORT(int32_t, p[1], p[2]); QMF_SORT(int32_t, p[4], p[5]); QMF_SORT(int32_t, p[7], p[8]);
    QMF_SORT(int32_t, p[0], p[1]); QMF_SORT(int32_t, p[3], p[4]); QMF_SORT(int32_t, p[6], p[7]);
    QMF_SORT(int32_t, p[1], p[2]); QMF_SORT(int32_t, p[4], p[5]); QMF_SORT(int32_t, p[7], p[8]);
    QMF_SORT(int32_t, p[0], p[3]); QMF_SORT(int32_t, p[5], p[8]); QMF_SORT(int32_t, p[4], p[7]);
    QMF_SORT(int32_t, p[3], p[6]); QMF_SORT(int32_t, p[1], p[4]); QMF_SORT(int32_t, p[2], p[5]);
    QMF_SORT(int32_t, p[4], p[7]); QMF_SORT(int32_t, p[4], p[2]); QMF_SORT(int32_t, p[6], p[4]);
    QMF_SORT(int32_t, p[4], p[2]);
    return p[4];
}

void arraySubInt32(int32_t *dest, int32_t *array1, int32_t *array2, int count)
{
    for (int i = 0; i < count; i++) {
        dest[i] = array1[i] - array2[i];
    }
}

static uint32_t baroCalibrationTimeout = 0;
static bool baroCalibrationFinished = false;
static float baroGroundAltitude = 0;
static float baroGroundPressure = 101325.0f; // 101325 pascal, 1 standard atmosphere

bool isBaroCalibrationFinished()
{
    return baroCalibrationFinished;
}

float getBaroGroundAltitude()
{
    return baroGroundAltitude;
}

//气压值转换为海拔
/**
 * @brief h = (T0 / L) * (1 - (P / P0)^(R * L / g))
    h 是海拔高度（单位可以是米、英尺等）。
    T0 是标准温度，通常是288.15K。
    L 是标准温度梯度，通常是0.0065 K/m。
    P 是您从传感器读取的气压值（Pa）。
    P0 是海平面上的标准气压（101325 Pa）。
    R 是气体常数，通常是8.314 J/(mol·K)。
    g 是地球表面的重力加速度，通常是9.80665 m/s²。
 *
 */
float pressureToAltitude(const float pressure)
{
    return (1.0f - pow(pressure / 101325.0f, 0.190295f)) * 443300.7692f;
}

//气压计1个标准大气压校准
void performBaroCalibrationCycle(float baroPressureSamp)
{
    //慢慢收敛校准
    const float baroGroundPressureError = baroPressureSamp - baroGroundPressure;
    baroGroundPressure += baroGroundPressureError * 0.15f;

    if (fabs(baroGroundPressureError) < (baroGroundPressure * 0.00005f))  // 0.005% calibration error (should give c. 10cm calibration error)
    {
        if ((esp_timer_get_time() - baroCalibrationTimeout) > 1000)
        {
            baroGroundAltitude = pressureToAltitude(baroGroundPressure);
            baroCalibrationFinished = true;
        }
    }
    else
    {
        baroCalibrationTimeout = esp_timer_get_time();
    }
}

#define PRESSURE_SAMPLES_MEDIAN 3

/*
altitude pressure
      0m   101325Pa
    100m   100129Pa delta = 1196
   1000m    89875Pa
   1100m    88790Pa delta = 1085
At sea level an altitude change of 100m results in a pressure change of 1196Pa, at 1000m pressure change is 1085Pa
So set glitch threshold at 1000 - this represents an altitude change of approximately 100m.
*/
#define PRESSURE_DELTA_GLITCH_THRESHOLD 1000

int32_t applyBarometerMedianFilter(int32_t newPressureReading)
{
    static int32_t barometerFilterSamples[PRESSURE_SAMPLES_MEDIAN];
    static int currentFilterSampleIndex = 0;
    static bool medianFilterReady = false;

    int nextSampleIndex = currentFilterSampleIndex + 1;
    if (nextSampleIndex == PRESSURE_SAMPLES_MEDIAN)
    {
        nextSampleIndex = 0;
        medianFilterReady = true;
    }
    int previousSampleIndex = currentFilterSampleIndex - 1;
    if (previousSampleIndex < 0)
    {
        previousSampleIndex = PRESSURE_SAMPLES_MEDIAN - 1;
    }
    const int previousPressureReading = barometerFilterSamples[previousSampleIndex];

    if (medianFilterReady)
    {
        if (fabs(previousPressureReading - newPressureReading) < PRESSURE_DELTA_GLITCH_THRESHOLD)
        {
            barometerFilterSamples[currentFilterSampleIndex] = newPressureReading;
            currentFilterSampleIndex = nextSampleIndex;
            return quickMedianFilter3(barometerFilterSamples);
        }
        else
        {
            // glitch threshold exceeded, so just return previous reading and don't add the glitched reading to the filter array
            return barometerFilterSamples[previousSampleIndex];
        }
    }
    else
    {
        barometerFilterSamples[currentFilterSampleIndex] = newPressureReading;
        currentFilterSampleIndex = nextSampleIndex;
        return newPressureReading;
    }
}
