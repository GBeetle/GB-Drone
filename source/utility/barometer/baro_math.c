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

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "io_define.h"
#include "gb_timer.h"
#include "error_handle.h"

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
    return (1.0f - powf(pressure / 101325.0f, 0.190295f)) * 443300.7692f;
}

//气压计1个标准大气压校准
void performBaroCalibrationCycle(float baroPressureSamp)
{
    //慢慢收敛校准
    const float baroGroundPressureError = baroPressureSamp - baroGroundPressure;
    uint64_t getTimerMs = 0;

    baroGroundPressure += baroGroundPressureError * 0.15f;

    if (GB_OK != GB_GetTimerMs(&getTimerMs))
    {
        GB_DEBUGE(ERROR_TAG, "GB_GetTimerMs FAILED");
        return;
    }

    if (fabs(baroGroundPressureError) < (baroGroundPressure * 0.0005f))  // 0.05% calibration error (should give c. 10cm calibration error)
    {
        if ((getTimerMs - baroCalibrationTimeout) > 10000)
        {
            baroGroundAltitude = pressureToAltitude(baroGroundPressure);
            baroCalibrationFinished = true;
        }
    }
    else
    {
        baroCalibrationTimeout = getTimerMs;
        //GB_DEBUGE(ERROR_TAG, "Start Calibration");
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
