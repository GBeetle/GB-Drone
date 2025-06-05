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

#include "controller.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "lora_state.h"

#define DEFAULT_VREF 0
#define ADC_12BITS_SAMPLE_MAX 0xfff

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_bits_width_t adc_width = ADC_WIDTH_BIT_12;
static const adc_unit_t adc1_unit = ADC_UNIT_1;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_channel_t throttle_ch = ADC_CHANNEL_0; // GPIO0 => ADC1_CHANNEL0
static const adc_channel_t yaw_ch = ADC_CHANNEL_1;      // GPIO1 => ADC1_CHANNEL1
static const adc_channel_t pitch_ch = ADC_CHANNEL_4;    // GPIO15 => ADC2_CHANNEL4
static const adc_channel_t roll_ch = ADC_CHANNEL_5;     // GPIO16 => ADC2_CHANNEL5

static uint16_t sample_middle;

void adc_wrapper_init(void)
{
    adc1_config_width(adc_width);
    adc1_config_channel_atten(yaw_ch, atten);
    adc1_config_channel_atten(throttle_ch, atten);
    adc2_config_channel_atten((adc2_channel_t)roll_ch, atten);
    adc2_config_channel_atten((adc2_channel_t)pitch_ch, atten);

    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(adc1_unit, atten, adc_width, DEFAULT_VREF, adc_chars);

    // yaw pitch roll 通道分别采样10次，作为中间基准值
    uint16_t sample_sum = 0;
    for (int i = 0; i < 10; i++)
    {
        uint16_t adc_val;

        adc_read_by_item(ADC_YAW, &adc_val, false);
        sample_sum += adc_val;
        adc_read_by_item(ADC_PITCH, &adc_val, false);
        sample_sum += adc_val;
        adc_read_by_item(ADC_ROLL, &adc_val, false);
        sample_sum += adc_val;
    }
    sample_middle = sample_sum / 30;
}

// constrain output to 0 ~ ADC_CONSTRAIN_MAX(1000)
static uint16_t _constrain_adc_output(int origin)
{
    return origin / (ADC_12BITS_SAMPLE_MAX / (float)ADC_CONSTRAIN_MAX);
}

//      0 ~ middle                => 0 ~ ADC_CONSTRAIN_MAX/2
// middle ~ ADC_12BITS_SAMPLE_MAX => ADC_CONSTRAIN_MAX/2 ~ ADC_CONSTRAIN_MAX
static uint16_t _constrain_adc_by_section(int origin)
{
    if (origin <= sample_middle)
        return origin / (sample_middle / ((float)ADC_CONSTRAIN_MAX / 2));
    else
    {
        uint16_t section = ADC_12BITS_SAMPLE_MAX - sample_middle;
        origin -= sample_middle;
        return (ADC_CONSTRAIN_MAX / 2) + origin / ((float)section / (ADC_CONSTRAIN_MAX / 2));
    }
}

/*
 * This function is used to read adc data by item
 * @item: defined in ADC_SAMPLE_ITEM
 * @adc_val: read value, range 0 ~ ADC_CONSTRAIN_MAX
 */
void adc_read_by_item(uint8_t item, uint16_t *adc_val, uint8_t is_constrained)
{
    int adc_val_origin = 0;

    *adc_val = 0;
    if (item >= ADC_TYPE_MAX)
        return;
    switch (item)
    {
    case ADC_THROTTLE:
        adc_val_origin = adc1_get_raw((adc1_channel_t)throttle_ch);
        break;
    case ADC_YAW:
        adc_val_origin = adc1_get_raw((adc1_channel_t)yaw_ch);
        break;
    case ADC_PITCH:
        adc2_get_raw((adc2_channel_t)pitch_ch, adc_width, (int *)&adc_val_origin);
        break;
    case ADC_ROLL:
        adc2_get_raw((adc2_channel_t)roll_ch, adc_width, (int *)&adc_val_origin);
        break;
    default:
        adc_val_origin = -1;
    }

    if (adc_val_origin >= 0 && is_constrained)
    {
        if (ADC_THROTTLE == item)
            *adc_val = _constrain_adc_output(adc_val_origin);
        else
            *adc_val = _constrain_adc_by_section(adc_val_origin);
    }
    else if (!is_constrained)
        *adc_val = adc_val_origin;
}
