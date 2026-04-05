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
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "lora_state.h"

#define DEFAULT_VREF 0
#define ADC_12BITS_SAMPLE_MAX 0xfff

static uint16_t sample_middle;

static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t adc_cali_handle = NULL;

static const adc_unit_t adc_unit = ADC_UNIT_1;
static const adc_atten_t atten = ADC_ATTEN_DB_11;

static const adc_channel_t throttle_ch = ADC_CHANNEL_0;
static const adc_channel_t yaw_ch = ADC_CHANNEL_1;
static const adc_channel_t pitch_ch = ADC_CHANNEL_3;
static const adc_channel_t roll_ch = ADC_CHANNEL_4;

void adc_wrapper_init(void)
{
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = adc_unit,
    };
    adc_oneshot_new_unit(&init_config, &adc_handle);

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = atten,
    };

    adc_oneshot_config_channel(adc_handle, throttle_ch, &config);
    adc_oneshot_config_channel(adc_handle, yaw_ch, &config);
    adc_oneshot_config_channel(adc_handle, pitch_ch, &config);
    adc_oneshot_config_channel(adc_handle, roll_ch, &config);


#if 0
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = adc_unit,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_12,
    };
    adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle);
#endif

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
    int raw = 0;
    adc_channel_t ch;

    *adc_val = 0;
    if (item >= ADC_TYPE_MAX)
        return;

    switch (item)
    {
    case ADC_THROTTLE:
        ch = throttle_ch;
        break;
    case ADC_YAW:
        ch = yaw_ch;
        break;
    case ADC_PITCH:
        ch = pitch_ch;
        break;
    case ADC_ROLL:
        ch = roll_ch;
        break;
    default:
        return;
    }

    adc_oneshot_read(adc_handle, ch, &raw);

    if (raw >= 0 && is_constrained)
    {
        if (ADC_THROTTLE == item)
            *adc_val = _constrain_adc_output(raw);
        else
            *adc_val = _constrain_adc_by_section(raw);
    }
    else
    {
        *adc_val = raw;
    }
}