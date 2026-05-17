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

#include "i2s_speaker.h"

#include <math.h>
#include <string.h>
#include "driver/i2s_std.h"
#include "log_sys.h"
#include "io_define.h"

static const char *TAG = "I2S_SPK";

static i2s_chan_handle_t s_tx_handle = NULL;
static uint32_t s_sample_rate = 44100;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define I2S_DMA_BUF_LEN    1024
#define I2S_DMA_BUF_COUNT   4

GB_RESULT i2s_speaker_init(const i2s_speaker_config_t *config)
{
    if (config == NULL) return GB_I2S_INIT_FAIL;
    s_sample_rate = config->sample_rate;

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = I2S_DMA_BUF_COUNT;
    chan_cfg.dma_frame_num = I2S_DMA_BUF_LEN;

    esp_err_t err = i2s_new_channel(&chan_cfg, &s_tx_handle, NULL);
    if (err != ESP_OK) {
        GB_DEBUGE(TAG, "Failed to create I2S channel: %d", err);
        return GB_I2S_INIT_FAIL;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(config->sample_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = config->bclk_gpio,
            .ws = config->ws_gpio,
            .dout = config->dout_gpio,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    err = i2s_channel_init_std_mode(s_tx_handle, &std_cfg);
    if (err != ESP_OK) {
        GB_DEBUGE(TAG, "Failed to init I2S std mode: %d", err);
        i2s_del_channel(s_tx_handle);
        s_tx_handle = NULL;
        return GB_I2S_INIT_FAIL;
    }

    GB_DEBUGI(TAG, "I2S speaker initialized: rate=%lu, BCLK=%d, WS=%d, DOUT=%d",
    config->sample_rate, config->bclk_gpio, config->ws_gpio, config->dout_gpio);
    return GB_OK;
}

GB_RESULT i2s_speaker_deinit(void)
{
    if (s_tx_handle == NULL) return GB_OK;
    i2s_channel_disable(s_tx_handle);
    esp_err_t err = i2s_del_channel(s_tx_handle);
    s_tx_handle = NULL;
    return (err == ESP_OK) ? GB_OK : GB_I2S_DEINIT_FAIL;
}

GB_RESULT i2s_speaker_enable(void)
{
    if (s_tx_handle == NULL) return GB_I2S_ENABLE_FAIL;
    esp_err_t err = i2s_channel_enable(s_tx_handle);
    return (err == ESP_OK) ? GB_OK : GB_I2S_ENABLE_FAIL;
}

GB_RESULT i2s_speaker_disable(void)
{
    if (s_tx_handle == NULL) return GB_I2S_DISABLE_FAIL;
    esp_err_t err = i2s_channel_disable(s_tx_handle);
    return (err == ESP_OK) ? GB_OK : GB_I2S_DISABLE_FAIL;
}

GB_RESULT i2s_speaker_write(const int16_t *data, size_t num_bytes, uint32_t timeout_ms)
{
    if (s_tx_handle == NULL || data == NULL) return GB_I2S_WRITE_FAIL;
    size_t bytes_written = 0;
    esp_err_t err = i2s_channel_write(s_tx_handle, data, num_bytes, &bytes_written, timeout_ms);
    return (err == ESP_OK) ? GB_OK : GB_I2S_WRITE_FAIL;
}

void i2s_speaker_play_tone(uint32_t freq_hz, uint32_t duration_ms, uint8_t volume)
{
    if (s_tx_handle == NULL) return;

    if (volume > 100) volume = 100;
    float vol_scale = (float)volume / 100.0f;
    int16_t amplitude = (int16_t)(32767.0f * vol_scale);

    uint32_t total_samples = (s_sample_rate * duration_ms) / 1000;
    int16_t buf[I2S_DMA_BUF_LEN];
    uint32_t samples_written = 0;

    i2s_channel_enable(s_tx_handle);

    while (samples_written < total_samples) {
        uint32_t chunk = total_samples - samples_written;
        if (chunk > I2S_DMA_BUF_LEN) chunk = I2S_DMA_BUF_LEN;

        for (uint32_t i = 0; i < chunk; i++) {
            float phase = 2.0f * (float)M_PI * freq_hz * (samples_written + i) / (float)s_sample_rate;
            buf[i] = (int16_t)(amplitude * sinf(phase));
        }

        size_t bytes_written = 0;
        i2s_channel_write(s_tx_handle, buf, chunk * sizeof(int16_t), &bytes_written, 1000);
        samples_written += chunk;
    }

    i2s_channel_disable(s_tx_handle);
}

void i2s_speaker_demo(void)
{
    GB_DEBUGI(TAG, "=== I2S Speaker Demo Start ===");

    i2s_speaker_config_t cfg = {
        .bclk_gpio = I2S_SPEAKER_BCLK,
        .ws_gpio = I2S_SPEAKER_WS,
        .dout_gpio = I2S_SPEAKER_DOUT,
        .sample_rate = 44100,
    };

    if (i2s_speaker_init(&cfg) != GB_OK) {
        GB_DEBUGE(TAG, "Demo: init failed!");
        return;
    }

    // C-major arpeggio: C4, E4, G4, C5
    i2s_speaker_play_tone(262, 300, 50);
    i2s_speaker_play_tone(330, 300, 50);
    i2s_speaker_play_tone(392, 300, 50);
    i2s_speaker_play_tone(523, 500, 50);

    // A440 test tone for 1 second
    i2s_speaker_play_tone(440, 1000, 60);

    i2s_speaker_deinit();

    GB_DEBUGI(TAG, "=== I2S Speaker Demo Complete ===");
}
