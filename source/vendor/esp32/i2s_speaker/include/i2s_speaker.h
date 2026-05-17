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

#ifndef _I2S_SPEAKER_
#define _I2S_SPEAKER_

#include "results.h"
#include <stdint.h>
#include <stddef.h>

typedef struct {
    int bclk_gpio;
    int ws_gpio;
    int dout_gpio;
    uint32_t sample_rate;
} i2s_speaker_config_t;

GB_RESULT i2s_speaker_init(const i2s_speaker_config_t *config);
GB_RESULT i2s_speaker_deinit(void);
GB_RESULT i2s_speaker_enable(void);
GB_RESULT i2s_speaker_disable(void);
GB_RESULT i2s_speaker_write(const int16_t *data, size_t num_bytes, uint32_t timeout_ms);

void i2s_speaker_play_tone(uint32_t freq_hz, uint32_t duration_ms, uint8_t volume);
void i2s_speaker_demo(void);

#endif /* end of include guard: _I2S_SPEAKER_ */
