/*
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of itscontributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "qmc5883l.h"

#define REG_XOUT_L 0x00
#define REG_XOUT_H 0x01
#define REG_YOUT_L 0x02
#define REG_YOUT_H 0x03
#define REG_ZOUT_L 0x04
#define REG_ZOUT_H 0x05
#define REG_STATE  0x06
#define REG_TOUT_L 0x07
#define REG_TOUT_H 0x08
#define REG_CTRL1  0x09
#define REG_CTRL2  0x0a
#define REG_FBR    0x0b
#define REG_ID     0x0d

#define MASK_MODE  0xfe
#define MASK_ODR   0xf3

inline static GB_RESULT write_reg(qmc5883l_t *dev, uint8_t reg, uint8_t val)
{
    return dev->writeBytes(dev, dev->addr, reg, 1, &val);
}

inline static GB_RESULT read_reg(qmc5883l_t *dev, uint8_t reg, uint8_t *val)
{
    return dev->readBytes(dev, dev->addr, reg, 1, &val);
}

///////////////////////////////////////////////////////////////////////////////

GB_RESULT qmc5883l_init_desc(qmc5883l_t *dev, compass_bus_t *bus, compass_addr_t addr)
{
    GB_RESULT res = GB_OK;

    CHK_NULL(dev, GB_COMPASS_DEVICE_NULL);

    dev->bus = bus;
    dev->addr = addr;

error_exit:
    return res;
}

GB_RESULT qmc5883l_free_desc(qmc5883l_t *dev)
{
    return GB_OK;
}

GB_RESULT qmc5883l_reset(qmc5883l_t *dev)
{
    GB_RESULT res = GB_OK;

    CHK_RES(write_reg(dev, REG_CTRL2, 0x80));
    dev->range = QMC5883L_RNG_2;

error_exit:
    return res;
}

GB_RESULT qmc5883l_get_chip_id(qmc5883l_t *dev, uint8_t *id)
{
    return read_reg(dev, REG_ID, id);
}

GB_RESULT qmc5883l_set_mode(qmc5883l_t *dev, qmc5883l_mode_t mode)
{
    GB_RESULT res = GB_OK;
    uint8_t v;

    CHK_NULL(dev, GB_COMPASS_DEVICE_NULL);

    CHK_RES(read_reg(dev, REG_CTRL1, &v));
    CHK_RES(write_reg(dev, REG_CTRL1, (v & 0xfe) | mode));

error_exit:
    return res;
}

GB_RESULT qmc5883l_get_mode(qmc5883l_t *dev, qmc5883l_mode_t *mode)
{
    GB_RESULT res = GB_OK;
    uint8_t v;

    CHK_NULL(dev && mode, GB_COMPASS_DEVICE_NULL);
    CHK_RES(read_reg(dev, REG_CTRL1, &v));
    *mode = v & 1;

error_exit:
    return res;
}

GB_RESULT qmc5883l_set_config(qmc5883l_t *dev, qmc5883l_odr_t odr, qmc5883l_osr_t osr, qmc5883l_range_t rng)
{
    GB_RESULT res = GB_OK;
    uint8_t v;

    CHK_NULL(dev, GB_COMPASS_DEVICE_NULL);
    CHK_RES(read_reg(dev, REG_CTRL1, &v));
    dev->range = rng;
    CHK_RES(write_reg(dev, REG_FBR, 1)); // Define set/reset period
    CHK_RES(write_reg(dev, REG_CTRL1,
            (v & 0x03) | ((odr & 3) << 2) | ((rng & 1) << 4) | ((osr & 3) << 6)));

error_exit:
    return res;
}

GB_RESULT qmc5883l_get_config(qmc5883l_t *dev, qmc5883l_odr_t *odr, qmc5883l_osr_t *osr, qmc5883l_range_t *rng)
{
    GB_RESULT res = GB_OK;
    uint8_t v;

    CHK_NULL(dev && odr && osr && rng, GB_COMPASS_DEVICE_NULL);
    CHK_RES(read_reg(dev, REG_CTRL1, &v));
    *odr = (v >> 2) & 3;
    *osr = (v >> 6) & 3;
    *rng = (v >> 4) & 1;

error_exit:
    return res;
}

GB_RESULT qmc5883l_set_int(qmc5883l_t *dev, bool enable)
{
    return write_reg(dev, REG_CTRL2, enable ? 1 : 0);
}

GB_RESULT qmc5883l_get_int(qmc5883l_t *dev, bool *enable)
{
    GB_RESULT res = GB_OK;
    uint8_t v;

    CHK_NULL(dev && enable, GB_COMPASS_DEVICE_NULL);
    CHK_RES(read_reg(dev, REG_CTRL2, &v));
    *enable = v & 1;

error_exit:
    return res;
}

GB_RESULT qmc5883l_data_ready(qmc5883l_t *dev, bool *ready)
{
    GB_RESULT res = GB_OK;
    uint8_t v;

    CHK_NULL(dev && ready, GB_COMPASS_DEVICE_NULL);
    CHK_RES(read_reg(dev, REG_STATE, &v));
    *ready = v & 1;

error_exit:
    return res;
}

GB_RESULT qmc5883l_get_raw_data(qmc5883l_t *dev, qmc5883l_raw_data_t *raw)
{
    GB_RESULT res = GB_OK;
    uint8_t v;

    CHK_NULL(dev && raw, GB_COMPASS_DEVICE_NULL);

    CHK_RES(dev->readBytes(dev, dev->addr, REG_XOUT_L, 6, raw));

error_exit:
    return res;
}

GB_RESULT qmc5883l_raw_to_mg(qmc5883l_t *dev, qmc5883l_raw_data_t *raw, qmc5883l_data_t *data)
{
    GB_RESULT res = GB_OK;

    CHK_NULL(dev && raw && data);

    float f = (dev->range == QMC5883L_RNG_2 ? 2000.0 : 8000.0) / 32768;

    data->x = raw->x * f;
    data->y = raw->y * f;
    data->z = raw->z * f;

error_exit:
    return res;
}

GB_RESULT qmc5883l_get_data(qmc5883l_t *dev, qmc5883l_data_t *data)
{
    GB_RESULT res = GB_OK;

    qmc5883l_raw_data_t raw;
    CHK_RES(qmc5883l_get_raw_data(dev, &raw));
    CHK_RES(qmc5883l_raw_to_mg(dev, &raw, data));

error_exit:
    return res;
}

GB_RESULT qmc5883l_get_raw_temp(qmc5883l_t *dev, int16_t *temp)
{
    GB_RESULT res = GB_OK;

    CHK_NULL(dev && temp);
    CHK_RES(dev->readBytes(dev, dev->addr, REG_TOUT_L, 2, temp));

error_exit:
    return res;
}
