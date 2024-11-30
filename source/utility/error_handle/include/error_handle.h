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
#ifndef _ERROR_HANDLE__
#define _ERROR_HANDLE__

#include "results.h"
#include "log_sys.h"

#define CHK_RES(val) do {           \
        if (val != GB_OK) {         \
            res = val;              \
            GB_DEBUGE(CHK_TAG, "[CHK_RES] failed at file: %s, func: %s, line: %d, res = %08x", __FILE__, __FUNCTION__, __LINE__, val); \
            goto error_exit;        \
        }                           \
    } while(0)

#define CHK_BOOL(val) do {          \
        if (!(val)) {                 \
            GB_DEBUGE(CHK_TAG, "[CHK_BOOL] failed at file: %s, func: %s, line: %d, val = %d", __FILE__, __FUNCTION__, __LINE__, val); \
            res = GB_CHK_BOOL_FAIL; \
            goto error_exit;        \
        }                           \
    } while(0)

#define CHK_NULL(val, error_code) do {         \
        if (val == NULL) {         \
            GB_DEBUGE(CHK_TAG, "[CHK_NULL] failed at file: %s, func: %s, line: %d, res = %08x", __FILE__, __FUNCTION__, __LINE__, error_code); \
            res = error_code;      \
            goto error_exit;       \
        }                          \
    } while(0)

#define CHK_LOGE(x, msg, ...) do { \
        GB_RESULT __ = x; \
        if (__ != GB_OK) { \
            GB_DEBUGE(CHK_TAG, msg, ## __VA_ARGS__); \
        } \
    } while (0)

#define CHK_VAL(val) do {           \
        if (val != GB_OK) {         \
            GB_DEBUGE(CHK_TAG, "[CHK_VAL] failed at file: %s, func: %s, line: %d, res = %08x", __FILE__, __FUNCTION__, __LINE__, val); \
        }                           \
    } while(0)

#define CHK_EXIT(val) do {          \
        if (val != GB_OK) {         \
            GB_DEBUGE(CHK_TAG, "[CHK_EXIT] failed at file: %s, func: %s, line: %d, res = %08x", __FILE__, __FUNCTION__, __LINE__, val); \
            return;                 \
        }                           \
    } while(0)

#define CHK_ESP_ERROR(val, res) do {          \
        if (val != ESP_OK) {                  \
            GB_DEBUGE(CHK_TAG, "[CHK_ESP_ERROR] failed at file: %s, func: %s, line: %d, res = %08x", __FILE__, __FUNCTION__, __LINE__, val); \
            goto error_exit;                  \
        }                                     \
    } while(0)

#define CHK_GB_ERROR(val) do {                \
        if (val != GB_OK) {                   \
            GB_DEBUGE(CHK_TAG, "[CHK_GB_ERROR] failed at file: %s, func: %s, line: %d, res = %08x", __FILE__, __FUNCTION__, __LINE__, val); \
            return val;                       \
        }                                     \
    } while(0)

#endif /* end of include guard: _ERROR_HANDLE__ */
