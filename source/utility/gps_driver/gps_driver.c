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

#include <string.h>
#include "log_sys.h"
#include "uart_bus.h"
#include "gps_driver.h"
#include "io_define.h"
#include "minmea.h"
#include "gb_timer.h"

typedef struct __attribute__((packed)) {
    uint32_t iTOW;
    uint16_t year;
    uint8_t  month, day, hour, min, sec;
    uint8_t  valid;
    uint32_t tAcc;
    int32_t  nano;
    uint8_t  fixType;
    uint8_t  flags;
    uint8_t  flags2;
    uint8_t  numSV;
    int32_t  lon;
    int32_t  lat;
    int32_t  height;
    int32_t  hMSL;
    int32_t  hAcc;
    int32_t  vAcc;
} GB_UBX_T;

typedef enum {
    UBX_SYNC1,
    UBX_SYNC2,
    UBX_ID,
    UBX_CLASS,
    UBX_LEN1,
    UBX_LEN2,
    UBX_PAYLOAD,
    UBX_CKA,
    UBX_CKB
} GB_UBX_STATE_T;

static void ubx_checksum(uint8_t *data, int len, uint8_t *cka, uint8_t *ckb)
{
    *cka = 0;
    *ckb = 0;
    for (int i = 0; i < len; i++) {
        *cka += data[i];
        *ckb += *cka;
    }
}

static void ubx_send_cfg_msg(uint8_t cls, uint8_t id, uint8_t rate)
{
    uint8_t payload[] = { cls, id, rate, 0, 0, 0 };

    uint8_t msg[6 + sizeof(payload)];
    msg[0] = 0x06;  // CFG
    msg[1] = 0x01;  // MSG
    msg[2] = sizeof(payload);
    msg[3] = 0x00;
    memcpy(&msg[4], payload, sizeof(payload));

    uint8_t ck_a, ck_b;
    ubx_checksum(msg, sizeof(msg), &ck_a, &ck_b);

    uint8_t pkt[sizeof(msg) + 4];
    pkt[0] = 0xB5;
    pkt[1] = 0x62;
    memcpy(&pkt[2], msg, sizeof(msg));
    pkt[sizeof(msg) + 2] = ck_a;
    pkt[sizeof(msg) + 3] = ck_b;

    GB_DUMPI(GPS_TAG, pkt, sizeof(pkt));

    uart1.writeBytes(&uart1, sizeof(pkt), pkt);
}


GB_RESULT GB_GPS_Init()
{
    GB_RESULT res = GB_OK;

    CHK_RES(uart1.begin(&uart1, 115200, GPS_TX_PIN, GPS_RX_PIN));
    GB_SleepMs(300);

    // TODO: Not effective
    /* diable NMEA */
    static const uint8_t nmea_ids[][2] = {
        {0xF0, 0x00}, // GGA
        {0xF0, 0x01}, // GLL
        {0xF0, 0x02}, // GSA
        {0xF0, 0x03}, // GSV
        {0xF0, 0x04}, // RMC
        {0xF0, 0x05}, // VTG
    };

    for (int i = 0; i < 6; i++) {
        ubx_send_cfg_msg(nmea_ids[i][0], nmea_ids[i][1], 0);
        GB_SleepMs(50);
    }

    /* enable UBX-NAV-PVT */
    ubx_send_cfg_msg(0x01, 0x07, 1);

error_exit:
    return res;
}

GB_RESULT GB_GPS_ReadUBX(GB_GPS_INFO_T *info)
{
    uint8_t ch;
    GB_UBX_STATE_T ubx_state = UBX_SYNC1;
    uint8_t ubx_cls, ubx_id;
    uint16_t ubx_len, ubx_cnt;
    uint8_t ubx_payload[128];
    uint8_t ubx_ck_a, ubx_ck_b;

    while (uart1.readBytes(&uart1, 1, &ch) == GB_OK) {

        switch (ubx_state) {

        case UBX_SYNC1:
            GB_DEBUGD(GPS_TAG, "UBX_SYNC1: %x", ch);
            if (ch == 0xB5) ubx_state = UBX_SYNC2;
            break;

        case UBX_SYNC2:
            GB_DEBUGD(GPS_TAG, "UBX_SYNC2: %x", ch);
            if (ch == 0x62) ubx_state = UBX_CLASS;
            else ubx_state = UBX_SYNC1;
            break;

        case UBX_CLASS:
            GB_DEBUGD(GPS_TAG, "UBX_CLASS: %x", ch);
            ubx_cls = ch;
            ubx_ck_a = ch;
            ubx_ck_b = ubx_ck_a;
            ubx_state = UBX_ID;
            break;

        case UBX_ID:
            GB_DEBUGD(GPS_TAG, "UBX_ID: %x", ch);
            ubx_id = ch;
            ubx_ck_a += ch;
            ubx_ck_b += ubx_ck_a;
            ubx_state = UBX_LEN1;
            break;

        case UBX_LEN1:
            GB_DEBUGD(GPS_TAG, "UBX_LEN1: %x", ch);
            ubx_len = ch;
            ubx_ck_a += ch;
            ubx_ck_b += ubx_ck_a;
            ubx_state = UBX_LEN2;
            break;

        case UBX_LEN2:
            GB_DEBUGD(GPS_TAG, "UBX_LEN2: %x", ch);
            ubx_len |= ch << 8;
            ubx_ck_a += ch;
            ubx_ck_b += ubx_ck_a;
            ubx_cnt = 0;
            ubx_state = UBX_PAYLOAD;
            break;

        case UBX_PAYLOAD:
            GB_DEBUGD(GPS_TAG, "UBX_PAYLOAD: %x", ch);
            ubx_payload[ubx_cnt++] = ch;
            ubx_ck_a += ch;
            ubx_ck_b += ubx_ck_a;
            if (ubx_cnt >= ubx_len) ubx_state = UBX_CKA;
            break;

        case UBX_CKA:
            GB_DEBUGD(GPS_TAG, "UBX_CKA: %x", ch);
            if (ch == ubx_ck_a) ubx_state = UBX_CKB;
            else ubx_state = UBX_SYNC1;
            break;

        case UBX_CKB:
            GB_DEBUGD(GPS_TAG, "UBX_CKB: %x", ch);
            ubx_state = UBX_SYNC1;
            if (ch == ubx_ck_b &&
                ubx_cls == 0x01 &&
                ubx_id == 0x07) {

                GB_UBX_T *pvt = (GB_UBX_T *)ubx_payload;

                GB_DUMPD(GPS_TAG, (uint8_t*)pvt, sizeof(GB_UBX_T));

                if (pvt->fixType < 3) {
                    return GB_FAIL;
                }

                info->latitude  = pvt->lat / 1e7;
                info->longitude = pvt->lon / 1e7;
                info->altitude  = pvt->hMSL / 1000.0;
                info->fix_type  = pvt->fixType;
                info->num_sv    = pvt->numSV;

                return GB_OK;
            }
            break;
        }
    }

    return GB_FAIL;
}

GB_RESULT GB_GPS_ReadNMEA(GB_GPS_INFO_T *gpsInfo)
{
    char line[GB_UART_RTX_BUF_SIZE];
    struct minmea_sentence_rmc rmc_frame;
    struct minmea_sentence_gga gga_frame;
    struct minmea_sentence_gst gst_frame;
    struct minmea_sentence_gsv gsv_frame;
    struct minmea_sentence_vtg vtg_frame;
    struct minmea_sentence_zda zda_frame;

    memset(&gga_frame, 0x00, sizeof(gga_frame));
    while (uart1.readLine(&uart1, GB_UART_RTX_BUF_SIZE, (uint8_t*)line, '$') == GB_OK) {
        GB_DEBUGI(GPS_TAG, "DEBUG: %s", line);

        switch (minmea_sentence_id(line, false)) {
            case MINMEA_SENTENCE_GGA: {
                if (minmea_parse_gga(&gga_frame, line)) {
                    GB_DEBUGI(GPS_TAG, "$xxGGA: fix quality: %d\n", gga_frame.fix_quality);
                    goto error_exit;
                }
                else {
                    GB_DEBUGI(GPS_TAG, "$xxGGA sentence is not parsed\n");
                }
            } break;

            case MINMEA_SENTENCE_RMC: {
                if (minmea_parse_rmc(&rmc_frame, line)) {
                    GB_DEBUGI(GPS_TAG, "$xxRMC: raw coordinates and speed: (%d/%d,%d/%d) %d/%d\n",
                            rmc_frame.latitude.value, rmc_frame.latitude.scale,
                            rmc_frame.longitude.value, rmc_frame.longitude.scale,
                            rmc_frame.speed.value, rmc_frame.speed.scale);
                    GB_DEBUGI(GPS_TAG, "$xxRMC fixed-point coordinates and speed scaled to three decimal places: (%d,%d) %d\n",
                            minmea_rescale(&rmc_frame.latitude, 1000),
                            minmea_rescale(&rmc_frame.longitude, 1000),
                            minmea_rescale(&rmc_frame.speed, 1000));
                    GB_DEBUGI(GPS_TAG, "$xxRMC floating point degree coordinates and speed: (%f,%f) %f\n",
                            minmea_tocoord(&rmc_frame.latitude),
                            minmea_tocoord(&rmc_frame.longitude),
                            minmea_tofloat(&rmc_frame.speed));
                }
                else {
                    GB_DEBUGI(GPS_TAG, "$xxRMC sentence is not parsed\n");
                }
            } break;

            case MINMEA_SENTENCE_GST: {
                if (minmea_parse_gst(&gst_frame, line)) {
                    GB_DEBUGI(GPS_TAG, "$xxGST: raw latitude,longitude and altitude error deviation: (%d/%d,%d/%d,%d/%d)\n",
                            gst_frame.latitude_error_deviation.value, gst_frame.latitude_error_deviation.scale,
                            gst_frame.longitude_error_deviation.value, gst_frame.longitude_error_deviation.scale,
                            gst_frame.altitude_error_deviation.value, gst_frame.altitude_error_deviation.scale);
                    GB_DEBUGI(GPS_TAG, "$xxGST fixed point latitude,longitude and altitude error deviation"
                           " scaled to one decimal place: (%d,%d,%d)\n",
                            minmea_rescale(&gst_frame.latitude_error_deviation, 10),
                            minmea_rescale(&gst_frame.longitude_error_deviation, 10),
                            minmea_rescale(&gst_frame.altitude_error_deviation, 10));
                    GB_DEBUGI(GPS_TAG, "$xxGST floating point degree latitude, longitude and altitude error deviation: (%f,%f,%f)",
                            minmea_tofloat(&gst_frame.latitude_error_deviation),
                            minmea_tofloat(&gst_frame.longitude_error_deviation),
                            minmea_tofloat(&gst_frame.altitude_error_deviation));
                }
                else {
                    GB_DEBUGI(GPS_TAG, "$xxGST sentence is not parsed\n");
                }
            } break;

            case MINMEA_SENTENCE_GSV: {
                if (minmea_parse_gsv(&gsv_frame, line)) {
                    GB_DEBUGI(GPS_TAG, "$xxGSV: message %d of %d\n", gsv_frame.msg_nr, gsv_frame.total_msgs);
                    GB_DEBUGI(GPS_TAG, "$xxGSV: satellites in view: %d\n", gsv_frame.total_sats);
                    for (int i = 0; i < 4; i++)
                        GB_DEBUGI(GPS_TAG, "$xxGSV: sat nr %d, elevation: %d, azimuth: %d, snr: %d dbm\n",
                            gsv_frame.sats[i].nr,
                            gsv_frame.sats[i].elevation,
                            gsv_frame.sats[i].azimuth,
                            gsv_frame.sats[i].snr);
                }
                else {
                    GB_DEBUGI(GPS_TAG, "$xxGSV sentence is not parsed\n");
                }
            } break;

            case MINMEA_SENTENCE_VTG: {
               if (minmea_parse_vtg(&vtg_frame, line)) {
                    GB_DEBUGI(GPS_TAG, "$xxVTG: true track degrees = %f\n",
                           minmea_tofloat(&vtg_frame.true_track_degrees));
                    GB_DEBUGI(GPS_TAG, "        magnetic track degrees = %f\n",
                           minmea_tofloat(&vtg_frame.magnetic_track_degrees));
                    GB_DEBUGI(GPS_TAG, "        speed knots = %f\n",
                            minmea_tofloat(&vtg_frame.speed_knots));
                    GB_DEBUGI(GPS_TAG, "        speed kph = %f\n",
                            minmea_tofloat(&vtg_frame.speed_kph));
               }
               else {
                    GB_DEBUGI(GPS_TAG, "$xxVTG sentence is not parsed\n");
               }
            } break;

            case MINMEA_SENTENCE_ZDA: {
                if (minmea_parse_zda(&zda_frame, line)) {
                    GB_DEBUGI(GPS_TAG, "$xxZDA: %d:%d:%d %02d.%02d.%d UTC%+03d:%02d\n",
                           zda_frame.time.hours,
                           zda_frame.time.minutes,
                           zda_frame.time.seconds,
                           zda_frame.date.day,
                           zda_frame.date.month,
                           zda_frame.date.year,
                           zda_frame.hour_offset,
                           zda_frame.minute_offset);
                }
                else {
                    GB_DEBUGI(GPS_TAG, "$xxZDA sentence is not parsed\n");
                }
            } break;

            case MINMEA_INVALID: {
                GB_DEBUGI(GPS_TAG, "$xxxxx sentence is not valid\n");
            } break;

            default: {
                GB_DEBUGI(GPS_TAG, "$xxxxx sentence is not parsed\n");
            } break;
        }
    }

error_exit:
    if (gga_frame.fix_quality > 0) {
        gpsInfo->latitude = minmea_tocoord(&gga_frame.latitude);
        gpsInfo->longitude = minmea_tocoord(&gga_frame.longitude);
    }

    if (gga_frame.altitude.scale) {
        gpsInfo->altitude = minmea_tofloat(&(gga_frame.altitude));
    }

    return GB_OK;
}
