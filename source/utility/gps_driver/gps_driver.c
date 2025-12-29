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

#include "log_sys.h"
#include "uart_bus.h"
#include "gps_driver.h"
#include "io_define.h"
#include "minmea.h"

GB_RESULT GB_GPS_Init()
{
    return uart1.begin(&uart1, 115200, GPS_TX_PIN, GPS_RX_PIN);
}

GB_RESULT GB_GPS_ReadData(GB_GPS_INFO_T *gpsInfo)
{
    char line[GB_UART_RTX_BUF_SIZE];
    struct minmea_sentence_rmc rmc_frame;
    struct minmea_sentence_gga gga_frame;
    struct minmea_sentence_gst gst_frame;
    struct minmea_sentence_gsv gsv_frame;
    struct minmea_sentence_vtg vtg_frame;
    struct minmea_sentence_zda zda_frame;

    while (uart1.readLine(&uart1, GB_UART_RTX_BUF_SIZE, (uint8_t*)line) == GB_OK) {
        GB_DEBUGI(GPS_TAG, "%s", line);
        switch (minmea_sentence_id(line, false)) {
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

            case MINMEA_SENTENCE_GGA: {
                if (minmea_parse_gga(&gga_frame, line)) {
                    GB_DEBUGI(GPS_TAG, "$xxGGA: fix quality: %d\n", gga_frame.fix_quality);
                }
                else {
                    GB_DEBUGI(GPS_TAG, "$xxGGA sentence is not parsed\n");
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

    gpsInfo->latitude = minmea_tocoord(&gga_frame.latitude);
    gpsInfo->longitude = minmea_tocoord(&gga_frame.latitude);
    gpsInfo->altitude = minmea_tofloat(&(gga_frame.altitude));

    return GB_OK;
}
