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

#include "esp_log.h"
#include "tusb_cdc_acm.h"
#include "tusb_console.h"
#include "class/cdc/cdc_device.h"
#include "log_sys.h"

const char* SENSOR_TAG = "[SENSOR_CHECK]";
const char* ERROR_TAG = "[ERROR]";
const char* ST_TAG = "[SELF_TEST]";
const char* CHK_TAG = "[CHK]";
const char* BMP_TAG = "[BMP]";
const char* RF24_TAG = "[RF24]";
const char* GB_INFO = "[GB_COMMON]";
const char* LORA_TAG = "[LORA]";
const char* FS_TAG = "[FS]";

void gb_log_system_init()
{
    esp_log_level_set(SENSOR_TAG, ESP_LOG_DEBUG);
    esp_log_level_set(ERROR_TAG, ESP_LOG_ERROR);
    esp_log_level_set(ST_TAG, ESP_LOG_ERROR);
    esp_log_level_set(CHK_TAG, ESP_LOG_ERROR);
    esp_log_level_set(RF24_TAG, ESP_LOG_DEBUG);
    esp_log_level_set(GB_INFO, ESP_LOG_INFO);
    esp_log_level_set(LORA_TAG, ESP_LOG_INFO);
    esp_log_level_set(FS_TAG, ESP_LOG_INFO);

#ifdef CONFIG_UART_LOG_ENABLE
    uart_config_t uart_config = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 2*1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
#endif

#ifdef CONFIG_USB_ENABLED
    ESP_LOGI(GB_INFO, "USB device initialization");
    tinyusb_config_t tusb_cfg = {}; // the configuration using default values
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_config_cdcacm_t amc_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 64,
        .callback_rx = NULL,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL
    };
    // SET wanted char => ' '
    tud_cdc_n_set_wanted_char(amc_cfg.cdc_port, ' ');
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&amc_cfg));
    esp_tusb_init_console(TINYUSB_CDC_ACM_0); // log to usb
#endif
}

void gb_print_log(GB_LOG_LEVEL level, const char *tag, const char* format, ...)
{
#if 0
    switch (level)
    {
        case GB_LOG_LEVEL_ERROR:
            ESP_LOGE(tag, format, ##__VA_ARGS__);
            break;
        case GB_LOG_LEVEL_WARNING:
            ESP_LOGW(tag, format, ##__VA_ARGS__);
            break;
        case GB_LOG_LEVEL_INFO:
            ESP_LOGI(tag, format, ##__VA_ARGS__);
            break;
        case GB_LOG_LEVEL_DEBUG:
            ESP_LOGD(tag, format, ##__VA_ARGS__);
            break;
        case GB_LOG_LEVEL_VERBOSE:
            ESP_LOGV(tag, format, ##__VA_ARGS__);
            break;
        default:
            ESP_LOGE(tag, format, ##__VA_ARGS__);
    }
#endif
}

GB_RESULT gb_usb_read_bytes(int cdc_intf, uint8_t *buf, size_t *rx_size)
{
    GB_RESULT res = GB_OK;
    /* read */
    esp_err_t ret = tinyusb_cdcacm_read(cdc_intf, buf, CONFIG_TINYUSB_CDC_RX_BUFSIZE, rx_size);
    if (ret == ESP_OK) {
        *rx_size -= 1;        // remove usb last byte '0x0a' = \n
        buf[*rx_size] = '\0';
    } else {
        res = GB_LOG_USB_READ_FAIL;
    }
    return res;
}

GB_RESULT gb_usb_write_bytes(int cdc_intf, const uint8_t *buf, size_t tx_size)
{
    GB_RESULT res = GB_OK;
    /* write */
    esp_err_t ret = tinyusb_cdcacm_write_queue(cdc_intf, buf, tx_size);
    if (ret != ESP_OK) {
        res = GB_LOG_USB_READ_FAIL;
    }
    tinyusb_cdcacm_write_flush(cdc_intf, 10);
    return res;
}

