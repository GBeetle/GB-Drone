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
#include "tinyusb.h"
#include "tinyusb_net.h"
#include "class/cdc/cdc_device.h"
#include "log_sys.h"

#define GB_LOG_BUFFER_SIZE 1024

const char* SENSOR_TAG = "[SENSOR_CHECK]";
const char* ERROR_TAG = "[ERROR]";
const char* ST_TAG = "[SELF_TEST]";
const char* CHK_TAG = "[CHK]";
const char* BMP_TAG = "[BMP]";
const char* RF24_TAG = "[RF24]";
const char* GB_INFO = "[GB_COMMON]";
const char* LORA_TAG = "[LORA]";
const char* FS_TAG = "[FS]";

void GB_LogSystemInit()
{
    esp_log_level_set(SENSOR_TAG, ESP_LOG_INFO);
    esp_log_level_set(ERROR_TAG, ESP_LOG_ERROR);
    esp_log_level_set(ST_TAG, ESP_LOG_DEBUG);
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

#ifdef CONFIG_USB_LOG_ENABLE
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

void GB_PrintLog(GB_LOG_LEVEL level, const char *tag, const char* format, ...)
{
    if (level > (GB_LOG_LEVEL)esp_log_level_get(tag))
        return;

    char log_buffer[GB_LOG_BUFFER_SIZE] = {0};
    va_list list;

    va_start(list, format);
    vsnprintf(log_buffer, sizeof(log_buffer), format, list);
    va_end(list);

    switch (level)
    {
        case GB_LOG_LEVEL_ERROR:
            ESP_LOGE(tag, "%s", log_buffer);
            break;
        case GB_LOG_LEVEL_WARNING:
            ESP_LOGW(tag, "%s", log_buffer);
            break;
        case GB_LOG_LEVEL_INFO:
            ESP_LOGI(tag, "%s", log_buffer);
            break;
        case GB_LOG_LEVEL_DEBUG:
            ESP_LOGD(tag, "%s", log_buffer);
            break;
        case GB_LOG_LEVEL_VERBOSE:
            ESP_LOGV(tag, "%s", log_buffer);
            break;
        default:
            ESP_LOGE(tag, "%s", log_buffer);
    }
}

void GB_DumpLog(GB_LOG_LEVEL level, const char *tag, const uint8_t *data, uint32_t size)
{
    switch (level)
    {
        case GB_LOG_LEVEL_ERROR:
            ESP_LOG_BUFFER_HEXDUMP(tag, data, size, ESP_LOG_ERROR);
            break;
        case GB_LOG_LEVEL_WARNING:
            ESP_LOG_BUFFER_HEXDUMP(tag, data, size, ESP_LOG_WARN);
            break;
        case GB_LOG_LEVEL_INFO:
            ESP_LOG_BUFFER_HEXDUMP(tag, data, size, ESP_LOG_INFO);
            break;
        case GB_LOG_LEVEL_DEBUG:
            ESP_LOG_BUFFER_HEXDUMP(tag, data, size, ESP_LOG_DEBUG);
            break;
        case GB_LOG_LEVEL_VERBOSE:
            ESP_LOG_BUFFER_HEXDUMP(tag, data, size, ESP_LOG_VERBOSE);
            break;
        default:
            ESP_LOG_BUFFER_HEXDUMP(tag, data, size, ESP_LOG_ERROR);
    }
}

GB_RESULT GB_ReadBytes(uint8_t *buf, size_t *rx_size)
{
    GB_RESULT res = GB_OK;

#ifdef CONFIG_UART_LOG_ENABLE
    rxBytes = uart_read_bytes(UART_NUM_0, buf, rx_size, 10 / portTICK_RATE_MS);
#elif CONFIG_USB_LOG_ENABLE
    esp_err_t ret = tinyusb_cdcacm_read(TINYUSB_CDC_ACM_0, buf, CONFIG_TINYUSB_CDC_RX_BUFSIZE, rx_size);
    if (ret == ESP_OK) {
        *rx_size -= 1;        // remove usb last byte '0x0a' = \n
        buf[*rx_size] = '\0';
    } else {
        res = GB_LOG_USB_READ_FAIL;
    }
#endif
    return res;
}

GB_RESULT GB_WriteBytes(const uint8_t *buf, size_t tx_size)
{
    GB_RESULT res = GB_OK;

#ifdef CONFIG_UART_LOG_ENABLE
    uart_write_bytes(UART_NUM_0, (const uint8_t *)buf, tx_size);
#elif CONFIG_USB_LOG_ENABLE
    esp_err_t ret = tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, buf, tx_size);
    if (ret != ESP_OK) {
        res = GB_LOG_USB_READ_FAIL;
    }
    tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 10);
#endif
    return res;
}

