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

#include <stdatomic.h>
#include <string.h>
#include "sdkconfig.h"
#include "gb_timer.h"
#include "task_manager.h"
#include "log_sys.h"
#include "tft_sprite.h"
#include "file_system.h"
#include "disp_driver.h"
#include "lvgl.h"
#include "lvgl_driver.h"
#include "controller.h"
#include "esp_timer.h"
#include "quad_3d.h"
#include "lora_state.h"
#include "disp_dsi.h"
#include "dashboard_widgets.h"
#include "gpio_setting.h"
#include "io_define.h"
#include "driver/gpio.h"

#define LV_TICK_PERIOD_MS 10

extern void welkin_widgets();
extern int battery_level;
extern volatile uint32_t g_fps_frame_count;
extern lv_timer_t *draw_task;
typedef void (*create_demo)(void);

SemaphoreHandle_t xGuiSemaphore;
atomic_uint_fast32_t lora_send_config = ATOMIC_VAR_INIT(LORA_SEND_NA);

static void lv_tick_task(void *arg)
{
    (void)arg;

    lv_tick_inc(LV_TICK_PERIOD_MS);
}

void gui_task(void *pvParameter)
{
    (void)pvParameter;
    create_demo demo;

    xGuiSemaphore = xSemaphoreCreateMutex();
    if (pvParameter == NULL)
        demo = welkin_widgets;
    else
        demo = pvParameter;

    lv_init();

#if (CONFIG_DISPLAY_ORIENTATION_PORTRAIT)
    lv_display_t *disp = lv_display_create(LV_VER_RES_MAX, LV_HOR_RES_MAX);
    assert(disp != NULL);
    GB_DEBUGI(DISP_TAG, "lv_display_create hor_res: %d, ver_res: %d (rotated by ppa)", LV_VER_RES_MAX, LV_HOR_RES_MAX);
#else
    lv_display_t *disp = lv_display_create(LV_HOR_RES_MAX, LV_VER_RES_MAX);
    assert(disp != NULL);
    GB_DEBUGI(DISP_TAG, "lv_display_create hor_res: %d, ver_res: %d", LV_HOR_RES_MAX, LV_VER_RES_MAX);
#endif

    lv_display_set_flush_cb(disp, lvgl_driver_flush);

#if defined CONFIG_TFT_DISPLAY_PROTOCOL_DSI
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB888);
#endif

    // MALLOC_CAP_DMA for SPI
    uint32_t buffer_size = DISP_BUF_SIZE * sizeof(lv_color_t);
    lv_color_t *buf1 = heap_caps_aligned_alloc(64, buffer_size, MALLOC_CAP_SPIRAM);
    assert(buf1 != NULL);
    lv_color_t *buf2 = heap_caps_aligned_alloc(64, buffer_size, MALLOC_CAP_SPIRAM);
    assert(buf2 != NULL);

    lv_display_set_buffers(disp, buf1, buf2, buffer_size, LV_DISPLAY_RENDER_MODE_PARTIAL);

#if (CONFIG_DISPLAY_ORIENTATION_PORTRAIT)
    disp_dsi_ppa_create(buffer_size);
#endif

    /* Register an input device when enabled on the menuconfig */
#if CONFIG_LV_TOUCH_CONTROLLER != TOUCH_CONTROLLER_NONE
    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, touch_driver_read);
#endif

    /* Create and start a periodic timer interrupt to call lv_tick_inc */
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &lv_tick_task,
        .name = "periodic_gui"};
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));

    /* Create the demo application */
    demo();

    while (1)
    {
        /* Delay 1 tick (assumes FreeRTOS tick is 10ms */
        vTaskDelay(pdMS_TO_TICKS(10));

        /* Try to take the semaphore, call lvgl related function on success */
        if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY))
        {
            lv_timer_handler();
            g_fps_frame_count++;
            xSemaphoreGive(xGuiSemaphore);
        }
    }

    /* A task should NEVER return */
    free(buf1);
    free(buf2);
    vTaskDelete(NULL);
}

void controller_task(void *pvParameter)
{
    GB_CONTROL_EVENT event;
    GB_TOGGLE_SWITCH_STATE prev_toggle_state = {0};
    GB_TOGGLE_SWITCH_STATE current_toggle_state = {0};
    //uint16_t throttle_adc, pitch_adc, roll_adc, yaw_adc;

    adc_wrapper_init();
    controller_input_init(); // Initialize unified button and toggle switch system
    QueueHandle_t event_queue = controller_get_event_queue();

    // Read initial toggle switch state
    controller_get_toggle_state(&current_toggle_state);
    prev_toggle_state = current_toggle_state;

    GB_DEBUGI(GB_INFO, "Controller task started - unified event-driven mode, sleep 5s");

    while (1)
    {
        //adc_read_by_item(ADC_THROTTLE, &throttle_adc, true);
        //adc_read_by_item(ADC_PITCH, &pitch_adc, true);
        //adc_read_by_item(ADC_ROLL, &roll_adc, true);
        //adc_read_by_item(ADC_YAW, &yaw_adc, true);

        // GB_DEBUGI(GB_INFO, "T: %d, P: %d, R: %d, Y: %d", throttle_adc, pitch_adc, roll_adc, yaw_adc);

        // Block waiting for control events (button or toggle switch)
        // Use 5 second timeout - task will sleep if no events
        if (xQueueReceive(event_queue, &event, pdMS_TO_TICKS(5000)) == pdTRUE)
        {
            if (event.type == GB_EVENT_BUTTON)
            {
                // Button pressed
                GB_DEBUGI(GB_INFO, "Button event: %d", event.data.button_id);
                gb_remote_single_control(event.data.button_id);

            }
            else if (event.type == GB_EVENT_TOGGLE_SWITCH)
            {
                // Toggle switch changed
                GB_SleepMs(30);

                // Read all toggle switch states using gpio_setting wrapper
                uint32_t level;
                GB_TOGGLE_SWITCH_STATE new_state;

                GB_GPIO_Get(TOGGLE_SW_1_GPIO, &level);
                new_state.sw1_state = !level; // Active low

                GB_GPIO_Get(TOGGLE_SW_2_GPIO, &level);
                new_state.sw2_state = !level;

                GB_GPIO_Get(TOGGLE_SW_3_GPIO, &level);
                new_state.sw3_state = !level;

                GB_GPIO_Get(TOGGLE_SW_4_GPIO, &level);
                new_state.sw4_state = !level;

                current_toggle_state = new_state;
                controller_set_toggle_state(&new_state);
            }
        }

        // Check if state actually changed (debounce)
        if (current_toggle_state.sw1_state != prev_toggle_state.sw1_state ||
            current_toggle_state.sw2_state != prev_toggle_state.sw2_state ||
            current_toggle_state.sw3_state != prev_toggle_state.sw3_state ||
            current_toggle_state.sw4_state != prev_toggle_state.sw4_state)
        {
            GB_DEBUGI(GB_INFO, "Toggle switch changed: SW1=%d, SW2=%d, SW3=%d, SW4=%d",
                current_toggle_state.sw1_state, current_toggle_state.sw2_state,
                current_toggle_state.sw3_state, current_toggle_state.sw4_state);

            // Handle SW1: Fly mode
            if (current_toggle_state.sw1_state != prev_toggle_state.sw1_state)
            {
                if (current_toggle_state.sw1_state)
                {
                    GB_DEBUGI(GB_INFO, "SW1 ON: Fly Mode");
                    atomic_store(&lora_send_config, LORA_SEND_CONTROL_COMMAND);
                }
                else
                {
                    GB_DEBUGI(GB_INFO, "SW1 OFF: Exit Fly Mode");
                    atomic_store(&lora_send_config, LORA_SEND_NA);
                }
            }

            // Handle SW3: Pull PID table from master (rising edge)
            if (current_toggle_state.sw3_state && !prev_toggle_state.sw3_state)
            {
                if (!current_toggle_state.sw1_state)
                {
                    GB_DEBUGI(GB_INFO, "SW3 triggered: Requesting PID table from master");
                    atomic_store(&lora_send_config, LORA_SEND_PID_GET_INFO);
                }
                else
                {
                    GB_DEBUGI(GB_INFO, "SW3 ignored: Cannot request PID during fly mode");
                }
            }

            // cancel PID pull request
            if (!current_toggle_state.sw3_state && prev_toggle_state.sw3_state)
            {
                GB_SEND_CONFIG current = (GB_SEND_CONFIG)atomic_load(&lora_send_config);
                if (current == LORA_SEND_PID_GET_INFO)
                {
                    GB_DEBUGI(GB_INFO, "SW3 OFF: Cancelling PID request");
                    atomic_store(&lora_send_config, LORA_GET_MOTION_STATE);
                }
            }

            // Handle SW4: Push PID table to master (rising edge)
            if (current_toggle_state.sw4_state && !prev_toggle_state.sw4_state)
            {
                if (!current_toggle_state.sw1_state)
                {
                    GB_DEBUGI(GB_INFO, "SW4 triggered: Sending PID table to master");
                    atomic_store(&lora_send_config, LORA_SEND_PID_SET_INFO);
                }
                else
                {
                    GB_DEBUGI(GB_INFO, "SW3 ignored: Cannot push PID during fly mode");
                }
            }

            if (!current_toggle_state.sw4_state && prev_toggle_state.sw4_state)
            {
                GB_SEND_CONFIG current = (GB_SEND_CONFIG)atomic_load(&lora_send_config);
                if (current == LORA_SEND_PID_SET_INFO)
                {
                    GB_DEBUGI(GB_INFO, "SW4 OFF: Cancelling PID push");
                    atomic_store(&lora_send_config, LORA_GET_MOTION_STATE);
                }
            }

            prev_toggle_state = current_toggle_state;

            // Send notification to GUI task that toggle state changed
            gb_remote_single_control(TOGGLE_SWITHCH_CHANGED);
        }
    }
}

typedef struct {
    GB_PID_TABLE_T pid_table;
    GB_REASSEMBLY_CTX_T reassembly_ctx;
    uint8_t msg_id_counter;
    uint64_t receive_waittime;
    uint32_t delay_time;
    uint8_t send_retry;
    GB_LORA_STATE lora_state;
    GB_LORA_PACKAGE_T send_package;
    union {
        GB_LORA_PACKAGE_T pkg;
        GB_LORA_FRAGMENT_T frag;
    } receive_buf;
} rf_context_t;

static void rf_prepare_package(rf_context_t *ctx, GB_SEND_CONFIG config)
{
    GB_LORA_PACKAGE_T *pkg = &ctx->send_package;

    switch (config) {
    case LORA_SEND_NA:
        pkg->type = GB_INIT_DATA;
        pkg->sync = 0xaa;
        GB_DEBUGI(RF24_TAG, "Sending init data, %d", pkg->sync);
        break;
    case LORA_SEND_SKY_WAL_CONFIG:
        pkg->type = GB_SET_CONFIG;
        pkg->sync = 0xab;
        pkg->config.set_type = GB_SET_THROTTLE;
        adc_read_by_item(ADC_THROTTLE, &pkg->config.throttle, true);
        GB_DEBUGI(RF24_TAG, "Setting throttle, %d", pkg->config.throttle);
        break;
    case LORA_SEND_CONTROL_COMMAND:
        pkg->type = GB_SET_CONFIG;
        pkg->sync = 0xac;
        pkg->config.set_type = GB_SET_CONTROL_ARG;
        adc_read_by_item(ADC_THROTTLE, &pkg->config.control_arg.throttle, true);
        adc_read_by_item(ADC_YAW, &pkg->config.control_arg.yaw, true);
        adc_read_by_item(ADC_PITCH, &pkg->config.control_arg.pitch, true);
        adc_read_by_item(ADC_ROLL, &pkg->config.control_arg.roll, true);
        GB_DEBUGD(RF24_TAG, "Commander throttle, %d, yaw: %d, pitch: %d, roll: %d",
                  pkg->config.control_arg.throttle, pkg->config.control_arg.yaw,
                  pkg->config.control_arg.pitch, pkg->config.control_arg.roll);
        break;
    case LORA_SEND_PID_GET_INFO:
        pkg->type = GB_GET_REQUEST;
        pkg->sync = 0xae;
        pkg->request.get_type = GB_GET_PID_TABLE;
        GB_DEBUGI(RF24_TAG, "Requesting PID table from master");
        break;
    case LORA_GET_MOTION_STATE:
        pkg->type = GB_GET_REQUEST;
        pkg->sync = 0xad;
        pkg->request.get_type = GB_GET_MOTION_STATE;
        GB_DEBUGD(RF24_TAG, "GB_GET_MOTION_STATE");
        break;
    default:
        GB_DEBUGE(ERROR_TAG, "UNKNOWN LORA SEND CONFIG!");
        break;
    }
}

static bool rf_send_pid_table(rf_context_t *ctx)
{
    GB_DEBUGI(RF24_TAG, "Sending PID table to master via fragments...");

    gui_pid_table_get(&ctx->pid_table);
    ctx->pid_table.crc16 = GB_PidTableCalculateCRC(&ctx->pid_table);
    ctx->msg_id_counter++;

    if (GB_LoraFragmentSend(
            (const uint8_t *)&ctx->pid_table,
            sizeof(ctx->pid_table),
            ctx->msg_id_counter,
            &ctx->lora_state) == GB_OK)
    {
        GB_DEBUGI(RF24_TAG, "PID table sent successfully!");
    }
    else
    {
        GB_DEBUGI(RF24_TAG, "Failed to send PID table");
    }

    ctx->lora_state = LORA_SEND;
    atomic_store(&lora_send_config, LORA_SEND_NA);
    return true; // signal caller to continue
}

static bool rf_handle_send(rf_context_t *ctx)
{
    GB_SEND_CONFIG current_config = (GB_SEND_CONFIG)atomic_load(&lora_send_config);

    if (current_config == LORA_SEND_PID_SET_INFO)
        return rf_send_pid_table(ctx);

    rf_prepare_package(ctx, current_config);

    // Transmit
    GB_RESULT report;
    if (current_config == LORA_SEND_CONTROL_COMMAND)
        report = radio.write_data(&radio, &ctx->send_package, sizeof(GB_LORA_PACKAGE_T), true);
    else
        report = radio.write(&radio, &ctx->send_package, sizeof(GB_LORA_PACKAGE_T));

    bool is_fire_and_forget = (current_config == LORA_SEND_SKY_WAL_CONFIG ||
                               current_config == LORA_SEND_CONTROL_COMMAND);

    if (report == GB_OK) {
        if (is_fire_and_forget) {
            ctx->send_retry = 0;
            GB_SleepMs(20);
            return true;
        }
        ctx->lora_state = LORA_RECEIVE;
        radio.startListening(&radio);
        GB_GetTimerUs(&ctx->receive_waittime);
    } else {
        GB_DEBUGI(RF24_TAG, "Transmission failed or timed out, %08x, retry: %d", report, ctx->send_retry);
        if (is_fire_and_forget) {
            GB_SleepMs(20);
            return true;
        }
        ctx->send_retry++;
        if (ctx->send_retry >= 5) {
            ctx->send_retry = 0;
            atomic_store(&lora_send_config, LORA_SEND_NA);
        }
    }
    return false;
}

static bool rf_handle_pid_fragment(rf_context_t *ctx)
{
    GB_LORA_FRAGMENT_T *frag = (GB_LORA_FRAGMENT_T *)&ctx->receive_buf.frag;
    uint8_t complete_msg[GB_MAX_FRAGMENTS * GB_FRAGMENT_PAYLOAD_SIZE];
    size_t msg_len = 0;

    GB_DEBUGI(RF24_TAG, "Received PID fragment: msg_id=%d, frag=%d/%d",
             frag->msg_id, frag->frag_index + 1, frag->frag_total);

    GB_FRAG_RESULT frag_res = GB_LoraFragmentReceive(frag, &ctx->reassembly_ctx,
                                                     complete_msg, &msg_len);

    // Send ACK for this fragment
    GB_LORA_FRAGMENT_T ack_frag = {0};
    ack_frag.type = GB_PID_FRAGMENT;
    ack_frag.sync = frag->sync + 1;
    ack_frag.msg_id = frag->msg_id;
    ack_frag.frag_index = frag->frag_index;
    ack_frag.frag_total = frag->frag_total;

    radio.stopListening(&radio);
    radio.write(&radio, &ack_frag, sizeof(ack_frag));
    radio.startListening(&radio);

    if (frag_res == GB_FRAG_COMPLETE) {
        GB_DEBUGI(RF24_TAG, "PID table reassembly complete!");

        if (msg_len >= sizeof(GB_PID_TABLE_T)) {
            memcpy(&ctx->pid_table, complete_msg, sizeof(GB_PID_TABLE_T));

            if (GB_PidTableValidate(&ctx->pid_table) == GB_OK) {
                GB_DEBUGI(RF24_TAG, "PID table received and validated!");
                for (int i = 0; i < PID_MAX; i++) {
                    GB_DEBUGI(RF24_TAG, "PID[%d]: Kp=%d, Ki=%d, Kd=%d",
                              i, ctx->pid_table.params[i].kp,
                              ctx->pid_table.params[i].ki,
                              ctx->pid_table.params[i].kd);
                }
                gui_pid_table_set(&ctx->pid_table);
                ctx->send_retry = 0;
                atomic_store(&lora_send_config, LORA_SEND_NA);
            } else {
                GB_DEBUGE(RF24_TAG, "PID table CRC validation failed!");
                ctx->send_retry++;
                if (ctx->send_retry >= 3)
                {
                    GB_DEBUGE(RF24_TAG, "PID table CRC failed %d times, aborting", ctx->send_retry);
                    ctx->send_retry = 0;
                    atomic_store(&lora_send_config, LORA_GET_MOTION_STATE);
                }
            }
        }

        ctx->lora_state = LORA_SEND;
        radio.stopListening(&radio);
    }

    return true; // signal caller to continue
}

static void rf_process_response(rf_context_t *ctx)
{
    if (ctx->receive_buf.pkg.sync != ctx->send_package.sync + 1)
        GB_DEBUGE(ERROR_TAG, "Receive error!");

    GB_DEBUGD(RF24_TAG, "type = %d, sync = %d", ctx->receive_buf.pkg.type, ctx->receive_buf.pkg.sync);

    GB_SEND_CONFIG current_config = (GB_SEND_CONFIG)atomic_load(&lora_send_config);

    if (LORA_SEND_NA == current_config) {
        battery_level = ctx->receive_buf.pkg.init.battery_capacity;

        // Update dashboard with init data
        flight_data_t *fdata = dashboard_get_flight_data();
        fdata->connected = (ctx->receive_buf.pkg.init.system_state == GB_SYSTEM_INITIALIZE_PASS ||
                            ctx->receive_buf.pkg.init.system_state == GB_SYSTEM_UNLOCK);
        fdata->armed = (ctx->receive_buf.pkg.init.system_state == GB_SYSTEM_UNLOCK);

        // Parse sensor_state bits: MPU | COMPASS | BMP280 | CAMERA
        uint8_t sensors = ctx->receive_buf.pkg.init.sensor_state;
        fdata->imu_status = (sensors & 0x08) ? SENSOR_STATUS_GOOD : SENSOR_STATUS_ERROR;
        fdata->mag_status = (sensors & 0x04) ? SENSOR_STATUS_GOOD : SENSOR_STATUS_ERROR;
        fdata->baro_status = (sensors & 0x02) ? SENSOR_STATUS_GOOD : SENSOR_STATUS_ERROR;
        fdata->radio_status = SENSOR_STATUS_GOOD;

        if (fdata->connected)
            strcpy(fdata->flight_mode, fdata->armed ? "UNLOCK" : "READY");
        else
            strcpy(fdata->flight_mode, "INIT");

        if (pdTRUE == xSemaphoreTake(xGuiSemaphore, pdMS_TO_TICKS(50)))
        {
            dashboard_update(fdata);
            xSemaphoreGive(xGuiSemaphore);
        }

        // Auto-enter motion state polling after successful init
        uint_fast32_t expected_state = LORA_SEND_NA;
        if (atomic_compare_exchange_strong(&lora_send_config, &expected_state, LORA_GET_MOTION_STATE))
            ctx->delay_time = 10;
    } else if (LORA_GET_MOTION_STATE == current_config) {
        float roll = (float)ctx->receive_buf.pkg.request.motion_status.roll / GB_ERLER_SCALE_RATE;
        float pitch = (float)ctx->receive_buf.pkg.request.motion_status.pitch / GB_ERLER_SCALE_RATE;
        float yaw = (float)ctx->receive_buf.pkg.request.motion_status.yaw / GB_ERLER_SCALE_RATE;
        int16_t altitude = ctx->receive_buf.pkg.request.motion_status.altitude;
        battery_level = ctx->receive_buf.pkg.request.motion_status.battery;
        GB_SYSTEM_STATE sys_state = ctx->receive_buf.pkg.request.motion_status.system_state;

        GB_DEBUGD(RF24_TAG, "Received motion_status roll: %d, pitch: %d, yaw: %d, alt: %d, battery: %d",
                 ctx->receive_buf.pkg.request.motion_status.roll,
                 ctx->receive_buf.pkg.request.motion_status.pitch,
                 ctx->receive_buf.pkg.request.motion_status.yaw,
                 altitude,
                 ctx->receive_buf.pkg.request.motion_status.battery);

        // Update dashboard with motion data
        flight_data_t *fdata = dashboard_get_flight_data();
        fdata->roll = roll;
        fdata->pitch = pitch;
        fdata->yaw = yaw;
        fdata->heading = (int)yaw;
        if (fdata->heading < 0) fdata->heading += 360;
        fdata->altitude = (int)altitude;
        fdata->connected = (sys_state == GB_SYSTEM_INITIALIZE_PASS || sys_state == GB_SYSTEM_UNLOCK);
        fdata->armed = (sys_state == GB_SYSTEM_UNLOCK);
        if (fdata->connected)
            strcpy(fdata->flight_mode, fdata->armed ? "UNLOCK" : "READY");

        if (pdTRUE == xSemaphoreTake(xGuiSemaphore, pdMS_TO_TICKS(50)))
        {
            dashboard_update(fdata);
            xSemaphoreGive(xGuiSemaphore);
        }

        // Only update 3D model when it is enabled
        if (draw_task != NULL)
            quad3d_set_angle(roll, pitch, yaw);

        uint_fast32_t expected_state = LORA_GET_MOTION_STATE;
        if (atomic_compare_exchange_strong(&lora_send_config, &expected_state, LORA_GET_MOTION_STATE))
            ctx->delay_time = 10;
    } else {
        GB_DEBUGI(RF24_TAG, "Config changed externally to %d, skipping response", current_config);
    }

    ctx->lora_state = LORA_SEND;
    radio.stopListening(&radio);
}

static bool rf_handle_receive(rf_context_t *ctx)
{
    uint8_t rf_status = radio.get_status(&radio);
    uint8_t pipe;
    bool has_data = (rf_status & _BV(RX_DR)) || radio.available(&radio, &pipe);

    if (!has_data)
    {
        GB_SEND_CONFIG current_config = (GB_SEND_CONFIG)atomic_load(&lora_send_config);
        bool config_changed = false;
        if (current_config == LORA_SEND_CONTROL_COMMAND ||
            current_config == LORA_SEND_SKY_WAL_CONFIG) {
            config_changed = true;
        }

        uint64_t time_now;
        GB_GetTimerUs(&time_now);
        if (!config_changed && time_now - ctx->receive_waittime < NRF24_RECEIVE_WAITTING_TIME)
            return true; // continue waiting

        ctx->send_retry ++;
        if (config_changed)
        {
            GB_DEBUGI(RF24_TAG, "Config changed during receiving, switching to send");
            ctx->send_retry = 0;
        }
        else
        {
            GB_DEBUGI(RF24_TAG, "Waiting to receive package, retry: %d", ctx->send_retry);
        }

        ctx->lora_state = LORA_SEND;
        radio.stopListening(&radio);
        radio.write_register(&radio, NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT), false);

        if (ctx->send_retry >= 5)
        {
            ctx->send_retry = 0;
            GB_DEBUGI(RF24_TAG, "Max receive retries, resetting to init data");
            radio.flush_rx(&radio);
            atomic_store(&lora_send_config, LORA_SEND_NA);
            ctx->delay_time = 1000;
        }
    }
    else
    {
        radio.write_register(&radio, NRF_STATUS, _BV(RX_DR), false);

        if (!radio.available(&radio, &pipe))
        {
            GB_DEBUGI(RF24_TAG, "Received nothing..., try to send again");
            ctx->lora_state = LORA_SEND;
            radio.stopListening(&radio);
            radio.write_register(&radio, NRF_STATUS, _BV(TX_DS) | _BV(MAX_RT), false);
            radio.flush_rx(&radio);
        }
        else
        {
            uint8_t bytes = radio.getPayloadSize(&radio);
            radio.read(&radio, &ctx->receive_buf, bytes);
            ctx->send_retry = 0;

            if (ctx->receive_buf.pkg.type == GB_PID_FRAGMENT)
                return rf_handle_pid_fragment(ctx);

            rf_process_response(ctx);
        }
    }

    return false;
}

static volatile TaskHandle_t nrf24_rf_task_handle = NULL;

static void IRAM_ATTR nrf24_rx_isr_handle(void *arg)
{
    if (nrf24_rf_task_handle) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(nrf24_rf_task_handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void rf_loop(void *arg)
{
    static rf_context_t ctx = {0};

    nrf24_rf_task_handle = xTaskGetCurrentTaskHandle();

    GB_LoraSystemInit(LORA_SEND, 1, &ctx.lora_state);
    radio.enableDynamicAck(&radio);
    GB_ReassemblyInit(&ctx.reassembly_ctx);

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .pin_bit_mask = (1ULL << NRF24_INT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 0,
        .pull_down_en = 1,
    };
    gpio_config(&io_conf);
    gpio_isr_handler_add(NRF24_INT, nrf24_rx_isr_handle, NULL);

    ctx.delay_time = 1000;

    while (true)
    {
        if (LORA_SEND == ctx.lora_state)
        {
            if (rf_handle_send(&ctx))
                continue;
        }
        else if (LORA_RECEIVE == ctx.lora_state)
        {
            if (rf_handle_receive(&ctx))
                continue;
        }

        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(ctx.delay_time));
    }
}
