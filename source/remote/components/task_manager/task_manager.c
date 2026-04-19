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
#include "sdkconfig.h"
#include "disp_dsi.h"

#define LV_TICK_PERIOD_MS 10

extern void welkin_widgets();
extern int battery_level;
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
                current_toggle_state = event.data.switch_state;
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

            prev_toggle_state = current_toggle_state;

            // Add small delay for hardware debouncing
            vTaskDelay(pdMS_TO_TICKS(50));

            // Send notification to GUI task that toggle state changed
            gb_remote_single_control(TOGGLE_SWITHCH_CHANGED);
        }
    }
}

void rf_loop(void *arg)
{
    uint8_t send_retry = 0;
    uint64_t receive_waittime = 0;
    uint64_t time_now = 0;
    uint32_t delay_time = 5000;
    GB_LORA_STATE lora_state;
    GB_LORA_PACKAGE_T send_package;
    GB_LORA_PACKAGE_T receive_package;

    GB_LoraSystemInit(LORA_SEND, 1, &lora_state);
    while (true)
    {
        if (LORA_SEND == lora_state)
        {
            GB_SEND_CONFIG current_config = (GB_SEND_CONFIG)atomic_load(&lora_send_config);
            // GB_DEBUGI(RF24_TAG, "Transmission begin, %d", current_config);  // payload was delivered
            switch (current_config)
            {
            case LORA_SEND_NA:
                send_package.type = GB_INIT_DATA;
                send_package.sync = 0xaa;
                GB_DEBUGI(RF24_TAG, "Sending init data, %d", send_package.sync);
                break;
            case LORA_SEND_SKY_WAL_CONFIG:
                send_package.type = GB_SET_CONFIG;
                send_package.sync = 0xab;
                send_package.config.set_type = GB_SET_THROTTLE;
                adc_read_by_item(ADC_THROTTLE, &send_package.config.throttle, true);
                GB_DEBUGI(RF24_TAG, "Setting throttle, %d", send_package.config.throttle);
                break;
            case LORA_SEND_CONTROL_COMMAND:
                send_package.type = GB_SET_CONFIG;
                send_package.sync = 0xac;
                send_package.config.set_type = GB_SET_CONTROL_ARG;
                adc_read_by_item(ADC_THROTTLE, &send_package.config.control_arg.throttle, true);
                adc_read_by_item(ADC_YAW, &send_package.config.control_arg.yaw, true);
                adc_read_by_item(ADC_PITCH, &send_package.config.control_arg.pitch, true);
                adc_read_by_item(ADC_ROLL, &send_package.config.control_arg.roll, true);
                GB_DEBUGI(RF24_TAG, "Commander throttle, %d, yaw: %d, pitch: %d, roll: %d", send_package.config.control_arg.throttle,
                          send_package.config.control_arg.yaw, send_package.config.control_arg.pitch, send_package.config.control_arg.roll);
                break;
            case LORA_SEND_PID_SET_INFO:
                GB_DEBUGI(RF24_TAG, "LORA_SEND_PID_SET_INFO");
                break;
            case LORA_SEND_PID_GET_INFO:
                GB_DEBUGI(RF24_TAG, "LORA_SEND_PID_GET_INFO");
                break;
            case LORA_GET_MOTION_STATE:
                send_package.type = GB_GET_REQUEST;
                send_package.sync = 0xad;
                send_package.config.set_type = GB_GET_MOTION_STATE;
                GB_DEBUGD(RF24_TAG, "GB_GET_MOTION_STATE");
                break;
            default:
                GB_DEBUGE(ERROR_TAG, "UNKNOWN LORA SEND CONFIG!");
            }

            // This device is a TX node
            GB_RESULT report = radio.write(&radio, &send_package, sizeof(GB_LORA_PACKAGE_T)); // transmit & save the report

            if (report == GB_OK)
            {
                GB_SEND_CONFIG current_config = (GB_SEND_CONFIG)atomic_load(&lora_send_config);
                // GB_DEBUGI(RF24_TAG, "Transmission successful!, config: %02x", radio.read_register(&radio, NRF_CONFIG));
                if (LORA_SEND_SKY_WAL_CONFIG == current_config || LORA_SEND_CONTROL_COMMAND == current_config) // don't need ack for esc setting
                    continue;
                if (LORA_SEND_PID_SET_INFO == current_config && GB_GET_PID_INFO_0_7 == send_package.config.set_type)
                {
                    //uint16_t fake_tbl[1][1];
                    //sendPIDTblInfo(1, 1, fake_tbl);
                    continue;
                }
                lora_state = LORA_RECEIVE;
                radio.startListening(&radio);
                send_retry = 0;
                GB_GetTimerUs(&receive_waittime);
            }
            else
            {
                GB_DEBUGE(RF24_TAG, "Transmission failed or timed out"); // payload was not delivered
                send_retry++;
                if (send_retry >= 5)
                {
                    send_retry = 0;
                    lora_state = LORA_RECEIVE;
                    radio.startListening(&radio);
                }
            }
            continue;
        }
        else if (LORA_RECEIVE == lora_state)
        {

            uint8_t rf_status = radio.get_status(&radio);
            if ((rf_status & _BV(RX_DR)) && lora_state == LORA_RECEIVE)
            {
                // This device is a RX node
                uint8_t pipe;
                if (radio.available(&radio, &pipe))
                {                                                 // is there a payload? get the pipe number that recieved it
                    uint8_t bytes = radio.getPayloadSize(&radio); // get the size of the payload
                    radio.read(&radio, &receive_package, bytes);  // fetch payload from FIFO
                    if (receive_package.sync != send_package.sync + 1)
                    {
                        GB_DEBUGE(ERROR_TAG, "Receive error!");
                    }
                    GB_DEBUGD(RF24_TAG, "type = %d, sync = %d", receive_package.type, receive_package.sync);
                    GB_SEND_CONFIG current_config = (GB_SEND_CONFIG)atomic_load(&lora_send_config);
                    if (LORA_SEND_NA == current_config)
                    {
                        battery_level = receive_package.init.battery_capacity;
                    }
                    else if (LORA_SEND_PID_SET_INFO == current_config)
                    {
                        //wk_remote_single_control(SEND_PID_DONE);
                        atomic_store(&lora_send_config, LORA_SEND_NA);
                    }
                    else if (LORA_SEND_PID_GET_INFO == current_config)
                    {
                        //uint16_t fake_tbl[1][1];
                        //getPIDInfoTable(1, 1, fake_tbl, &(out_package.config.pid));
                        //sendReceivePIDTblInfoWithType(GB_GET_PID_INFO_8_15);
                        atomic_store(&lora_send_config, LORA_SEND_NA);
                    }
                    else if (LORA_GET_MOTION_STATE == current_config)
                    {
                        GB_DEBUGD(RF24_TAG, "Received Quad status roll: %d, pitch: %d, yaw: %d",
                                  receive_package.request.quad_status.roll,
                                  receive_package.request.quad_status.pitch,
                                  receive_package.request.quad_status.yaw);
                        atomic_store(&lora_send_config, LORA_GET_MOTION_STATE);
                        quad3d_set_angle((float)receive_package.request.quad_status.roll / GB_ERLER_SCALE_RATE,
                                         (float)receive_package.request.quad_status.pitch / GB_ERLER_SCALE_RATE,
                                         (float)receive_package.request.quad_status.yaw / GB_ERLER_SCALE_RATE);
                        delay_time = 10;
                    }
                    else
                    {
                        atomic_store(&lora_send_config, LORA_SEND_NA);
                    }
                    lora_state = LORA_SEND;
                    radio.stopListening(&radio);
                }
                else
                {
                    GB_DEBUGI(RF24_TAG, "Received nothing..., try to send again");
                    lora_state = LORA_SEND;
                    radio.stopListening(&radio);
                }
            }
            else
            {
                GB_GetTimerUs(&time_now);
                if (time_now - receive_waittime < NRF24_RECEIVE_WAITTING_TIME)
                {
                    // GB_DEBUGI(RF24_TAG, "Waitting to receive... ");
                    continue;
                }
                else
                {
                    lora_state = LORA_SEND;
                    radio.stopListening(&radio);
                    GB_DEBUGI(RF24_TAG, "Waitting to receive timeout, re-send package");
                }
            }
            if (rf_status & _BV(TX_DS))
            {
                GB_DEBUGI(RF24_TAG, "Transmission successful! ");
            }
            if (rf_status & _BV(MAX_RT))
            {
                GB_DEBUGI(RF24_TAG, "Transmission MAX_RT! ");
            }
        }
        /* Wait to be notified that the transmission is complete.  Note
        the first parameter is pdTRUE, which has the effect of clearing
        the task's notification value back to 0, making the notification
        value act like a binary (rather than a counting) semaphore.  */
        uint32_t ul_notification_value;
        const TickType_t max_block_time = pdMS_TO_TICKS(delay_time);
        ul_notification_value = ulTaskNotifyTake(pdTRUE, max_block_time);

        if (ul_notification_value == 1)
        {
            /* The transmission ended as expected. */
        }
        else
        {
            /* The call to ulTaskNotifyTake() timed out. */
        }
    }
}
