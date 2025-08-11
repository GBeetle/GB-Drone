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

#define LV_TICK_PERIOD_MS 10

extern void welkin_widgets();
extern int battery_level;
typedef void (*create_demo)(void);

SemaphoreHandle_t xGuiSemaphore;
GB_SEND_CONFIG lora_send_config = LORA_SEND_NA;

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

    lv_color_t *buf1 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1 != NULL);

    /* Use double buffered when not working with monochrome displays */
    lv_color_t *buf2 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2 != NULL);

    static lv_disp_buf_t disp_buf;

    uint32_t size_in_px = DISP_BUF_SIZE;

    /* Initialize the working buffer depending on the selected display.
     * NOTE: buf2 == NULL when using monochrome displays. */
    lv_disp_buf_init(&disp_buf, buf1, buf2, size_in_px);

    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.flush_cb = lvgl_driver_flush;

    GB_DEBUGI(DISP_TAG, "lv_disp_drv_init hor_res: %d, ver_res: %d", disp_drv.hor_res, disp_drv.ver_res);

#if defined CONFIG_DISPLAY_ORIENTATION_PORTRAIT || defined CONFIG_DISPLAY_ORIENTATION_PORTRAIT_INVERTED
    disp_drv.rotated = 1;
#endif

    disp_drv.buffer = &disp_buf;
    lv_disp_drv_register(&disp_drv);

    /* Register an input device when enabled on the menuconfig */
#if CONFIG_LV_TOUCH_CONTROLLER != TOUCH_CONTROLLER_NONE
    lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.read_cb = touch_driver_read;
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    lv_indev_drv_register(&indev_drv);
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
            lv_task_handler();
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
    uint64_t control_waittime = 0;
    uint64_t time_now = 0;
    uint8_t ui_operation_mode = 0;
    uint8_t left_up_op = 0;
    uint16_t throttle_adc, pitch_adc, roll_adc, yaw_adc;

    adc_wrapper_init();
    while (1)
    {
        ui_operation_mode = 0;

        adc_read_by_item(ADC_THROTTLE, &throttle_adc, true);
        adc_read_by_item(ADC_PITCH, &pitch_adc, true);
        adc_read_by_item(ADC_ROLL, &roll_adc, true);
        adc_read_by_item(ADC_YAW, &yaw_adc, true);

        //GB_DEBUGI(GB_INFO, "T: %d, P: %d, R: %d, Y: %d", throttle_adc, pitch_adc, roll_adc, yaw_adc);

        // 飞控设置，双摇杆推到最低点3s，将方向摇杆回中
        if (GB_FLY_MODE == gb_get_user_mode() && throttle_adc == 0 && pitch_adc == 0)
        {
            if (control_waittime == 0)
                GB_GetTimerUs(&control_waittime);
            GB_GetTimerUs(&time_now);
            if (time_now - control_waittime > NRF24_CONTROL_WAITTING_TIME && throttle_adc == 0 && pitch_adc == 0)
            {
                GB_DEBUGI(GB_INFO, "Ready to send control command, should reset right controller");
                while (pitch_adc < ADC_CONSTRAIN_MIDDLE - 50 || pitch_adc > ADC_CONSTRAIN_MIDDLE + 50)
                    adc_read_by_item(ADC_PITCH, &pitch_adc, true);
                lora_send_config = LORA_SEND_CONTROL_COMMAND;
                control_waittime = 0;
            }
        }
        else if (throttle_adc != 0 || pitch_adc != 0)
        {
            control_waittime = 0;
        }

        if (LORA_SEND_CONTROL_COMMAND != lora_send_config) // not fly mode, only control UI
        {
#if 0
            // set angle
            quad3d_set_angle(_scale_to(roll_adc, 0, ADC_CONSTRAIN_MAX, -30, 30),
                             _scale_to(pitch_adc, 0, ADC_CONSTRAIN_MAX, -30, 30),
                             _scale_to(yaw_adc, 0, ADC_CONSTRAIN_MAX, -180, 180));
#endif

            // 右摇杆左右滑动
            if (roll_adc <= ADC_CONSTRAIN_MIDDLE / 2)
            {
                gb_remote_single_control(R_RIGHT);
                ui_operation_mode = 1;
                //GB_DEBUGI(GB_INFO, "[R : R]");
            }
            else if (roll_adc >= ADC_CONSTRAIN_MIDDLE + ADC_CONSTRAIN_MIDDLE / 2)
            {
                gb_remote_single_control(R_LEFT);
                ui_operation_mode = 1;
                //GB_DEBUGI(GB_INFO, "[R : L]");
            }
            else if (pitch_adc <= ADC_CONSTRAIN_MIDDLE / 2)
            {
                gb_remote_single_control(L_UP_R_DOWN);
                ui_operation_mode = 1;
                //GB_DEBUGI(GB_INFO, "[R : D]");
            }
            else if (pitch_adc >= ADC_CONSTRAIN_MIDDLE + ADC_CONSTRAIN_MIDDLE / 2)
            {
                gb_remote_single_control(L_DOWN_R_UP);
                ui_operation_mode = 1;
                //GB_DEBUGI(GB_INFO, "[R : U]");
            }

            // only need for use mode
            if (GB_PID_SETTING_MODE == gb_get_user_mode())
            {
                // 左摇杆下，右摇杆上下滑动
                if ((throttle_adc <= ADC_CONSTRAIN_MIDDLE / 2) && (pitch_adc <= ADC_CONSTRAIN_MIDDLE / 2))
                {
                    gb_remote_single_control(L_DOWN_R_DOWN);
                    ui_operation_mode = 1;
                }
                else if ((throttle_adc <= ADC_CONSTRAIN_MIDDLE / 2) &&
                         (pitch_adc >= ADC_CONSTRAIN_MIDDLE + ADC_CONSTRAIN_MIDDLE / 2))
                {
                    gb_remote_single_control(L_DOWN_R_UP);
                    ui_operation_mode = 1;
                }

                // 左摇杆上，右摇杆上下滑动
                if ((throttle_adc >= ADC_CONSTRAIN_MIDDLE + ADC_CONSTRAIN_MIDDLE / 2) &&
                    (pitch_adc <= ADC_CONSTRAIN_MIDDLE / 2))
                {
                    gb_remote_single_control(L_UP_R_DOWN);
                    ui_operation_mode = 2;
                }
                else if ((throttle_adc >= ADC_CONSTRAIN_MIDDLE + ADC_CONSTRAIN_MIDDLE / 2) &&
                         (pitch_adc >= ADC_CONSTRAIN_MIDDLE + ADC_CONSTRAIN_MIDDLE / 2))
                {
                    gb_remote_single_control(L_UP_R_UP);
                    ui_operation_mode = 2;
                }

                // 左摇杆向上滑动, 右摇杆居中
                if ((throttle_adc >= ADC_CONSTRAIN_MIDDLE + ADC_CONSTRAIN_MIDDLE / 2) && (0 == left_up_op) &&
                    (pitch_adc >= ADC_CONSTRAIN_MIDDLE - 50 && pitch_adc <= ADC_CONSTRAIN_MIDDLE + 50))
                {
                    gb_remote_single_control(L_UP);
                    ui_operation_mode = 1;
                    left_up_op = 1;
                }
                // 左摇杆向下，重置
                else if (throttle_adc <= ADC_CONSTRAIN_MIDDLE / 2)
                {
                    left_up_op = 0;
                }
            }

            // 左摇杆左右滑动
            if (yaw_adc <= ADC_CONSTRAIN_MIDDLE / 2)
            {
                gb_remote_single_control(L_RIGHT);
                ui_operation_mode = 1;
            }
            else if (yaw_adc >= ADC_CONSTRAIN_MIDDLE + ADC_CONSTRAIN_MIDDLE / 2)
            {
                gb_remote_single_control(L_LEFT);
                ui_operation_mode = 1;
            }
        }

        if (ui_operation_mode == 2)
            vTaskDelay(pdMS_TO_TICKS(100));
        else if (ui_operation_mode == 1)
            vTaskDelay(pdMS_TO_TICKS(300));
        else
            vTaskDelay(pdMS_TO_TICKS(10));

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
            // GB_DEBUGI(RF24_TAG, "Transmission begin, %d", lora_send_config);  // payload was delivered
            switch (lora_send_config)
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

            if (report >= 0)
            {
                // GB_DEBUGI(RF24_TAG, "Transmission successful!, config: %02x", radio.read_register(&radio, NRF_CONFIG));
                if (LORA_SEND_SKY_WAL_CONFIG == lora_send_config || LORA_SEND_CONTROL_COMMAND == lora_send_config) // don't need ack for esc setting
                    continue;
                if (LORA_SEND_PID_SET_INFO == lora_send_config && GB_GET_PID_INFO_0_7 == send_package.config.set_type)
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
                    if (LORA_SEND_NA == lora_send_config)
                    {
                        battery_level = receive_package.init.battery_capacity;
                    }
                    else if (LORA_SEND_PID_SET_INFO == lora_send_config)
                    {
                        //wk_remote_single_control(SEND_PID_DONE);
                        lora_send_config = LORA_SEND_NA;
                    }
                    else if (LORA_SEND_PID_GET_INFO == lora_send_config && GB_SET_PID_0_7 == send_package.request.get_type)
                    {
                        //uint16_t fake_tbl[1][1];
                        //getPIDInfoTable(1, 1, fake_tbl, &(out_package.config.pid));
                        //sendReceivePIDTblInfoWithType(GB_GET_PID_INFO_8_15);
                    }
                    else if (LORA_SEND_PID_GET_INFO == lora_send_config && GB_SET_PID_8_15 == send_package.request.get_type)
                    {
                        //wk_remote_single_control(FLASH_PID_TBL);
                        lora_send_config = LORA_SEND_NA;
                    }
                    else if (LORA_GET_MOTION_STATE == lora_send_config)
                    {
                        GB_DEBUGD(RF24_TAG, "Received Quad status roll: %d, pitch: %d, yaw: %d",
                                  receive_package.request.quad_status.roll,
                                  receive_package.request.quad_status.pitch,
                                  receive_package.request.quad_status.yaw);
                        lora_send_config = LORA_GET_MOTION_STATE;
                        quad3d_set_angle((float)receive_package.request.quad_status.roll / GB_ERLER_SCALE_RATE,
                                         (float)receive_package.request.quad_status.pitch / GB_ERLER_SCALE_RATE,
                                         (float)receive_package.request.quad_status.yaw / GB_ERLER_SCALE_RATE);
                        delay_time = 10;
                    }
                    else
                    {
                        lora_send_config = LORA_SEND_NA;
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
