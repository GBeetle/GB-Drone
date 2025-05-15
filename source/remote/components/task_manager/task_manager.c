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

#define LV_TICK_PERIOD_MS 10

typedef enum
{
    GB_THROTTLE,
    GB_YAW,
    GB_PITCH,
    GB_ROLL,
    GB_TYPE_MAX,
} GB_SAMPLE_ITEM;

extern void welkin_widgets();
typedef void (*create_demo)(void);

SemaphoreHandle_t xGuiSemaphore;
static GB_SEND_CONFIG lora_send_config = LORA_SEND_NA;

/*
 * G = ((S - Smin) * (Gmax - Gmin)) / (Smax - Smin) + Gmin
 */
static float _scale_to(float val, float src_min, float src_max, float dst_min, float dst_max)
{
    return ((val - src_min) * (dst_max - dst_min)) / (src_max - src_min) + dst_min;
}

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
                control_waittime = esp_timer_get_time();
            if (esp_timer_get_time() - control_waittime > NRF24_CONTROL_WAITTING_TIME && throttle_adc == 0 && pitch_adc == 0)
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
            // set angle
            quad3d_set_angle(_scale_to(roll_adc, 0, ADC_CONSTRAIN_MAX, -30, 30),
                             _scale_to(pitch_adc, 0, ADC_CONSTRAIN_MAX, -30, 30),
                             _scale_to(yaw_adc, 0, ADC_CONSTRAIN_MAX, -180, 180));

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
