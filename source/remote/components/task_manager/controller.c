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

#include "controller.h"
#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "lora_state.h"
#include "log_sys.h"
#include "gb_timer.h"

#define ADC_12BITS_SAMPLE_MAX 0xfff
#define ADC_OUTPUT_MAX        1000
#define DEADZONE              30
#define FILTER_ALPHA          0.2f

typedef struct {
    int min;
    int max;
    int center;

    float filtered;
} adc_proc_t;

static adc_oneshot_unit_handle_t adc_handle;

static const adc_unit_t adc_unit = ADC_UNIT_2;
static const adc_atten_t atten = ADC_ATTEN_DB_12;

static const adc_channel_t throttle_ch = ADC_CHANNEL_2;
static const adc_channel_t yaw_ch = ADC_CHANNEL_3;
static const adc_channel_t pitch_ch = ADC_CHANNEL_5;
static const adc_channel_t roll_ch = ADC_CHANNEL_4;

static adc_cali_handle_t throttle_cali_handle = NULL;
static adc_cali_handle_t yaw_cali_handle = NULL;
static adc_cali_handle_t pitch_cali_handle = NULL;
static adc_cali_handle_t roll_cali_handle = NULL;

static adc_proc_t throttle_proc = {
    .min = 50,
    .max = 3200,
    .center = 1600,
    .filtered = 500,
};

static adc_proc_t yaw_proc = {
    .min = 0,
    .max = 3200,
    .center = 1620,
    .filtered = 500,
};

static adc_proc_t pitch_proc = {
    .min = 105,
    .max = 2980,
    .center = 1560,
    .filtered = 500,
};

static adc_proc_t roll_proc = {
    .min = 130,
    .max = 3120,
    .center = 1700,
    .filtered = 500,
};


static inline int clamp(int x, int min, int max)
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

// 0~1000
static int normalize(int raw, adc_proc_t *p)
{
    raw = clamp(raw, p->min, p->max);

    return (raw - p->min) * ADC_OUTPUT_MAX / (p->max - p->min);
}

// DEADZONE center = 500
static int apply_deadzone(int val)
{
    if (abs(val - 500) < DEADZONE)
        return 500;
    return val;
}

// one order filter
static float lowpass_filter(float input, float prev)
{
    return FILTER_ALPHA * input + (1 - FILTER_ALPHA) * prev;
}

static int adc_process(adc_proc_t *p, int raw)
{
    int val = normalize(raw, p);

    val = apply_deadzone(val);

    p->filtered = lowpass_filter((float)val, p->filtered);

    return (int)(p->filtered + 0.5f);
}

GB_RESULT adc_wrapper_init(void)
{
    GB_RESULT res = GB_OK;

    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = adc_unit,
    };
    adc_oneshot_new_unit(&init_config, &adc_handle);

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = atten,
    };

    CHK_ESP_ERROR(adc_oneshot_config_channel(adc_handle, throttle_ch, &config), GB_ADC_INIT_FAIL);
    CHK_ESP_ERROR(adc_oneshot_config_channel(adc_handle, yaw_ch, &config), GB_ADC_INIT_FAIL);
    CHK_ESP_ERROR(adc_oneshot_config_channel(adc_handle, pitch_ch, &config), GB_ADC_INIT_FAIL);
    CHK_ESP_ERROR(adc_oneshot_config_channel(adc_handle, roll_ch, &config), GB_ADC_INIT_FAIL);

    adc_cali_curve_fitting_config_t throttle_cali_config = {
        .unit_id = adc_unit,
        .chan = throttle_ch,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_12,
    };
    CHK_ESP_ERROR(adc_cali_create_scheme_curve_fitting(&throttle_cali_config, &throttle_cali_handle), GB_ADC_CALI_FAIL);

    adc_cali_curve_fitting_config_t yaw_cali_config = {
        .unit_id = adc_unit,
        .chan = yaw_ch,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_12,
    };
    CHK_ESP_ERROR(adc_cali_create_scheme_curve_fitting(&yaw_cali_config, &yaw_cali_handle), GB_ADC_CALI_FAIL);

    adc_cali_curve_fitting_config_t pitch_cali_config = {
        .unit_id = adc_unit,
        .chan = pitch_ch,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_12,
    };
    CHK_ESP_ERROR(adc_cali_create_scheme_curve_fitting(&pitch_cali_config, &pitch_cali_handle), GB_ADC_CALI_FAIL);

    adc_cali_curve_fitting_config_t roll_cali_config = {
        .unit_id = adc_unit,
        .chan = roll_ch,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_12,
    };
    CHK_ESP_ERROR(adc_cali_create_scheme_curve_fitting(&roll_cali_config, &roll_cali_handle), GB_ADC_CALI_FAIL);

error_exit:
    return res;
}

/*
 * This function is used to read adc data by item
 * @item: defined in ADC_SAMPLE_ITEM
 * @adc_val: read value, range 0 ~ ADC_CONSTRAIN_MAX
 */
void adc_read_by_item(uint8_t item, uint16_t *adc_val, uint8_t is_constrained)
{
    int raw = 0;
    adc_channel_t ch;
    //adc_cali_handle_t cali_handle;
    adc_proc_t *proc = NULL;

    *adc_val = 0;
    if (item >= ADC_TYPE_MAX)
        return;

    switch (item)
    {
    case ADC_THROTTLE:
        ch = throttle_ch;
        proc = &throttle_proc;
        //cali_handle = throttle_cali_handle;
        break;
    case ADC_YAW:
        ch = yaw_ch;
        proc = &yaw_proc;
        //cali_handle = yaw_cali_handle;
        break;
    case ADC_PITCH:
        ch = pitch_ch;
        proc = &pitch_proc;
        //cali_handle = pitch_cali_handle;
        break;
    case ADC_ROLL:
        ch = roll_ch;
        proc = &roll_proc;
        //cali_handle = roll_cali_handle;
        break;
    default:
        return;
    }

    adc_oneshot_read(adc_handle, ch, &raw);
    //printf("CH %d RAW: %d\n", ch, raw);
    //adc_cali_raw_to_voltage(cali_handle, raw, (int*)adc_val);
    *adc_val = adc_process(proc, raw);
}

#define BTN_COUNT    13
#define BTN_DEBUGMODE_MS   200 // 200ms debounce
#define TOGGLE_DEBUGMODE_MS 100 // 100ms debounce for toggle switches

// Unified event queue for both buttons and toggle switches
static QueueHandle_t control_event_queue = NULL;

// Mutex to protect toggle switch state
static SemaphoreHandle_t toggle_state_mutex = NULL;
static GB_TOGGLE_SWITCH_STATE g_current_toggle_state = {0};

typedef struct {
    int gpio_num;
    GB_REMOTE_CONTROL_ID btn_id;
    uint64_t last_isr_time;
} btn_entry_t;

static btn_entry_t btn_table[BTN_COUNT] = {
    { BTN_GPIO_DPAD_UP,    BTN_DPAD_UP,    0 },
    { BTN_GPIO_DPAD_DOWN,  BTN_DPAD_DOWN,  0 },
    { BTN_GPIO_DPAD_LEFT,  BTN_DPAD_LEFT,  0 },
    { BTN_GPIO_DPAD_RIGHT, BTN_DPAD_RIGHT, 0 },
    { BTN_GPIO_DPAD_MID,   BTN_DPAD_MID,   0 },
    { BTN_GPIO_OP_A,    BTN_OP_A,    0 },
    { BTN_GPIO_OP_B,    BTN_OP_B,    0 },
    { BTN_GPIO_OP_X,    BTN_OP_X,    0 },
    { BTN_GPIO_OP_Y,    BTN_OP_Y,    0 },
    { BTN_GPIO_OP_MENU,    BTN_OP_MENU,    0 },
    { BTN_GPIO_OP_OPTION,  BTN_OP_OPTION,  0 },
    { BTN_GPIO_OP_START,   BTN_OP_START,   0 },
    { BTN_GPIO_OP_SELECT,  BTN_OP_SELECT,  0 }
};

static uint64_t toggle_last_isr_time = 0;

// Button ISR handler - sends button event to unified queue
static void IRAM_ATTR button_isr_handler(void *arg)
{
    btn_entry_t *entry = (btn_entry_t *)arg;
    uint64_t now = 0;

    GB_GetTimerMs(&now);
    if (now - entry->last_isr_time < BTN_DEBUGMODE_MS) {
        return;
    }

    entry->last_isr_time = now;

    GB_CONTROL_EVENT event = {
        .type = GB_EVENT_BUTTON,
        .data.button_id = entry->btn_id
    };

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(control_event_queue, &event, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken)
        portYIELD_FROM_ISR();
}

static void IRAM_ATTR toggle_switch_isr_handler(void *arg)
{
    uint64_t now = 0;

    GB_GetTimerMs(&now);
    if (now - toggle_last_isr_time < TOGGLE_DEBUGMODE_MS) {
        return;
    }
    toggle_last_isr_time = now;

    GB_CONTROL_EVENT event = {
        .type = GB_EVENT_TOGGLE_SWITCH,
        .data.switch_state = {0}
    };

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(control_event_queue, &event, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken)
        portYIELD_FROM_ISR();
}

void controller_input_init(void)
{
    // Create unified event queue - larger to handle both buttons and switches
    control_event_queue = xQueueCreate(16, sizeof(GB_CONTROL_EVENT));

    // Create mutex for toggle state protection
    toggle_state_mutex = xSemaphoreCreateMutex();

    // Initialize button GPIOs using gpio_setting wrapper
    for (int i = 0; i < BTN_COUNT; i++)
    {
        GB_GPIO_Init(btn_table[i].gpio_num, GB_GPIO_INPUT, GB_GPIO_PULLUP);
    }

    // Initialize toggle switch GPIOs using gpio_setting wrapper
    GB_GPIO_Init(TOGGLE_SW_1_GPIO, GB_GPIO_INPUT, GB_GPIO_PULLUP);
    GB_GPIO_Init(TOGGLE_SW_2_GPIO, GB_GPIO_INPUT, GB_GPIO_PULLUP);
    GB_GPIO_Init(TOGGLE_SW_3_GPIO, GB_GPIO_INPUT, GB_GPIO_PULLUP);
    GB_GPIO_Init(TOGGLE_SW_4_GPIO, GB_GPIO_INPUT, GB_GPIO_PULLUP);

    // Configure interrupts using ESP-IDF GPIO driver (since gpio_setting doesn't support ISR yet)
    gpio_config_t btn_conf = {
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE, // Active low: trigger on falling edge
        .pin_bit_mask = 0,
    };

    for (int i = 0; i < BTN_COUNT; i++)
    {
        btn_conf.pin_bit_mask = 1ULL << btn_table[i].gpio_num;
        gpio_config(&btn_conf);
    }

    gpio_config_t toggle_conf = {
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE, // Trigger on any edge (both rising and falling)
        .pin_bit_mask = (1ULL << TOGGLE_SW_1_GPIO) |
        (1ULL << TOGGLE_SW_2_GPIO) |
        (1ULL << TOGGLE_SW_3_GPIO) |
        (1ULL << TOGGLE_SW_4_GPIO),
    };

    gpio_config(&toggle_conf);

    // Register button ISR handlers
    for (int i = 0; i < BTN_COUNT; i++)
    {
        gpio_isr_handler_add(btn_table[i].gpio_num, button_isr_handler, &btn_table[i]);
    }

    // Register toggle switch ISR handlers
    gpio_isr_handler_add(TOGGLE_SW_1_GPIO, toggle_switch_isr_handler, NULL);
    gpio_isr_handler_add(TOGGLE_SW_2_GPIO, toggle_switch_isr_handler, NULL);
    gpio_isr_handler_add(TOGGLE_SW_3_GPIO, toggle_switch_isr_handler, NULL);
    gpio_isr_handler_add(TOGGLE_SW_4_GPIO, toggle_switch_isr_handler, NULL);

    // Read initial toggle switch state with mutex protection
    if (toggle_state_mutex != NULL)
    {
        uint32_t level;
        GB_TOGGLE_SWITCH_STATE initial_state;

        GB_GPIO_Get(TOGGLE_SW_1_GPIO, &level);
        initial_state.sw1_state = !level;

        GB_GPIO_Get(TOGGLE_SW_2_GPIO, &level);
        initial_state.sw2_state = !level;

        GB_GPIO_Get(TOGGLE_SW_3_GPIO, &level);
        initial_state.sw3_state = !level;

        GB_GPIO_Get(TOGGLE_SW_4_GPIO, &level);
        initial_state.sw4_state = !level;

        // Protect write to global state with mutex
        if (xSemaphoreTake(toggle_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            g_current_toggle_state = initial_state;
            xSemaphoreGive(toggle_state_mutex);
        }
    }

    GB_DEBUGI(GB_INFO, "Controller input initialized - buttons: %d, toggle switches: 4", BTN_COUNT);
}

// Get the unified event queue
QueueHandle_t controller_get_event_queue(void)
{
    return control_event_queue;
}

// Thread-safe getter for toggle switch state
void controller_get_toggle_state(GB_TOGGLE_SWITCH_STATE *state)
{
    if (state == NULL) {
        return;
    }

    if (xSemaphoreTake(toggle_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        *state = g_current_toggle_state;
        xSemaphoreGive(toggle_state_mutex);
    }
}

void controller_set_toggle_state(const GB_TOGGLE_SWITCH_STATE *state)
{
    if (state == NULL) {
        return;
    }

    if (xSemaphoreTake(toggle_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        g_current_toggle_state = *state;
        xSemaphoreGive(toggle_state_mutex);
    }
}
