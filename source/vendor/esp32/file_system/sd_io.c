/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <stdio.h>
#include <inttypes.h>
#include <unistd.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "driver/gpio.h"
#include "esp_cpu.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "io_define.h"
#include "log_sys.h"
#include "error_handle.h"
#include "results.h"
#include "file_system.h"
#include "sd_io.h"

#define GPIO_INPUT_PIN_SEL(pin) (1ULL << pin)
#define EXAMPLE_IS_UHS1 (CONFIG_SDMMC_SPEED_UHS_I_SDR50 || CONFIG_SDMMC_SPEED_UHS_I_DDR50)


extern char *gb_file_system_partition[GB_FILE_MAX];

const char *names[] = {"CLK", "CMD", "D0", "D1", "D2", "D3"};
const int pins[] = {SD_CLK,
                    SD_CMD,
                    SD_D0
#ifdef CONFIG_SDMMC_BUS_WIDTH_4
                    ,
                    CONFIG_EXAMPLE_PIN_D1,
                    CONFIG_EXAMPLE_PIN_D2,
                    CONFIG_EXAMPLE_PIN_D3
#endif
};

const int pin_count = sizeof(pins) / sizeof(pins[0]);

pin_configuration_t config = {
    .names = names,
    .pins = pins,
};

static uint32_t get_cycles_until_pin_level(int i, int level, int timeout)
{
    uint32_t start = esp_cpu_get_cycle_count();
    while (gpio_get_level(i) == !level && esp_cpu_get_cycle_count() - start < timeout)
    {
        ;
    }
    uint32_t end = esp_cpu_get_cycle_count();
    return end - start;
}

void check_sd_card_pins(pin_configuration_t *config, const int pin_count)
{
    GB_DEBUGI(FS_TAG, "Testing SD pin connections and pullup strength");
    gpio_config_t io_conf = {};
    for (int i = 0; i < pin_count; ++i)
    {
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_INPUT_OUTPUT_OD;
        io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL(config->pins[i]);
        io_conf.pull_down_en = 0;
        io_conf.pull_up_en = 0;
        gpio_config(&io_conf);
    }

    GB_DEBUGI(FS_TAG, "\n**** PIN recovery time ****\n\n");

    for (int i = 0; i < pin_count; ++i)
    {
        gpio_set_direction(config->pins[i], GPIO_MODE_INPUT_OUTPUT_OD);
        gpio_set_level(config->pins[i], 0);
        usleep(100);
        gpio_set_level(config->pins[i], 1);
        uint32_t cycles = get_cycles_until_pin_level(config->pins[i], 1, 10000);
        GB_DEBUGI(FS_TAG, "PIN %2d %3s  %" PRIu32 " cycles\n", config->pins[i], config->names[i], cycles);
    }

    GB_DEBUGI(FS_TAG, "\n**** PIN recovery time with weak pullup ****\n\n");

    for (int i = 0; i < pin_count; ++i)
    {
        gpio_set_direction(config->pins[i], GPIO_MODE_INPUT_OUTPUT_OD);
        gpio_pullup_en(config->pins[i]);
        gpio_set_level(config->pins[i], 0);
        usleep(100);
        gpio_set_level(config->pins[i], 1);
        uint32_t cycles = get_cycles_until_pin_level(config->pins[i], 1, 10000);
        GB_DEBUGI(FS_TAG, "PIN %2d %3s  %" PRIu32 " cycles\n", config->pins[i], config->names[i], cycles);
        gpio_pullup_dis(config->pins[i]);
    }
}

GB_RESULT GB_SDCardFileSystem_Init()
{
    GB_RESULT res = GB_OK;
    esp_err_t ret;

    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024};
    sdmmc_card_t *card;
    GB_DEBUGI(FS_TAG, "Initializing SD card");

    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
    // Please check its source code and implement error recovery when developing
    // production applications.

    GB_DEBUGI(FS_TAG, "Using SDMMC peripheral");

    // By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
    // For setting a specific frequency, use host.max_freq_khz (range 400kHz - 40MHz for SDMMC)
    // Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000;
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
#if CONFIG_SDMMC_SPEED_HS
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
#elif CONFIG_SDMMC_SPEED_UHS_I_SDR50
    host.slot = SDMMC_HOST_SLOT_0;
    host.max_freq_khz = SDMMC_FREQ_SDR50;
    host.flags &= ~SDMMC_HOST_FLAG_DDR;
#elif CONFIG_SDMMC_SPEED_UHS_I_DDR50
    host.slot = SDMMC_HOST_SLOT_0;
    host.max_freq_khz = SDMMC_FREQ_DDR50;
#endif

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
#if EXAMPLE_IS_UHS1
    slot_config.flags |= SDMMC_SLOT_FLAG_UHS1;
#endif

    // Set bus width to use:
#ifdef CONFIG_SDMMC_BUS_WIDTH_4
    slot_config.width = 4;
#else
    slot_config.width = 1;
#endif

    slot_config.clk = SD_CLK;
    slot_config.cmd = SD_CMD;
    slot_config.d0 = SD_D0;
#ifdef CONFIG_SDMMC_BUS_WIDTH_4
    slot_config.d1 = SD_D1;
    slot_config.d2 = SD_D2;
    slot_config.d3 = SD_D3;
#endif // CONFIG_SDMMC_BUS_WIDTH_4

    // Enable internal pullups on enabled pins. The internal pullups
    // are insufficient however, please make sure 10k external pullups are
    // connected on the bus. This is for debug / example purpose only.
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    GB_DEBUGI(FS_TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdmmc_mount(gb_file_system_partition[GB_FILE_SD_CARD], &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            GB_DEBUGE(FS_TAG, "Failed to mount filesystem. "
                          "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        }
        else
        {
            GB_DEBUGE(FS_TAG, "Failed to initialize the card (%s). "
                          "Make sure SD card lines have pull-up resistors in place.",
                     esp_err_to_name(ret));
            check_sd_card_pins(&config, pin_count);
        }
    }
    GB_DEBUGI(FS_TAG, "Filesystem mounted");

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);

    // All done, unmount partition and disable SDMMC peripheral
    // esp_vfs_fat_sdcard_unmount(gb_file_system_partition[GB_FILE_SD_CARD], card);
    // GB_DEBUGI(FS_TAG, "Card unmounted");

    return res;
}
