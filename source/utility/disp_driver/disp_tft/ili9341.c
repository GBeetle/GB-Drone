/**
 * @file ili9341.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "ili9341.h"
#include "disp_spi.h"
#include "gpio_setting.h"
#include "gb_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "spi_bus.h"

/*********************
 *      DEFINES
 *********************/
 #define TAG "ILI9341"

/**********************
 *      TYPEDEFS
 **********************/

/*The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct. */
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void ili9341_set_orientation(uint8_t orientation);

static void ili9341_send_cmd(uint8_t cmd);
static void ili9341_send_data(void * data, uint16_t length);
static void ili9341_send_color(void * data, uint16_t length);

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void ili9341_init(void)
{
    lcd_init_cmd_t ili_init_cmds[]={
        {0xCF, {0x00, 0x83, 0X30}, 3},
        {0xED, {0x64, 0x03, 0X12, 0X81}, 4},
        {0xE8, {0x85, 0x01, 0x79}, 3},
        {0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},
        {0xF7, {0x20}, 1},
        {0xEA, {0x00, 0x00}, 2},
        {0xC0, {0x26}, 1},
        {0xC1, {0x11}, 1},
        {0xC5, {0x35, 0x3E}, 2},
        {0xC7, {0xBE}, 1},
        {0x36, {0x28}, 1},
        {0x3A, {0x55}, 1},
        {0xB1, {0x00, 0x1B}, 2},
        {0xF2, {0x08}, 1},
        {0x26, {0x01}, 1},
        {0xE0, {0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0X87, 0x32, 0x0A, 0x07, 0x02, 0x07, 0x05, 0x00}, 15},
        {0XE1, {0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 0x4D, 0x05, 0x18, 0x0D, 0x38, 0x3A, 0x1F}, 15},
        {0x2A, {0x00, 0x00, 0x00, 0xEF}, 4},
        {0x2B, {0x00, 0x00, 0x01, 0x3f}, 4},
        {0x2C, {0}, 0},
        {0xB7, {0x07}, 1},
        {0xB6, {0x0A, 0x82, 0x27, 0x00}, 4},
        {0x11, {0}, 0x80},
        {0x29, {0}, 0x80},
        {0, {0}, 0xff},
    };

    //Initialize non-SPI GPIOs
    GB_GPIO_Reset(ILI9341_DC);
    GB_GPIO_SetDirection(ILI9341_DC, GB_GPIO_OUTPUT);
    GB_GPIO_Set(ILI9341_DC, 0);

    GB_GPIO_Reset(ILI9341_RST);
    GB_GPIO_SetDirection(ILI9341_RST, GB_GPIO_OUTPUT);
    GB_GPIO_Set(ILI9341_RST, 0);
    GB_SleepMs(120);
    GB_GPIO_Set(ILI9341_RST, 1);
    GB_SleepMs(100);

#if ILI9341_ENABLE_BACKLIGHT_CONTROL
    GB_GPIO_Reset(ILI9341_BCKL);
    GB_GPIO_SetDirection(ILI9341_BCKL, GB_GPIO_OUTPUT);
#endif

    GB_DEBUGI(DISP_TAG, "ILI9341 Initialization.");

    //Send all the commands
    uint16_t cmd = 0;
    while (ili_init_cmds[cmd].databytes!=0xff) {
        ili9341_send_cmd(ili_init_cmds[cmd].cmd);
        if (ili_init_cmds[cmd].databytes & 0x80) {
            GB_SleepMs(120);
        }
        else {
            ili9341_send_data(ili_init_cmds[cmd].data, ili_init_cmds[cmd].databytes);
        }
        cmd++;
    }

    ili9341_enable_backlight(true);

    ili9341_set_orientation(CONFIG_DISPLAY_ORIENTATION);

#if ILI9341_INVERT_COLORS == 1
    ili9341_send_cmd(0x21);
#else
    ili9341_send_cmd(0x20);
#endif
}

void ili9341_flush(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, void *color_map)
{
    uint8_t data[4];

    /*Column addresses*/
    ili9341_send_cmd(0x2A);
    data[0] = (x1 >> 8) & 0xFF;
    data[1] = x1 & 0xFF;
    data[2] = (x2 >> 8) & 0xFF;
    data[3] = x2 & 0xFF;
    ili9341_send_data(data, 4);

    /*Page addresses*/
    ili9341_send_cmd(0x2B);
    data[0] = (y1 >> 8) & 0xFF;
    data[1] = y1 & 0xFF;
    data[2] = (y2 >> 8) & 0xFF;
    data[3] = y2 & 0xFF;
    ili9341_send_data(data, 4);

    /*Memory write*/
    ili9341_send_cmd(0x2C);

    uint32_t size = (x2 - x1 + 1) * (y2 - y1 + 1);
    ili9341_send_color((void*)color_map, size * 2);
}

void ili9341_enable_backlight(bool backlight)
{
#if ILI9341_ENABLE_BACKLIGHT_CONTROL
    GB_DEBUGI(DISP_TAG, "%s backlight.", backlight ? "Enabling" : "Disabling");
    uint32_t tmp = 0;

#if (ILI9341_BCKL_ACTIVE_LVL==1)
    tmp = backlight ? 1 : 0;
#else
    tmp = backlight ? 0 : 1;
#endif

    GB_GPIO_Set(ILI9341_BCKL, tmp);
#endif
}

void ili9341_sleep_in()
{
    uint8_t data[] = {0x08};
    ili9341_send_cmd(0x10);
    ili9341_send_data(&data, 1);
}

void ili9341_sleep_out()
{
    uint8_t data[] = {0x08};
    ili9341_send_cmd(0x11);
    ili9341_send_data(&data, 1);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/


static void ili9341_send_cmd(uint8_t cmd)
{
    disp_wait_for_pending_transactions();
    GB_GPIO_Set(ILI9341_DC, 0);     /*Command mode*/
    disp_spi_send_data(&cmd, 1);
}

static void ili9341_send_data(void * data, uint16_t length)
{
    disp_wait_for_pending_transactions();
    GB_GPIO_Set(ILI9341_DC, 1);     /*Data mode*/
    disp_spi_send_data(data, length);
}

static void ili9341_send_color(void * data, uint16_t length)
{
    disp_wait_for_pending_transactions();
    GB_GPIO_Set(ILI9341_DC, 1);   /*Data mode*/
    disp_spi_send_colors(data, length);
}

static void ili9341_set_orientation(uint8_t orientation)
{
    // ESP_ASSERT(orientation < 4);

    const char *orientation_str[] = {
        "PORTRAIT", "PORTRAIT_INVERTED", "LANDSCAPE", "LANDSCAPE_INVERTED"
    };

    GB_DEBUGI(DISP_TAG, "Display orientation: %s", orientation_str[orientation]);

#if defined CONFIG_PREDEFINED_DISPLAY_M5STACK
    uint8_t data[] = {0x68, 0x68, 0x08, 0x08};
#elif defined (CONFIG_PREDEFINED_DISPLAY_WROVER4)
    uint8_t data[] = {0x4C, 0x88, 0x28, 0xE8};
#elif defined (CONFIG_PREDEFINED_DISPLAY_NONE)
    uint8_t data[] = {0x48, 0x88, 0x28, 0xE8};
#endif

    GB_DEBUGI(DISP_TAG, "0x36 command value: 0x%02X", data[orientation]);

    ili9341_send_cmd(0x36);
    ili9341_send_data((void *) &data[orientation], 1);
}
