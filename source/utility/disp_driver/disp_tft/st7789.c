/**
 * @file st7789.c
 *
 * Mostly taken from lbthomsen/esp-idf-littlevgl github.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "st7789.h"

#include "disp_spi.h"
#include "gpio_setting.h"
#include "gb_timer.h"
#include "log_sys.h"

/*********************
 *      DEFINES
 *********************/
#define TAG "st7789"
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
static void st7789_set_orientation(uint8_t orientation);

static void st7789_send_cmd(uint8_t cmd);
static void st7789_send_data(void *data, uint16_t length);
static void st7789_send_color(void *data, uint16_t length);

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void st7789_init(void)
{
    lcd_init_cmd_t st7789_init_cmds[] = {
        {0xCF, {0x00, 0x83, 0X30}, 3},
        {0xED, {0x64, 0x03, 0X12, 0X81}, 4},
        {ST7789_PWCTRL2, {0x85, 0x01, 0x79}, 3},
        {0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},
        {0xF7, {0x20}, 1},
        {0xEA, {0x00, 0x00}, 2},
        {ST7789_LCMCTRL, {0x26}, 1},
        {ST7789_IDSET, {0x11}, 1},
        {ST7789_VCMOFSET, {0x35, 0x3E}, 2},
        {ST7789_CABCCTRL, {0xBE}, 1},
        {ST7789_MADCTL, {0x00}, 1}, // Set to 0x28 if your display is flipped
        {ST7789_COLMOD, {0x55}, 1},

#if ST7789_INVERT_COLORS == 1
		{ST7789_INVON, {0}, 0}, // set inverted mode
#else
 		{ST7789_INVOFF, {0}, 0}, // set non-inverted mode
#endif

        {ST7789_RGBCTRL, {0x00, 0x1B}, 2},
        {0xF2, {0x08}, 1},
        {ST7789_GAMSET, {0x01}, 1},
        {ST7789_PVGAMCTRL, {0xD0, 0x00, 0x02, 0x07, 0x0A, 0x28, 0x32, 0x44, 0x42, 0x06, 0x0E, 0x12, 0x14, 0x17}, 14},
        {ST7789_NVGAMCTRL, {0xD0, 0x00, 0x02, 0x07, 0x0A, 0x28, 0x31, 0x54, 0x47, 0x0E, 0x1C, 0x17, 0x1B, 0x1E}, 14},
        {ST7789_CASET, {0x00, 0x00, 0x00, 0xEF}, 4},
        {ST7789_RASET, {0x00, 0x00, 0x01, 0x3f}, 4},
        {ST7789_RAMWR, {0}, 0},
        {ST7789_GCTRL, {0x07}, 1},
        {0xB6, {0x0A, 0x82, 0x27, 0x00}, 4},
        {ST7789_SLPOUT, {0}, 0x80},
        {ST7789_DISPON, {0}, 0x80},
        {0, {0}, 0xff},
    };

    //Initialize non-SPI GPIOs
    GB_GPIO_Reset(ST7789_DC);
    GB_GPIO_SetDirection(ST7789_DC, GB_GPIO_OUTPUT);

#if !defined(CONFIG_DISP_ST7789_SOFT_RESET)
    GB_GPIO_Reset(ST7789_RST);
    GB_GPIO_SetDirection(ST7789_RST, GB_GPIO_OUTPUT);
#endif

#if ST7789_ENABLE_BACKLIGHT_CONTROL
    GB_GPIO_Reset(ST7789_BCKL);
    GB_GPIO_SetDirection(ST7789_BCKL, GB_GPIO_OUTPUT);
#endif

    //Reset the display
#if !defined(CONFIG_DISP_ST7789_SOFT_RESET)
    GB_GPIO_Set(ST7789_RST, 0);
    GB_SleepMs(100);
    GB_GPIO_Set(ST7789_RST, 1);
    GB_SleepMs(100);
#else
    st7789_send_cmd(ST7789_SWRESET);
#endif

    GB_DEBUGI(DISP_TAG, "ST7789 initialization.\n");

    //Send all the commands
    uint16_t cmd = 0;
    while (st7789_init_cmds[cmd].databytes!=0xff) {
        st7789_send_cmd(st7789_init_cmds[cmd].cmd);
        st7789_send_data(st7789_init_cmds[cmd].data, st7789_init_cmds[cmd].databytes&0x1F);
        if (st7789_init_cmds[cmd].databytes & 0x80) {
            GB_SleepMs(100);
        }
        cmd++;
    }

    st7789_enable_backlight(true);

    st7789_set_orientation(CONFIG_DISPLAY_ORIENTATION);
}

void st7789_enable_backlight(bool backlight)
{
#if ST7789_ENABLE_BACKLIGHT_CONTROL
    GB_DEBUGI(DISP_TAG, "%s backlight.\n", backlight ? "Enabling" : "Disabling");
    uint32_t tmp = 0;

#if (ST7789_BCKL_ACTIVE_LVL==1)
    tmp = backlight ? 1 : 0;
#else
    tmp = backlight ? 0 : 1;
#endif

    GB_GPIO_Set(ST7789_BCKL, tmp);
#endif
}

/* The ST7789 display controller can drive 320*240 displays, when using a 240*240
 * display there's a gap of 80px, we need to edit the coordinates to take into
 * account that gap, this is not necessary in all orientations. */
void st7789_flush(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, void *color_map)
{
    uint8_t data[4] = {0};

    uint16_t offsetx1 = x1;
    uint16_t offsetx2 = x2;
    uint16_t offsety1 = y1;
    uint16_t offsety2 = y2;

    //GB_DEBUGI(DISP_TAG, "st7789_flush (%d, %d), (%d, %d)", x1, y1, x2, y2);

#if (CONFIG_TFT_DISPLAY_OFFSETS)
    offsetx1 += CONFIG_TFT_DISPLAY_X_OFFSET;
    offsetx2 += CONFIG_TFT_DISPLAY_X_OFFSET;
    offsety1 += CONFIG_TFT_DISPLAY_Y_OFFSET;
    offsety2 += CONFIG_TFT_DISPLAY_Y_OFFSET;

#elif (LV_HOR_RES_MAX == 240) && (LV_VER_RES_MAX == 240)
#if (CONFIG_DISPLAY_ORIENTATION_PORTRAIT)
    offsetx1 += 80;
    offsetx2 += 80;
#elif (CONFIG_DISPLAY_ORIENTATION_LANDSCAPE_INVERTED)
    offsety1 += 80;
    offsety2 += 80;
#endif
#endif

    /*Column addresses*/
    st7789_send_cmd(ST7789_CASET);
    data[0] = (offsetx1 >> 8) & 0xFF;
    data[1] = offsetx1 & 0xFF;
    data[2] = (offsetx2 >> 8) & 0xFF;
    data[3] = offsetx2 & 0xFF;
    st7789_send_data(data, 4);

    /*Page addresses*/
    st7789_send_cmd(ST7789_RASET);
    data[0] = (offsety1 >> 8) & 0xFF;
    data[1] = offsety1 & 0xFF;
    data[2] = (offsety2 >> 8) & 0xFF;
    data[3] = offsety2 & 0xFF;
    st7789_send_data(data, 4);

    /*Memory write*/
    st7789_send_cmd(ST7789_RAMWR);

    uint32_t size = (x2 - x1 + 1) * (y2 - y1 + 1);

    st7789_send_color((void*)color_map, size * 2);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/
static void st7789_send_cmd(uint8_t cmd)
{
    disp_wait_for_pending_transactions();
    GB_GPIO_Set(ST7789_DC, 0);
    disp_spi_send_data(&cmd, 1);
}

static void st7789_send_data(void * data, uint16_t length)
{
    disp_wait_for_pending_transactions();
    GB_GPIO_Set(ST7789_DC, 1);
    disp_spi_send_data(data, length);
}

static void st7789_send_color(void * data, uint16_t length)
{
    disp_wait_for_pending_transactions();
    GB_GPIO_Set(ST7789_DC, 1);
    disp_spi_send_colors(data, length);
}

static void st7789_set_orientation(uint8_t orientation)
{
    // ESP_ASSERT(orientation < 4);

    const char *orientation_str[] = {
        "PORTRAIT", "PORTRAIT_INVERTED", "LANDSCAPE", "LANDSCAPE_INVERTED"
    };

    GB_DEBUGI(DISP_TAG, "Display orientation: %s", orientation_str[orientation]);

    uint8_t data[] =
    {
#if CONFIG_PREDEFINED_DISPLAY_TTGO
	0x60, 0xA0, 0x00, 0xC0
#else
	0xC0, 0x00, 0x60, 0xA0
#endif
    };

    GB_DEBUGI(DISP_TAG, "0x36 command value: 0x%02X", data[orientation]);

    st7789_send_cmd(ST7789_MADCTL);
    st7789_send_data((void *) &data[orientation], 1);
}
