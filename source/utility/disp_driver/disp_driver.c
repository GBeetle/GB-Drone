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

#include "log_sys.h"
#include "disp_driver.h"
#include "spi_bus.h"
#include "gb_timer.h"

void tft_driver_init(void)
{
#if defined CONFIG_TFT_DISPLAY_CONTROLLER_ILI9341
    ili9341_init();
#elif defined CONFIG_TFT_DISPLAY_CONTROLLER_ILI9481
    ili9481_init();
#elif defined CONFIG_TFT_DISPLAY_CONTROLLER_ILI9488
    ili9488_init();
#elif defined CONFIG_TFT_DISPLAY_CONTROLLER_ST7789
    st7789_init();
#elif defined CONFIG_TFT_DISPLAY_CONTROLLER_ST7796S
    st7796s_init();
#elif defined CONFIG_TFT_DISPLAY_CONTROLLER_ST7735S
    st7735s_init();
#elif defined CONFIG_TFT_DISPLAY_CONTROLLER_HX8357
    hx8357_init();
#elif defined CONFIG_TFT_DISPLAY_CONTROLLER_ILI9486
    ili9486_init();
#elif defined CONFIG_TFT_DISPLAY_CONTROLLER_SH1107
    sh1107_init();
#elif defined CONFIG_TFT_DISPLAY_CONTROLLER_SSD1306
    ssd1306_init();
#elif defined CONFIG_TFT_DISPLAY_CONTROLLER_FT81X
    FT81x_init();
#elif defined CONFIG_TFT_DISPLAY_CONTROLLER_IL3820
    il3820_init();
#elif defined CONFIG_TFT_DISPLAY_CONTROLLER_RA8875
    ra8875_init();
#elif defined CONFIG_TFT_DISPLAY_CONTROLLER_GC9A01
   GC9A01_init();
#elif defined CONFIG_TFT_DISPLAY_CONTROLLER_JD79653A
   jd79653a_init();
#elif defined CONFIG_TFT_DISPLAY_CONTROLLER_UC8151D
   uc8151d_init();
#endif
}

void disp_driver_init(void)
{
    GB_DEBUGI(DISP_TAG, "Display hor size: %d, ver size: %d", LV_HOR_RES_MAX, LV_VER_RES_MAX);
    GB_DEBUGI(DISP_TAG, "Display buffer size: %d", DISP_BUF_SIZE);

#if defined CONFIG_TFT_DISPLAY_PROTOCOL_SPI
    CHK_EXIT(hspi.begin(&hspi, CONFIG_DISP_SPI_MOSI, CONFIG_DISP_SPI_MISO, CONFIG_DISP_SPI_CLK, 0));
    CHK_EXIT(hspi.addDevice(&hspi, GB_SPI_DEV_0, 0, 2, SPI_DEVICE_NO_DUMMY, SPI_TFT_CLOCK_SPEED_HZ, -1));
#elif defined CONFIG_TFT_DISPLAY_PROTOCOL_I2C
    // TODO
#else
    #error "No protocol defined for display controller"
#endif
    tft_driver_init();

/* Touch controller initialization */
#if CONFIG_TOUCH_CONTROLLER != TOUCH_CONTROLLER_NONE
#if defined (CONFIG_TOUCH_DRIVER_PROTOCOL_SPI)
    GB_DEBUGI(DISP_TAG, "Initializing SPI master for touch");

    // add TP SPI device
#elif defined (CONFIG_TOUCH_DRIVER_PROTOCOL_I2C)
    GB_DEBUGI(DISP_TAG, "Initializing I2C master for touch");

    // add TP I2C device
#error "No protocol defined for touch controller"
#endif
    touch_driver_init();
#endif

}

// TODO
void disp_driver_set_rotation(uint8_t m)
{

}
