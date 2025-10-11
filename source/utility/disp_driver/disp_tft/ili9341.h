/**
 * @file lv_templ.h
 *
 */

#ifndef ILI9341_H
#define ILI9341_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include <stdbool.h>

#include "../disp_driver.h"

/*********************
 *      DEFINES
 *********************/
#define ILI9341_DC   CONFIG_DISP_PIN_DC
#define ILI9341_RST  CONFIG_DISP_PIN_RST
#define ILI9341_BCKL CONFIG_DISP_PIN_BCKL

#define ILI9341_ENABLE_BACKLIGHT_CONTROL CONFIG_ENABLE_BACKLIGHT_CONTROL

#if CONFIG_BACKLIGHT_ACTIVE_LVL
  #define ILI9341_BCKL_ACTIVE_LVL 1
#else
  #define ILI9341_BCKL_ACTIVE_LVL 0
#endif

#define ILI9341_INVERT_COLORS CONFIG_INVERT_COLORS

#define  TFT_DRIVER 0x9341

// Delay between some initialisation commands
#define TFT_INIT_DELAY 0x80 // Not used unless commandlist invoked


// Generic commands used by TFT_eSPI.cpp
#define TFT_NOP     0x00
#define TFT_SWRST   0x01

#define TFT_SLPIN   0x10
#define TFT_SLPOUT  0x11

#define TFT_INVOFF  0x20
#define TFT_INVON   0x21

#define TFT_DISPOFF 0x28
#define TFT_DISPON  0x29

#define TFT_CASET   0x2A
#define TFT_PASET   0x2B
#define TFT_RAMWR   0x2C

#define TFT_RAMRD   0x2E

#define TFT_MADCTL  0x36

#define TFT_MAD_MY  0x80
#define TFT_MAD_MX  0x40
#define TFT_MAD_MV  0x20
#define TFT_MAD_ML  0x10
#define TFT_MAD_RGB 0x00
#define TFT_MAD_BGR 0x08
#define TFT_MAD_MH  0x04
#define TFT_MAD_SS  0x02
#define TFT_MAD_GS  0x01

#define TFT_IDXRD   0x00 // ILI9341 only, indexed control register read

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/

void ili9341_init(void);
void ili9341_flush(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, void *color_map);
void ili9341_enable_backlight(bool backlight);
void ili9341_sleep_in(void);
void ili9341_sleep_out(void);

/**********************
 *      MACROS
 **********************/


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*ILI9341_H*/
