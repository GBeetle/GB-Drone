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

#ifndef _TFT_eSPIH_
#define _TFT_eSPIH_

#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "spi_bus.h"
#include "error_handle.h"
#include "io_define.h"
#include "gpio_setting.h"
#include "disp_driver.h"

// Bit level feature flags
// Bit 0 set: viewport capability
#define TFT_ESPI_FEATURES 1

/***************************************************************************************
**                         Section 1: Load required header files
***************************************************************************************/



/***************************************************************************************
**                         Section 2: Load library and processor specific header files
***************************************************************************************/
// Include header file that defines the fonts loaded, the TFT drivers
// available and the pins to be used, etc, etc
#ifdef CONFIG_TFT_eSPI_ESPIDF
  #include "tft_config.h"
#endif

// Handle FLASH based storage e.g. PROGMEM
#undef pgm_read_byte
#define pgm_read_byte(addr)   (*(const unsigned char *)(addr))
#undef pgm_read_word
#define pgm_read_word(addr) ({ \
  typeof(addr) _addr = (addr); \
  *(const unsigned short *)(_addr); \
})
#undef pgm_read_dword
#define pgm_read_dword(addr) ({ \
  typeof(addr) _addr = (addr); \
  *(const unsigned long *)(_addr); \
})

/***************************************************************************************
**                         Section 3: Interface setup
***************************************************************************************/
#ifndef TAB_COLOUR
  #define TAB_COLOUR 0
#endif

// If the SPI frequency is not defined, set a default
#ifndef SPI_FREQUENCY
  #define SPI_FREQUENCY  20000000
#endif

// If the SPI read frequency is not defined, set a default
#ifndef SPI_READ_FREQUENCY
  #define SPI_READ_FREQUENCY 10000000
#endif

// Some ST7789 boards do not work with Mode 0
#ifndef TFT_SPI_MODE
  #if defined(ST7789_DRIVER) || defined(ST7789_2_DRIVER)
    #define TFT_SPI_MODE SPI_MODE3
  #else
    #define TFT_SPI_MODE SPI_MODE0
  #endif
#endif

// If the XPT2046 SPI frequency is not defined, set a default
#ifndef SPI_TOUCH_FREQUENCY
  #define SPI_TOUCH_FREQUENCY  2500000
#endif

#ifndef SPI_BUSY_CHECK
  #define SPI_BUSY_CHECK
#endif

#define DMA_BUSY_CHECK // Not used so leave blank

#define DC_C GB_GPIO_Set(TFT_SPI_DC, false)
#define DC_D GB_GPIO_Set(TFT_SPI_DC, true)

#define CS_L
#define CS_H

#define tft_Read_8(C) \
  CHK_LOGE(tft_dev->bus->readByte(tft_dev->bus, GB_SPI_DEV_0, 0x00, (C)), "TFT readByte failed\n")
#define tft_Write_8(C) \
  CHK_LOGE(tft_dev->bus->writeByte(tft_dev->bus, GB_SPI_DEV_0, 0x00, (C)), "TFT writeByte failed\n")
#define tft_Write_16(C) \
  CHK_LOGE(tft_dev->bus->writeByte(tft_dev->bus, GB_SPI_DEV_0, 0x00, (uint8_t)((C)>>8)), "TFT writeByte failed\n"); \
  CHK_LOGE(tft_dev->bus->writeByte(tft_dev->bus, GB_SPI_DEV_0, 0x00, (uint8_t)((C)>>0)), "TFT writeByte failed\n")
#define tft_Write_16S(C) \
  CHK_LOGE(tft_dev->bus->writeByte(tft_dev->bus, GB_SPI_DEV_0, 0x00, (uint8_t)((C)>>0)), "TFT writeByte failed\n"); \
  CHK_LOGE(tft_dev->bus->writeByte(tft_dev->bus, GB_SPI_DEV_0, 0x00, (uint8_t)((C)>>8)), "TFT writeByte failed\n")

#define tft_Write_32(C) \
  tft_Write_16((uint16_t) ((C)>>16)); \
  tft_Write_16((uint16_t) ((C)>>0))

#define tft_Write_32C(C,D) \
  tft_Write_16((uint16_t) (C)); \
  tft_Write_16((uint16_t) (D))

#define tft_Write_32D(C) \
  tft_Write_16((uint16_t) (C)); \
  tft_Write_16((uint16_t) (C))

/***************************************************************************************
**                         Section 4: Setup fonts
***************************************************************************************/
// Use GLCD font in error case where user requests a smooth font file
// that does not exist (this is a temporary fix to stop ESP32 reboot)
#ifdef SMOOTH_FONT
  #ifndef LOAD_GLCD
    #define LOAD_GLCD
  #endif
#endif

// Only load the fonts defined in User_Setup.h (to save space)
// Set flag so RLE rendering code is optionally compiled
#ifdef LOAD_GLCD
  #include <Fonts/glcdfont.c>
#endif

#ifdef LOAD_FONT2
  #include <Fonts/Font16.h>
#endif

#ifdef LOAD_FONT4
  #include <Fonts/Font32rle.h>
  #define LOAD_RLE
#endif

#ifdef LOAD_FONT6
  #include <Fonts/Font64rle.h>
  #ifndef LOAD_RLE
    #define LOAD_RLE
  #endif
#endif

#ifdef LOAD_FONT7
  #include <Fonts/Font7srle.h>
  #ifndef LOAD_RLE
    #define LOAD_RLE
  #endif
#endif

#ifdef LOAD_FONT8
  #include <Fonts/Font72rle.h>
  #ifndef LOAD_RLE
    #define LOAD_RLE
  #endif
#elif defined LOAD_FONT8N // Optional narrower version
  #define LOAD_FONT8
  #include <Fonts/Font72x53rle.h>
  #ifndef LOAD_RLE
    #define LOAD_RLE
  #endif
#endif

#ifdef LOAD_GFXFF
  // We can include all the free fonts and they will only be built into
  // the sketch if they are used
  #include <Fonts/GFXFF/gfxfont.h>
  // Call up any user custom fonts
  #include <User_Setups/User_Custom_Fonts.h>
#endif // #ifdef LOAD_GFXFF

// This is a structure to conveniently hold information on the default fonts
// Stores pointer to font character image address table, width table and height
typedef struct {
  const uint8_t *chartbl;
  const uint8_t *widthtbl;
  uint8_t height;
  uint8_t baseline;
} fontinfo;

typedef struct spi tft_bus_t;
typedef spi_device_handle_t tft_addr_handle_t;

/***************************************************************************************
**                         Section 5: Font datum enumeration
***************************************************************************************/
//These enumerate the text plotting alignment (reference datum point)
#define TL_DATUM 0 // Top left (default)
#define TC_DATUM 1 // Top centre
#define TR_DATUM 2 // Top right
#define ML_DATUM 3 // Middle left
#define CL_DATUM 3 // Centre left, same as above
#define MC_DATUM 4 // Middle centre
#define CC_DATUM 4 // Centre centre, same as above
#define MR_DATUM 5 // Middle right
#define CR_DATUM 5 // Centre right, same as above
#define BL_DATUM 6 // Bottom left
#define BC_DATUM 7 // Bottom centre
#define BR_DATUM 8 // Bottom right
#define L_BASELINE  9 // Left character baseline (Line the 'A' character would sit on)
#define C_BASELINE 10 // Centre character baseline
#define R_BASELINE 11 // Right character baseline

/***************************************************************************************
**                         Section 6: Colour enumeration
***************************************************************************************/
// Default color definitions
#define TFT_BLACK       0x0000      /*   0,   0,   0 */
#define TFT_NAVY        0x000F      /*   0,   0, 128 */
#define TFT_DARKGREEN   0x03E0      /*   0, 128,   0 */
#define TFT_DARKCYAN    0x03EF      /*   0, 128, 128 */
#define TFT_MAROON      0x7800      /* 128,   0,   0 */
#define TFT_PURPLE      0x780F      /* 128,   0, 128 */
#define TFT_OLIVE       0x7BE0      /* 128, 128,   0 */
#define TFT_LIGHTGREY   0xD69A      /* 211, 211, 211 */
#define TFT_DARKGREY    0x7BEF      /* 128, 128, 128 */
#define TFT_BLUE        0x001F      /*   0,   0, 255 */
#define TFT_GREEN       0x07E0      /*   0, 255,   0 */
#define TFT_CYAN        0x07FF      /*   0, 255, 255 */
#define TFT_RED         0xF800      /* 255,   0,   0 */
#define TFT_MAGENTA     0xF81F      /* 255,   0, 255 */
#define TFT_YELLOW      0xFFE0      /* 255, 255,   0 */
#define TFT_WHITE       0xFFFF      /* 255, 255, 255 */
#define TFT_ORANGE      0xFDA0      /* 255, 180,   0 */
#define TFT_GREENYELLOW 0xB7E0      /* 180, 255,   0 */
#define TFT_PINK        0xFE19      /* 255, 192, 203 */ //Lighter pink, was 0xFC9F
#define TFT_BROWN       0x9A60      /* 150,  75,   0 */
#define TFT_GOLD        0xFEA0      /* 255, 215,   0 */
#define TFT_SILVER      0xC618      /* 192, 192, 192 */
#define TFT_SKYBLUE     0x867D      /* 135, 206, 235 */
#define TFT_VIOLET      0x915C      /* 180,  46, 226 */

// Next is a special 16 bit colour value that encodes to 8 bits
// and will then decode back to the same 16 bit value.
// Convenient for 8 bit and 16 bit transparent sprites.
#define TFT_TRANSPARENT 0x0120 // This is actually a dark green

// Default palette for 4 bit colour sprites
static const uint16_t default_4bit_palette[] = {
  TFT_BLACK,    //  0  ^
  TFT_BROWN,    //  1  |
  TFT_RED,      //  2  |
  TFT_ORANGE,   //  3  |
  TFT_YELLOW,   //  4  Colours 0-9 follow the resistor colour code!
  TFT_GREEN,    //  5  |
  TFT_BLUE,     //  6  |
  TFT_PURPLE,   //  7  |
  TFT_DARKGREY, //  8  |
  TFT_WHITE,    //  9  v
  TFT_CYAN,     // 10  Blue+green mix
  TFT_MAGENTA,  // 11  Blue+red mix
  TFT_MAROON,   // 12  Darker red colour
  TFT_DARKGREEN,// 13  Darker green colour
  TFT_NAVY,     // 14  Darker blue colour
  TFT_PINK      // 15
};

/***************************************************************************************
**                         Section 7: Diagnostic support
***************************************************************************************/
// #define TFT_eSPI_DEBUG     // Switch on debug support serial messages  (not used yet)
// #define TFT_eSPI_FNx_DEBUG // Switch on debug support for function "x" (not used yet)

// This structure allows sketches to retrieve the user setup parameters at runtime
// by calling getSetup(), zero impact on code size unless used, mainly for diagnostics
typedef struct
{
  char  *version;
  char  *setup_info;  // Setup reference name available to use in a user setup

  uint32_t setup_id;   // ID available to use in a user setup
  int32_t esp;         // Processor code
  uint8_t trans;       // SPI transaction support
  uint8_t serial;      // Serial (SPI) or parallel
  uint8_t  port;       // SPI port
  uint8_t overlap;     // ESP8266 overlap mode
  uint8_t interface;   // Interface type

  uint16_t tft_driver; // Hexadecimal code
  uint16_t tft_width;  // Rotation 0 width and height
  uint16_t tft_height;

  uint8_t r0_x_offset; // Display offsets, not all used yet
  uint8_t r0_y_offset;
  uint8_t r1_x_offset;
  uint8_t r1_y_offset;
  uint8_t r2_x_offset;
  uint8_t r2_y_offset;
  uint8_t r3_x_offset;
  uint8_t r3_y_offset;

  int8_t pin_tft_mosi; // SPI pins
  int8_t pin_tft_miso;
  int8_t pin_tft_clk;
  int8_t pin_tft_cs;

  int8_t pin_tft_dc;   // Control pins
  int8_t pin_tft_rd;
  int8_t pin_tft_wr;
  int8_t pin_tft_rst;

  int8_t pin_tft_d0;   // Parallel port pins
  int8_t pin_tft_d1;
  int8_t pin_tft_d2;
  int8_t pin_tft_d3;
  int8_t pin_tft_d4;
  int8_t pin_tft_d5;
  int8_t pin_tft_d6;
  int8_t pin_tft_d7;

  int8_t pin_tft_led;
  int8_t pin_tft_led_on;

  int8_t pin_tch_cs;   // Touch chip select pin

  int16_t tft_spi_freq;// TFT write SPI frequency
  int16_t tft_rd_freq; // TFT read  SPI frequency
  int16_t tch_spi_freq;// Touch controller read/write SPI frequency
} setup_t;

/***************************************************************************************
**                         Section 8: Class member and support functions
***************************************************************************************/
// Swap any type
//template <typename T> static inline void
//swap_coord(T& a, T& b) { T t = a; a = b; b = t; }

// Callback prototype for smooth font pixel colour read
typedef uint16_t (*getColorCallback)(uint16_t x, uint16_t y);

// Class functions and variables
struct TFT_eSPI {
  tft_bus_t *bus;            //!< SPI device descriptor

  uint8_t  spiBusyCheck;      // Number of ESP32 transfer buffers to check

  uint32_t textcolor, textbgcolor;         // Text foreground and background colours

  uint32_t bitmap_fg, bitmap_bg;           // Bitmap foreground (bit=1) and background (bit=0) colours

  uint8_t  textfont,  // Current selected font number
           textsize,  // Current font size multiplier
           textdatum, // Text reference datum
           rotation;  // Display rotation (0-3)

  uint8_t  decoderState;   // UTF8 decoder state        - not for user access
  uint16_t decoderBuffer;      // Unicode code-point buffer - not for user access

           // Display variant settings
  uint8_t  tabcolor;                   // ST7735 screen protector "tab" colour (now invalid)
  uint8_t  colstart;
  uint8_t  rowstart; // Screen display area to CGRAM area coordinate offsets

           // Port and pin masks for control signals (ESP826 only) - TODO: remove need for this
  volatile uint32_t *dcport, *csport;
  uint32_t cspinmask, dcpinmask, wrpinmask, sclkpinmask;

#if defined(ESP32_PARALLEL)
           // Bit masks for ESP32 parallel bus interface
  uint32_t xclr_mask;
  uint32_t xdir_mask; // Port set/clear and direction control masks

           // Lookup table for ESP32 parallel bus interface uses 1kbyte RAM,
  uint32_t xset_mask[256]; // Makes Sprite rendering test 33% faster, for slower macro equivalent
                           // see commented out #define set_mask(C) within TFT_eSPI_ESP32.h
#endif

  //uint32_t lastColor = 0xFFFF; // Last colour - used to minimise bit shifting overhead

  getColorCallback getColor; // Smooth font callback function pointer

  bool     locked, inTransaction, lockTransaction; // SPI transaction and mutex lock flags

 //-------------------------------------- protected ----------------------------------//

  //int32_t  win_xe, win_ye;          // Window end coords - not needed

  int32_t  _init_width, _init_height; // Display w/h as input, used by setRotation()
  int32_t  _width, _height;           // Display w/h as modified by current rotation
  int32_t  addr_row, addr_col;        // Window position - used to minimise window commands

  int16_t  _xPivot;   // TFT x pivot point coordinate for rotated Sprites
  int16_t  _yPivot;   // TFT x pivot point coordinate for rotated Sprites

  // Viewport variables
  int32_t  _vpX, _vpY, _vpW, _vpH;    // Note: x start, y start, x end + 1, y end + 1
  int32_t  _xDatum;
  int32_t  _yDatum;
  int32_t  _xWidth;
  int32_t  _yHeight;
  bool     _vpDatum;
  bool     _vpOoB;

  int32_t  cursor_x, cursor_y, padX;       // Text cursor x,y and padding setting
  int32_t  bg_cursor_x;                    // Background fill cursor
  int32_t  last_cursor_x;                  // Previous text cursor position when fill used

  uint32_t fontsloaded;               // Bit field of fonts loaded

  uint8_t  glyph_ab,   // Smooth font glyph delta Y (height) above baseline
           glyph_bb;   // Smooth font glyph delta Y (height) below baseline

  bool     isDigits;   // adjust bounding box for numbers to reduce visual jiggling
  bool     textwrapX, textwrapY;  // If set, 'wrap' text at right and optionally bottom edge of display
  bool     _swapBytes; // Swap the byte order for TFT pushImage()

  bool     _booted;    // init() or begin() has already run once

                       // User sketch manages these via set/getAttribute()
  bool     _cp437;        // If set, use correct CP437 charset (default is ON)
  bool     _utf8;         // If set, use UTF-8 decoder in print stream 'write()' function (default ON)
  bool     _psram_enable; // Enable PSRAM use for library functions (TBD) and Sprites

  uint32_t _lastColor; // Buffered value of last colour used

  bool     _fillbg;    // Fill background flag (just for for smooth fonts at the moment)

#if defined (SSD1963_DRIVER)
  uint16_t Cswap;      // Swap buffer for SSD1963
  uint8_t r6, g6, b6;  // RGB buffer for SSD1963
#endif

#ifdef LOAD_GFXFF
  GFXfont  *gfxFont;
#endif

  // These are virtual so the TFT_eSprite class can override them with sprite specific functions
  void     (*drawPixel)(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, uint32_t color);
  void     (*drawChar)(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, uint16_t c, uint32_t color, uint32_t bg, uint8_t size);
  void     (*drawLine)(struct TFT_eSPI * tft_dev, int32_t xs, int32_t ys, int32_t xe, int32_t ye, uint32_t color);
  void     (*drawFastVLine)(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t h, uint32_t color);
  void     (*drawFastHLine)(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, uint32_t color);
  void     (*fillRect)(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, uint32_t color);

  int16_t  (*drawCharUniFont)(struct TFT_eSPI * tft_dev, uint16_t uniCode, int32_t x, int32_t y, uint8_t font);
  int16_t  (*drawCharUni)(struct TFT_eSPI * tft_dev, uint16_t uniCode, int32_t x, int32_t y);
  int16_t  (*height)(struct TFT_eSPI * tft_dev);
  int16_t  (*width)(struct TFT_eSPI * tft_dev);

                   // Read the colour of a pixel at x,y and return value in 565 format
  uint16_t (*readPixel)(struct TFT_eSPI * tft_dev, int32_t x, int32_t y);

  void     (*setWindow)(struct TFT_eSPI * tft_dev, int32_t xs, int32_t ys, int32_t xe, int32_t ye);   // Note: start + end coordinates

                   // Push (aka write pixel) colours to the set window
  void     (*pushColor)(struct TFT_eSPI * tft_dev, uint16_t color);

  void     (*setRotation)(struct TFT_eSPI * tft_dev, uint8_t r); // Set the display image orientation to 0, 1, 2 or 3
  uint8_t  (*getRotation)(struct TFT_eSPI * tft_dev);      // Read the current rotation

  void     (*invertDisplay)(struct TFT_eSPI * tft_dev, bool i);  // Tell TFT to invert all displayed colours


  // The TFT_eSprite class inherits the following functions (not all are useful to Sprite class
  void     (*setAddrWindow)(struct TFT_eSPI * tft_dev, int32_t xs, int32_t ys, int32_t w, int32_t h); // Note: start coordinates + width and height

  // Viewport commands, see "Viewport_Demo" sketch
  void     (*setViewport)(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, bool vpDatum);  //vpDatum default = true
  bool     (*checkViewport)(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h);
  int32_t  (*getViewportX)(struct TFT_eSPI * tft_dev);
  int32_t  (*getViewportY)(struct TFT_eSPI * tft_dev);
  int32_t  (*getViewportWidth)(struct TFT_eSPI * tft_dev);
  int32_t  (*getViewportHeight)(struct TFT_eSPI * tft_dev);
  bool     (*getViewportDatum)(struct TFT_eSPI * tft_dev);
  void     (*frameViewport)(struct TFT_eSPI * tft_dev, uint16_t color, int32_t w);
  void     (*resetViewport)(struct TFT_eSPI * tft_dev);

           // Clip input window to viewport bounds, return false if whole area is out of bounds
  bool     (*clipAddrWindow)(struct TFT_eSPI * tft_dev, int32_t* x, int32_t* y, int32_t* w, int32_t* h);
           // Clip input window area to viewport bounds, return false if whole area is out of bounds
  bool     (*clipWindow)(struct TFT_eSPI * tft_dev, int32_t* xs, int32_t* ys, int32_t* xe, int32_t* ye);

           // Push (aka write pixel) colours to the TFT (use setAddrWindow() first)
  void     (*pushColorLen)(struct TFT_eSPI * tft_dev, uint16_t color, uint32_t len);  // Deprecated, use pushBlock()
  void     (*pushColorsSwap)(struct TFT_eSPI * tft_dev, uint16_t  *data, uint32_t len, bool swap); // With byte swap option: true
  void     (*pushColors)(struct TFT_eSPI * tft_dev, uint8_t  *data, uint32_t len); // Deprecated, use pushPixels()

           // Write a solid block of a single colour
  void     (*pushBlock)(struct TFT_eSPI * tft_dev, uint16_t color, uint32_t len);

           // Write a set of pixels stored in memory, use setSwapBytes(true/false) function to correct endianess
  void     (*pushPixels)(struct TFT_eSPI * tft_dev, const void * data_in, uint32_t len);

           // Support for half duplex (bi-directional SDA) SPI bus where MOSI must be switched to input
#ifdef TFT_SDA_READ
#if defined (TFT_eSPI_ENABLE_8_BIT_READ)
  uint8_t  (*tft_Read_8)(struct TFT_eSPI * tft_dev);     // Read 8 bit value from TFT command register
#endif
  void     (*begin_SDA_Read)(struct TFT_eSPI * tft_dev); // Begin a read on a half duplex (bi-directional SDA) SPI bus - sets MOSI to input
  void     (*end_SDA_Read)(struct TFT_eSPI * tft_dev);   // Restore MOSI to output
#endif

  // Graphics drawing
  void     (*fillScreen)(struct TFT_eSPI * tft_dev, uint32_t color);
  void     (*drawRect)(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, uint32_t color);
  void     (*drawRoundRect)(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, int32_t radius, uint32_t color);
  void     (*fillRoundRect)(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, int32_t radius, uint32_t color);

  void     (*fillRectVGradient)(struct TFT_eSPI * tft_dev, int16_t x, int16_t y, int16_t w, int16_t h, uint32_t color1, uint32_t color2);
  void     (*fillRectHGradient)(struct TFT_eSPI * tft_dev, int16_t x, int16_t y, int16_t w, int16_t h, uint32_t color1, uint32_t color2);

           // Draw a pixel blended with the pixel colour on the TFT or sprite, return blended colour
           // If bg_color is not included the background pixel colour will be read from TFT or sprite
  uint16_t (*drawPixelAlpha)(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, uint32_t color, uint8_t alpha, uint32_t bg_color); // bg_color default = 0x00FFFFFF

           // Draw a small anti-aliased filled circle at ax,ay with radius r (uses drawWideLine)
           // If bg_color is not included the background pixel colour will be read from TFT or sprite
  void     (*drawSpot)(struct TFT_eSPI * tft_dev, float ax, float ay, float r, uint32_t fg_color, uint32_t bg_color); // bg_color default = 0x00FFFFFF

           // Draw an anti-aliased filled circle at x, y with radius r
           // If bg_color is not included the background pixel colour will be read from TFT or sprite
  void     (*fillSmoothCircle)(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t r, uint32_t color, uint32_t bg_color); // bg_color default = 0x00FFFFFF

  void     (*fillSmoothRoundRect)(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, int32_t radius, uint32_t color, uint32_t bg_color); // bg_color default = 0x00FFFFFF

           // Draw an anti-aliased wide line from ax,ay to bx,by width wd with radiused ends (radius is wd/2)
           // If bg_color is not included the background pixel colour will be read from TFT or sprite
  void     (*drawWideLine)(struct TFT_eSPI * tft_dev, float ax, float ay, float bx, float by, float wd, uint32_t fg_color, uint32_t bg_color); // bg_color default = 0x00FFFFFF

           // Draw an anti-aliased wide line from ax,ay to bx,by with different width at each end aw, bw and with radiused ends
           // If bg_color is not included the background pixel colour will be read from TFT or sprite
  void     (*drawWedgeLine)(struct TFT_eSPI * tft_dev, float ax, float ay, float bx, float by, float aw, float bw, uint32_t fg_color, uint32_t bg_color); // bg_color default = 0x00FFFFFF

  void     (*drawCircle)(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t r, uint32_t color);
  void     (*drawCircleHelper)(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t r, uint8_t cornername, uint32_t color);
  void     (*fillCircle)(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t r, uint32_t color);
  void     (*fillCircleHelper)(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t r, uint8_t cornername, int32_t delta, uint32_t color);

  void     (*drawEllipse)(struct TFT_eSPI * tft_dev, int16_t x, int16_t y, int32_t rx, int32_t ry, uint16_t color);
  void     (*fillEllipse)(struct TFT_eSPI * tft_dev, int16_t x, int16_t y, int32_t rx, int32_t ry, uint16_t color);

           //                 Corner 1               Corner 2               Corner 3
  void     (*drawTriangle)(struct TFT_eSPI * tft_dev, int32_t x1,int32_t y1, int32_t x2,int32_t y2, int32_t x3,int32_t y3, uint32_t color);
  void     (*fillTriangle)(struct TFT_eSPI * tft_dev, int32_t x1,int32_t y1, int32_t x2,int32_t y2, int32_t x3,int32_t y3, uint32_t color);

  // Image rendering
    // Swap the byte order for pushImage() and pushPixels() - corrects endianness
  void     (*setSwapBytes)(struct TFT_eSPI * tft_dev, bool swap);
  bool     (*getSwapBytes)(struct TFT_eSPI * tft_dev);

           // Draw bitmap
  void     (*drawBitmap)(struct TFT_eSPI * tft_dev, int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t fgcolor);
  void     (*drawBitmapBg)(struct TFT_eSPI * tft_dev, int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t fgcolor, uint16_t bgcolor);
  void     (*drawXBitmap)(struct TFT_eSPI * tft_dev, int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t fgcolor);
  void     (*drawXBitmapBg)(struct TFT_eSPI * tft_dev, int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t fgcolor, uint16_t bgcolor);
  void     (*setBitmapColor)(struct TFT_eSPI * tft_dev, uint16_t fgcolor, uint16_t bgcolor); // Define the 2 colours for 1bpp sprites

           // Set TFT pivot point (use when rendering rotated sprites)
  void     (*setPivot)(struct TFT_eSPI * tft_dev, int16_t x, int16_t y);
  int16_t  (*getPivotX)(struct TFT_eSPI * tft_dev); // Get pivot x
  int16_t  (*getPivotY)(struct TFT_eSPI * tft_dev); // Get pivot y

           // The next functions can be used as a pair to copy screen blocks (or horizontal/vertical lines) to another location
           // Read a block of pixels to a data buffer, buffer is 16 bit and the size must be at least w * h
  void     (*readRect)(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, uint16_t *data);
           // Write a block of pixels to the screen which have been read by readRect()
  void     (*pushRect)(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, uint16_t *data);

           // These are used to render images or sprites stored in RAM arrays (used by Sprite class for 16bpp Sprites)
  void     (*pushImage)(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, uint16_t *data);
  void     (*pushImageTrans)(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, uint16_t *data, uint16_t transparent);

           // These are used to render images stored in FLASH (PROGMEM)
  void     (*pushImageFlashTrans)(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, const uint16_t *data, uint16_t transparent);
  void     (*pushImageFlash)(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, const uint16_t *data);

           // These are used by Sprite class pushSprite() member function for 1, 4 and 8 bits per pixel (bpp) colours
           // They are not intended to be used with user sketches (but could be)
           // Set bpp8 true for 8bpp sprites, false otherwise. The cmap pointer must be specified for 4bpp
  void     (*pushImageBpp8)(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, uint8_t  *data, bool bpp8, uint16_t *cmap);  // bpp8=true, cmap = nullptr
  void     (*pushImageBpp8Trans)(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, uint8_t  *data, uint8_t  transparent, bool bpp8, uint16_t *cmap); // cmap = nullptr
           // FLASH version
  void     (*pushImageBpp8Flash)(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, const uint8_t *data, bool bpp8,  uint16_t *cmap);  // cmap = nullptr
           // This next function has been used successfully to dump the TFT screen to a PC for documentation purposes
           // It reads a screen area and returns the 3 RGB 8 bit colour values of each pixel in the buffer
           // Set w and h to 1 to read 1 pixel's colour. The data buffer must be at least w * h * 3 bytes
  void     (*readRectRGB)(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, uint8_t *data);

  // Text rendering - value returned is the pixel width of the rendered text
  int16_t  (*drawNumberFont)(struct TFT_eSPI * tft_dev, long intNumber, int32_t x, int32_t y, uint8_t font); // Draw integer using specified font number
  int16_t  (*drawNumber)(struct TFT_eSPI * tft_dev, long intNumber, int32_t x, int32_t y);               // Draw integer using current font

           // Decimal is the number of decimal places to render
           // Use with setTextDatum() to position values on TFT, and setTextPadding() to blank old displayed values
  int16_t  (*drawFloatFont)(struct TFT_eSPI * tft_dev, float floatNumber, uint8_t decimal, int32_t x, int32_t y, uint8_t font); // Draw float using specified font number
  int16_t  (*drawFloat)(struct TFT_eSPI * tft_dev, float floatNumber, uint8_t decimal, int32_t x, int32_t y);               // Draw float using current font

           // Handle char arrays
           // Use with setTextDatum() to position string on TFT, and setTextPadding() to blank old displayed strings
  int16_t  (*drawStringFont)(struct TFT_eSPI * tft_dev, const char *string, int32_t x, int32_t y, uint8_t font);  // Draw string using specified font number
  int16_t  (*drawString)(struct TFT_eSPI * tft_dev, const char *string, int32_t x, int32_t y);                // Draw string using current font
  //int16_t  (*drawString)(const String& string, int32_t x, int32_t y, uint8_t font);// Draw string using specified font number
  //int16_t  (*drawString)(const String& string, int32_t x, int32_t y);              // Draw string using current font

  int16_t  (*drawCentreString)(struct TFT_eSPI * tft_dev, const char *string, int32_t x, int32_t y, uint8_t font);  // Deprecated, use setTextDatum() and drawString()
  int16_t  (*drawRightString)(struct TFT_eSPI * tft_dev, const char *string, int32_t x, int32_t y, uint8_t font);   // Deprecated, use setTextDatum() and drawString()
  //int16_t  (*drawCentreString)(const String& string, int32_t x, int32_t y, uint8_t font);// Deprecated, use setTextDatum() and drawString()
  //int16_t  (*drawRightString)(const String& string, int32_t x, int32_t y, uint8_t font); // Deprecated, use setTextDatum() and drawString()

  // Text rendering and font handling support funtions
  void     (*setCursor)(struct TFT_eSPI * tft_dev, int16_t x, int16_t y);                 // Set cursor for tft.print()
  void     (*setCursorFont)(struct TFT_eSPI * tft_dev, int16_t x, int16_t y, uint8_t font);   // Set cursor and font number for tft.print()

  int16_t  (*getCursorX)(struct TFT_eSPI * tft_dev);                                // Read current cursor x position (moves with tft.print())
  int16_t  (*getCursorY)(struct TFT_eSPI * tft_dev);                                // Read current cursor y position

  void     (*setTextColor)(struct TFT_eSPI * tft_dev, uint16_t color);                    // Set character (glyph) color only (background not over-written)
  void     (*setTextColorFill)(struct TFT_eSPI * tft_dev, uint16_t fgcolor, uint16_t bgcolor, bool bgfill);  // Set character (glyph) foreground and background colour, optional background fill for smooth fonts, bgfill=false
  void     (*setTextSize)(struct TFT_eSPI * tft_dev, uint8_t size);                       // Set character size multiplier (this increases pixel size)

  void     (*setTextWrap)(struct TFT_eSPI * tft_dev, bool wrapX, bool wrapY);     // Turn on/off wrapping of text in TFT width and/or height  wrapY=false

  void     (*setTextDatum)(struct TFT_eSPI * tft_dev, uint8_t datum);                     // Set text datum position (default is top left), see Section 6 above
  uint8_t  (*getTextDatum)(struct TFT_eSPI * tft_dev);

  void     (*setTextPadding)(struct TFT_eSPI * tft_dev, uint16_t x_width);                // Set text padding (background blanking/over-write) width in pixels
  uint16_t (*getTextPadding)(struct TFT_eSPI * tft_dev);                            // Get text padding

#ifdef LOAD_GFXFF
  void     (*setFreeFont)(struct TFT_eSPI * tft_dev, const GFXfont *f);                 // Select the GFX Free Font f = null
  void     (*setTextFont)(struct TFT_eSPI * tft_dev, uint8_t font);                       // Set the font number to use in future
#else
  void     (*setFreeFont)(struct TFT_eSPI * tft_dev, uint8_t font);                       // Not used, historical fix to prevent an error
  void     (*setTextFont)(struct TFT_eSPI * tft_dev, uint8_t font);                       // Set the font number to use in future
#endif

  int16_t  (*textWidthFont)(struct TFT_eSPI * tft_dev, const char *string, uint8_t font);     // Returns pixel width of string in specified font
  int16_t  (*textWidth)(struct TFT_eSPI * tft_dev, const char *string);                   // Returns pixel width of string in current font
           //textWidth(const String& string, uint8_t font),   // As above for String types
           //textWidth(const String& string),
  int16_t  (*fontHeight)(struct TFT_eSPI * tft_dev, int16_t font);                        // Returns pixel height of string in specified font

           // Used by library and Smooth font class to extract Unicode point codes from a UTF8 encoded string
  uint16_t (*decodeUTF8)(struct TFT_eSPI * tft_dev, uint8_t *buf, uint16_t *index, uint16_t remaining);
  uint16_t (*decodeUTF8C)(struct TFT_eSPI * tft_dev, uint8_t c);

           // Support function to UTF8 decode and draw characters piped through print stream
  size_t   (*write)(struct TFT_eSPI * tft_dev, uint8_t c);
  // size_t   write(const uint8_t *buf, size_t len);

           // Used by Smooth font class to fetch a pixel colour for the anti-aliasing
  void     (*setCallback)(struct TFT_eSPI * tft_dev, getColorCallback getCol);

  uint16_t (*fontsLoaded)(struct TFT_eSPI * tft_dev); // Each bit in returned value represents a font type that is loaded - used for debug/error handling only

#ifndef RM68120_DRIVER
  void     (*writecommand)(struct TFT_eSPI * tft_dev, uint8_t c);  // Send a command, function resets DC/RS high ready for data
#else
  void     (*writecommand)(struct TFT_eSPI * tft_dev, uint16_t c); // Send a command, function resets DC/RS high ready for data
  void     (*writeRegister)(struct TFT_eSPI * tft_dev, uint16_t c, uint8_t d); // Write data to 16 bit command register
#endif
  void     (*writedata)(struct TFT_eSPI * tft_dev, uint8_t d);     // Send data with DC/RS set high

  void     (*commandList)(struct TFT_eSPI * tft_dev, const uint8_t *addr); // Send a initialisation sequence to TFT stored in FLASH

  uint8_t  (*readcommand8)(struct TFT_eSPI * tft_dev, uint8_t cmd_function, uint8_t index); // read 8 bits from TFT
  uint16_t (*readcommand16)(struct TFT_eSPI * tft_dev, uint8_t cmd_function, uint8_t index); // read 16 bits from TFT
  uint32_t (*readcommand32)(struct TFT_eSPI * tft_dev, uint8_t cmd_function, uint8_t index); // read 32 bits from TFT


  // Colour conversion
           // Convert 8 bit red, green and blue to 16 bits
  uint16_t (*color565)(struct TFT_eSPI * tft_dev, uint8_t red, uint8_t green, uint8_t blue);

           // Convert 8 bit colour to 16 bits
  uint16_t (*color8to16)(struct TFT_eSPI * tft_dev, uint8_t color332);
           // Convert 16 bit colour to 8 bits
  uint8_t  (*color16to8)(struct TFT_eSPI * tft_dev, uint16_t color565);

           // Convert 16 bit colour to/from 24 bit, R+G+B concatenated into LS 24 bits
  uint32_t (*color16to24)(struct TFT_eSPI * tft_dev, uint16_t color565);
  uint32_t (*color24to16)(struct TFT_eSPI * tft_dev, uint32_t color888);

           // Alpha blend 2 colours, see generic "alphaBlend_Test" example
           // alpha =   0 = 100% background colour
           // alpha = 255 = 100% foreground colour
  uint16_t (*alphaBlend)(struct TFT_eSPI * tft_dev, uint8_t alpha, uint16_t fgc, uint16_t bgc);
           // 16 bit colour alphaBlend with alpha dither (dither reduces colour banding)
  uint16_t (*alphaBlendDither)(struct TFT_eSPI * tft_dev, uint8_t alpha, uint16_t fgc, uint16_t bgc, uint8_t dither);
           // 24 bit colour alphaBlend with optional alpha dither
  uint32_t (*alphaBlend24)(struct TFT_eSPI * tft_dev, uint8_t alpha, uint32_t fgc, uint32_t bgc, uint8_t dither); //dither = 0

  // Bare metal functions
  void     (*startWrite)(struct TFT_eSPI * tft_dev);                         // Begin SPI transaction
  void     (*writeColor)(struct TFT_eSPI * tft_dev, uint16_t color, uint32_t len); // Deprecated, use pushBlock()
  void     (*endWrite)(struct TFT_eSPI * tft_dev);                           // End SPI transaction

  // Set/get an arbitrary library configuration attribute or option
  //       Use to switch ON/OFF capabilities such as UTF8 decoding - each attribute has a unique ID
  //       id = 0: reserved - may be used in future to reset all attributes to a default state
  //       id = 1: Turn on (a=true) or off (a=false) GLCD cp437 font character error correction
  //       id = 2: Turn on (a=true) or off (a=false) UTF8 decoding
  //       id = 3: Enable or disable use of ESP32 PSRAM (if available)
#define CP437_SWITCH 1
#define UTF8_SWITCH  2
#define PSRAM_ENABLE 3
  void     (*setAttribute)(struct TFT_eSPI * tft_dev, uint8_t id, uint8_t a); // Set attribute value
  uint8_t  (*getAttribute)(struct TFT_eSPI * tft_dev, uint8_t id);                // Get attribute value

           // Used for diagnostic sketch to see library setup adopted by compiler, see Section 7 above
  void     (*getSetup)(struct TFT_eSPI * tft_dev, setup_t *tft_settings); // Sketch provides the instance to populate
  bool     (*verifySetupID)(struct TFT_eSPI * tft_dev, uint32_t id);

  // Global variables
  //static   SPIClass& (*getSPIinstance)(void); // Get SPI class handle

           // New begin and end prototypes
           // begin/end a TFT write transaction
           // For SPI bus the transmit clock rate is set
  void     (*begin_tft_write)(struct TFT_eSPI * tft_dev);
  void     (*end_tft_write)(struct TFT_eSPI * tft_dev);

           // begin/end a TFT read transaction
           // For SPI bus: begin lowers SPI clock rate, end reinstates transmit clock rate
  void     (*begin_tft_read)(struct TFT_eSPI * tft_dev);
  void     (*end_tft_read)(struct TFT_eSPI * tft_dev);

           // Temporary  library development function  TODO: remove need for this
  void     (*pushSwapBytePixels)(struct TFT_eSPI * tft_dev, const void* data_in, uint32_t len);

           // Same as setAddrWindow but exits with CGRAM in read mode
  void     (*readAddrWindow)(struct TFT_eSPI * tft_dev, int32_t xs, int32_t ys, int32_t w, int32_t h);

           // Byte read prototype
  uint8_t  (*readByte)(struct TFT_eSPI * tft_dev);

           // GPIO parallel bus input/output direction control
  void     (*busDir)(struct TFT_eSPI * tft_dev, uint32_t mask, uint8_t mode);

           // Single GPIO input/output direction control
  void     (*gpioMode)(struct TFT_eSPI * tft_dev, uint8_t gpio, uint8_t mode);

           // Helper function: calculate distance of a point from a finite length line between two points
  float    (*wedgeLineDistance)(struct TFT_eSPI * tft_dev, float pax, float pay, float bax, float bay, float dr);
}; // End of class TFT_eSPI


void  TFT_eSpi_init(struct TFT_eSPI * tft_dev, int16_t w, int16_t h, uint8_t tc);  // default TAB_COLOUR

#endif // ends #ifndef _TFT_eSPIH_
