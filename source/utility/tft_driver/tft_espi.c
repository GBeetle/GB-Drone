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

#include "tft_espi.h"
#include "disp_driver.h"

static void    drawPixel(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, uint32_t color);
static void    drawChar(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, uint16_t c, uint32_t color, uint32_t bg, uint8_t size);
static void    drawLine(struct TFT_eSPI * tft_dev, int32_t xs, int32_t ys, int32_t xe, int32_t ye, uint32_t color);
static void    drawFastVLine(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t h, uint32_t color);
static void    drawFastHLine(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, uint32_t color);
static void    fillRect(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, uint32_t color);

static int16_t  drawCharUniFont(struct TFT_eSPI * tft_dev, uint16_t uniCode, int32_t x, int32_t y, uint8_t font);
static int16_t  drawCharUni(struct TFT_eSPI * tft_dev, uint16_t uniCode, int32_t x, int32_t y);
static int16_t  height(struct TFT_eSPI * tft_dev);
static int16_t  width(struct TFT_eSPI * tft_dev);
static uint16_t readPixel(struct TFT_eSPI * tft_dev, int32_t x, int32_t y);

static void    setWindow(struct TFT_eSPI * tft_dev, int32_t xs, int32_t ys, int32_t xe, int32_t ye);   // Note: start + end coordinates
static void    pushColor(struct TFT_eSPI * tft_dev, uint16_t color);

static void     setRotation(struct TFT_eSPI * tft_dev, uint8_t r); // Set the display image orientation to 0, 1, 2 or 3
static uint8_t  getRotation(struct TFT_eSPI * tft_dev);      // Read the current rotation

static void    invertDisplay(struct TFT_eSPI * tft_dev, bool i);  // Tell TFT to invert all displayed colours
static void    setAddrWindow(struct TFT_eSPI * tft_dev, int32_t xs, int32_t ys, int32_t w, int32_t h); // Note: start coordinates + width and height
static void    setViewport(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, bool vpDatum);  //vpDatum default = true
static bool    checkViewport(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h);
static int32_t  getViewportX(struct TFT_eSPI * tft_dev);
static int32_t  getViewportY(struct TFT_eSPI * tft_dev);
static int32_t  getViewportWidth(struct TFT_eSPI * tft_dev);
static int32_t  getViewportHeight(struct TFT_eSPI * tft_dev);
static bool    getViewportDatum(struct TFT_eSPI * tft_dev);
static void    frameViewport(struct TFT_eSPI * tft_dev, uint16_t color, int32_t w);
static void    resetViewport(struct TFT_eSPI * tft_dev);
static bool    clipAddrWindow(struct TFT_eSPI * tft_dev, int32_t* x, int32_t* y, int32_t* w, int32_t* h);
static bool    clipWindow(struct TFT_eSPI * tft_dev, int32_t* xs, int32_t* ys, int32_t* xe, int32_t* ye);
static void    pushColorLen(struct TFT_eSPI * tft_dev, uint16_t color, uint32_t len);  // Deprecated, use pushBlock()
static void    pushColorsSwap(struct TFT_eSPI * tft_dev, uint16_t *data, uint32_t len, bool swap); // With byte swap option: true
static void    pushColors(struct TFT_eSPI * tft_dev, uint8_t *data, uint32_t len); // Deprecated, use pushPixels()
static void    fillScreen(struct TFT_eSPI * tft_dev, uint32_t color);
static void    drawRect(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, uint32_t color);
static void    drawRoundRect(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, int32_t radius, uint32_t color);
static void    fillRoundRect(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, int32_t radius, uint32_t color);
static void    fillRectVGradient(struct TFT_eSPI * tft_dev, int16_t x, int16_t y, int16_t w, int16_t h, uint32_t color1, uint32_t color2);
static void    fillRectHGradient(struct TFT_eSPI * tft_dev, int16_t x, int16_t y, int16_t w, int16_t h, uint32_t color1, uint32_t color2);
static uint16_t drawPixelAlpha(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, uint32_t color, uint8_t alpha, uint32_t bg_color); // bg_color default = 0x00FFFFFF
static void    drawSpot(struct TFT_eSPI * tft_dev, float ax, float ay, float r, uint32_t fg_color, uint32_t bg_color); // bg_color default = 0x00FFFFFF
static void    fillSmoothCircle(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t r, uint32_t color, uint32_t bg_color); // bg_color default = 0x00FFFFFF
static void    fillSmoothRoundRect(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, int32_t radius, uint32_t color, uint32_t bg_color); // bg_color default = 0x00FFFFFF
static void    drawWideLine(struct TFT_eSPI * tft_dev, float ax, float ay, float bx, float by, float wd, uint32_t fg_color, uint32_t bg_color); // bg_color default = 0x00FFFFFF
static void    drawWedgeLine(struct TFT_eSPI * tft_dev, float ax, float ay, float bx, float by, float aw, float bw, uint32_t fg_color, uint32_t bg_color); // bg_color default = 0x00FFFFFF
static void    drawCircle(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t r, uint32_t color);
static void    drawCircleHelper(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t r, uint8_t cornername, uint32_t color);
static void    fillCircle(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t r, uint32_t color);
static void    fillCircleHelper(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t r, uint8_t cornername, int32_t delta, uint32_t color);
static void    drawEllipse(struct TFT_eSPI * tft_dev, int16_t x, int16_t y, int32_t rx, int32_t ry, uint16_t color);
static void    fillEllipse(struct TFT_eSPI * tft_dev, int16_t x, int16_t y, int32_t rx, int32_t ry, uint16_t color);
static void    drawTriangle(struct TFT_eSPI * tft_dev, int32_t x1,int32_t y1, int32_t x2,int32_t y2, int32_t x3,int32_t y3, uint32_t color);
static void    fillTriangle(struct TFT_eSPI * tft_dev, int32_t x1,int32_t y1, int32_t x2,int32_t y2, int32_t x3,int32_t y3, uint32_t color);
static void    setSwapBytes(struct TFT_eSPI * tft_dev, bool swap);
static bool    getSwapBytes(struct TFT_eSPI * tft_dev);
static void    drawBitmap(struct TFT_eSPI * tft_dev, int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t fgcolor);
static void    drawBitmapBg(struct TFT_eSPI * tft_dev, int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t fgcolor, uint16_t bgcolor);
static void    drawXBitmap(struct TFT_eSPI * tft_dev, int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t fgcolor);
static void    drawXBitmapBg(struct TFT_eSPI * tft_dev, int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t fgcolor, uint16_t bgcolor);
static void    setBitmapColor(struct TFT_eSPI * tft_dev, uint16_t fgcolor, uint16_t bgcolor); // Define the 2 colours for 1bpp sprites
static void    setPivot(struct TFT_eSPI * tft_dev, int16_t x, int16_t y);
static int16_t  getPivotX(struct TFT_eSPI * tft_dev); // Get pivot x
static int16_t  getPivotY(struct TFT_eSPI * tft_dev); // Get pivot y
static void    readRect(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, uint16_t *data);
static void    pushRect(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, uint16_t *data);
static void    pushImage(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, uint16_t *data);
static void    pushImageTrans(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, uint16_t *data, uint16_t transparent);
static void    pushImageFlashTrans(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, const uint16_t *data, uint16_t transparent);
static void    pushImageFlash(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, const uint16_t *data);
static void    pushImageBpp8(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, uint8_t *data, bool bpp8, uint16_t *cmap);  // bpp8=true, cmap = nullptr
static void    pushImageBpp8Trans(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, uint8_t *data, uint8_t transparent, bool bpp8, uint16_t *cmap); // cmap = nullptr
static void    pushImageBpp8Flash(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, const uint8_t *data, bool bpp8,  uint16_t *cmap);  // cmap = nullptr
static void    readRectRGB(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, uint8_t *data);
static int16_t  drawNumberFont(struct TFT_eSPI * tft_dev, long intNumber, int32_t x, int32_t y, uint8_t font); // Draw integer using specified font number
static int16_t  drawNumber(struct TFT_eSPI * tft_dev, long intNumber, int32_t x, int32_t y);               // Draw integer using current font
static int16_t  drawFloatFont(struct TFT_eSPI * tft_dev, float floatNumber, uint8_t decimal, int32_t x, int32_t y, uint8_t font); // Draw float using specified font number
static int16_t  drawFloat(struct TFT_eSPI * tft_dev, float floatNumber, uint8_t decimal, int32_t x, int32_t y);               // Draw float using current font
static int16_t  drawStringFont(struct TFT_eSPI * tft_dev, const char *string, int32_t x, int32_t y, uint8_t font);  // Draw string using specified font number
static int16_t  drawString(struct TFT_eSPI * tft_dev, const char *string, int32_t x, int32_t y);                // Draw string using current font
static int16_t  drawCentreString(struct TFT_eSPI * tft_dev, const char *string, int32_t x, int32_t y, uint8_t font);  // Deprecated, use setTextDatum() and drawString()
static int16_t  drawRightString(struct TFT_eSPI * tft_dev, const char *string, int32_t x, int32_t y, uint8_t font);   // Deprecated, use setTextDatum() and drawString()
static void    setCursor(struct TFT_eSPI * tft_dev, int16_t x, int16_t y);                 // Set cursor for tft.print()
static void    setCursorFont(struct TFT_eSPI * tft_dev, int16_t x, int16_t y, uint8_t font);   // Set cursor and font number for tft.print()
static int16_t  getCursorX(struct TFT_eSPI * tft_dev);                                // Read current cursor x position (moves with tft.print())
static int16_t  getCursorY(struct TFT_eSPI * tft_dev);                                // Read current cursor y position
static void    setTextColor(struct TFT_eSPI * tft_dev, uint16_t color);                    // Set character (glyph) color only (background not over-written)
static void    setTextColorFill(struct TFT_eSPI * tft_dev, uint16_t fgcolor, uint16_t bgcolor, bool bgfill);  // Set character (glyph) foreground and background colour, optional background fill for smooth fonts, bgfill=false
static void    setTextSize(struct TFT_eSPI * tft_dev, uint8_t size);                       // Set character size multiplier (this increases pixel size)
static void    setTextWrap(struct TFT_eSPI * tft_dev, bool wrapX, bool wrapY);     // Turn on/off wrapping of text in TFT width and/or height  wrapY=false
static void    setTextDatum(struct TFT_eSPI * tft_dev, uint8_t datum);                     // Set text datum position (default is top left), see Section 6 above
static uint8_t getTextDatum(struct TFT_eSPI * tft_dev);
static void    setTextPadding(struct TFT_eSPI * tft_dev, uint16_t x_width);                // Set text padding (background blanking/over-write) width in pixels
static uint16_t getTextPadding(struct TFT_eSPI * tft_dev);                            // Get text padding
#ifdef LOAD_GFXFF
static void    setFreeFont(struct TFT_eSPI * tft_dev, const GFXfont *f);                 // Select the GFX Free Font f = null
static void    setTextFont(struct TFT_eSPI * tft_dev, uint8_t font);                       // Set the font number to use in future
#else
static void    setFreeFont(struct TFT_eSPI * tft_dev, uint8_t font);                       // Not used, historical fix to prevent an error
static void    setTextFont(struct TFT_eSPI * tft_dev, uint8_t font);                       // Set the font number to use in future
#endif
static int16_t  textWidthFont(struct TFT_eSPI * tft_dev, const char *string, uint8_t font);     // Returns pixel width of string in specified font
static int16_t  textWidth(struct TFT_eSPI * tft_dev, const char *string);                   // Returns pixel width of string in current font
static int16_t  fontHeight(struct TFT_eSPI * tft_dev, int16_t font);                        // Returns pixel height of string in specified font
static uint16_t  decodeUTF8(struct TFT_eSPI * tft_dev, uint8_t *buf, uint16_t *index, uint16_t remaining);
static uint16_t  decodeUTF8C(struct TFT_eSPI * tft_dev, uint8_t c);
static size_t    write(struct TFT_eSPI * tft_dev, uint8_t c);
static void    setCallback(struct TFT_eSPI * tft_dev, getColorCallback getCol);
static uint16_t fontsLoaded(struct TFT_eSPI * tft_dev); // Each bit in returned value represents a font type that is loaded - used for debug/error handling only
#ifndef RM68120_DRIVE
static void    writecommand(struct TFT_eSPI * tft_dev, uint8_t c);  // Send a command, function resets DC/RS high ready for data
#else
static void    writecommand(struct TFT_eSPI * tft_dev, uint16_t c); // Send a command, function resets DC/RS high ready for data
static void    writeRegister(struct TFT_eSPI * tft_dev, uint16_t c, uint8_t d); // Write data to 16 bit command register
#endif
static void    writedata(struct TFT_eSPI * tft_dev, uint8_t d);     // Send data with DC/RS set high
static void    commandList(struct TFT_eSPI * tft_dev, const uint8_t *addr); // Send a initialisation sequence to TFT stored in FLASH
static uint8_t     readcommand8(struct TFT_eSPI * tft_dev, uint8_t cmd_function, uint8_t index); // read 8 bits from TFT
static uint16_t   readcommand16(struct TFT_eSPI * tft_dev, uint8_t cmd_function, uint8_t index); // read 16 bits from TFT
static uint32_t   readcommand32(struct TFT_eSPI * tft_dev, uint8_t cmd_function, uint8_t index); // read 32 bits from TFT
static uint16_t   color565(struct TFT_eSPI * tft_dev, uint8_t red, uint8_t green, uint8_t blue);
static uint16_t   color8to16(struct TFT_eSPI * tft_dev, uint8_t color332);
static uint8_t    color16to8(struct TFT_eSPI * tft_dev, uint16_t color565);
static uint32_t   color16to24(struct TFT_eSPI * tft_dev, uint16_t color565);
static uint32_t   color24to16(struct TFT_eSPI * tft_dev, uint32_t color888);
static uint16_t   alphaBlend(struct TFT_eSPI * tft_dev, uint8_t alpha, uint16_t fgc, uint16_t bgc);
static uint16_t   alphaBlendDither(struct TFT_eSPI * tft_dev, uint8_t alpha, uint16_t fgc, uint16_t bgc, uint8_t dither);
static uint32_t   alphaBlend24(struct TFT_eSPI * tft_dev, uint8_t alpha, uint32_t fgc, uint32_t bgc, uint8_t dither); //dither = 0
static void    startWrite(struct TFT_eSPI * tft_dev);                         // Begin SPI transaction
static void    writeColor(struct TFT_eSPI * tft_dev, uint16_t color, uint32_t len); // Deprecated, use pushBlock()
static void    endWrite(struct TFT_eSPI * tft_dev);                           // End SPI transaction
static void    setAttribute(struct TFT_eSPI * tft_dev, uint8_t id, uint8_t a); // Set attribute value
static uint8_t getAttribute(struct TFT_eSPI * tft_dev, uint8_t id);                // Get attribute value
static void    getSetup(struct TFT_eSPI * tft_dev, setup_t *tft_settings); // Sketch provides the instance to populate
static bool    verifySetupID(struct TFT_eSPI * tft_dev, uint32_t id);
static void    begin_tft_write(struct TFT_eSPI * tft_dev);
static void    end_tft_write(struct TFT_eSPI * tft_dev);
static void    begin_tft_read(struct TFT_eSPI * tft_dev);
static void    end_tft_read(struct TFT_eSPI * tft_dev);
static void    readAddrWindow(struct TFT_eSPI * tft_dev, int32_t xs, int32_t ys, int32_t w, int32_t h);
static float   wedgeLineDistance(struct TFT_eSPI * tft_dev, float pax, float pay, float bax, float bay, float dr);
static void    pushBlock(struct TFT_eSPI * tft_dev, uint16_t color, uint32_t len);
static void    pushPixels(struct TFT_eSPI * tft_dev, const void *data_in, uint32_t len);

// Create a null default font in case some fonts not used (to prevent crash)
const  uint8_t widtbl_null[1] = {0};
const uint8_t chr_null[1] = {0};
const uint8_t* const chrtbl_null[1] = {chr_null};

// Now fill the structure
const fontinfo fontdata [] = {
  #ifdef LOAD_GLCD
   { (const uint8_t *)font, widtbl_null, 0, 0 },
  #else
   { (const uint8_t *)chrtbl_null, widtbl_null, 0, 0 },
  #endif
   // GLCD font (Font 1) does not have all parameters
   { (const uint8_t *)chrtbl_null, widtbl_null, 8, 7 },

  #ifdef LOAD_FONT2
   { (const uint8_t *)chrtbl_f16, widtbl_f16, chr_hgt_f16, baseline_f16},
  #else
   { (const uint8_t *)chrtbl_null, widtbl_null, 0, 0 },
  #endif

   // Font 3 current unused
   { (const uint8_t *)chrtbl_null, widtbl_null, 0, 0 },

  #ifdef LOAD_FONT4
   { (const uint8_t *)chrtbl_f32, widtbl_f32, chr_hgt_f32, baseline_f32},
  #else
   { (const uint8_t *)chrtbl_null, widtbl_null, 0, 0 },
  #endif

   // Font 5 current unused
   { (const uint8_t *)chrtbl_null, widtbl_null, 0, 0 },

  #ifdef LOAD_FONT6
   { (const uint8_t *)chrtbl_f64, widtbl_f64, chr_hgt_f64, baseline_f64},
  #else
   { (const uint8_t *)chrtbl_null, widtbl_null, 0, 0 },
  #endif

  #ifdef LOAD_FONT7
   { (const uint8_t *)chrtbl_f7s, widtbl_f7s, chr_hgt_f7s, baseline_f7s},
  #else
   { (const uint8_t *)chrtbl_null, widtbl_null, 0, 0 },
  #endif

  #ifdef LOAD_FONT8
   { (const uint8_t *)chrtbl_f72, widtbl_f72, chr_hgt_f72, baseline_f72}
  #else
   { (const uint8_t *)chrtbl_null, widtbl_null, 0, 0 }
  #endif
};

#ifndef SPI_BUSY_CHECK
    #define SPI_BUSY_CHECK
#endif

// Clipping macro for pushImage
#define PI_CLIP(dev)                                                    \
    if (dev->_vpOoB) return;                                            \
    x+= dev->_xDatum;                                                   \
    y+= dev->_yDatum;                                                   \
                                                                        \
    if ((x >= dev->_vpW) || (y >= dev->_vpH)) return;                   \
                                                                        \
    int32_t dx = 0;                                                     \
    int32_t dy = 0;                                                     \
    int32_t dw = w;                                                     \
    int32_t dh = h;                                                     \
                                                                        \
    if (x < dev->_vpX) { dx = dev->_vpX - x; dw -= dx; x = dev->_vpX; } \
    if (y < dev->_vpY) { dy = dev->_vpY - y; dh -= dy; y = dev->_vpY; } \
                                                                        \
    if ((x + dw) > dev->_vpW ) dw = dev->_vpW - x;                      \
    if ((y + dh) > dev->_vpH ) dh = dev->_vpH - y;                      \
                                                                        \
    if (dw < 1 || dh < 1) return;

static inline void delay(int32_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

static inline void swap_coord(int32_t *a, int32_t *b)
{ int32_t t = *a; *a = *b; *b = t; }

/***************************************************************************************
** Function name:                     initBus
** Description:                         initialise the SPI or parallel bus
***************************************************************************************/
static GB_RESULT initBus(struct TFT_eSPI * tft_dev)
{
    GB_RESULT res = GB_OK;

    tft_dev->bus = &hspi;

#if defined (TFT_PARALLEL_8_BIT)

    // Make sure read is high before we set the bus to output
    pinMode(TFT_RD, OUTPUT);
    digitalWrite(TFT_RD, HIGH);

    #if    !defined (ARDUINO_ARCH_RP2040)    && !defined (ARDUINO_ARCH_MBED)// PIO manages pins
        // Set TFT data bus lines to output
        pinMode(TFT_D0, OUTPUT); digitalWrite(TFT_D0, HIGH);
        pinMode(TFT_D1, OUTPUT); digitalWrite(TFT_D1, HIGH);
        pinMode(TFT_D2, OUTPUT); digitalWrite(TFT_D2, HIGH);
        pinMode(TFT_D3, OUTPUT); digitalWrite(TFT_D3, HIGH);
        pinMode(TFT_D4, OUTPUT); digitalWrite(TFT_D4, HIGH);
        pinMode(TFT_D5, OUTPUT); digitalWrite(TFT_D5, HIGH);
        pinMode(TFT_D6, OUTPUT); digitalWrite(TFT_D6, HIGH);
        pinMode(TFT_D7, OUTPUT); digitalWrite(TFT_D7, HIGH);
    #endif

    PARALLEL_INIT_TFT_DATA_BUS;

#endif
    return res;
}

//extern void st7789_init(struct TFT_eSPI * tft_dev, uint8_t tc);
/***************************************************************************************
** Function name:                     TFT_eSpi_init (tc is tab colour for ST7735 displays only)
** Description:                         Reset, then initialise the TFT display registers
***************************************************************************************/
void TFT_eSpi_init(struct TFT_eSPI * tft_dev, int16_t w, int16_t h, uint8_t tc)
{
    tft_dev->_init_width    = tft_dev->_width  = w; // Set by specific xxxxx_Defines.h file or by users sketch
    tft_dev->_init_height   = tft_dev->_height = h; // Set by specific xxxxx_Defines.h file or by users sketch

    tft_dev->rotation        = 0;
    tft_dev->cursor_y        = tft_dev->cursor_x  = tft_dev->last_cursor_x = tft_dev->bg_cursor_x = 0;
    tft_dev->textfont        = 1;
    tft_dev->textsize        = 1;
    tft_dev->textcolor       = tft_dev->bitmap_fg = 0xFFFF;                                                                                         // White
    tft_dev->textbgcolor     = tft_dev->bitmap_bg = 0x0000;                                                                                         // Black
    tft_dev->padX            = 0;                                                                                                                                             // No padding

    tft_dev->_fillbg = false;    // Smooth font only at the moment, force text background fill

    tft_dev->isDigits      = false;         // No bounding box adjustment
    tft_dev->textwrapX     = true;          // Wrap text at end of line when using print stream
    tft_dev->textwrapY     = false;         // Wrap text at bottom of screen when using print stream
    tft_dev->textdatum     = TL_DATUM;      // Top Left text alignment is default
    tft_dev->fontsloaded   = 0;

    tft_dev->_swapBytes = false;    // Do not swap colour bytes by default for big edain pictures

    tft_dev->locked          = true;     // Transaction mutex lock flag to ensure begin/endTranaction pairing
    tft_dev->inTransaction   = false;    // Flag to prevent multiple sequential functions to keep bus access open
    tft_dev->lockTransaction = false;    // start/endWrite lock flag to allow sketch to keep SPI bus access open

    tft_dev->_booted = true;    // Default attributes
    tft_dev->_cp437  = true;    // Legacy GLCD font bug fix
    tft_dev->_utf8   = true;    // UTF8 decoding enabled

#if defined (FONT_FS_AVAILABLE) && defined (SMOOTH_FONT)
    tft_dev->fs_font    = true;         // Smooth font filing system or array (fs_font = false) flag
#endif

#if defined (CONFIG_SPIRAM_SUPPORT)
    tft_dev->_psram_enable = true; // Enable the use of PSRAM (if available)
    else
#endif
    tft_dev->_psram_enable = false;

    tft_dev->addr_row = 0xFFFF;    // drawPixel command length optimiser
    tft_dev->addr_col = 0xFFFF;    // drawPixel command length optimiser

    tft_dev->_xPivot = 0;
    tft_dev->_yPivot = 0;

// Legacy support for bit GPIO masks
    tft_dev->cspinmask     = 0;
    tft_dev->dcpinmask     = 0;
    tft_dev->wrpinmask     = 0;
    tft_dev->sclkpinmask   = 0;

// Flags for which fonts are loaded
#ifdef LOAD_GLCD
    tft_dev->fontsloaded  = 0x0002; // Bit 1 set
#endif

#ifdef LOAD_FONT2
    tft_dev->fontsloaded |= 0x0004; // Bit 2 set
#endif

#ifdef LOAD_FONT4
    tft_dev->fontsloaded |= 0x0010; // Bit 4 set
#endif

#ifdef LOAD_FONT6
    tft_dev->fontsloaded |= 0x0040; // Bit 6 set
#endif

#ifdef LOAD_FONT7
    tft_dev->fontsloaded |= 0x0080; // Bit 7 set
#endif

#ifdef LOAD_FONT8
    tft_dev->fontsloaded |= 0x0100; // Bit 8 set
#endif

#ifdef LOAD_FONT8N
    tft_dev->fontsloaded |= 0x0200; // Bit 9 set
#endif

#ifdef SMOOTH_FONT
    tft_dev->fontsloaded |= 0x8000; // Bit 15 set
#endif

    tft_dev->drawPixel           = &drawPixel;
    tft_dev->drawChar            = &drawChar;
    tft_dev->drawLine            = &drawLine;
    tft_dev->drawFastVLine       = &drawFastVLine;
    tft_dev->drawFastHLine       = &drawFastHLine;
    tft_dev->fillRect            = &fillRect;
    tft_dev->drawCharUniFont     = &drawCharUniFont;
    tft_dev->drawCharUni         = &drawCharUni;
    tft_dev->height              = &height;
    tft_dev->width               = &width;
    tft_dev->readPixel           = &readPixel;
    tft_dev->setWindow           = &setWindow;
    tft_dev->pushColor           = &pushColor;
    tft_dev->setRotation         = &setRotation;
    tft_dev->getRotation         = &getRotation;
    tft_dev->invertDisplay       = &invertDisplay;
    tft_dev->setAddrWindow       = &setAddrWindow;
    tft_dev->setViewport         = &setViewport;
    tft_dev->checkViewport       = &checkViewport;
    tft_dev->getViewportX        = &getViewportX;
    tft_dev->getViewportY        = &getViewportY;
    tft_dev->getViewportWidth    = &getViewportWidth;
    tft_dev->getViewportHeight   = &getViewportHeight;
    tft_dev->getViewportDatum    = &getViewportDatum;
    tft_dev->frameViewport       = &frameViewport;
    tft_dev->resetViewport       = &resetViewport;
    tft_dev->clipAddrWindow      = &clipAddrWindow;
    tft_dev->clipWindow          = &clipWindow;
    tft_dev->pushColorLen        = &pushColorLen;
    tft_dev->pushColorsSwap      = &pushColorsSwap;
    tft_dev->pushColors          = &pushColors;
    tft_dev->fillScreen          = &fillScreen;
    tft_dev->drawRect            = &drawRect;
    tft_dev->drawRoundRect       = &drawRoundRect;
    tft_dev->fillRoundRect       = &fillRoundRect;
    tft_dev->fillRectVGradient   = &fillRectVGradient;
    tft_dev->fillRectHGradient   = &fillRectHGradient;
    tft_dev->drawPixelAlpha      = &drawPixelAlpha;
    tft_dev->drawSpot            = &drawSpot;
    tft_dev->fillSmoothCircle    = &fillSmoothCircle;
    tft_dev->fillSmoothRoundRect = &fillSmoothRoundRect;
    tft_dev->drawWideLine        = &drawWideLine;
    tft_dev->drawWedgeLine       = &drawWedgeLine;
    tft_dev->drawCircle          = &drawCircle;
    tft_dev->drawCircleHelper    = &drawCircleHelper;
    tft_dev->fillCircle          = &fillCircle;
    tft_dev->fillCircleHelper    = &fillCircleHelper;
    tft_dev->drawEllipse         = &drawEllipse;
    tft_dev->fillEllipse         = &fillEllipse;
    tft_dev->drawTriangle        = &drawTriangle;
    tft_dev->fillTriangle        = &fillTriangle;
    tft_dev->setSwapBytes        = &setSwapBytes;
    tft_dev->getSwapBytes        = &getSwapBytes;
    tft_dev->drawBitmap          = &drawBitmap;
    tft_dev->drawBitmapBg        = &drawBitmapBg;
    tft_dev->drawXBitmap         = &drawXBitmap;
    tft_dev->drawXBitmapBg       = &drawXBitmapBg;
    tft_dev->setBitmapColor      = &setBitmapColor;
    tft_dev->setPivot            = &setPivot;
    tft_dev->getPivotX           = &getPivotX;
    tft_dev->getPivotY           = &getPivotY;
    tft_dev->readRect            = &readRect;
    tft_dev->pushRect            = &pushRect;
    tft_dev->pushImage        = &pushImage;
    tft_dev->pushImageTrans   = &pushImageTrans;
    tft_dev->pushImageFlashTrans = &pushImageFlashTrans;
    tft_dev->pushImageFlash      = &pushImageFlash;
    tft_dev->pushImageBpp8       = &pushImageBpp8;
    tft_dev->pushImageBpp8Trans  = &pushImageBpp8Trans;
    tft_dev->pushImageBpp8Flash  = &pushImageBpp8Flash;
    tft_dev->readRectRGB         = &readRectRGB;
    tft_dev->drawNumberFont      = &drawNumberFont;
    tft_dev->drawNumber          = &drawNumber;
    tft_dev->drawFloatFont       = &drawFloatFont;
    tft_dev->drawFloat           = &drawFloat;
    tft_dev->drawStringFont      = &drawStringFont;
    tft_dev->drawString          = &drawString;
    tft_dev->drawCentreString    = &drawCentreString;
    tft_dev->drawRightString     = &drawRightString;
    tft_dev->setCursor           = &setCursor;
    tft_dev->setCursorFont       = &setCursorFont;
    tft_dev->getCursorX          = &getCursorX;
    tft_dev->getCursorY          = &getCursorY;
    tft_dev->setTextColor        = &setTextColor;
    tft_dev->setTextColorFill    = &setTextColorFill;
    tft_dev->setTextSize         = &setTextSize;
    tft_dev->setTextWrap         = &setTextWrap;
    tft_dev->setTextDatum        = &setTextDatum;
    tft_dev->getTextDatum        = &getTextDatum;
    tft_dev->setTextPadding      = &setTextPadding;
    tft_dev->getTextPadding      = &getTextPadding;
#ifdef LOAD_GFXFF
    tft_dev->setFreeFont = &setFreeFont;
    tft_dev->setTextFont = &setTextFont;
#else
    tft_dev->setFreeFont = &setFreeFont;
    tft_dev->setTextFont = &setTextFont;
#endif
    tft_dev->textWidthFont = &textWidthFont;
    tft_dev->textWidth     = &textWidth;
    tft_dev->fontHeight    = &fontHeight;
    tft_dev->decodeUTF8    = &decodeUTF8;
    tft_dev->decodeUTF8C   = &decodeUTF8C;
    tft_dev->write         = &write;
    tft_dev->setCallback   = &setCallback;
    tft_dev->fontsLoaded   = &fontsLoaded;
#ifndef RM68120_DRIVE
    tft_dev->writecommand = &writecommand;
#else
    tft_dev->writecommand  = &writecommand;
    tft_dev->writeRegister = &writeRegister;
#endif
    tft_dev->writedata         = &writedata;
    tft_dev->commandList       = &commandList;
    tft_dev->readcommand8      = &readcommand8;
    tft_dev->readcommand16     = &readcommand16;
    tft_dev->readcommand32     = &readcommand32;
    tft_dev->color565          = &color565;
    tft_dev->color8to16        = &color8to16;
    tft_dev->color16to8        = &color16to8;
    tft_dev->color16to24       = &color16to24;
    tft_dev->color24to16       = &color24to16;
    tft_dev->alphaBlend        = &alphaBlend;
    tft_dev->alphaBlendDither  = &alphaBlendDither;
    tft_dev->alphaBlend24      = &alphaBlend24;
    tft_dev->startWrite        = &startWrite;
    tft_dev->writeColor        = &writeColor;
    tft_dev->endWrite          = &endWrite;
    tft_dev->setAttribute      = &setAttribute;
    tft_dev->getAttribute      = &getAttribute;
    tft_dev->getSetup          = &getSetup;
    tft_dev->verifySetupID     = &verifySetupID;
    tft_dev->begin_tft_write   = &begin_tft_write;
    tft_dev->end_tft_write     = &end_tft_write;
    tft_dev->begin_tft_read    = &begin_tft_read;
    tft_dev->end_tft_read      = &end_tft_read;
    tft_dev->wedgeLineDistance = &wedgeLineDistance;
    tft_dev->readAddrWindow    = &readAddrWindow;
    tft_dev->pushBlock           = &pushBlock;
    tft_dev->pushPixels          = &pushPixels;

    // Reset the viewport to the whole screen
    tft_dev->resetViewport(tft_dev);

    if (tft_dev->_booted)
    {
        initBus(tft_dev);

        tft_dev->lockTransaction = false;
        tft_dev->inTransaction   = false;
        tft_dev->locked          = true;
        tft_dev->_booted         = false;

        tft_dev->end_tft_write(tft_dev);
    } // end of: if just _booted

    GB_DEBUGI(DISP_TAG, "TFT_eSpi_init Done");
}

/***************************************************************************************
** Function name:                     begin_tft_write (was called spi_begin)
** Description:                         Start SPI transaction for writes and select TFT
***************************************************************************************/
static void begin_tft_write(struct TFT_eSPI * tft_dev)
{
    if (tft_dev->locked) {
        tft_dev->locked = false; // Flag to show SPI access now unlocked
        CS_L;
        //SET_BUS_WRITE_MODE;
    }
}

/***************************************************************************************
** Function name:                     end_tft_write (was called spi_end)
** Description:                         End transaction for write and deselect TFT
***************************************************************************************/
static void end_tft_write(struct TFT_eSPI * tft_dev)
{
    if(!tft_dev->inTransaction) {            // Flag to stop ending transaction during multiple graphics calls
        if (!tft_dev->locked) {                    // Locked when beginTransaction has been called
            tft_dev->locked = true;                // Flag to show SPI access now locked
            //SPI_BUSY_CHECK;             // Check send complete and clean out unused rx data
            CS_H;
        }
    }
}

/***************************************************************************************
** Function name:                     begin_tft_read    (was called spi_begin_read)
** Description:                         Start transaction for reads and select TFT
***************************************************************************************/
// Reads require a lower SPI clock rate than writes
static void begin_tft_read(struct TFT_eSPI * tft_dev)
{
    if (tft_dev->locked) {
        tft_dev->locked = false;
        CS_L;
    }
}


/***************************************************************************************
** Function name:                     end_tft_read (was called spi_end_read)
** Description:                         End transaction for reads and deselect TFT
***************************************************************************************/
static void end_tft_read(struct TFT_eSPI * tft_dev)
{
    if(!tft_dev->inTransaction) {
        if (!tft_dev->locked) {
            tft_dev->locked = true;
            CS_H;
        }
    }
}

/***************************************************************************************
** Function name:                     setViewport
** Description:                         Set the clipping region for the TFT screen
***************************************************************************************/
static void setViewport(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, bool vpDatum)
{
    // Viewport metrics (not clipped)
    tft_dev->_xDatum    = x; // Datum x position in screen coordinates
    tft_dev->_yDatum    = y; // Datum y position in screen coordinates
    tft_dev->_xWidth    = w; // Viewport width
    tft_dev->_yHeight   = h; // Viewport height

    // Full size default viewport
    tft_dev->_vpDatum = false;                     // Datum is at top left corner of screen (true = top left of viewport)
    tft_dev->_vpOoB   = false;                     // Out of Bounds flag (true is all of viewport is off screen)
    tft_dev->_vpX     = 0;                         // Viewport top left corner x coordinate
    tft_dev->_vpY     = 0;                         // Viewport top left corner y coordinate
    tft_dev->_vpW     = tft_dev->width(tft_dev);   // Equivalent of TFT width    (Nb: viewport right edge coord + 1)
    tft_dev->_vpH     = tft_dev->height(tft_dev);  // Equivalent of TFT height (Nb: viewport bottom edge coord + 1)

    // Clip viewport to screen area
    if (x<0) { w += x; x = 0; }
    if (y<0) { h += y; y = 0; }
    if ((x + w) > tft_dev->width(tft_dev) ) { w = tft_dev->width(tft_dev)    - x; }
    if ((y + h) > tft_dev->height(tft_dev) ) { h = tft_dev->height(tft_dev) - y; }

    //Serial.print(" x=");Serial.print( x);Serial.print(", y=");Serial.print( y);
    //Serial.print(", w=");Serial.print(w);Serial.print(", h=");Serial.println(h);

    // Check if viewport is entirely out of bounds
    if (w < 1 || h < 1)
    {
        // Set default values and Out of Bounds flag in case of error
        tft_dev->_xDatum  = 0;
        tft_dev->_yDatum  = 0;
        tft_dev->_xWidth  = tft_dev->width(tft_dev);
        tft_dev->_yHeight = tft_dev->height(tft_dev);
        tft_dev->_vpOoB   = true;                      // Set Out of Bounds flag to inhibit all drawing
        return;
    }

    if (!vpDatum)
    {
        tft_dev->_xDatum  = 0;                         // Reset to top left of screen if not using a viewport datum
        tft_dev->_yDatum  = 0;
        tft_dev->_xWidth  = tft_dev->width(tft_dev);
        tft_dev->_yHeight = tft_dev->height(tft_dev);
    }

    // Store the clipped screen viewport metrics and datum position
    tft_dev->_vpX     = x;
    tft_dev->_vpY     = y;
    tft_dev->_vpW     = x + w;
    tft_dev->_vpH     = y + h;
    tft_dev->_vpDatum = vpDatum;

    //Serial.print(" _xDatum=");Serial.print( _xDatum);Serial.print(", _yDatum=");Serial.print( _yDatum);
    //Serial.print(", _xWidth=");Serial.print(_xWidth);Serial.print(", _yHeight=");Serial.println(_yHeight);

    //Serial.print(" _vpX=");Serial.print( _vpX);Serial.print(", _vpY=");Serial.print( _vpY);
    //Serial.print(", _vpW=");Serial.print(_vpW);Serial.print(", _vpH=");Serial.println(_vpH);

}

/***************************************************************************************
** Function name:                     checkViewport
** Description:                         Check if any part of specified area is visible in viewport
***************************************************************************************/
// Note: Setting w and h to 1 will check if coordinate x,y is in area
static bool checkViewport(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h)
{
    if (tft_dev->_vpOoB) return false;
    x+= tft_dev->_xDatum;
    y+= tft_dev->_yDatum;

    if ((x >= tft_dev->_vpW) || (y >= tft_dev->_vpH)) return false;

    int32_t dx = 0;
    int32_t dy = 0;
    int32_t dw = w;
    int32_t dh = h;

    if (x < tft_dev->_vpX) { dx = tft_dev->_vpX - x; dw -= dx; x = tft_dev->_vpX; }
    if (y < tft_dev->_vpY) { dy = tft_dev->_vpY - y; dh -= dy; y = tft_dev->_vpY; }

    if ((x + dw) > tft_dev->_vpW ) dw = tft_dev->_vpW - x;
    if ((y + dh) > tft_dev->_vpH ) dh = tft_dev->_vpH - y;

    if (dw < 1 || dh < 1) return false;

    return true;
}

/***************************************************************************************
** Function name:                     resetViewport
** Description:                         Reset viewport to whole TFT screen, datum at 0,0
***************************************************************************************/
static void resetViewport(struct TFT_eSPI * tft_dev)
{
    // Reset viewport to the whole screen (or sprite) area
    tft_dev->_vpDatum = false;
    tft_dev->_vpOoB   = false;
    tft_dev->_xDatum  = 0;
    tft_dev->_yDatum  = 0;
    tft_dev->_vpX     = 0;
    tft_dev->_vpY     = 0;
    tft_dev->_vpW     = tft_dev->width(tft_dev);
    tft_dev->_vpH     = tft_dev->height(tft_dev);
    tft_dev->_xWidth  = tft_dev->width(tft_dev);
    tft_dev->_yHeight = tft_dev->height(tft_dev);
}

/***************************************************************************************
** Function name:                     getViewportX
** Description:                         Get x position of the viewport datum
***************************************************************************************/
static int32_t getViewportX(struct TFT_eSPI * tft_dev)
{
    return tft_dev->_xDatum;
}

/***************************************************************************************
** Function name:                     getViewportY
** Description:                         Get y position of the viewport datum
***************************************************************************************/
static int32_t getViewportY(struct TFT_eSPI * tft_dev)
{
    return tft_dev->_yDatum;
}

/***************************************************************************************
** Function name:                     getViewportWidth
** Description:                         Get width of the viewport
***************************************************************************************/
static int32_t getViewportWidth(struct TFT_eSPI * tft_dev)
{
    return tft_dev->_xWidth;
}

/***************************************************************************************
** Function name:                     getViewportHeight
** Description:                         Get height of the viewport
***************************************************************************************/
static int32_t getViewportHeight(struct TFT_eSPI * tft_dev)
{
    return tft_dev->_yHeight;
}

/***************************************************************************************
** Function name:                     getViewportDatum
** Description:                         Get datum flag of the viewport (true = viewport corner)
***************************************************************************************/
static bool getViewportDatum(struct TFT_eSPI * tft_dev)
{
    return tft_dev->_vpDatum;
}

/***************************************************************************************
** Function name:                     frameViewport
** Description:                         Draw a frame inside or outside the viewport of width w
***************************************************************************************/
static void frameViewport(struct TFT_eSPI * tft_dev, uint16_t color, int32_t w)
{
    // Save datum position
    bool _dT = tft_dev->_vpDatum;

    // If w is positive the frame is drawn inside the viewport
    // a large positive width will clear the screen inside the viewport
    if (w>0)
    {
        // Set vpDatum true to simplify coordinate derivation
        tft_dev->_vpDatum = true;
        tft_dev->fillRect(tft_dev, 0, 0, tft_dev->_vpW - tft_dev->_vpX, w, color);                                // Top
        tft_dev->fillRect(tft_dev, 0, w, w, tft_dev->_vpH - tft_dev->_vpY - w - w, color);                // Left
        tft_dev->fillRect(tft_dev, tft_dev->_xWidth - w, w, w, tft_dev->_yHeight - w - w, color); // Right
        tft_dev->fillRect(tft_dev, 0, tft_dev->_yHeight - w, tft_dev->_xWidth, w, color);                 // Bottom
    }
    else
    // If w is negative the frame is drawn outside the viewport
    // a large negative width will clear the screen outside the viewport
    {
        w = -w;

        // Save old values
        int32_t _xT = tft_dev->_vpX;
        tft_dev->_vpX = 0;
        int32_t _yT = tft_dev->_vpY;
        tft_dev->_vpY = 0;
        int32_t _wT = tft_dev->_vpW;
        int32_t _hT = tft_dev->_vpH;

        // Set vpDatum false so frame can be drawn outside window
        tft_dev->_vpDatum = false; // When false the full width and height is accessed
        tft_dev->_vpH = tft_dev->height(tft_dev);
        tft_dev->_vpW = tft_dev->width(tft_dev);

        // Draw frame
        tft_dev->fillRect(tft_dev, _xT - w - tft_dev->_xDatum, _yT - w - tft_dev->_yDatum, _wT - _xT + w + w, w, color); // Top
        tft_dev->fillRect(tft_dev, _xT - w - tft_dev->_xDatum, _yT - tft_dev->_yDatum, w, _hT - _yT, color);                         // Left
        tft_dev->fillRect(tft_dev, _wT - tft_dev->_xDatum, _yT - tft_dev->_yDatum, w, _hT - _yT, color);                                 // Right
        tft_dev->fillRect(tft_dev, _xT - w - tft_dev->_xDatum, _hT - tft_dev->_yDatum, _wT - _xT + w + w, w, color);         // Bottom

        // Restore old values
        tft_dev->_vpX = _xT;
        tft_dev->_vpY = _yT;
        tft_dev->_vpW = _wT;
        tft_dev->_vpH = _hT;
    }

    // Restore vpDatum
    tft_dev->_vpDatum = _dT;
}

/***************************************************************************************
** Function name:                     clipAddrWindow
** Description:                         Clip address window x,y,w,h to screen and viewport
***************************************************************************************/
static bool clipAddrWindow(struct TFT_eSPI * tft_dev, int32_t *x, int32_t *y, int32_t *w, int32_t *h)
{
    if (tft_dev->_vpOoB) return false; // Area is outside of viewport

    *x+= tft_dev->_xDatum;
    *y+= tft_dev->_yDatum;

    if ((*x >= tft_dev->_vpW) || (*y >= tft_dev->_vpH)) return false;    // Area is outside of viewport

    // Crop drawing area bounds
    if (*x < tft_dev->_vpX) { *w -= tft_dev->_vpX - *x; *x = tft_dev->_vpX; }
    if (*y < tft_dev->_vpY) { *h -= tft_dev->_vpY - *y; *y = tft_dev->_vpY; }

    if ((*x + *w) > tft_dev->_vpW ) *w = tft_dev->_vpW - *x;
    if ((*y + *h) > tft_dev->_vpH ) *h = tft_dev->_vpH - *y;

    if (*w < 1 || *h < 1) return false; // No area is inside viewport

    return true;    // Area is wholly or partially inside viewport
}

/***************************************************************************************
** Function name:                     clipWindow
** Description:                         Clip window xs,yx,xe,ye to screen and viewport
***************************************************************************************/
static bool clipWindow(struct TFT_eSPI * tft_dev, int32_t *xs, int32_t *ys, int32_t *xe, int32_t *ye)
{
    if (tft_dev->_vpOoB) return false; // Area is outside of viewport

    *xs+= tft_dev->_xDatum;
    *ys+= tft_dev->_yDatum;
    *xe+= tft_dev->_xDatum;
    *ye+= tft_dev->_yDatum;

    if ((*xs >= tft_dev->_vpW) || (*ys >= tft_dev->_vpH)) return false;    // Area is outside of viewport
    if ((*xe <    tft_dev->_vpX) || (*ye <    tft_dev->_vpY)) return false;    // Area is outside of viewport

    // Crop drawing area bounds
    if (*xs < tft_dev->_vpX) *xs = tft_dev->_vpX;
    if (*ys < tft_dev->_vpY) *ys = tft_dev->_vpY;

    if (*xe > tft_dev->_vpW) *xe = tft_dev->_vpW - 1;
    if (*ye > tft_dev->_vpH) *ye = tft_dev->_vpH - 1;

    return true;    // Area is wholly or partially inside viewport
}

/***************************************************************************************
** Function name:                     setRotation
** Description:                         rotate the screen orientation m = 0-3 or 4-7 for BMP drawing
***************************************************************************************/
static void setRotation(struct TFT_eSPI * tft_dev, uint8_t m)
{
    disp_driver_set_rotation(m);

    tft_dev->addr_row = 0xFFFF;
    tft_dev->addr_col = 0xFFFF;

    // Reset the viewport to the whole screen
    tft_dev->resetViewport(tft_dev);
}


/***************************************************************************************
** Function name:                     commandList, used for FLASH based lists only (e.g. ST7735)
** Description:                         Get initialisation commands from FLASH and send to TFT
***************************************************************************************/
static void commandList(struct TFT_eSPI * tft_dev, const uint8_t *addr)
{
    uint8_t    numCommands;
    uint8_t    numArgs;
    uint8_t    ms;

    numCommands = pgm_read_byte(addr++);     // Number of commands to follow

    while (numCommands--)                                    // For each command...
    {
        tft_dev->writecommand(tft_dev, pgm_read_byte(addr++)); // Read, issue command
        numArgs = pgm_read_byte(addr++);         // Number of args to follow
        ms = numArgs & TFT_INIT_DELAY;             // If hibit set, delay follows args
        numArgs &= ~TFT_INIT_DELAY;                    // Mask out delay bit

        while (numArgs--)                                        // For each argument...
        {
            tft_dev->writedata(tft_dev, pgm_read_byte(addr++));    // Read, issue argument
        }

        if (ms)
        {
            ms = pgm_read_byte(addr++);                // Read post-command delay time (ms)
            delay( (ms==255 ? 500 : ms) );
        }
    }
}

/***************************************************************************************
** Function name:                     writecommand
** Description:                         Send an 8 bit command to the TFT
***************************************************************************************/
#ifndef RM68120_DRIVER
static void writecommand(struct TFT_eSPI * tft_dev, uint8_t c)
{
    tft_dev->begin_tft_write(tft_dev);
    DC_C;

    tft_Write_8(c);

    DC_D;
    tft_dev->end_tft_write(tft_dev);
}
#else
static void writecommand(struct TFT_eSPI * tft_dev, uint16_t c)
{
    tft_dev->begin_tft_write(tft_dev);
    DC_C;

    tft_Write_16(c);

    DC_D;
    tft_dev->end_tft_write(tft_dev);
}
static void writeRegister(struct TFT_eSPI * tft_dev, uint16_t c, uint8_t d)
{

    tft_dev->begin_tft_write(tft_dev);
    DC_C;

    tft_Write_16(c);

    DC_D;

    tft_Write_8(d);

    tft_dev->end_tft_write(tft_dev);
}
#endif

/***************************************************************************************
** Function name:                     writedata
** Description:                         Send a 8 bit data value to the TFT
***************************************************************************************/
static void writedata(struct TFT_eSPI * tft_dev, uint8_t d)
{
    tft_dev->begin_tft_write(tft_dev);
    DC_D;                // Play safe, but should already be in data mode

    tft_Write_8(d);

    CS_L;                // Allow more hold time for low VDI rail
    tft_dev->end_tft_write(tft_dev);
}


/***************************************************************************************
** Function name:                     readcommand8
** Description:                         Read a 8 bit data value from an indexed command register
***************************************************************************************/
static uint8_t readcommand8(struct TFT_eSPI * tft_dev, uint8_t cmd_function, uint8_t index)
{
    uint8_t reg;

#if defined(TFT_PARALLEL_8_BIT) || defined(RP2040_PIO_INTERFACE)

    writecommand(cmd_function); // Sets DC and CS high

    busDir(dir_mask, INPUT);

    CS_L;

    // Read nth parameter (assumes caller discards 1st parameter or points index to 2nd)
    while(index--) reg = readByte();

    busDir(dir_mask, OUTPUT);

    CS_H;

#else // SPI interface
    // Tested with ILI9341 set to Interface II i.e. IM [3:0] = "1101"
    tft_dev->begin_tft_read(tft_dev);
    index = 0x10 + (index & 0x0F);

    DC_C; tft_Write_8(0xD9);
    DC_D; tft_Write_8(index);

    CS_H; // Some displays seem to need CS to be pulsed here, or is just a delay needed?
    CS_L;

    DC_C; tft_Write_8(cmd_function);
    DC_D;
    tft_Read_8(&reg);

    tft_dev->end_tft_read(tft_dev);
#endif
    return reg;
}


/***************************************************************************************
** Function name:                     readcommand16
** Description:                         Read a 16 bit data value from an indexed command register
***************************************************************************************/
static uint16_t readcommand16(struct TFT_eSPI * tft_dev, uint8_t cmd_function, uint8_t index)
{
    uint32_t reg;

    reg  = (tft_dev->readcommand8(tft_dev, cmd_function, index + 0) <<    8);
    reg |= (tft_dev->readcommand8(tft_dev, cmd_function, index + 1) <<    0);

    return reg;
}


/***************************************************************************************
** Function name:                     readcommand32
** Description:                         Read a 32 bit data value from an indexed command register
***************************************************************************************/
static uint32_t readcommand32(struct TFT_eSPI * tft_dev, uint8_t cmd_function, uint8_t index)
{
    uint32_t reg;

    reg  = ((uint32_t)tft_dev->readcommand8(tft_dev, cmd_function, index + 0) << 24);
    reg |= ((uint32_t)tft_dev->readcommand8(tft_dev, cmd_function, index + 1) << 16);
    reg |= ((uint32_t)tft_dev->readcommand8(tft_dev, cmd_function, index + 2) <<    8);
    reg |= ((uint32_t)tft_dev->readcommand8(tft_dev, cmd_function, index + 3) <<    0);

    return reg;
}


/***************************************************************************************
** Function name:                     read pixel (for SPI Interface II i.e. IM [3:0] = "1101")
** Description:                         Read 565 pixel colours from a pixel
***************************************************************************************/
static uint16_t readPixel(struct TFT_eSPI * tft_dev, int32_t x0, int32_t y0)
{
    if (tft_dev->_vpOoB) return 0;

    x0+= tft_dev->_xDatum;
    y0+= tft_dev->_yDatum;

    // Range checking
    if ((x0 < tft_dev->_vpX) || (y0 < tft_dev->_vpY) ||(x0 >= tft_dev->_vpW) || (y0 >= tft_dev->_vpH)) return 0;

#if defined(TFT_PARALLEL_8_BIT) || defined(RP2040_PIO_INTERFACE)

    CS_L;

    readAddrWindow(x0, y0, 1, 1);

    // Set masked pins D0- D7 to input
    busDir(dir_mask, INPUT);

    #if    !defined (SSD1963_DRIVER)
    // Dummy read to throw away don't care value
    readByte();
    #endif

    // Fetch the 16 bit BRG pixel
    //uint16_t rgb = (readByte() << 8) | readByte();

    #if defined (ILI9341_DRIVER)    || defined(ILI9341_2_DRIVER) || defined (ILI9488_DRIVER) || defined (SSD1963_DRIVER)// Read 3 bytes

        // Read window pixel 24 bit RGB values and fill in LS bits
        uint16_t rgb = ((readByte() & 0xF8) << 8) | ((readByte() & 0xFC) << 3) | (readByte() >> 3);

        CS_H;

        // Set masked pins D0- D7 to output
        busDir(dir_mask, OUTPUT);

        return rgb;

    #else // ILI9481 or ILI9486 16 bit read

        // Fetch the 16 bit BRG pixel
        uint16_t bgr = (readByte() << 8) | readByte();

        CS_H;

        // Set masked pins D0- D7 to output
        busDir(dir_mask, OUTPUT);

        #ifdef ILI9486_DRIVER
            return    bgr;
        #else
            // Swap Red and Blue (could check MADCTL setting to see if this is needed)
            return    (bgr>>11) | (bgr<<11) | (bgr & 0x7E0);
        #endif

    #endif

#else // Not TFT_PARALLEL_8_BIT

    // This function can get called during anti-aliased font rendering
    // so a transaction may be in progress
    bool wasInTransaction = tft_dev->inTransaction;
    if (tft_dev->inTransaction)
    {
        tft_dev->inTransaction= false;
        tft_dev->end_tft_write(tft_dev);
    }

    uint16_t color = 0;

    tft_dev->begin_tft_read(tft_dev); // Sets CS low

    tft_dev->readAddrWindow(tft_dev, x0, y0, 1, 1);

    #ifdef TFT_SDA_READ
        begin_SDA_Read();
    #endif

    // Dummy read to throw away don't care value
    uint8_t tmp;
    tft_Read_8(&tmp);

    #if defined (ST7796_DRIVER)
        // Read the 2 bytes
        color = ((tft_Read_8()) << 8) | (tft_Read_8());
    #else
        // Read the 3 RGB bytes, colour is actually only in the top 6 bits of each byte
        // as the TFT stores colours as 18 bits
        uint8_t r, g, b;
        tft_Read_8(&r);
        tft_Read_8(&g);
        tft_Read_8(&b);
        color = tft_dev->color565(tft_dev, r, g, b);
    #endif

    CS_H;

    #ifdef TFT_SDA_READ
        end_SDA_Read();
    #endif

    tft_dev->end_tft_read(tft_dev);

    // Reinstate the transaction if one was in progress
    if(wasInTransaction)
    {
        tft_dev->begin_tft_write(tft_dev);
        tft_dev->inTransaction = true;
    }

    return color;

#endif
}

static void setCallback(struct TFT_eSPI *tft_dev, getColorCallback getCol)
{
    tft_dev->getColor = getCol;
}


/***************************************************************************************
** Function name:                     read rectangle (for SPI Interface II i.e. IM [3:0] = "1101")
** Description:                         Read 565 pixel colours from a defined area
***************************************************************************************/
static void readRect(struct TFT_eSPI *tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, uint16_t *data)
{
    PI_CLIP(tft_dev);

#if defined(TFT_PARALLEL_8_BIT) || defined(RP2040_PIO_INTERFACE)

    CS_L;

    readAddrWindow(x, y, dw, dh);

    data += dx + dy * w;

    // Set masked pins D0- D7 to input
    busDir(dir_mask, INPUT);

    #if defined (ILI9341_DRIVER)    || defined(ILI9341_2_DRIVER) || defined (ILI9488_DRIVER) // Read 3 bytes
        // Dummy read to throw away don't care value
        readByte();

        // Fetch the 24 bit RGB value
        while (dh--) {
            int32_t lw = dw;
            uint16_t* line = data;
            while (lw--) {
                // Assemble the RGB 16 bit colour
                uint16_t rgb = ((readByte() & 0xF8) << 8) | ((readByte() & 0xFC) << 3) | (readByte() >> 3);

                // Swapped byte order for compatibility with pushRect()
                *line++ = (rgb<<8) | (rgb>>8);
            }
            data += w;
        }

    #elif    defined (SSD1963_DRIVER)
        // Fetch the 18 bit BRG pixels
        while (dh--) {
            int32_t lw = dw;
            uint16_t* line = data;
            while (lw--) {
                uint16_t bgr = ((readByte() & 0xF8) >> 3);; // CS_L adds a small delay
                bgr |= ((readByte() & 0xFC) << 3);
                bgr |= (readByte() << 8);
                // Swap Red and Blue (could check MADCTL setting to see if this is needed)
                uint16_t rgb = (bgr>>11) | (bgr<<11) | (bgr & 0x7E0);
                // Swapped byte order for compatibility with pushRect()
                *line++ = (rgb<<8) | (rgb>>8);
            }
            data += w;
        }

    #else // ILI9481 reads as 16 bits
        // Dummy read to throw away don't care value
        readByte();

        // Fetch the 16 bit BRG pixels
        while (dh--) {
            int32_t lw = dw;
            uint16_t* line = data;
            while (lw--) {
            #ifdef ILI9486_DRIVER
                // Read the RGB 16 bit colour
                *line++ = readByte() | (readByte() << 8);
            #else
                // Read the BRG 16 bit colour
                uint16_t bgr = (readByte() << 8) | readByte();
                // Swap Red and Blue (could check MADCTL setting to see if this is needed)
                uint16_t rgb = (bgr>>11) | (bgr<<11) | (bgr & 0x7E0);
                // Swapped byte order for compatibility with pushRect()
                *line++ = (rgb<<8) | (rgb>>8);
            #endif
            }
            data += w;
        }
    #endif

    CS_H;

    // Set masked pins D0- D7 to output
    busDir(dir_mask, OUTPUT);

#else // SPI interface

    // This function can get called after a begin_tft_write
    // so a transaction may be in progress
    bool wasInTransaction = tft_dev->inTransaction;
    if (tft_dev->inTransaction) { tft_dev->inTransaction= false; tft_dev->end_tft_write(tft_dev);}

    uint16_t color = 0;

    tft_dev->begin_tft_read(tft_dev);

    tft_dev->readAddrWindow(tft_dev, x, y, dw, dh);

    data += dx + dy * w;

    #ifdef TFT_SDA_READ
        begin_SDA_Read();
    #endif

    // Dummy read to throw away don't care value
    uint8_t tmp;
    tft_Read_8(&tmp);

    // Read window pixel 24 bit RGB values
    while (dh--) {
        int32_t lw = dw;
        uint16_t* line = data;
        while (lw--) {

#if !defined (ILI9488_DRIVER)
    #if defined (ST7796_DRIVER)
        // Read the 2 bytes
        color = ((tft_Read_8()) << 8) | (tft_Read_8());
    #else
        // Read the 3 RGB bytes, colour is actually only in the top 6 bits of each byte
        // as the TFT stores colours as 18 bits
        uint8_t r, g, b;
        tft_Read_8(&r);
        tft_Read_8(&g);
        tft_Read_8(&b);
        color = tft_dev->color565(tft_dev, r, g, b);
    #endif
#else
        // The 6 colour bits are in MS 6 bits of each byte but we do not include the extra clock pulse
        // so we use a trick and mask the middle 6 bits of the byte, then only shift 1 place left
        uint8_t r = (tft_Read_8()&0x7E)<<1;
        uint8_t g = (tft_Read_8()&0x7E)<<1;
        uint8_t b = (tft_Read_8()&0x7E)<<1;
        color = color565(r, g, b);
#endif

            // Swapped colour byte order for compatibility with pushRect()
            *line++ = color << 8 | color >> 8;
        }
        data += w;
    }

    //CS_H;

    #ifdef TFT_SDA_READ
        end_SDA_Read();
    #endif

    tft_dev->end_tft_read(tft_dev);

    // Reinstate the transaction if one was in progress
    if(wasInTransaction) { tft_dev->begin_tft_write(tft_dev); tft_dev->inTransaction = true; }
#endif
}


/***************************************************************************************
** Function name:                     push rectangle
** Description:                         push 565 pixel colours into a defined area
***************************************************************************************/
static void pushRect(struct TFT_eSPI *tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, uint16_t *data)
{
    bool swap = tft_dev->_swapBytes;

    tft_dev->_swapBytes = false;
    tft_dev->pushImage(tft_dev, x, y, w, h, data);
    tft_dev->_swapBytes = swap;
}


/***************************************************************************************
** Function name:                     pushImage
** Description:                         plot 16 bit colour sprite or image onto TFT
***************************************************************************************/
static void pushImage(struct TFT_eSPI *tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, uint16_t *data)
{
    PI_CLIP(tft_dev);

    tft_dev->begin_tft_write(tft_dev);
    tft_dev->inTransaction = true;

    tft_dev->setWindow(tft_dev, x, y, x + dw - 1, y + dh - 1);

    data += dx + dy * w;

    // Check if whole image can be pushed
    if (dw == w)
        tft_dev->pushPixels(tft_dev, data, dw * dh);
    else {
        // Push line segments to crop image
        while (dh--)
        {
            tft_dev->pushPixels(tft_dev, data, dw);
            data += w;
        }
    }

    tft_dev->inTransaction = tft_dev->lockTransaction;
    tft_dev->end_tft_write(tft_dev);
}

/***************************************************************************************
** Function name:                     pushImage
** Description:                         plot 16 bit sprite or image with 1 colour being transparent
***************************************************************************************/
static void pushImageTrans(struct TFT_eSPI *tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, uint16_t *data, uint16_t transp)
{
    PI_CLIP(tft_dev);

    tft_dev->begin_tft_write(tft_dev);
    tft_dev->inTransaction = true;

    data += dx + dy * w;


    uint16_t    lineBuf[dw]; // Use buffer to minimise setWindow call count

    // The little endian transp color must be byte swapped if the image is big endian
    if (!tft_dev->_swapBytes)
        transp = transp >> 8 | transp << 8;

    while (dh--)
    {
        int32_t len = dw;
        uint16_t* ptr = data;
        int32_t px = x, sx = x;
        bool move = true;
        uint16_t np = 0;

        while (len--)
        {
            if (transp != *ptr)
            {
                if (move) { move = false; sx = px; }
                lineBuf[np] = *ptr;
                np++;
            }
            else
            {
                move = true;
                if (np)
                {
                    tft_dev->setWindow(tft_dev, sx, y, sx + np - 1, y);
                    tft_dev->pushPixels(tft_dev, (uint16_t*)lineBuf, np);
                    np = 0;
                }
            }
            px++;
            ptr++;
        }
        if (np) { tft_dev->setWindow(tft_dev, sx, y, sx + np - 1, y); tft_dev->pushPixels(tft_dev, (uint16_t*)lineBuf, np); }

        y++;
        data += w;
    }

    tft_dev->inTransaction = tft_dev->lockTransaction;
    tft_dev->end_tft_write(tft_dev);
}


/***************************************************************************************
** Function name:                     pushImage - for FLASH (PROGMEM) stored images
** Description:                         plot 16 bit image
***************************************************************************************/
static void pushImageFlash(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, const uint16_t *data)
{
    // Requires 32 bit aligned access, so use PROGMEM 16 bit word functions
    PI_CLIP(tft_dev);

    tft_dev->begin_tft_write(tft_dev);
    tft_dev->inTransaction = true;

    data += dx + dy * w;

    uint16_t    buffer[dw];

    tft_dev->setWindow(tft_dev, x, y, x + dw - 1, y + dh - 1);

    // Fill and send line buffers to TFT
    for (int32_t i = 0; i < dh; i++) {
        for (int32_t j = 0; j < dw; j++) {
            buffer[j] = pgm_read_word(&data[i * w + j]);
        }
        tft_dev->pushPixels(tft_dev, buffer, dw);
    }

    tft_dev->inTransaction = tft_dev->lockTransaction;
    tft_dev->end_tft_write(tft_dev);
}

/***************************************************************************************
** Function name:                     pushImage - for FLASH (PROGMEM) stored images
** Description:                         plot 16 bit image with 1 colour being transparent
***************************************************************************************/
static void pushImageFlashTrans(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, const uint16_t *data, uint16_t transp)
{
    // Requires 32 bit aligned access, so use PROGMEM 16 bit word functions
    PI_CLIP(tft_dev);

    tft_dev->begin_tft_write(tft_dev);
    tft_dev->inTransaction = true;

    data += dx + dy * w;


    uint16_t    lineBuf[dw];

    // The little endian transp color must be byte swapped if the image is big endian
    if (!tft_dev->_swapBytes) transp = transp >> 8 | transp << 8;

    while (dh--) {
        int32_t len = dw;
        uint16_t* ptr = (uint16_t*)data;
        int32_t px = x, sx = x;
        bool move = true;

        uint16_t np = 0;

        while (len--) {
            uint16_t color = pgm_read_word(ptr);
            if (transp != color) {
                if (move) { move = false; sx = px; }
                lineBuf[np] = color;
                np++;
            }
            else {
                move = true;
                if (np) {
                    tft_dev->setWindow(tft_dev, sx, y, sx + np - 1, y);
                    tft_dev->pushPixels(tft_dev, lineBuf, np);
                    np = 0;
                }
            }
            px++;
            ptr++;
        }
        if (np) { tft_dev->setWindow(tft_dev, sx, y, sx + np - 1, y); tft_dev->pushPixels(tft_dev, lineBuf, np); }

        y++;
        data += w;
    }

    tft_dev->inTransaction = tft_dev->lockTransaction;
    tft_dev->end_tft_write(tft_dev);
}

/***************************************************************************************
** Function name:                     pushImage
** Description:                         plot 8 bit or 4 bit or 1 bit image or sprite using a line buffer
***************************************************************************************/
static void pushImageBpp8Flash(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, const uint8_t *data, bool bpp8, uint16_t *cmap)
{
    PI_CLIP(tft_dev);

    tft_dev->begin_tft_write(tft_dev);
    tft_dev->inTransaction = true;
    bool swap = tft_dev->_swapBytes;

    tft_dev->setWindow(tft_dev, x, y, x + dw - 1, y + dh - 1); // Sets CS low and sent RAMWR

    // Line buffer makes plotting faster
    uint16_t    lineBuf[dw];

    if (bpp8)
    {
        tft_dev->_swapBytes = false;

        uint8_t    blue[] = {0, 11, 21, 31}; // blue 2 to 5 bit colour lookup table

        tft_dev->_lastColor = -1; // Set to illegal value

        // Used to store last shifted colour
        uint8_t msbColor = 0;
        uint8_t lsbColor = 0;

        data += dx + dy * w;
        while (dh--) {
            uint32_t len = dw;
            uint8_t* ptr = (uint8_t*)data;
            uint8_t* linePtr = (uint8_t*)lineBuf;

            while(len--) {
                uint32_t color = pgm_read_byte(ptr++);

                // Shifts are slow so check if colour has changed first
                if (color != tft_dev->_lastColor) {
                    //                    =====Green=====         ===============Red==============
                    msbColor = (color & 0x1C)>>2 | (color & 0xC0)>>3 | (color & 0xE0);
                    //                    =====Green=====        =======Blue======
                    lsbColor = (color & 0x1C)<<3 | blue[color & 0x03];
                    tft_dev->_lastColor = color;
                }

             *linePtr++ = msbColor;
             *linePtr++ = lsbColor;
            }

            tft_dev->pushPixels(tft_dev, lineBuf, dw);

            data += w;
        }
        tft_dev->_swapBytes = swap; // Restore old value
    }
    else if (cmap != NULL) // Must be 4bpp
    {
        tft_dev->_swapBytes = true;

        w = (w+1) & 0xFFFE;     // if this is a sprite, w will already be even; this does no harm.
        bool splitFirst = (dx & 0x01) != 0; // split first means we have to push a single px from the left of the sprite / image

        if (splitFirst) {
            data += ((dx - 1 + dy * w) >> 1);
        }
        else {
            data += ((dx + dy * w) >> 1);
        }

        while (dh--) {
            uint32_t len = dw;
            uint8_t * ptr = (uint8_t*)data;
            uint16_t *linePtr = lineBuf;
            uint8_t colors; // two colors in one byte
            uint16_t index;

            if (splitFirst) {
                colors = pgm_read_byte(ptr);
                index = (colors & 0x0F);
                *linePtr++ = cmap[index];
                len--;
                ptr++;
            }

            while (len--)
            {
                colors = pgm_read_byte(ptr);
                index = ((colors & 0xF0) >> 4) & 0x0F;
                *linePtr++ = cmap[index];

                if (len--)
                {
                    index = colors & 0x0F;
                    *linePtr++ = cmap[index];
                } else {
                    break;    // nothing to do here
                }

                ptr++;
            }

            tft_dev->pushPixels(tft_dev, lineBuf, dw);
            data += (w >> 1);
        }
        tft_dev->_swapBytes = swap; // Restore old value
    }
    else // Must be 1bpp
    {
        tft_dev->_swapBytes = false;
        uint8_t * ptr = (uint8_t*)data;
        uint32_t ww =    (w+7)>>3; // Width of source image line in bytes
        for (int32_t yp = dy;    yp < dy + dh; yp++)
        {
            uint8_t* linePtr = (uint8_t*)lineBuf;
            for (int32_t xp = dx; xp < dx + dw; xp++)
            {
                uint16_t col = (pgm_read_byte(ptr + (xp>>3)) & (0x80 >> (xp & 0x7)) );
                if (col) {*linePtr++ = tft_dev->bitmap_fg>>8; *linePtr++ = (uint8_t) tft_dev->bitmap_fg;}
                else         {*linePtr++ = tft_dev->bitmap_bg>>8; *linePtr++ = (uint8_t) tft_dev->bitmap_bg;}
            }
            ptr += ww;
            tft_dev->pushPixels(tft_dev, lineBuf, dw);
        }
    }

    tft_dev->_swapBytes = swap; // Restore old value
    tft_dev->inTransaction = tft_dev->lockTransaction;
    tft_dev->end_tft_write(tft_dev);
}


/***************************************************************************************
** Function name:                     pushImage
** Description:                         plot 8 bit or 4 bit or 1 bit image or sprite using a line buffer
***************************************************************************************/
static void pushImageBpp8(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, uint8_t *data, bool bpp8, uint16_t *cmap)
{
    PI_CLIP(tft_dev);

    tft_dev->begin_tft_write(tft_dev);
    tft_dev->inTransaction = true;
    bool swap = tft_dev->_swapBytes;

    tft_dev->setWindow(tft_dev, x, y, x + dw - 1, y + dh - 1); // Sets CS low and sent RAMWR

    // Line buffer makes plotting faster
    uint16_t    lineBuf[dw];

    if (bpp8)
    {
        tft_dev->_swapBytes = false;

        uint8_t    blue[] = {0, 11, 21, 31}; // blue 2 to 5 bit colour lookup table

        tft_dev->_lastColor = -1; // Set to illegal value

        // Used to store last shifted colour
        uint8_t msbColor = 0;
        uint8_t lsbColor = 0;

        data += dx + dy * w;
        while (dh--) {
            uint32_t len = dw;
            uint8_t* ptr = data;
            uint8_t* linePtr = (uint8_t*)lineBuf;

            while(len--) {
                uint32_t color = *ptr++;

                // Shifts are slow so check if colour has changed first
                if (color != tft_dev->_lastColor) {
                    //                    =====Green=====         ===============Red==============
                    msbColor = (color & 0x1C)>>2 | (color & 0xC0)>>3 | (color & 0xE0);
                    //                    =====Green=====        =======Blue======
                    lsbColor = (color & 0x1C)<<3 | blue[color & 0x03];
                    tft_dev->_lastColor = color;
                }

             *linePtr++ = msbColor;
             *linePtr++ = lsbColor;
            }

            tft_dev->pushPixels(tft_dev, lineBuf, dw);

            data += w;
        }
        tft_dev->_swapBytes = swap; // Restore old value
    }
    else if (cmap != NULL) // Must be 4bpp
    {
        tft_dev->_swapBytes = true;

        w = (w+1) & 0xFFFE;     // if this is a sprite, w will already be even; this does no harm.
        bool splitFirst = (dx & 0x01) != 0; // split first means we have to push a single px from the left of the sprite / image

        if (splitFirst) {
            data += ((dx - 1 + dy * w) >> 1);
        }
        else {
            data += ((dx + dy * w) >> 1);
        }

        while (dh--) {
            uint32_t len = dw;
            uint8_t * ptr = data;
            uint16_t *linePtr = lineBuf;
            uint8_t colors; // two colors in one byte
            uint16_t index;

            if (splitFirst) {
                colors = *ptr;
                index = (colors & 0x0F);
                *linePtr++ = cmap[index];
                len--;
                ptr++;
            }

            while (len--)
            {
                colors = *ptr;
                index = ((colors & 0xF0) >> 4) & 0x0F;
                *linePtr++ = cmap[index];

                if (len--)
                {
                    index = colors & 0x0F;
                    *linePtr++ = cmap[index];
                } else {
                    break;    // nothing to do here
                }

                ptr++;
            }

            tft_dev->pushPixels(tft_dev, lineBuf, dw);
            data += (w >> 1);
        }
        tft_dev->_swapBytes = swap; // Restore old value
    }
    else // Must be 1bpp
    {
        tft_dev->_swapBytes = false;

        uint32_t ww =    (w+7)>>3; // Width of source image line in bytes
        for (int32_t yp = dy;    yp < dy + dh; yp++)
        {
            uint8_t* linePtr = (uint8_t*)lineBuf;
            for (int32_t xp = dx; xp < dx + dw; xp++)
            {
                uint16_t col = (data[(xp>>3)] & (0x80 >> (xp & 0x7)) );
                if (col) {*linePtr++ = tft_dev->bitmap_fg>>8; *linePtr++ = (uint8_t) tft_dev->bitmap_fg;}
                else         {*linePtr++ = tft_dev->bitmap_bg>>8; *linePtr++ = (uint8_t) tft_dev->bitmap_bg;}
            }
            data += ww;
            tft_dev->pushPixels(tft_dev, lineBuf, dw);
        }
    }

    tft_dev->_swapBytes = swap; // Restore old value
    tft_dev->inTransaction = tft_dev->lockTransaction;
    tft_dev->end_tft_write(tft_dev);
}


/***************************************************************************************
** Function name:                     pushImage
** Description:                         plot 8 or 4 or 1 bit image or sprite with a transparent colour
***************************************************************************************/
static void pushImageBpp8Trans(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, uint8_t *data, uint8_t transp, bool bpp8, uint16_t *cmap)
{
    PI_CLIP(tft_dev);

    tft_dev->begin_tft_write(tft_dev);
    tft_dev->inTransaction = true;
    bool swap = tft_dev->_swapBytes;


    // Line buffer makes plotting faster
    uint16_t    lineBuf[dw];

    if (bpp8) { // 8 bits per pixel
        tft_dev->_swapBytes = false;

        data += dx + dy * w;

        uint8_t    blue[] = {0, 11, 21, 31}; // blue 2 to 5 bit colour lookup table

        tft_dev->_lastColor = -1; // Set to illegal value

        // Used to store last shifted colour
        uint8_t msbColor = 0;
        uint8_t lsbColor = 0;

        while (dh--) {
            int32_t len = dw;
            uint8_t* ptr = data;
            uint8_t* linePtr = (uint8_t*)lineBuf;

            int32_t px = x, sx = x;
            bool move = true;
            uint16_t np = 0;

            while (len--) {
                if (transp != *ptr) {
                    if (move) { move = false; sx = px; }
                    uint8_t color = *ptr;

                    // Shifts are slow so check if colour has changed first
                    if (color != tft_dev->_lastColor) {
                        //                    =====Green=====         ===============Red==============
                        msbColor = (color & 0x1C)>>2 | (color & 0xC0)>>3 | (color & 0xE0);
                        //                    =====Green=====        =======Blue======
                        lsbColor = (color & 0x1C)<<3 | blue[color & 0x03];
                        tft_dev->_lastColor = color;
                    }
                    *linePtr++ = msbColor;
                    *linePtr++ = lsbColor;
                    np++;
                }
                else {
                    move = true;
                    if (np) {
                        tft_dev->setWindow(tft_dev, sx, y, sx + np - 1, y);
                        tft_dev->pushPixels(tft_dev, lineBuf, np);
                        linePtr = (uint8_t*)lineBuf;
                        np = 0;
                    }
                }
                px++;
                ptr++;
            }

            if (np) { tft_dev->setWindow(tft_dev, sx, y, sx + np - 1, y); tft_dev->pushPixels(tft_dev, lineBuf, np); }
            y++;
            data += w;
        }
    }
    else if (cmap != NULL) // 4bpp with color map
    {
        tft_dev->_swapBytes = true;

        w = (w+1) & 0xFFFE; // here we try to recreate iwidth from dwidth.
        bool splitFirst = ((dx & 0x01) != 0);
        if (splitFirst) {
            data += ((dx - 1 + dy * w) >> 1);
        }
        else {
            data += ((dx + dy * w) >> 1);
        }

        while (dh--) {
            uint32_t len = dw;
            uint8_t * ptr = data;

            int32_t px = x, sx = x;
            bool move = true;
            uint16_t np = 0;

            uint8_t index;    // index into cmap.

            if (splitFirst) {
                index = (*ptr & 0x0F);    // odd = bits 3 .. 0
                if (index != transp) {
                    move = false; sx = px;
                    lineBuf[np] = cmap[index];
                    np++;
                }
                px++; ptr++;
                len--;
            }

            while (len--)
            {
                uint8_t color = *ptr;

                // find the actual color you care about.    There will be two pixels here!
                // but we may only want one at the end of the row
                uint16_t index = ((color & 0xF0) >> 4) & 0x0F;    // high bits are the even numbers
                if (index != transp) {
                    if (move) {
                        move = false; sx = px;
                    }
                    lineBuf[np] = cmap[index];
                    np++; // added a pixel
                }
                else {
                    move = true;
                    if (np) {
                        tft_dev->setWindow(tft_dev, sx, y, sx + np - 1, y);
                        tft_dev->pushPixels(tft_dev, lineBuf, np);
                        np = 0;
                    }
                }
                px++;

                if (len--)
                {
                    index = color & 0x0F; // the odd number is 3 .. 0
                    if (index != transp) {
                        if (move) {
                            move = false; sx = px;
                         }
                        lineBuf[np] = cmap[index];
                        np++;
                    }
                    else {
                        move = true;
                        if (np) {
                            tft_dev->setWindow(tft_dev, sx, y, sx + np - 1, y);
                            tft_dev->pushPixels(tft_dev, lineBuf, np);
                            np = 0;
                        }
                    }
                    px++;
                }
                else {
                    break;    // we are done with this row.
                }
                ptr++;    // we only increment ptr once in the loop (deliberate)
            }

            if (np) {
                tft_dev->setWindow(tft_dev, sx, y, sx + np - 1, y);
                tft_dev->pushPixels(tft_dev, lineBuf, np);
                np = 0;
            }
            data += (w>>1);
            y++;
        }
    }
    else { // 1 bit per pixel
        tft_dev->_swapBytes = false;

        uint32_t ww =    (w+7)>>3; // Width of source image line in bytes
        uint16_t np = 0;

        for (int32_t yp = dy;    yp < dy + dh; yp++)
        {
            int32_t px = x, sx = x;
            bool move = true;
            for (int32_t xp = dx; xp < dx + dw; xp++)
            {
                if (data[(xp>>3)] & (0x80 >> (xp & 0x7))) {
                    if (move) {
                        move = false;
                        sx = px;
                    }
                    np++;
                }
                else {
                    move = true;
                    if (np) {
                        tft_dev->setWindow(tft_dev, sx, y, sx + np - 1, y);
                        tft_dev->pushBlock(tft_dev, tft_dev->bitmap_fg, np);
                        np = 0;
                    }
                }
                px++;
            }
            if (np) { tft_dev->setWindow(tft_dev, sx, y, sx + np - 1, y); tft_dev->pushBlock(tft_dev, tft_dev->bitmap_fg, np); np = 0; }
            y++;
            data += ww;
        }
    }
    tft_dev->_swapBytes = swap; // Restore old value
    tft_dev->inTransaction = tft_dev->lockTransaction;
    tft_dev->end_tft_write(tft_dev);
}


/***************************************************************************************
** Function name:                     setSwapBytes
** Description:                         Used by 16 bit pushImage() to swap byte order in colours
***************************************************************************************/
static void setSwapBytes(struct TFT_eSPI * tft_dev, bool swap)
{
    tft_dev->_swapBytes = swap;
}


/***************************************************************************************
** Function name:                     getSwapBytes
** Description:                         Return the swap byte order for colours
***************************************************************************************/
static bool getSwapBytes(struct TFT_eSPI * tft_dev)
{
    return tft_dev->_swapBytes;
}


/***************************************************************************************
** Function name:                     read rectangle (for SPI Interface II i.e. IM [3:0] = "1101")
** Description:                         Read RGB pixel colours from a defined area
***************************************************************************************/
// If w and h are 1, then 1 pixel is read, *data array size must be 3 bytes per pixel
static void readRectRGB(struct TFT_eSPI * tft_dev, int32_t x0, int32_t y0, int32_t w, int32_t h, uint8_t *data)
{
#if defined(TFT_PARALLEL_8_BIT) || defined(RP2040_PIO_INTERFACE)

    uint32_t len = w * h;
    uint8_t* buf565 = data + len;

    readRect(x0, y0, w, h, (uint16_t*)buf565);

    while (len--) {
        uint16_t pixel565 = (*buf565++)<<8;
        pixel565 |= *buf565++;
        uint8_t red     = (pixel565 & 0xF800) >> 8; red     |= red     >> 5;
        uint8_t green = (pixel565 & 0x07E0) >> 3; green |= green >> 6;
        uint8_t blue    = (pixel565 & 0x001F) << 3; blue    |= blue    >> 5;
        *data++ = red;
        *data++ = green;
        *data++ = blue;
    }

#else    // Not TFT_PARALLEL_8_BIT

    tft_dev->begin_tft_read(tft_dev);

    tft_dev->readAddrWindow(tft_dev, x0, y0, w, h); // Sets CS low

    #ifdef TFT_SDA_READ
        begin_SDA_Read();
    #endif

    // Dummy read to throw away don't care value
    uint8_t tmp;
    tft_Read_8(&tmp);

    // Read window pixel 24 bit RGB values, buffer must be set in sketch to 3 * w * h
    uint32_t len = w * h;
    while (len--) {

    #if !defined (ILI9488_DRIVER)

        // Read the 3 RGB bytes, colour is actually only in the top 6 bits of each byte
        // as the TFT stores colours as 18 bits
        tft_Read_8(data++);
        tft_Read_8(data++);
        tft_Read_8(data++);

    #else

        // The 6 colour bits are in MS 6 bits of each byte, but the ILI9488 needs an extra clock pulse
        // so bits appear shifted right 1 bit, so mask the middle 6 bits then shift 1 place left
        *data++ = (tft_Read_8()&0x7E)<<1;
        *data++ = (tft_Read_8()&0x7E)<<1;
        *data++ = (tft_Read_8()&0x7E)<<1;

    #endif

    }

    CS_H;

    #ifdef TFT_SDA_READ
        end_SDA_Read();
    #endif

    tft_dev->end_tft_read(tft_dev);

#endif
}


/***************************************************************************************
** Function name:                     drawCircle
** Description:                         Draw a circle outline
***************************************************************************************/
// Optimised midpoint circle algorithm
static void drawCircle(struct TFT_eSPI * tft_dev, int32_t x0, int32_t y0, int32_t r, uint32_t color)
{
    if ( r <= 0 ) return;

    //begin_tft_write();                    // Sprite class can use this function, avoiding begin_tft_write()
    tft_dev->inTransaction = true;

        int32_t f         = 1 - r;
        int32_t ddF_y = -2 * r;
        int32_t ddF_x = 1;
        int32_t xs        = -1;
        int32_t xe        = 0;
        int32_t len     = 0;

        bool first = true;
        do {
            while (f < 0) {
                ++xe;
                f += (ddF_x += 2);
            }
            f += (ddF_y += 2);

            if (xe-xs>1) {
                if (first) {
                    len = 2*(xe - xs)-1;
                    tft_dev->drawFastHLine(tft_dev, x0 - xe, y0 + r, len, color);
                    tft_dev->drawFastHLine(tft_dev, x0 - xe, y0 - r, len, color);
                    tft_dev->drawFastVLine(tft_dev, x0 + r, y0 - xe, len, color);
                    tft_dev->drawFastVLine(tft_dev, x0 - r, y0 - xe, len, color);
                    first = false;
                }
                else {
                    len = xe - xs++;
                    tft_dev->drawFastHLine(tft_dev, x0 - xe, y0 + r, len, color);
                    tft_dev->drawFastHLine(tft_dev, x0 - xe, y0 - r, len, color);
                    tft_dev->drawFastHLine(tft_dev, x0 + xs, y0 - r, len, color);
                    tft_dev->drawFastHLine(tft_dev, x0 + xs, y0 + r, len, color);

                    tft_dev->drawFastVLine(tft_dev, x0 + r, y0 + xs, len, color);
                    tft_dev->drawFastVLine(tft_dev, x0 + r, y0 - xe, len, color);
                    tft_dev->drawFastVLine(tft_dev, x0 - r, y0 - xe, len, color);
                    tft_dev->drawFastVLine(tft_dev, x0 - r, y0 + xs, len, color);
                }
            }
            else {
                ++xs;
                tft_dev->drawPixel(tft_dev, x0 - xe, y0 + r, color);
                tft_dev->drawPixel(tft_dev, x0 - xe, y0 - r, color);
                tft_dev->drawPixel(tft_dev, x0 + xs, y0 - r, color);
                tft_dev->drawPixel(tft_dev, x0 + xs, y0 + r, color);

                tft_dev->drawPixel(tft_dev, x0 + r, y0 + xs, color);
                tft_dev->drawPixel(tft_dev, x0 + r, y0 - xe, color);
                tft_dev->drawPixel(tft_dev, x0 - r, y0 - xe, color);
                tft_dev->drawPixel(tft_dev, x0 - r, y0 + xs, color);
            }
            xs = xe;
        } while (xe < --r);

    tft_dev->inTransaction = tft_dev->lockTransaction;
    tft_dev->end_tft_write(tft_dev);                            // Does nothing if Sprite class uses this function
}


/***************************************************************************************
** Function name:                     drawCircleHelper
** Description:                         Support function for drawRoundRect()
***************************************************************************************/
static void drawCircleHelper(struct TFT_eSPI * tft_dev, int32_t x0, int32_t y0, int32_t rr, uint8_t cornername, uint32_t color)
{
    if (rr <= 0) return;
    int32_t f         = 1 - rr;
    int32_t ddF_x = 1;
    int32_t ddF_y = -2 * rr;
    int32_t xe        = 0;
    int32_t xs        = 0;
    int32_t len     = 0;

    //begin_tft_write();                    // Sprite class can use this function, avoiding begin_tft_write()
    tft_dev->inTransaction = true;

    while (xe < rr--)
    {
        while (f < 0) {
            ++xe;
            f += (ddF_x += 2);
        }
        f += (ddF_y += 2);

        if (xe-xs==1) {
            if (cornername & 0x1) { // left top
                tft_dev->drawPixel(tft_dev, x0 - xe, y0 - rr, color);
                tft_dev->drawPixel(tft_dev, x0 - rr, y0 - xe, color);
            }
            if (cornername & 0x2) { // right top
                tft_dev->drawPixel(tft_dev, x0 + rr        , y0 - xe, color);
                tft_dev->drawPixel(tft_dev, x0 + xs + 1, y0 - rr, color);
            }
            if (cornername & 0x4) { // right bottom
                tft_dev->drawPixel(tft_dev, x0 + xs + 1, y0 + rr        , color);
                tft_dev->drawPixel(tft_dev, x0 + rr, y0 + xs + 1, color);
            }
            if (cornername & 0x8) { // left bottom
                tft_dev->drawPixel(tft_dev, x0 - rr, y0 + xs + 1, color);
                tft_dev->drawPixel(tft_dev, x0 - xe, y0 + rr        , color);
            }
        }
        else {
            len = xe - xs++;
            if (cornername & 0x1) { // left top
                tft_dev->drawFastHLine(tft_dev, x0 - xe, y0 - rr, len, color);
                tft_dev->drawFastVLine(tft_dev, x0 - rr, y0 - xe, len, color);
            }
            if (cornername & 0x2) { // right top
                tft_dev->drawFastVLine(tft_dev, x0 + rr, y0 - xe, len, color);
                tft_dev->drawFastHLine(tft_dev, x0 + xs, y0 - rr, len, color);
            }
            if (cornername & 0x4) { // right bottom
                tft_dev->drawFastHLine(tft_dev, x0 + xs, y0 + rr, len, color);
                tft_dev->drawFastVLine(tft_dev, x0 + rr, y0 + xs, len, color);
            }
            if (cornername & 0x8) { // left bottom
                tft_dev->drawFastVLine(tft_dev, x0 - rr, y0 + xs, len, color);
                tft_dev->drawFastHLine(tft_dev, x0 - xe, y0 + rr, len, color);
            }
        }
        xs = xe;
    }
    tft_dev->inTransaction = tft_dev->lockTransaction;
    tft_dev->end_tft_write(tft_dev);                            // Does nothing if Sprite class uses this function
}

/***************************************************************************************
** Function name:                     fillCircle
** Description:                         draw a filled circle
***************************************************************************************/
// Optimised midpoint circle algorithm, changed to horizontal lines (faster in sprites)
// Improved algorithm avoids repetition of lines
static void fillCircle(struct TFT_eSPI * tft_dev, int32_t x0, int32_t y0, int32_t r, uint32_t color)
{
    int32_t    x    = 0;
    int32_t    dx = 1;
    int32_t    dy = r+r;
    int32_t    p    = -(r>>1);

    //begin_tft_write();                    // Sprite class can use this function, avoiding begin_tft_write()
    tft_dev->inTransaction = true;

    tft_dev->drawFastHLine(tft_dev, x0 - r, y0, dy+1, color);

    while(x<r){

        if(p>=0) {
            tft_dev->drawFastHLine(tft_dev, x0 - x, y0 + r, dx, color);
            tft_dev->drawFastHLine(tft_dev, x0 - x, y0 - r, dx, color);
            dy-=2;
            p-=dy;
            r--;
        }

        dx+=2;
        p+=dx;
        x++;

        tft_dev->drawFastHLine(tft_dev, x0 - r, y0 + x, dy+1, color);
        tft_dev->drawFastHLine(tft_dev, x0 - r, y0 - x, dy+1, color);

    }

    tft_dev->inTransaction = tft_dev->lockTransaction;
    tft_dev->end_tft_write(tft_dev);                            // Does nothing if Sprite class uses this function
}

/***************************************************************************************
** Function name:                     fillCircleHelper
** Description:                         Support function for fillRoundRect()
***************************************************************************************/
// Support drawing roundrects, changed to horizontal lines (faster in sprites)
static void fillCircleHelper(struct TFT_eSPI * tft_dev, int32_t x0, int32_t y0, int32_t r, uint8_t cornername, int32_t delta, uint32_t color)
{
    int32_t f     = 1 - r;
    int32_t ddF_x = 1;
    int32_t ddF_y = -r - r;
    int32_t y     = 0;

    delta++;

    while (y < r) {
        if (f >= 0) {
            if (cornername & 0x1) tft_dev->drawFastHLine(tft_dev, x0 - y, y0 + r, y + y + delta, color);
            if (cornername & 0x2) tft_dev->drawFastHLine(tft_dev, x0 - y, y0 - r, y + y + delta, color);
            r--;
            ddF_y += 2;
            f     += ddF_y;
        }

        y++;
        ddF_x += 2;
        f     += ddF_x;

        if (cornername & 0x1) tft_dev->drawFastHLine(tft_dev, x0 - r, y0 + y, r + r + delta, color);
        if (cornername & 0x2) tft_dev->drawFastHLine(tft_dev, x0 - r, y0 - y, r + r + delta, color);
    }
}


/***************************************************************************************
** Function name:                     drawEllipse
** Description:                         Draw a ellipse outline
***************************************************************************************/
static void drawEllipse(struct TFT_eSPI * tft_dev, int16_t x0, int16_t y0, int32_t rx, int32_t ry, uint16_t color)
{
    if (rx<2) return;
    if (ry<2) return;
    int32_t x, y;
    int32_t rx2 = rx * rx;
    int32_t ry2 = ry * ry;
    int32_t fx2 = 4 * rx2;
    int32_t fy2 = 4 * ry2;
    int32_t s;

    //begin_tft_write();                    // Sprite class can use this function, avoiding begin_tft_write()
    tft_dev->inTransaction = true;

    for (x = 0, y = ry, s = 2*ry2+rx2*(1-2*ry); ry2*x <= rx2*y; x++) {
        // These are ordered to minimise coordinate changes in x or y
        // drawPixel can then send fewer bounding box commands
        tft_dev->drawPixel(tft_dev, x0 + x, y0 + y, color);
        tft_dev->drawPixel(tft_dev, x0 - x, y0 + y, color);
        tft_dev->drawPixel(tft_dev, x0 - x, y0 - y, color);
        tft_dev->drawPixel(tft_dev, x0 + x, y0 - y, color);
        if (s >= 0) {
            s += fx2 * (1 - y);
            y--;
        }
        s += ry2 * ((4 * x) + 6);
    }

    for (x = rx, y = 0, s = 2*rx2+ry2*(1-2*rx); rx2*y <= ry2*x; y++) {
        // These are ordered to minimise coordinate changes in x or y
        // drawPixel can then send fewer bounding box commands
        tft_dev->drawPixel(tft_dev, x0 + x, y0 + y, color);
        tft_dev->drawPixel(tft_dev, x0 - x, y0 + y, color);
        tft_dev->drawPixel(tft_dev, x0 - x, y0 - y, color);
        tft_dev->drawPixel(tft_dev, x0 + x, y0 - y, color);
        if (s >= 0)
        {
            s += fy2 * (1 - x);
            x--;
        }
        s += rx2 * ((4 * y) + 6);
    }

    tft_dev->inTransaction = tft_dev->lockTransaction;
    tft_dev->end_tft_write(tft_dev);                            // Does nothing if Sprite class uses this function
}


/***************************************************************************************
** Function name:                     fillEllipse
** Description:                         draw a filled ellipse
***************************************************************************************/
static void fillEllipse(struct TFT_eSPI * tft_dev, int16_t x0, int16_t y0, int32_t rx, int32_t ry, uint16_t color)
{
    if (rx<2) return;
    if (ry<2) return;
    int32_t x, y;
    int32_t rx2 = rx * rx;
    int32_t ry2 = ry * ry;
    int32_t fx2 = 4 * rx2;
    int32_t fy2 = 4 * ry2;
    int32_t s;

    //begin_tft_write();                    // Sprite class can use this function, avoiding begin_tft_write()
    tft_dev->inTransaction = true;

    for (x = 0, y = ry, s = 2*ry2+rx2*(1-2*ry); ry2*x <= rx2*y; x++) {
        tft_dev->drawFastHLine(tft_dev, x0 - x, y0 - y, x + x + 1, color);
        tft_dev->drawFastHLine(tft_dev, x0 - x, y0 + y, x + x + 1, color);

        if (s >= 0) {
            s += fx2 * (1 - y);
            y--;
        }
        s += ry2 * ((4 * x) + 6);
    }

    for (x = rx, y = 0, s = 2*rx2+ry2*(1-2*rx); rx2*y <= ry2*x; y++) {
        tft_dev->drawFastHLine(tft_dev, x0 - x, y0 - y, x + x + 1, color);
        tft_dev->drawFastHLine(tft_dev, x0 - x, y0 + y, x + x + 1, color);

        if (s >= 0) {
            s += fy2 * (1 - x);
            x--;
        }
        s += rx2 * ((4 * y) + 6);
    }

    tft_dev->inTransaction = tft_dev->lockTransaction;
    tft_dev->end_tft_write(tft_dev);                            // Does nothing if Sprite class uses this function
}


/***************************************************************************************
** Function name:                     fillScreen
** Description:                         Clear the screen to defined colour
***************************************************************************************/
static void fillScreen(struct TFT_eSPI * tft_dev, uint32_t color)
{
    tft_dev->fillRect(tft_dev, 0, 0, tft_dev->_width, tft_dev->_height, color);
}


/***************************************************************************************
** Function name:                     drawRect
** Description:                         Draw a rectangle outline
***************************************************************************************/
// Draw a rectangle
static void drawRect(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, uint32_t color)
{
    //begin_tft_write();                    // Sprite class can use this function, avoiding begin_tft_write()
    tft_dev->inTransaction = true;

    tft_dev->drawFastHLine(tft_dev, x, y, w, color);
    tft_dev->drawFastHLine(tft_dev, x, y + h - 1, w, color);
    // Avoid drawing corner pixels twice
    tft_dev->drawFastVLine(tft_dev, x, y+1, h-2, color);
    tft_dev->drawFastVLine(tft_dev, x + w - 1, y+1, h-2, color);

    tft_dev->inTransaction = tft_dev->lockTransaction;
    tft_dev->end_tft_write(tft_dev);                            // Does nothing if Sprite class uses this function
}


/***************************************************************************************
** Function name:                     drawRoundRect
** Description:                         Draw a rounded corner rectangle outline
***************************************************************************************/
// Draw a rounded rectangle
static void drawRoundRect(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, int32_t r, uint32_t color)
{
    //begin_tft_write();                    // Sprite class can use this function, avoiding begin_tft_write()
    tft_dev->inTransaction = true;

    // smarter version
    tft_dev->drawFastHLine(tft_dev, x + r    , y        , w - r - r, color); // Top
    tft_dev->drawFastHLine(tft_dev, x + r    , y + h - 1, w - r - r, color); // Bottom
    tft_dev->drawFastVLine(tft_dev, x        , y + r    , h - r - r, color); // Left
    tft_dev->drawFastVLine(tft_dev, x + w - 1, y + r    , h - r - r, color); // Right
    // draw four corners
    tft_dev->drawCircleHelper(tft_dev, x + r        , y + r        , r, 1, color);
    tft_dev->drawCircleHelper(tft_dev, x + w - r - 1, y + r        , r, 2, color);
    tft_dev->drawCircleHelper(tft_dev, x + w - r - 1, y + h - r - 1, r, 4, color);
    tft_dev->drawCircleHelper(tft_dev, x + r        , y + h - r - 1, r, 8, color);

    tft_dev->inTransaction = tft_dev->lockTransaction;
    tft_dev->end_tft_write(tft_dev);                            // Does nothing if Sprite class uses this function
}


/***************************************************************************************
** Function name:                     fillRoundRect
** Description:                         Draw a rounded corner filled rectangle
***************************************************************************************/
// Fill a rounded rectangle, changed to horizontal lines (faster in sprites)
static void fillRoundRect(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, int32_t r, uint32_t color)
{
    //begin_tft_write();                    // Sprite class can use this function, avoiding begin_tft_write()
    tft_dev->inTransaction = true;

    // smarter version
    tft_dev->fillRect(tft_dev, x, y + r, w, h - r - r, color);

    // draw four corners
    tft_dev->fillCircleHelper(tft_dev, x + r, y + h - r - 1, r, 1, w - r - r - 1, color);
    tft_dev->fillCircleHelper(tft_dev, x + r        , y + r, r, 2, w - r - r - 1, color);

    tft_dev->inTransaction = tft_dev->lockTransaction;
    tft_dev->end_tft_write(tft_dev);                            // Does nothing if Sprite class uses this function
}


/***************************************************************************************
** Function name:                     drawTriangle
** Description:                         Draw a triangle outline using 3 arbitrary points
***************************************************************************************/
// Draw a triangle
static void drawTriangle(struct TFT_eSPI * tft_dev, int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t x2, int32_t y2, uint32_t color)
{
    //begin_tft_write();                    // Sprite class can use this function, avoiding begin_tft_write()
    tft_dev->inTransaction = true;

    tft_dev->drawLine(tft_dev, x0, y0, x1, y1, color);
    tft_dev->drawLine(tft_dev, x1, y1, x2, y2, color);
    tft_dev->drawLine(tft_dev, x2, y2, x0, y0, color);

    tft_dev->inTransaction = tft_dev->lockTransaction;
    tft_dev->end_tft_write(tft_dev);                            // Does nothing if Sprite class uses this function
}


/***************************************************************************************
** Function name:                     fillTriangle
** Description:                         Draw a filled triangle using 3 arbitrary points
***************************************************************************************/
// Fill a triangle - original Adafruit function works well and code footprint is small
static void fillTriangle(struct TFT_eSPI * tft_dev, int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t x2, int32_t y2, uint32_t color)
{
    int32_t a, b, y, last;

    // Sort coordinates by Y order (y2 >= y1 >= y0)
    if (y0 > y1) {
        swap_coord(&y0, &y1); swap_coord(&x0, &x1);
    }
    if (y1 > y2) {
        swap_coord(&y2, &y1); swap_coord(&x2, &x1);
    }
    if (y0 > y1) {
        swap_coord(&y0, &y1); swap_coord(&x0, &x1);
    }

    if (y0 == y2) { // Handle awkward all-on-same-line case as its own thing
        a = b = x0;
        if (x1 < a)            a = x1;
        else if (x1 > b) b = x1;
        if (x2 < a)            a = x2;
        else if (x2 > b) b = x2;
        tft_dev->drawFastHLine(tft_dev, a, y0, b - a + 1, color);
        return;
    }

    //begin_tft_write();                    // Sprite class can use this function, avoiding begin_tft_write()
    tft_dev->inTransaction = true;

    int32_t
    dx01 = x1 - x0,
    dy01 = y1 - y0,
    dx02 = x2 - x0,
    dy02 = y2 - y0,
    dx12 = x2 - x1,
    dy12 = y2 - y1,
    sa     = 0,
    sb     = 0;

    // For upper part of triangle, find scanline crossings for segments
    // 0-1 and 0-2.    If y1=y2 (flat-bottomed triangle), the scanline y1
    // is included here (and second loop will be skipped, avoiding a /0
    // error there), otherwise scanline y1 is skipped here and handled
    // in the second loop...which also avoids a /0 error here if y0=y1
    // (flat-topped triangle).
    if (y1 == y2) last = y1;    // Include y1 scanline
    else                 last = y1 - 1; // Skip it

    for (y = y0; y <= last; y++) {
        a     = x0 + sa / dy01;
        b     = x0 + sb / dy02;
        sa += dx01;
        sb += dx02;

        if (a > b) swap_coord(&a, &b);
        tft_dev->drawFastHLine(tft_dev, a, y, b - a + 1, color);
    }

    // For lower part of triangle, find scanline crossings for segments
    // 0-2 and 1-2.    This loop is skipped if y1=y2.
    sa = dx12 * (y - y1);
    sb = dx02 * (y - y0);
    for (; y <= y2; y++) {
        a     = x1 + sa / dy12;
        b     = x0 + sb / dy02;
        sa += dx12;
        sb += dx02;

        if (a > b) swap_coord(&a, &b);
        tft_dev->drawFastHLine(tft_dev, a, y, b - a + 1, color);
    }

    tft_dev->inTransaction = tft_dev->lockTransaction;
    tft_dev->end_tft_write(tft_dev);                            // Does nothing if Sprite class uses this function
}


/***************************************************************************************
** Function name:                     drawBitmap
** Description:                         Draw an image stored in an array on the TFT
***************************************************************************************/
static void drawBitmap(struct TFT_eSPI * tft_dev, int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color)
{
    //begin_tft_write();                    // Sprite class can use this function, avoiding begin_tft_write()
    tft_dev->inTransaction = true;

    int32_t i, j, byteWidth = (w + 7) / 8;

    for (j = 0; j < h; j++) {
        for (i = 0; i < w; i++ ) {
            if (pgm_read_byte(bitmap + j * byteWidth + i / 8) & (128 >> (i & 7))) {
                tft_dev->drawPixel(tft_dev, x + i, y + j, color);
            }
        }
    }

    tft_dev->inTransaction = tft_dev->lockTransaction;
    tft_dev->end_tft_write(tft_dev);                            // Does nothing if Sprite class uses this function
}


/***************************************************************************************
** Function name:                     drawBitmap
** Description:                         Draw an image stored in an array on the TFT
***************************************************************************************/
static void drawBitmapBg(struct TFT_eSPI * tft_dev, int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t fgcolor, uint16_t bgcolor)
{
    //begin_tft_write();                    // Sprite class can use this function, avoiding begin_tft_write()
    tft_dev->inTransaction = true;

    int32_t i, j, byteWidth = (w + 7) / 8;

    for (j = 0; j < h; j++) {
        for (i = 0; i < w; i++ ) {
            if (pgm_read_byte(bitmap + j * byteWidth + i / 8) & (128 >> (i & 7)))
                tft_dev->drawPixel(tft_dev, x + i, y + j, fgcolor);
            else
                tft_dev->drawPixel(tft_dev, x + i, y + j, bgcolor);
        }
    }

    tft_dev->inTransaction = tft_dev->lockTransaction;
    tft_dev->end_tft_write(tft_dev);                            // Does nothing if Sprite class uses this function
}

/***************************************************************************************
** Function name:                     drawXBitmap
** Description:                         Draw an image stored in an XBM array onto the TFT
***************************************************************************************/
static void drawXBitmap(struct TFT_eSPI * tft_dev, int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color)
{
    //begin_tft_write();                    // Sprite class can use this function, avoiding begin_tft_write()
    tft_dev->inTransaction = true;

    int32_t i, j, byteWidth = (w + 7) / 8;

    for (j = 0; j < h; j++) {
        for (i = 0; i < w; i++ ) {
            if (pgm_read_byte(bitmap + j * byteWidth + i / 8) & (1 << (i & 7))) {
                tft_dev->drawPixel(tft_dev, x + i, y + j, color);
            }
        }
    }

    tft_dev->inTransaction = tft_dev->lockTransaction;
    tft_dev->end_tft_write(tft_dev);                            // Does nothing if Sprite class uses this function
}


/***************************************************************************************
** Function name:                     drawXBitmap
** Description:                         Draw an XBM image with foreground and background colors
***************************************************************************************/
static void drawXBitmapBg(struct TFT_eSPI * tft_dev, int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color, uint16_t bgcolor)
{
    //begin_tft_write();                    // Sprite class can use this function, avoiding begin_tft_write()
    tft_dev->inTransaction = true;

    int32_t i, j, byteWidth = (w + 7) / 8;

    for (j = 0; j < h; j++) {
        for (i = 0; i < w; i++ ) {
            if (pgm_read_byte(bitmap + j * byteWidth + i / 8) & (1 << (i & 7)))
                tft_dev->drawPixel(tft_dev, x + i, y + j,     color);
            else
                tft_dev->drawPixel(tft_dev, x + i, y + j, bgcolor);
        }
    }

    tft_dev->inTransaction = tft_dev->lockTransaction;
    tft_dev->end_tft_write(tft_dev);                            // Does nothing if Sprite class uses this function
}


/***************************************************************************************
** Function name:                     setCursor
** Description:                         Set the text cursor x,y position
***************************************************************************************/
static void setCursor(struct TFT_eSPI * tft_dev, int16_t x, int16_t y)
{
    tft_dev->cursor_x = x;
    tft_dev->cursor_y = y;
}


/***************************************************************************************
** Function name:                     setCursor
** Description:                         Set the text cursor x,y position and font
***************************************************************************************/
static void setCursorFont(struct TFT_eSPI * tft_dev, int16_t x, int16_t y, uint8_t font)
{
    tft_dev->textfont = font;
    tft_dev->cursor_x = x;
    tft_dev->cursor_y = y;
}


/***************************************************************************************
** Function name:                     getCursorX
** Description:                         Get the text cursor x position
***************************************************************************************/
static int16_t getCursorX(struct TFT_eSPI * tft_dev)
{
    return tft_dev->cursor_x;
}

/***************************************************************************************
** Function name:                     getCursorY
** Description:                         Get the text cursor y position
***************************************************************************************/
static int16_t getCursorY(struct TFT_eSPI * tft_dev)
{
    return tft_dev->cursor_y;
}


/***************************************************************************************
** Function name:                     setTextSize
** Description:                         Set the text size multiplier
***************************************************************************************/
static void setTextSize(struct TFT_eSPI * tft_dev, uint8_t s)
{
    if (s>7) s = 7; // Limit the maximum size multiplier so byte variables can be used for rendering
    tft_dev->textsize = (s > 0) ? s : 1; // Don't allow font size 0
}


/***************************************************************************************
** Function name:                     setTextColor
** Description:                         Set the font foreground colour (background is transparent)
***************************************************************************************/
static void setTextColor(struct TFT_eSPI * tft_dev, uint16_t c)
{
    // For 'transparent' background, we'll set the bg
    // to the same as fg instead of using a flag
    tft_dev->textcolor = tft_dev->textbgcolor = c;
}


/***************************************************************************************
** Function name:                     setTextColor
** Description:                         Set the font foreground and background colour
***************************************************************************************/
// Smooth fonts use the background colour for anti-aliasing and by default the
// background is not filled. If bgfill = true, then a smooth font background fill will
// be used.
static void setTextColorFill(struct TFT_eSPI * tft_dev, uint16_t c, uint16_t b, bool bgfill)
{
    tft_dev->textcolor     = c;
    tft_dev->textbgcolor = b;
    tft_dev->_fillbg         = bgfill;
}


/***************************************************************************************
** Function name:                     setPivot
** Description:                         Set the pivot point on the TFT
*************************************************************************************x*/
static void setPivot(struct TFT_eSPI * tft_dev, int16_t x, int16_t y)
{
    tft_dev->_xPivot = x;
    tft_dev->_yPivot = y;
}


/***************************************************************************************
** Function name:                     getPivotX
** Description:                         Get the x pivot position
***************************************************************************************/
static int16_t getPivotX(struct TFT_eSPI * tft_dev)
{
    return tft_dev->_xPivot;
}


/***************************************************************************************
** Function name:                     getPivotY
** Description:                         Get the y pivot position
***************************************************************************************/
static int16_t getPivotY(struct TFT_eSPI * tft_dev)
{
    return tft_dev->_yPivot;
}


/***************************************************************************************
** Function name:                     setBitmapColor
** Description:                         Set the foreground foreground and background colour
***************************************************************************************/
static void setBitmapColor(struct TFT_eSPI * tft_dev, uint16_t c, uint16_t b)
{
    if (c == b) b = ~c;
    tft_dev->bitmap_fg = c;
    tft_dev->bitmap_bg = b;
}


/***************************************************************************************
** Function name:                     setTextWrap
** Description:                         Define if text should wrap at end of line
***************************************************************************************/
static void setTextWrap(struct TFT_eSPI * tft_dev, bool wrapX, bool wrapY)
{
    tft_dev->textwrapX = wrapX;
    tft_dev->textwrapY = wrapY;
}


/***************************************************************************************
** Function name:                     setTextDatum
** Description:                         Set the text position reference datum
***************************************************************************************/
static void setTextDatum(struct TFT_eSPI * tft_dev, uint8_t d)
{
    tft_dev->textdatum = d;
}


/***************************************************************************************
** Function name:                     setTextPadding
** Description:                         Define padding width (aids erasing old text and numbers)
***************************************************************************************/
static void setTextPadding(struct TFT_eSPI * tft_dev, uint16_t x_width)
{
    tft_dev->padX = x_width;
}

/***************************************************************************************
** Function name:                     setTextPadding
** Description:                         Define padding width (aids erasing old text and numbers)
***************************************************************************************/
static uint16_t getTextPadding(struct TFT_eSPI * tft_dev)
{
    return tft_dev->padX;
}

/***************************************************************************************
** Function name:                     getRotation
** Description:                         Return the rotation value (as used by setRotation())
***************************************************************************************/
static uint8_t getRotation(struct TFT_eSPI * tft_dev)
{
    return tft_dev->rotation;
}

/***************************************************************************************
** Function name:                     getTextDatum
** Description:                         Return the text datum value (as used by setTextDatum())
***************************************************************************************/
static uint8_t getTextDatum(struct TFT_eSPI * tft_dev)
{
    return tft_dev->textdatum;
}


/***************************************************************************************
** Function name:                     width
** Description:                         Return the pixel width of display (per current rotation)
***************************************************************************************/
// Return the size of the display (per current rotation)
static int16_t width(struct TFT_eSPI * tft_dev)
{
    if (tft_dev->_vpDatum) return tft_dev->_xWidth;
    return tft_dev->_width;
}


/***************************************************************************************
** Function name:                     height
** Description:                         Return the pixel height of display (per current rotation)
***************************************************************************************/
static int16_t height(struct TFT_eSPI * tft_dev)
{
    if (tft_dev->_vpDatum) return tft_dev->_yHeight;
    return tft_dev->_height;
}


/***************************************************************************************
** Function name:                     textWidth
** Description:                         Return the width in pixels of a string in a given font
***************************************************************************************/
static int16_t textWidth(struct TFT_eSPI * tft_dev, const char *string)
{
    return tft_dev->textWidthFont(tft_dev, string, tft_dev->textfont);
}

static int16_t textWidthFont(struct TFT_eSPI * tft_dev, const char *string, uint8_t font)
{
    int32_t str_width = 0;
    uint16_t uniCode    = 0;

#ifdef SMOOTH_FONT
    if(fontLoaded) {
        while (*string) {
            uniCode = decodeUTF8(*string++);
            if (uniCode) {
                if (uniCode == 0x20) str_width += gFont.spaceWidth;
                else {
                    uint16_t gNum = 0;
                    bool found = getUnicodeIndex(uniCode, &gNum);
                    if (found) {
                        if(str_width == 0 && gdX[gNum] < 0) str_width -= gdX[gNum];
                        if (*string || isDigits) str_width += gxAdvance[gNum];
                        else str_width += (gdX[gNum] + gWidth[gNum]);
                    }
                    else str_width += gFont.spaceWidth + 1;
                }
            }
        }
        isDigits = false;
        return str_width;
    }
#endif

    if (font>1 && font<9) {
        char *widthtable = (char *)pgm_read_dword( &(fontdata[font].widthtbl ) ) - 32; //subtract the 32 outside the loop

        while (*string) {
            uniCode = *(string++);
            if (uniCode > 31 && uniCode < 128)
            str_width += pgm_read_byte( widthtable + uniCode); // Normally we need to subtract 32 from uniCode
            else str_width += pgm_read_byte( widthtable + 32); // Set illegal character = space width
        }

    }
    else {

#ifdef LOAD_GFXFF
        if(gfxFont) { // New font
            while (*string) {
                uniCode = decodeUTF8(*string++);
                if ((uniCode >= pgm_read_word(&gfxFont->first)) && (uniCode <= pgm_read_word(&gfxFont->last ))) {
                    uniCode -= pgm_read_word(&gfxFont->first);
                    GFXglyph *glyph    = &(((GFXglyph *)pgm_read_dword(&gfxFont->glyph))[uniCode]);
                    // If this is not the    last character or is a digit then use xAdvance
                    if (*string    || isDigits) str_width += pgm_read_byte(&glyph->xAdvance);
                    // Else use the offset plus width since this can be bigger than xAdvance
                    else str_width += ((int8_t)pgm_read_byte(&glyph->xOffset) + pgm_read_byte(&glyph->width));
                }
            }
        }
        else
#endif
        {
#ifdef LOAD_GLCD
            while (*string++) str_width += 6;
#endif
        }
    }
    tft_dev->isDigits = false;
    return str_width * tft_dev->textsize;
}


/***************************************************************************************
** Function name:                     fontsLoaded
** Description:                         return an encoded 16 bit value showing the fonts loaded
***************************************************************************************/
// Returns a value showing which fonts are loaded (bit N set =    Font N loaded)
static uint16_t fontsLoaded(struct TFT_eSPI * tft_dev)
{
    return tft_dev->fontsloaded;
}


/***************************************************************************************
** Function name:                     fontHeight
** Description:                         return the height of a font (yAdvance for free fonts)
***************************************************************************************/
static int16_t fontHeight(struct TFT_eSPI * tft_dev, int16_t font)
{
#ifdef SMOOTH_FONT
    if(fontLoaded) return gFont.yAdvance;
#endif

#ifdef LOAD_GFXFF
    if (font==1) {
        if(gfxFont) { // New font
            return pgm_read_byte(&gfxFont->yAdvance) * textsize;
        }
    }
#endif
    return pgm_read_byte( &fontdata[font].height ) * tft_dev->textsize;
}

/***************************************************************************************
** Function name:                     drawChar
** Description:                         draw a single character in the GLCD or GFXFF font
***************************************************************************************/
static void drawChar(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, uint16_t c, uint32_t color, uint32_t bg, uint8_t size)
{
    if (tft_dev->_vpOoB) return;

    if (c < 32) return;
#ifdef LOAD_GLCD
//>>>>>>>>>>>>>>>>>>
    #ifdef LOAD_GFXFF
    if(!gfxFont) { // 'Classic' built-in font
    #endif
//>>>>>>>>>>>>>>>>>>

    int32_t xd = x + _xDatum;
    int32_t yd = y + _yDatum;

    if ((xd >= _vpW)                                 || // Clip right
         ( yd >= _vpH)                                 || // Clip bottom
         ((xd + 6 * size - 1) < _vpX)    || // Clip left
         ((yd + 8 * size - 1) < _vpY))        // Clip top
        return;

    bool fillbg = (bg != color);
    bool clip = xd < _vpX || xd + 6    * textsize >= _vpW || yd < _vpY || yd + 8 * textsize >= _vpH;

    if ((size==1) && fillbg && !clip) {
        uint8_t column[6];
        uint8_t mask = 0x1;
        begin_tft_write();

        setWindow(xd, yd, xd+5, yd+7);

        for (int8_t i = 0; i < 5; i++ ) column[i] = pgm_read_byte(font + (c * 5) + i);
        column[5] = 0;

        for (int8_t j = 0; j < 8; j++) {
            for (int8_t k = 0; k < 5; k++ ) {
                if (column[k] & mask) {tft_Write_16(color);}
                else {tft_Write_16(bg);}
            }
            mask <<= 1;
            tft_Write_16(bg);
        }

        end_tft_write();
    }
    else {
        //begin_tft_write();                    // Sprite class can use this function, avoiding begin_tft_write()
        inTransaction = true;

        for (int8_t i = 0; i < 6; i++ ) {
            uint8_t line;
            if (i == 5)
                line = 0x0;
            else
                line = pgm_read_byte(font + (c * 5) + i);

            if (size == 1 && !fillbg) { // default size
                for (int8_t j = 0; j < 8; j++) {
                    if (line & 0x1) drawPixel(x + i, y + j, color);
                    line >>= 1;
                }
            }
            else {    // big size or clipped
                for (int8_t j = 0; j < 8; j++) {
                    if (line & 0x1) fillRect(x + (i * size), y + (j * size), size, size, color);
                    else if (fillbg) fillRect(x + i * size, y + j * size, size, size, bg);
                    line >>= 1;
                }
            }
        }
        inTransaction = lockTransaction;
        end_tft_write();                            // Does nothing if Sprite class uses this function
    }

//>>>>>>>>>>>>>>>>>>>>>>>>>>>
    #ifdef LOAD_GFXFF
    } else { // Custom font
    #endif
//>>>>>>>>>>>>>>>>>>>>>>>>>>>
#endif // LOAD_GLCD

#ifdef LOAD_GFXFF
        // Filter out bad characters not present in font
        if ((c >= pgm_read_word(&gfxFont->first)) && (c <= pgm_read_word(&gfxFont->last ))) {
            //begin_tft_write();                    // Sprite class can use this function, avoiding begin_tft_write()
            inTransaction = true;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>

            c -= pgm_read_word(&gfxFont->first);
            GFXglyph *glyph    = &(((GFXglyph *)pgm_read_dword(&gfxFont->glyph))[c]);
            uint8_t    *bitmap = (uint8_t *)pgm_read_dword(&gfxFont->bitmap);

            uint32_t bo = pgm_read_word(&glyph->bitmapOffset);
            uint8_t    w    = pgm_read_byte(&glyph->width),
                             h    = pgm_read_byte(&glyph->height);
                             //xa = pgm_read_byte(&glyph->xAdvance);
            int8_t     xo = pgm_read_byte(&glyph->xOffset),
                             yo = pgm_read_byte(&glyph->yOffset);
            uint8_t    xx, yy, bits=0, bit=0;
            int16_t    xo16 = 0, yo16 = 0;

            if(size > 1) {
                xo16 = xo;
                yo16 = yo;
            }

            // GFXFF rendering speed up
            uint16_t hpc = 0; // Horizontal foreground pixel count
            for(yy=0; yy<h; yy++) {
                for(xx=0; xx<w; xx++) {
                    if(bit == 0) {
                        bits = pgm_read_byte(&bitmap[bo++]);
                        bit    = 0x80;
                    }
                    if(bits & bit) hpc++;
                    else {
                     if (hpc) {
                            if(size == 1) drawFastHLine(x+xo+xx-hpc, y+yo+yy, hpc, color);
                            else fillRect(x+(xo16+xx-hpc)*size, y+(yo16+yy)*size, size*hpc, size, color);
                            hpc=0;
                        }
                    }
                    bit >>= 1;
                }
                // Draw pixels for this line as we are about to increment yy
                if (hpc) {
                    if(size == 1) drawFastHLine(x+xo+xx-hpc, y+yo+yy, hpc, color);
                    else fillRect(x+(xo16+xx-hpc)*size, y+(yo16+yy)*size, size*hpc, size, color);
                    hpc=0;
                }
            }

            inTransaction = lockTransaction;
            end_tft_write();                            // Does nothing if Sprite class uses this function
        }
#endif

#ifdef LOAD_GLCD
    #ifdef LOAD_GFXFF
    } // End classic vs custom font
    #endif
#else
    #ifndef LOAD_GFXFF
        // Avoid warnings if fonts are disabled
        x = x;
        y = y;
        color = color;
        bg = bg;
        size = size;
    #endif
#endif

}


/***************************************************************************************
** Function name:                     setAddrWindow
** Description:                         define an area to receive a stream of pixels
***************************************************************************************/
// Chip select is high at the end of this function
static void setAddrWindow(struct TFT_eSPI * tft_dev, int32_t x0, int32_t y0, int32_t w, int32_t h)
{
    tft_dev->begin_tft_write(tft_dev);

    tft_dev->setWindow(tft_dev, x0, y0, x0 + w - 1, y0 + h - 1);

    tft_dev->end_tft_write(tft_dev);
}


/***************************************************************************************
** Function name:                     setWindow
** Description:                         define an area to receive a stream of pixels
***************************************************************************************/
// Chip select stays low, call begin_tft_write first. Use setAddrWindow() from sketches
static void setWindow(struct TFT_eSPI * tft_dev, int32_t x0, int32_t y0, int32_t x1, int32_t y1)
{
    //begin_tft_write(); // Must be called before setWindow
    tft_dev->addr_row = 0xFFFF;
    tft_dev->addr_col = 0xFFFF;

#if defined (ILI9225_DRIVER)
    if (rotation & 0x01) { swap_coord(x0, y0); swap_coord(x1, y1); }
    SPI_BUSY_CHECK;
    DC_C; tft_Write_8(TFT_CASET1);
    DC_D; tft_Write_16(x0);
    DC_C; tft_Write_8(TFT_CASET2);
    DC_D; tft_Write_16(x1);

    DC_C; tft_Write_8(TFT_PASET1);
    DC_D; tft_Write_16(y0);
    DC_C; tft_Write_8(TFT_PASET2);
    DC_D; tft_Write_16(y1);

    DC_C; tft_Write_8(TFT_RAM_ADDR1);
    DC_D; tft_Write_16(x0);
    DC_C; tft_Write_8(TFT_RAM_ADDR2);
    DC_D; tft_Write_16(y0);

    // write to RAM
    DC_C; tft_Write_8(TFT_RAMWR);
    DC_D;
    // Temporary solution is to include the RP2040 code here
    #if (defined(ARDUINO_ARCH_RP2040)    || defined (ARDUINO_ARCH_MBED)) && !defined(RP2040_PIO_INTERFACE)
        // For ILI9225 and RP2040 the slower Arduino SPI transfer calls were used, so need to swap back to 16 bit mode
        while (spi_get_hw(SPI_X)->sr & SPI_SSPSR_BSY_BITS) {};
        hw_write_masked(&spi_get_hw(SPI_X)->cr0, (16 - 1) << SPI_SSPCR0_DSS_LSB, SPI_SSPCR0_DSS_BITS);
    #endif
#elif defined (SSD1351_DRIVER)
    if (rotation & 1) {
        swap_coord(x0, y0);
        swap_coord(x1, y1);
    }
    SPI_BUSY_CHECK;
    DC_C; tft_Write_8(TFT_CASET);
    DC_D; tft_Write_16(x1 | (x0 << 8));
    DC_C; tft_Write_8(TFT_PASET);
    DC_D; tft_Write_16(y1 | (y0 << 8));
    DC_C; tft_Write_8(TFT_RAMWR);
    DC_D;
#else
    #if defined (SSD1963_DRIVER)
        if ((rotation & 0x1) == 0) { swap_coord(x0, y0); swap_coord(x1, y1); }
    #endif

    #ifdef CGRAM_OFFSET
        x0+=tft_dev->colstart;
        x1+=tft_dev->colstart;
        y0+=tft_dev->rowstart;
        y1+=tft_dev->rowstart;
    #endif

    SPI_BUSY_CHECK;
    DC_C; tft_Write_8(TFT_CASET);
    DC_D; tft_Write_32C(x0, x1);
    DC_C; tft_Write_8(TFT_PASET);
    DC_D; tft_Write_32C(y0, y1);
    DC_C; tft_Write_8(TFT_RAMWR);
    DC_D;
#endif
    //end_tft_write(); // Must be called after setWindow
}

/***************************************************************************************
** Function name:                     readAddrWindow
** Description:                         define an area to read a stream of pixels
***************************************************************************************/
static void readAddrWindow(struct TFT_eSPI * tft_dev, int32_t xs, int32_t ys, int32_t w, int32_t h)
{
    //begin_tft_write(); // Must be called before readAddrWindow or CS set low

    int32_t xe = xs + w - 1;
    int32_t ye = ys + h - 1;

    tft_dev->addr_col = 0xFFFF;
    tft_dev->addr_row = 0xFFFF;

#if defined (SSD1963_DRIVER)
    if ((rotation & 0x1) == 0) { swap_coord(xs, ys); swap_coord(xe, ye); }
#endif

#ifdef CGRAM_OFFSET
    xs += tft_dev->colstart;
    xe += tft_dev->colstart;
    ys += tft_dev->rowstart;
    ye += tft_dev->rowstart;
#endif

    // Column addr set
    DC_C; tft_Write_8(TFT_CASET);
    DC_D; tft_Write_32C(xs, xe);

    // Row addr set
    DC_C; tft_Write_8(TFT_PASET);
    DC_D; tft_Write_32C(ys, ye);

     // Read CGRAM command
    DC_C; tft_Write_8(TFT_RAMRD);

     DC_D;
    //end_tft_write(); // Must be called after readAddrWindow or CS set high
}


/***************************************************************************************
** Function name:                     drawPixel
** Description:                         push a single pixel at an arbitrary position
***************************************************************************************/
static void drawPixel(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, uint32_t color)
{
    if (tft_dev->_vpOoB) return;

    x+= tft_dev->_xDatum;
    y+= tft_dev->_yDatum;

    // Range checking
    if ((x < tft_dev->_vpX) || (y < tft_dev->_vpY) ||(x >= tft_dev->_vpW) || (y >= tft_dev->_vpH)) return;

#ifdef CGRAM_OFFSET
    x+=tft_dev->colstart;
    y+=tft_dev->rowstart;
#endif

#if (defined (MULTI_TFT_SUPPORT) || defined (GC9A01_DRIVER)) && !defined (ILI9225_DRIVER)
    addr_row = 0xFFFF;
    addr_col = 0xFFFF;
#endif

    tft_dev->begin_tft_write(tft_dev);

#if defined (ILI9225_DRIVER)
    if (rotation & 0x01) { swap_coord(x, y); }
    SPI_BUSY_CHECK;

    // Set window to full screen to optimise sequential pixel rendering
    if (addr_row != 0x9225) {
        addr_row = 0x9225; // addr_row used for flag
        DC_C; tft_Write_8(TFT_CASET1);
        DC_D; tft_Write_16(0);
        DC_C; tft_Write_8(TFT_CASET2);
        DC_D; tft_Write_16(175);

        DC_C; tft_Write_8(TFT_PASET1);
        DC_D; tft_Write_16(0);
        DC_C; tft_Write_8(TFT_PASET2);
        DC_D; tft_Write_16(219);
    }

    // Define pixel coordinate
    DC_C; tft_Write_8(TFT_RAM_ADDR1);
    DC_D; tft_Write_16(x);
    DC_C; tft_Write_8(TFT_RAM_ADDR2);
    DC_D; tft_Write_16(y);

    // write to RAM
    DC_C; tft_Write_8(TFT_RAMWR);
    #if defined(TFT_PARALLEL_8_BIT) || defined(TFT_PARALLEL_16_BIT) || !defined(ESP32)
        DC_D; tft_Write_16(color);
    #else
        DC_D; tft_Write_16N(color);
    #endif
#else

    #if defined (SSD1963_DRIVER)
        if ((rotation & 0x1) == 0) { swap_coord(x, y); }
    #endif

        SPI_BUSY_CHECK;

    #if defined (SSD1351_DRIVER)
        if (rotation & 0x1) { swap_coord(x, y); }
        // No need to send x if it has not changed (speeds things up)
        if (addr_col != x) {
            DC_C; tft_Write_8(TFT_CASET);
            DC_D; tft_Write_16(x | (x << 8));
            addr_col = x;
        }

        // No need to send y if it has not changed (speeds things up)
        if (addr_row != y) {
            DC_C; tft_Write_8(TFT_PASET);
            DC_D; tft_Write_16(y | (y << 8));
            addr_row = y;
        }
    #else
        // No need to send x if it has not changed (speeds things up)
        if (tft_dev->addr_col != x) {
            DC_C; tft_Write_8(TFT_CASET);
            DC_D; tft_Write_32D(x);
            tft_dev->addr_col = x;
        }

        // No need to send y if it has not changed (speeds things up)
        if (tft_dev->addr_row != y) {
            DC_C; tft_Write_8(TFT_PASET);
            DC_D; tft_Write_32D(y);
            tft_dev->addr_row = y;
        }
    #endif

    DC_C; tft_Write_8(TFT_RAMWR);

    #if defined(TFT_PARALLEL_8_BIT) || defined(TFT_PARALLEL_16_BIT) || !defined(ESP32)
        DC_D; tft_Write_16(color);
    #else
        DC_D; tft_Write_16N(color);
    #endif
#endif

    tft_dev->end_tft_write(tft_dev);
}

/***************************************************************************************
** Function name:                     pushColor
** Description:                         push a single pixel
***************************************************************************************/
static void pushColor(struct TFT_eSPI * tft_dev, uint16_t color)
{
    tft_dev->begin_tft_write(tft_dev);

    SPI_BUSY_CHECK;
    tft_Write_16(color);

    tft_dev->end_tft_write(tft_dev);
}


/***************************************************************************************
** Function name:                     pushColor
** Description:                         push a single colour to "len" pixels
***************************************************************************************/
static void pushColorLen(struct TFT_eSPI * tft_dev, uint16_t color, uint32_t len)
{
    tft_dev->begin_tft_write(tft_dev);

    tft_dev->pushBlock(tft_dev, color, len);

    tft_dev->end_tft_write(tft_dev);
}

/***************************************************************************************
** Function name:                     startWrite
** Description:                         begin transaction with CS low, MUST later call endWrite
***************************************************************************************/
static void startWrite(struct TFT_eSPI * tft_dev)
{
    tft_dev->begin_tft_write(tft_dev);
    tft_dev->lockTransaction = true; // Lock transaction for all sequentially run sketch functions
    tft_dev->inTransaction = true;
}

/***************************************************************************************
** Function name:                     endWrite
** Description:                         end transaction with CS high
***************************************************************************************/
static void endWrite(struct TFT_eSPI * tft_dev)
{
    tft_dev->lockTransaction = false; // Release sketch induced transaction lock
    tft_dev->inTransaction = false;
    DMA_BUSY_CHECK;                    // Safety check - user code should have checked this!
    tft_dev->end_tft_write(tft_dev);                 // Release SPI bus
}

/***************************************************************************************
** Function name:                     writeColor (use startWrite() and endWrite() before & after)
** Description:                         raw write of "len" pixels avoiding transaction check
***************************************************************************************/
static void writeColor(struct TFT_eSPI * tft_dev, uint16_t color, uint32_t len)
{
    tft_dev->pushBlock(tft_dev, color, len);
}

/***************************************************************************************
** Function name:                     pushColors
** Description:                         push an array of pixels for 16 bit raw image drawing
***************************************************************************************/
// Assumed that setAddrWindow() has previously been called
// len is number of bytes, not pixels
static void pushColors(struct TFT_eSPI * tft_dev, uint8_t *data, uint32_t len)
{
    tft_dev->begin_tft_write(tft_dev);

    tft_dev->pushPixels(tft_dev, data, len>>1);

    tft_dev->end_tft_write(tft_dev);
}


/***************************************************************************************
** Function name:                     pushColors
** Description:                         push an array of pixels, for image drawing
***************************************************************************************/
static void pushColorsSwap(struct TFT_eSPI * tft_dev, uint16_t *data, uint32_t len, bool swap)
{
    tft_dev->begin_tft_write(tft_dev);
    if (swap) {swap = tft_dev->_swapBytes; tft_dev->_swapBytes = true; }

    tft_dev->pushPixels(tft_dev, data, len);

    tft_dev->_swapBytes = swap; // Restore old value
    tft_dev->end_tft_write(tft_dev);
}


/***************************************************************************************
** Function name:                     drawLine
** Description:                         draw a line between 2 arbitrary points
***************************************************************************************/
// Bresenham's algorithm - thx wikipedia - speed enhanced by Bodmer to use
// an efficient FastH/V Line draw routine for line segments of 2 pixels or more
static void drawLine(struct TFT_eSPI * tft_dev, int32_t x0, int32_t y0, int32_t x1, int32_t y1, uint32_t color)
{
    if (tft_dev->_vpOoB) return;

    //begin_tft_write();             // Sprite class can use this function, avoiding begin_tft_write()
    tft_dev->inTransaction = true;

    //x+= _xDatum;                         // Not added here, added by drawPixel & drawFastXLine
    //y+= _yDatum;

    bool steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep) {
        swap_coord(&x0, &y0);
        swap_coord(&x1, &y1);
    }

    if (x0 > x1) {
        swap_coord(&x0, &x1);
        swap_coord(&y0, &y1);
    }

    int32_t dx = x1 - x0, dy = abs(y1 - y0);;

    int32_t err = dx >> 1, ystep = -1, xs = x0, dlen = 0;

    if (y0 < y1) ystep = 1;

    // Split into steep and not steep for FastH/V separation
    if (steep) {
        for (; x0 <= x1; x0++) {
            dlen++;
            err -= dy;
            if (err < 0) {
                if (dlen == 1) tft_dev->drawPixel(tft_dev, y0, xs, color);
                else tft_dev->drawFastVLine(tft_dev, y0, xs, dlen, color);
                dlen = 0;
                y0 += ystep; xs = x0 + 1;
                err += dx;
            }
        }
        if (dlen) tft_dev->drawFastVLine(tft_dev, y0, xs, dlen, color);
    }
    else
    {
        for (; x0 <= x1; x0++) {
            dlen++;
            err -= dy;
            if (err < 0) {
                if (dlen == 1) tft_dev->drawPixel(tft_dev, xs, y0, color);
                else tft_dev->drawFastHLine(tft_dev, xs, y0, dlen, color);
                dlen = 0;
                y0 += ystep; xs = x0 + 1;
                err += dx;
            }
        }
        if (dlen) tft_dev->drawFastHLine(tft_dev, xs, y0, dlen, color);
    }

    tft_dev->inTransaction = tft_dev->lockTransaction;
    tft_dev->end_tft_write(tft_dev);
}


/***************************************************************************************
** Description:    Constants for anti-aliased line drawing on TFT and in Sprites
***************************************************************************************/
const float PixelAlphaGain     = 255.0;
const float LoAlphaTheshold    = 1.0/32.0;
const float HiAlphaTheshold    = 1.0 - LoAlphaTheshold;

/***************************************************************************************
** Function name:                     drawPixel (alpha blended)
** Description:                         Draw a pixel blended with the screen or bg pixel colour
***************************************************************************************/
static uint16_t drawPixelAlpha(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, uint32_t color, uint8_t alpha, uint32_t bg_color)
{
    if (bg_color == 0x00FFFFFF) bg_color = tft_dev->readPixel(tft_dev, x, y);
    color = tft_dev->alphaBlend(tft_dev, alpha, color, bg_color);
    tft_dev->drawPixel(tft_dev, x, y, color);
    return color;
}

/***************************************************************************************
** Function name:                     fillSmoothCircle
** Description:                         Draw a filled anti-aliased circle
***************************************************************************************/
static void fillSmoothCircle(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t r, uint32_t color, uint32_t bg_color)
{
    if (r <= 0) return;

    tft_dev->inTransaction = true;

    tft_dev->drawFastHLine(tft_dev, x - r, y, 2 * r + 1, color);
    int32_t xs = 1;
    int32_t cx = 0;

    int32_t r1 = r * r;
    r++;
    int32_t r2 = r * r;

    for (int32_t cy = r - 1; cy > 0; cy--)
    {
        int32_t dy2 = (r - cy) * (r - cy);
        for (cx = xs; cx < r; cx++)
        {
            int32_t hyp2 = (r - cx) * (r - cx) + dy2;
            if (hyp2 <= r1) break;
            if (hyp2 >= r2) continue;
            float alphaf = (float)r - sqrtf(hyp2);
            if (alphaf > HiAlphaTheshold) break;
            xs = cx;
            if (alphaf < LoAlphaTheshold) continue;
            uint8_t alpha = alphaf * 255;

            if (bg_color == 0x00FFFFFF) {
                tft_dev->drawPixelAlpha(tft_dev, x + cx - r, y + cy - r, color, alpha, bg_color);
                tft_dev->drawPixelAlpha(tft_dev, x - cx + r, y + cy - r, color, alpha, bg_color);
                tft_dev->drawPixelAlpha(tft_dev, x - cx + r, y - cy + r, color, alpha, bg_color);
                tft_dev->drawPixelAlpha(tft_dev, x + cx - r, y - cy + r, color, alpha, bg_color);
            }
            else {
                uint16_t pcol = tft_dev->drawPixelAlpha(tft_dev, x + cx - r, y + cy - r, color, alpha, bg_color);
                tft_dev->drawPixel(tft_dev, x - cx + r, y + cy - r, pcol);
                tft_dev->drawPixel(tft_dev, x - cx + r, y - cy + r, pcol);
                tft_dev->drawPixel(tft_dev, x + cx - r, y - cy + r, pcol);
            }
        }
        tft_dev->drawFastHLine(tft_dev, x + cx - r, y + cy - r, 2 * (r - cx) + 1, color);
        tft_dev->drawFastHLine(tft_dev, x + cx - r, y - cy + r, 2 * (r - cx) + 1, color);
    }
    tft_dev->inTransaction = tft_dev->lockTransaction;
    tft_dev->end_tft_write(tft_dev);
}


/***************************************************************************************
** Function name:                     fillSmoothRoundRect
** Description:                         Draw a filled anti-aliased rounded corner rectangle
***************************************************************************************/
static void fillSmoothRoundRect(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, int32_t r, uint32_t color, uint32_t bg_color)
{
    tft_dev->inTransaction = true;
    int32_t xs = 0;
    int32_t cx = 0;

    // Limit radius to half width or height
    if (r > w/2) r = w/2;
    if (r > h/2) r = h/2;

    y += r;
    h -= 2*r;
    tft_dev->fillRect(tft_dev, x, y, w, h, color);
    h--;
    x += r;
    w -= 2*r+1;
    int32_t r1 = r * r;
    r++;
    int32_t r2 = r * r;

    for (int32_t cy = r - 1; cy > 0; cy--)
    {
        int32_t dy2 = (r - cy) * (r - cy);
        for (cx = xs; cx < r; cx++)
        {
            int32_t hyp2 = (r - cx) * (r - cx) + dy2;
            if (hyp2 <= r1) break;
            if (hyp2 >= r2) continue;
            float alphaf = (float)r - sqrtf(hyp2);
            if (alphaf > HiAlphaTheshold) break;
            xs = cx;
            if (alphaf < LoAlphaTheshold) continue;
            uint8_t alpha = alphaf * 255;

            tft_dev->drawPixelAlpha(tft_dev, x + cx - r, y + cy - r, color, alpha, bg_color);
            tft_dev->drawPixelAlpha(tft_dev, x - cx + r + w, y + cy - r, color, alpha, bg_color);
            tft_dev->drawPixelAlpha(tft_dev, x - cx + r + w, y - cy + r + h, color, alpha, bg_color);
            tft_dev->drawPixelAlpha(tft_dev, x + cx - r, y - cy + r + h, color, alpha, bg_color);
        }
        tft_dev->drawFastHLine(tft_dev, x + cx - r, y + cy - r, 2 * (r - cx) + 1 + w, color);
        tft_dev->drawFastHLine(tft_dev, x + cx - r, y - cy + r + h, 2 * (r - cx) + 1 + w, color);
    }
    tft_dev->inTransaction = tft_dev->lockTransaction;
    tft_dev->end_tft_write(tft_dev);
}

/***************************************************************************************
** Function name:                     drawSpot - maths intensive, so for small filled circles
** Description:                         Draw an anti-aliased filled circle at ax,ay with radius r
***************************************************************************************/
static void drawSpot(struct TFT_eSPI * tft_dev, float ax, float ay, float r, uint32_t fg_color, uint32_t bg_color)
{
    // Filled circle can be created by the wide line function with zero line length
    tft_dev->drawWedgeLine(tft_dev, ax, ay, ax, ay, r, r, fg_color, bg_color);
}

/***************************************************************************************
** Function name:                     drawWideLine - background colour specified or pixel read
** Description:                         draw an anti-aliased line with rounded ends, width wd
***************************************************************************************/
static void drawWideLine(struct TFT_eSPI * tft_dev, float ax, float ay, float bx, float by, float wd, uint32_t fg_color, uint32_t bg_color)
{
    tft_dev->drawWedgeLine(tft_dev, ax, ay, bx, by, wd/2.0, wd/2.0, fg_color, bg_color);
}

/***************************************************************************************
** Function name:                     drawWedgeLine
** Description:                         draw an anti-aliased line with different width radiused ends
***************************************************************************************/
static void drawWedgeLine(struct TFT_eSPI * tft_dev, float ax, float ay, float bx, float by, float ar, float br, uint32_t fg_color, uint32_t bg_color)
{
    if ( (abs(ax - bx) < 0.01f) && (abs(ay - by) < 0.01f) ) bx += 0.01f;    // Avoid divide by zero

    // Find line bounding box
    int32_t x0 = (int32_t)floorf(fminf(ax-ar, bx-br));
    int32_t x1 = (int32_t) ceilf(fmaxf(ax+ar, bx+br));
    int32_t y0 = (int32_t)floorf(fminf(ay-ar, by-br));
    int32_t y1 = (int32_t) ceilf(fmaxf(ay+ar, by+br));

    if (!tft_dev->clipWindow(tft_dev, &x0, &y0, &x1, &y1)) return;

    // Establish x start and y start
    int32_t ys = ay;
    if ((ax-ar)>(bx-br)) ys = by;

    float rdt = ar - br; // Radius delta
    float alpha = 1.0f;
    ar += 0.5;

    uint16_t bg = bg_color;
    float xpax, ypay, bax = bx - ax, bay = by - ay;

    tft_dev->begin_tft_write(tft_dev);
    tft_dev->inTransaction = true;

    int32_t xs = x0;
    // Scan bounding box from ys down, calculate pixel intensity from distance to line
    for (int32_t yp = ys; yp <= y1; yp++) {
        bool swin = true;    // Flag to start new window area
        bool endX = false; // Flag to skip pixels
        ypay = yp - ay;
        for (int32_t xp = xs; xp <= x1; xp++) {
            if (endX) if (alpha <= LoAlphaTheshold) break;    // Skip right side
            xpax = xp - ax;
            alpha = ar - tft_dev->wedgeLineDistance(tft_dev, xpax, ypay, bax, bay, rdt);
            if (alpha <= LoAlphaTheshold ) continue;
            // Track edge to minimise calculations
            if (!endX) { endX = true; xs = xp; }
            if (alpha > HiAlphaTheshold) {
                if (swin) { tft_dev->setWindow(tft_dev, xp, yp, tft_dev->width(tft_dev)-1, yp); swin = false; }
                tft_dev->pushColor(tft_dev, fg_color);
                continue;
            }
            //Blend color with background and plot
            if (bg_color == 0x00FFFFFF) {
                bg = tft_dev->readPixel(tft_dev, xp, yp); swin = true;
            }
            if (swin) { tft_dev->setWindow(tft_dev, xp, yp, tft_dev->width(tft_dev)-1, yp); swin = false; }
            tft_dev->pushColor(tft_dev, tft_dev->alphaBlend(tft_dev, (uint8_t)(alpha * PixelAlphaGain), fg_color, bg));
        }
    }

    // Reset x start to left side of box
    xs = x0;
    // Scan bounding box from ys-1 up, calculate pixel intensity from distance to line
    for (int32_t yp = ys-1; yp >= y0; yp--) {
        bool swin = true;    // Flag to start new window area
        bool endX = false; // Flag to skip pixels
        ypay = yp - ay;
        for (int32_t xp = xs; xp <= x1; xp++) {
            if (endX) if (alpha <= LoAlphaTheshold) break;    // Skip right side of drawn line
            xpax = xp - ax;
            alpha = ar - tft_dev->wedgeLineDistance(tft_dev, xpax, ypay, bax, bay, rdt);
            if (alpha <= LoAlphaTheshold ) continue;
            // Track line boundary
            if (!endX) { endX = true; xs = xp; }
            if (alpha > HiAlphaTheshold) {
                if (swin) { tft_dev->setWindow(tft_dev, xp, yp, tft_dev->width(tft_dev)-1, yp); swin = false; }
                tft_dev->pushColor(tft_dev, fg_color);
                continue;
            }
            //Blend color with background and plot
            if (bg_color == 0x00FFFFFF) {
                bg = tft_dev->readPixel(tft_dev, xp, yp); swin = true;
            }
            if (swin) { tft_dev->setWindow(tft_dev, xp, yp, tft_dev->width(tft_dev)-1, yp); swin = false; }
            tft_dev->pushColor(tft_dev, tft_dev->alphaBlend(tft_dev, (uint8_t)(alpha * PixelAlphaGain), fg_color, bg));
        }
    }

    tft_dev->inTransaction = tft_dev->lockTransaction;
    tft_dev->end_tft_write(tft_dev);
}

// Calculate distance of px,py to closest part of line
/***************************************************************************************
** Function name:                     lineDistance - private helper function for drawWedgeLine
** Description:                         returns distance of px,py to closest part of a to b wedge
***************************************************************************************/
static float wedgeLineDistance(struct TFT_eSPI * tft_dev, float xpax, float ypay, float bax, float bay, float dr)
{
    float h = fmaxf(fminf((xpax * bax + ypay * bay) / (bax * bax + bay * bay), 1.0f), 0.0f);
    float dx = xpax - bax * h, dy = ypay - bay * h;
    return sqrtf(dx * dx + dy * dy) + h * dr;
}


/***************************************************************************************
** Function name:                     drawFastVLine
** Description:                         draw a vertical line
***************************************************************************************/
static void drawFastVLine(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t h, uint32_t color)
{
    if (tft_dev->_vpOoB) return;

    x+= tft_dev->_xDatum;
    y+= tft_dev->_yDatum;

    // Clipping
    if ((x < tft_dev->_vpX) || (x >= tft_dev->_vpW) || (y >= tft_dev->_vpH)) return;

    if (y < tft_dev->_vpY) { h += y - tft_dev->_vpY; y = tft_dev->_vpY; }

    if ((y + h) > tft_dev->_vpH) h = tft_dev->_vpH - y;

    if (h < 1) return;

    tft_dev->begin_tft_write(tft_dev);

    tft_dev->setWindow(tft_dev, x, y, x, y + h - 1);

    tft_dev->pushBlock(tft_dev, color, h);

    tft_dev->end_tft_write(tft_dev);
}


/***************************************************************************************
** Function name:                     drawFastHLine
** Description:                         draw a horizontal line
***************************************************************************************/
static void drawFastHLine(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, uint32_t color)
{
    if (tft_dev->_vpOoB) return;

    x+= tft_dev->_xDatum;
    y+= tft_dev->_yDatum;

    // Clipping
    if ((y < tft_dev->_vpY) || (x >= tft_dev->_vpW) || (y >= tft_dev->_vpH)) return;

    if (x < tft_dev->_vpX) { w += x - tft_dev->_vpX; x = tft_dev->_vpX; }

    if ((x + w) > tft_dev->_vpW) w = tft_dev->_vpW - x;

    if (w < 1) return;

    tft_dev->begin_tft_write(tft_dev);

    tft_dev->setWindow(tft_dev, x, y, x + w - 1, y);

    tft_dev->pushBlock(tft_dev, color, w);

    tft_dev->end_tft_write(tft_dev);
}


/***************************************************************************************
** Function name:                     fillRect
** Description:                         draw a filled rectangle
***************************************************************************************/
static void fillRect(struct TFT_eSPI * tft_dev, int32_t x, int32_t y, int32_t w, int32_t h, uint32_t color)
{
    if(tft_dev->_vpOoB) return;

    x+= tft_dev->_xDatum;
    y+= tft_dev->_yDatum;

    // Clipping
    if((x >= tft_dev->_vpW) || (y >= tft_dev->_vpH)) return;

    if (x < tft_dev->_vpX) { w += x - tft_dev->_vpX; x = tft_dev->_vpX; }
    if (y < tft_dev->_vpY) { h += y - tft_dev->_vpY; y = tft_dev->_vpY; }

    if ((x + w) > tft_dev->_vpW) w = tft_dev->_vpW - x;
    if ((y + h) > tft_dev->_vpH) h = tft_dev->_vpH - y;

    if((w < 1) || (h < 1)) return;

    //Serial.print(" _xDatum=");Serial.print( _xDatum);Serial.print(", _yDatum=");Serial.print( _yDatum);
    //Serial.print(", _xWidth=");Serial.print(_xWidth);Serial.print(", _yHeight=");Serial.println(_yHeight);

    //Serial.print(" _vpX=");Serial.print( _vpX);Serial.print(", _vpY=");Serial.print( _vpY);
    //Serial.print(", _vpW=");Serial.print(_vpW);Serial.print(", _vpH=");Serial.println(_vpH);

    //Serial.print(" x=");Serial.print( y);Serial.print(", y=");Serial.print( y);
    //Serial.print(", w=");Serial.print(w);Serial.print(", h=");Serial.println(h);

    GB_DEBUGI(DISP_TAG, "window, x: %d, y: %d, w: %d, h: %d, color: %08x", x, y, w, h, color);

    tft_dev->begin_tft_write(tft_dev);
    tft_dev->setWindow(tft_dev, x, y, x + w - 1, y + h - 1);
    for (int i = 0; i < h; i++)
        tft_dev->pushBlock(tft_dev, color, w);
    tft_dev->end_tft_write(tft_dev);
}


/***************************************************************************************
** Function name:                     fillRectVGradient
** Description:                         draw a filled rectangle with a vertical colour gradient
***************************************************************************************/
static void fillRectVGradient(struct TFT_eSPI * tft_dev, int16_t x, int16_t y, int16_t w, int16_t h, uint32_t color1, uint32_t color2)
{
    if (tft_dev->_vpOoB) return;

    x+= tft_dev->_xDatum;
    y+= tft_dev->_yDatum;

    // Clipping
    if ((x >= tft_dev->_vpW) || (y >= tft_dev->_vpH)) return;

    if (x < tft_dev->_vpX) { w += x - tft_dev->_vpX; x = tft_dev->_vpX; }
    if (y < tft_dev->_vpY) { h += y - tft_dev->_vpY; y = tft_dev->_vpY; }

    if ((x + w) > tft_dev->_vpW) w = tft_dev->_vpW - x;
    if ((y + h) > tft_dev->_vpH) h = tft_dev->_vpH - y;

    if ((w < 1) || (h < 1)) return;

    tft_dev->begin_tft_write(tft_dev);

    tft_dev->setWindow(tft_dev, x, y, x + w - 1, y + h - 1);

    float delta = -255.0/h;
    float alpha = 255.0;
    uint32_t color = color1;

    while (h--) {
        tft_dev->pushBlock(tft_dev, color, w);
        alpha += delta;
        color = tft_dev->alphaBlend(tft_dev, (uint8_t)alpha, color1, color2);
    }

    tft_dev->end_tft_write(tft_dev);
}


/***************************************************************************************
** Function name:                     fillRectHGradient
** Description:                         draw a filled rectangle with a horizontal colour gradient
***************************************************************************************/
static void fillRectHGradient(struct TFT_eSPI * tft_dev, int16_t x, int16_t y, int16_t w, int16_t h, uint32_t color1, uint32_t color2)
{
    if (tft_dev->_vpOoB) return;

    x+= tft_dev->_xDatum;
    y+= tft_dev->_yDatum;

    // Clipping
    if ((x >= tft_dev->_vpW) || (y >= tft_dev->_vpH)) return;

    if (x < tft_dev->_vpX) { w += x - tft_dev->_vpX; x = tft_dev->_vpX; }
    if (y < tft_dev->_vpY) { h += y - tft_dev->_vpY; y = tft_dev->_vpY; }

    if ((x + w) > tft_dev->_vpW) w = tft_dev->_vpW - x;
    if ((y + h) > tft_dev->_vpH) h = tft_dev->_vpH - y;

    if ((w < 1) || (h < 1)) return;

    tft_dev->begin_tft_write(tft_dev);

    float delta = -255.0/w;
    float alpha = 255.0;
    uint32_t color = color1;

    while (w--) {
        tft_dev->drawFastVLine(tft_dev, x++, y, h, color);
        alpha += delta;
        color = tft_dev->alphaBlend(tft_dev, (uint8_t)alpha, color1, color2);
    }

    tft_dev->end_tft_write(tft_dev);
}


/***************************************************************************************
** Function name:                     color565
** Description:                         convert three 8 bit RGB levels to a 16 bit colour value
***************************************************************************************/
static uint16_t color565(struct TFT_eSPI * tft_dev, uint8_t r, uint8_t g, uint8_t b)
{
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}


/***************************************************************************************
** Function name:                     color16to8
** Description:                         convert 16 bit colour to an 8 bit 332 RGB colour value
***************************************************************************************/
static uint8_t color16to8(struct TFT_eSPI * tft_dev, uint16_t c)
{
    return ((c & 0xE000)>>8) | ((c & 0x0700)>>6) | ((c & 0x0018)>>3);
}


/***************************************************************************************
** Function name:                     color8to16
** Description:                         convert 8 bit colour to a 16 bit 565 colour value
***************************************************************************************/
static uint16_t color8to16(struct TFT_eSPI * tft_dev, uint8_t color)
{
    uint8_t    blue[] = {0, 11, 21, 31}; // blue 2 to 5 bit colour lookup table
    uint16_t color16 = 0;

    //                =====Green=====         ===============Red==============
    color16    = (color & 0x1C)<<6 | (color & 0xC0)<<5 | (color & 0xE0)<<8;
    //                =====Green=====        =======Blue======
    color16 |= (color & 0x1C)<<3 | blue[color & 0x03];

    return color16;
}

/***************************************************************************************
** Function name:                     color16to24
** Description:                         convert 16 bit colour to a 24 bit 888 colour value
***************************************************************************************/
static uint32_t color16to24(struct TFT_eSPI * tft_dev, uint16_t color565)
{
    uint8_t r = (color565 >> 8) & 0xF8; r |= (r >> 5);
    uint8_t g = (color565 >> 3) & 0xFC; g |= (g >> 6);
    uint8_t b = (color565 << 3) & 0xF8; b |= (b >> 5);

    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | ((uint32_t)b << 0);
}

/***************************************************************************************
** Function name:                     color24to16
** Description:                         convert 24 bit colour to a 16 bit 565 colour value
***************************************************************************************/
static uint32_t color24to16(struct TFT_eSPI * tft_dev, uint32_t color888)
{
    uint16_t r = (color888 >> 8) & 0xF800;
    uint16_t g = (color888 >> 5) & 0x07E0;
    uint16_t b = (color888 >> 3) & 0x001F;

    return (r | g | b);
}

/***************************************************************************************
** Function name:                     invertDisplay
** Description:                         invert the display colours i = 1 invert, i = 0 normal
***************************************************************************************/
static void invertDisplay(struct TFT_eSPI * tft_dev, bool i)
{
    tft_dev->begin_tft_write(tft_dev);
    // Send the command twice as otherwise it does not always work!
    tft_dev->writecommand(tft_dev, i ? TFT_INVON : TFT_INVOFF);
    tft_dev->writecommand(tft_dev, i ? TFT_INVON : TFT_INVOFF);
    tft_dev->end_tft_write(tft_dev);
}


/**************************************************************************
** Function name:                     setAttribute
** Description:                         Sets a control parameter of an attribute
**************************************************************************/
static void setAttribute(struct TFT_eSPI * tft_dev, uint8_t attr_id, uint8_t param) {
        switch (attr_id) {
                        break;
                case CP437_SWITCH:
                        tft_dev->_cp437 = param;
                        break;
                case UTF8_SWITCH:
                        tft_dev->_utf8    = param;
                        tft_dev->decoderState = 0;
                        break;
                case PSRAM_ENABLE:
#if defined (ESP32) && defined (CONFIG_SPIRAM_SUPPORT)
                        if (psramFound()) _psram_enable = param; // Enable the use of PSRAM (if available)
                        else
#endif
                        tft_dev->_psram_enable = false;
                        break;
                //case 4: // TBD future feature control
                //        _tbd = param;
                //        break;
        }
}


/**************************************************************************
** Function name:                     getAttribute
** Description:                         Get value of an attribute (control parameter)
**************************************************************************/
static uint8_t getAttribute(struct TFT_eSPI * tft_dev, uint8_t attr_id) {
        switch (attr_id) {
                case CP437_SWITCH: // ON/OFF control of full CP437 character set
                        return tft_dev->_cp437;
                case UTF8_SWITCH: // ON/OFF control of UTF-8 decoding
                        return tft_dev->_utf8;
                case PSRAM_ENABLE:
                        return tft_dev->_psram_enable;
                //case 3: // TBD future feature control
                //        return _tbd;
                //        break;
        }

        return false;
}

/***************************************************************************************
** Function name:                     decodeUTF8
** Description:                         Serial UTF-8 decoder with fall-back to extended ASCII
*************************************************************************************x*/
static uint16_t decodeUTF8C(struct TFT_eSPI * tft_dev, uint8_t c)
{
    if (!tft_dev->_utf8) return c;

    // 7 bit Unicode Code Point
    if ((c & 0x80) == 0x00) {
        tft_dev->decoderState = 0;
        return c;
    }

    if (tft_dev->decoderState == 0) {
        // 11 bit Unicode Code Point
        if ((c & 0xE0) == 0xC0) {
            tft_dev->decoderBuffer = ((c & 0x1F)<<6);
            tft_dev->decoderState = 1;
            return 0;
        }
        // 16 bit Unicode Code Point
        if ((c & 0xF0) == 0xE0) {
            tft_dev->decoderBuffer = ((c & 0x0F)<<12);
            tft_dev->decoderState = 2;
            return 0;
        }
        // 21 bit Unicode    Code Point not supported so fall-back to extended ASCII
        // if ((c & 0xF8) == 0xF0) return c;
    }
    else {
        if (tft_dev->decoderState == 2) {
            tft_dev->decoderBuffer |= ((c & 0x3F)<<6);
            tft_dev->decoderState--;
            return 0;
        }
        else {
            tft_dev->decoderBuffer |= (c & 0x3F);
            tft_dev->decoderState = 0;
            return tft_dev->decoderBuffer;
        }
    }

    tft_dev->decoderState = 0;

    return c; // fall-back to extended ASCII
}


/***************************************************************************************
** Function name:                     decodeUTF8
** Description:                         Line buffer UTF-8 decoder with fall-back to extended ASCII
*************************************************************************************x*/
static uint16_t decodeUTF8(struct TFT_eSPI * tft_dev, uint8_t *buf, uint16_t *index, uint16_t remaining)
{
    uint16_t c = buf[(*index)++];
    //Serial.print("Byte from string = 0x"); Serial.println(c, HEX);

    if (!tft_dev->_utf8) return c;

    // 7 bit Unicode
    if ((c & 0x80) == 0x00) return c;

    // 11 bit Unicode
    if (((c & 0xE0) == 0xC0) && (remaining > 1))
        return ((c & 0x1F)<<6) | (buf[(*index)++]&0x3F);

    // 16 bit Unicode
    if (((c & 0xF0) == 0xE0) && (remaining > 2)) {
        c = ((c & 0x0F)<<12) | ((buf[(*index)++]&0x3F)<<6);
        return    c | ((buf[(*index)++]&0x3F));
    }

    // 21 bit Unicode not supported so fall-back to extended ASCII
    // if ((c & 0xF8) == 0xF0) return c;

    return c; // fall-back to extended ASCII
}


/***************************************************************************************
** Function name:                     alphaBlend
** Description:                         Blend 16bit foreground and background
*************************************************************************************x*/
static uint16_t alphaBlend(struct TFT_eSPI * tft_dev, uint8_t alpha, uint16_t fgc, uint16_t bgc)
{
    // For speed use fixed point maths and rounding to permit a power of 2 division
    uint16_t fgR = ((fgc >> 10) & 0x3E) + 1;
    uint16_t fgG = ((fgc >>    4) & 0x7E) + 1;
    uint16_t fgB = ((fgc <<    1) & 0x3E) + 1;

    uint16_t bgR = ((bgc >> 10) & 0x3E) + 1;
    uint16_t bgG = ((bgc >>    4) & 0x7E) + 1;
    uint16_t bgB = ((bgc <<    1) & 0x3E) + 1;

    // Shift right 1 to drop rounding bit and shift right 8 to divide by 256
    uint16_t r = (((fgR * alpha) + (bgR * (255 - alpha))) >> 9);
    uint16_t g = (((fgG * alpha) + (bgG * (255 - alpha))) >> 9);
    uint16_t b = (((fgB * alpha) + (bgB * (255 - alpha))) >> 9);

    // Combine RGB565 colours into 16 bits
    //return ((r&0x18) << 11) | ((g&0x30) << 5) | ((b&0x18) << 0); // 2 bit greyscale
    //return ((r&0x1E) << 11) | ((g&0x3C) << 5) | ((b&0x1E) << 0); // 4 bit greyscale
    return (r << 11) | (g << 5) | (b << 0);
}

/***************************************************************************************
** Function name:                     alphaBlend
** Description:                         Blend 16bit foreground and background with dither
*************************************************************************************x*/
static uint16_t alphaBlendDither(struct TFT_eSPI * tft_dev, uint8_t alpha, uint16_t fgc, uint16_t bgc, uint8_t dither)
{
    if (dither) {
        int16_t alphaDither = (int16_t)alpha - dither + rand() % 2*dither - dither; // +/-4 randomised
        alpha = (uint8_t)alphaDither;
        if (alphaDither <    0) alpha = 0;
        if (alphaDither >255) alpha = 255;
    }

    return tft_dev->alphaBlend(tft_dev, alpha, fgc, bgc);
}

/***************************************************************************************
** Function name:                     alphaBlend
** Description:                         Blend 24bit foreground and background with optional dither
*************************************************************************************x*/
static uint32_t alphaBlend24(struct TFT_eSPI * tft_dev, uint8_t alpha, uint32_t fgc, uint32_t bgc, uint8_t dither)
{

    if (dither) {
        int16_t alphaDither = (int16_t)alpha - dither + rand() % 2*dither - dither; // +/-dither randomised
        alpha = (uint8_t)alphaDither;
        if (alphaDither <    0) alpha = 0;
        if (alphaDither >255) alpha = 255;
    }

    // For speed use fixed point maths and rounding to permit a power of 2 division
    uint16_t fgR = ((fgc >> 15) & 0x1FE) + 1;
    uint16_t fgG = ((fgc >>    7) & 0x1FE) + 1;
    uint16_t fgB = ((fgc <<    1) & 0x1FE) + 1;

    uint16_t bgR = ((bgc >> 15) & 0x1FE) + 1;
    uint16_t bgG = ((bgc >>    7) & 0x1FE) + 1;
    uint16_t bgB = ((bgc <<    1) & 0x1FE) + 1;

    // Shift right 1 to drop rounding bit and shift right 8 to divide by 256
    uint16_t r = (((fgR * alpha) + (bgR * (255 - alpha))) >> 9);
    uint16_t g = (((fgG * alpha) + (bgG * (255 - alpha))) >> 9);
    uint16_t b = (((fgB * alpha) + (bgB * (255 - alpha))) >> 9);

    // Combine RGB colours into 24 bits
    return (r << 16) | (g << 8) | (b << 0);
}

/***************************************************************************************
** Function name:                     write
** Description:                         draw characters piped through serial stream
***************************************************************************************/
/* // Not all processors support buffered write
#ifndef ESP8266 // Avoid ESP8266 board package bug
size_t write(const uint8_t *buf, size_t len)
{
    inTransaction = true;

    uint8_t *lbuf = (uint8_t *)buf;
    while(*lbuf !=0 && len--) write(*lbuf++);

    inTransaction = lockTransaction;
    end_tft_write();
    return 1;
}
#endif
*/
/***************************************************************************************
** Function name:                     write
** Description:                         draw characters piped through serial stream
***************************************************************************************/
static size_t write(struct TFT_eSPI * tft_dev, uint8_t utf8)
{
    if (tft_dev->_vpOoB) return 1;

    uint16_t uniCode = tft_dev->decodeUTF8C(tft_dev, utf8);

    if (!uniCode) return 1;

    if (utf8 == '\r') return 1;

#ifdef SMOOTH_FONT
    if(fontLoaded) {
        if (uniCode < 32 && utf8 != '\n') return 1;

        drawGlyph(uniCode);

        return 1;
    }
#endif

    if (uniCode == '\n') uniCode+=22; // Make it a valid space character to stop errors
    else if (uniCode < 32) return 1;

    uint16_t cwidth = 0;
    uint16_t cheight = 0;

//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv DEBUG vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
    //Serial.print((uint8_t) uniCode); // Debug line sends all printed TFT text to serial port
    //Serial.println(uniCode, HEX); // Debug line sends all printed TFT text to serial port
    //delay(5);                                         // Debug optional wait for serial port to flush through
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ DEBUG ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#ifdef LOAD_GFXFF
    if(!gfxFont) {
#endif
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

#ifdef LOAD_FONT2
    if (textfont == 2) {
        if (uniCode > 127) return 1;

        cwidth = pgm_read_byte(widtbl_f16 + uniCode-32);
        cheight = chr_hgt_f16;
        // Font 2 is rendered in whole byte widths so we must allow for this
        cwidth = (cwidth + 6) / 8;    // Width in whole bytes for font 2, should be + 7 but must allow for font width change
        cwidth = cwidth * 8;                // Width converted back to pixels
    }
    #ifdef LOAD_RLE
    else
    #endif
#endif

#ifdef LOAD_RLE
    {
        if ((textfont>2) && (textfont<9)) {
            if (uniCode > 127) return 1;
            // Uses the fontinfo struct array to avoid lots of 'if' or 'switch' statements
            cwidth = pgm_read_byte( (uint8_t *)pgm_read_dword( &(fontdata[textfont].widthtbl ) ) + uniCode-32 );
            cheight= pgm_read_byte( &fontdata[textfont].height );
        }
    }
#endif

#ifdef LOAD_GLCD
    if (textfont==1) {
            cwidth =    6;
            cheight = 8;
    }
#else
    if (tft_dev->textfont==1) return 1;
#endif

    cheight = cheight * tft_dev->textsize;

    if (utf8 == '\n') {
        tft_dev->cursor_y += cheight;
        tft_dev->cursor_x    = 0;
    }
    else {
        if (tft_dev->textwrapX && (tft_dev->cursor_x + cwidth * tft_dev->textsize > tft_dev->width(tft_dev))) {
            tft_dev->cursor_y += cheight;
            tft_dev->cursor_x = 0;
        }
        if (tft_dev->textwrapY && (tft_dev->cursor_y >= (int32_t) tft_dev->height(tft_dev))) tft_dev->cursor_y = 0;
        tft_dev->cursor_x += (int32_t)(tft_dev->drawCharUniFont(tft_dev, uniCode, tft_dev->cursor_x, tft_dev->cursor_y, tft_dev->textfont));
    }

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#ifdef LOAD_GFXFF
    } // Custom GFX font
    else {
        if(utf8 == '\n') {
            cursor_x    = 0;
            cursor_y += (int16_t)textsize * (uint8_t)pgm_read_byte(&gfxFont->yAdvance);
        } else {
            if (uniCode > pgm_read_word(&gfxFont->last )) return 1;
            if (uniCode < pgm_read_word(&gfxFont->first)) return 1;

            uint16_t     c2        = uniCode - pgm_read_word(&gfxFont->first);
            GFXglyph *glyph = &(((GFXglyph *)pgm_read_dword(&gfxFont->glyph))[c2]);
            uint8_t     w         = pgm_read_byte(&glyph->width),
                                h         = pgm_read_byte(&glyph->height);
            if((w > 0) && (h > 0)) { // Is there an associated bitmap?
                int16_t xo = (int8_t)pgm_read_byte(&glyph->xOffset);
                if(textwrapX && ((cursor_x + textsize * (xo + w)) > width())) {
                    // Drawing character would go off right edge; wrap to new line
                    cursor_x    = 0;
                    cursor_y += (int16_t)textsize * (uint8_t)pgm_read_byte(&gfxFont->yAdvance);
                }
                if (textwrapY && (cursor_y >= (int32_t) height())) cursor_y = 0;
                drawChar(cursor_x, cursor_y, uniCode, textcolor, textbgcolor, textsize);
            }
            cursor_x += pgm_read_byte(&glyph->xAdvance) * (int16_t)textsize;
        }
    }
#endif // LOAD_GFXFF
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    return 1;
}


/***************************************************************************************
** Function name:                     drawChar
** Description:                         draw a Unicode glyph onto the screen
***************************************************************************************/
    // TODO: Rationalise with TFT_eSprite
    // Any UTF-8 decoding must be done before calling drawChar()
static int16_t drawCharUni(struct TFT_eSPI * tft_dev, uint16_t uniCode, int32_t x, int32_t y)
{
    return tft_dev->drawCharUniFont(tft_dev, uniCode, x, y, tft_dev->textfont);
}

    // Any UTF-8 decoding must be done before calling drawChar()
static int16_t drawCharUniFont(struct TFT_eSPI * tft_dev, uint16_t uniCode, int32_t x, int32_t y, uint8_t font)
{
    if (tft_dev->_vpOoB || !uniCode) return 0;

    if (font==1) {
#ifdef LOAD_GLCD
    #ifndef LOAD_GFXFF
        drawChar(x, y, uniCode, textcolor, textbgcolor, textsize);
        return 6 * textsize;
    #endif
#else
    #ifndef LOAD_GFXFF
        return 0;
    #endif
#endif

#ifdef LOAD_GFXFF
        drawChar(x, y, uniCode, textcolor, textbgcolor, textsize);
        if(!gfxFont) { // 'Classic' built-in font
        #ifdef LOAD_GLCD
            return 6 * textsize;
        #else
            return 0;
        #endif
        }
        else {
            if((uniCode >= pgm_read_word(&gfxFont->first)) && (uniCode <= pgm_read_word(&gfxFont->last) )) {
                uint16_t     c2        = uniCode - pgm_read_word(&gfxFont->first);
                GFXglyph *glyph = &(((GFXglyph *)pgm_read_dword(&gfxFont->glyph))[c2]);
                return pgm_read_byte(&glyph->xAdvance) * textsize;
            }
            else {
                return 0;
            }
        }
#endif
    }

    if ((font>1) && (font<9) && ((uniCode < 32) || (uniCode > 127))) return 0;

    int32_t width    = 0;
    int32_t height = 0;
    uint32_t flash_address = 0;
    uniCode -= 32;

#ifdef LOAD_FONT2
    if (font == 2) {
        flash_address = pgm_read_dword(&chrtbl_f16[uniCode]);
        width = pgm_read_byte(widtbl_f16 + uniCode);
        height = chr_hgt_f16;
    }
    #ifdef LOAD_RLE
    else
    #endif
#endif

#ifdef LOAD_RLE
    {
        if ((font>2) && (font<9)) {
            flash_address = pgm_read_dword( (const void*)(pgm_read_dword( &(fontdata[font].chartbl ) ) + uniCode*sizeof(void *)) );
            width = pgm_read_byte( (uint8_t *)pgm_read_dword( &(fontdata[font].widthtbl ) ) + uniCode );
            height= pgm_read_byte( &fontdata[font].height );
        }
    }
#endif

    int32_t xd = x + tft_dev->_xDatum;
    int32_t yd = y + tft_dev->_yDatum;

    if ((xd + width * tft_dev->textsize < tft_dev->_vpX || xd >= tft_dev->_vpW) && (yd + height * tft_dev->textsize < tft_dev->_vpY || yd >= tft_dev->_vpH)) return width * tft_dev->textsize ;

    int32_t w = width;
    int32_t pX            = 0;
    int32_t pY            = y;
    uint8_t line = 0;
    bool clip = xd < tft_dev->_vpX || xd + width    * tft_dev->textsize >= tft_dev->_vpW || yd < tft_dev->_vpY || yd + height * tft_dev->textsize >= tft_dev->_vpH;

#ifdef LOAD_FONT2 // chop out code if we do not need it
    if (font == 2) {
        w = w + 6; // Should be + 7 but we need to compensate for width increment
        w = w / 8;

        if (textcolor == textbgcolor || textsize != 1 || clip) {
            //begin_tft_write();                    // Sprite class can use this function, avoiding begin_tft_write()
            inTransaction = true;

            for (int32_t i = 0; i < height; i++) {
                if (textcolor != textbgcolor) fillRect(x, pY, width * textsize, textsize, textbgcolor);

                for (int32_t k = 0; k < w; k++) {
                    line = pgm_read_byte((uint8_t *)flash_address + w * i + k);
                    if (line) {
                        if (textsize == 1) {
                            pX = x + k * 8;
                            if (line & 0x80) drawPixel(pX, pY, textcolor);
                            if (line & 0x40) drawPixel(pX + 1, pY, textcolor);
                            if (line & 0x20) drawPixel(pX + 2, pY, textcolor);
                            if (line & 0x10) drawPixel(pX + 3, pY, textcolor);
                            if (line & 0x08) drawPixel(pX + 4, pY, textcolor);
                            if (line & 0x04) drawPixel(pX + 5, pY, textcolor);
                            if (line & 0x02) drawPixel(pX + 6, pY, textcolor);
                            if (line & 0x01) drawPixel(pX + 7, pY, textcolor);
                        }
                        else {
                            pX = x + k * 8 * textsize;
                            if (line & 0x80) fillRect(pX, pY, textsize, textsize, textcolor);
                            if (line & 0x40) fillRect(pX + textsize, pY, textsize, textsize, textcolor);
                            if (line & 0x20) fillRect(pX + 2 * textsize, pY, textsize, textsize, textcolor);
                            if (line & 0x10) fillRect(pX + 3 * textsize, pY, textsize, textsize, textcolor);
                            if (line & 0x08) fillRect(pX + 4 * textsize, pY, textsize, textsize, textcolor);
                            if (line & 0x04) fillRect(pX + 5 * textsize, pY, textsize, textsize, textcolor);
                            if (line & 0x02) fillRect(pX + 6 * textsize, pY, textsize, textsize, textcolor);
                            if (line & 0x01) fillRect(pX + 7 * textsize, pY, textsize, textsize, textcolor);
                        }
                    }
                }
                pY += textsize;
            }

            inTransaction = lockTransaction;
            end_tft_write();
        }
        else { // Faster drawing of characters and background using block write

            begin_tft_write();

            setWindow(xd, yd, xd + width - 1, yd + height - 1);

            uint8_t mask;
            for (int32_t i = 0; i < height; i++) {
                pX = width;
                for (int32_t k = 0; k < w; k++) {
                    line = pgm_read_byte((uint8_t *) (flash_address + w * i + k) );
                    mask = 0x80;
                    while (mask && pX) {
                        if (line & mask) {tft_Write_16(textcolor);}
                        else {tft_Write_16(textbgcolor);}
                        pX--;
                        mask = mask >> 1;
                    }
                }
                if (pX) {tft_Write_16(textbgcolor);}
            }

            end_tft_write();
        }
    }

    #ifdef LOAD_RLE
    else
    #endif
#endif    //FONT2

#ifdef LOAD_RLE    //674 bytes of code
    // Font is not 2 and hence is RLE encoded
    {
        begin_tft_write();
        inTransaction = true;

        w *= height; // Now w is total number of pixels in the character
        if (textcolor == textbgcolor && !clip) {

            int32_t px = 0, py = pY; // To hold character block start and end column and row values
            int32_t pc = 0; // Pixel count
            uint8_t np = textsize * textsize; // Number of pixels in a drawn pixel

            uint8_t tnp = 0; // Temporary copy of np for while loop
            uint8_t ts = textsize - 1; // Temporary copy of textsize
            // 16 bit pixel count so maximum font size is equivalent to 180x180 pixels in area
            // w is total number of pixels to plot to fill character block
            while (pc < w) {
                line = pgm_read_byte((uint8_t *)flash_address);
                flash_address++;
                if (line & 0x80) {
                    line &= 0x7F;
                    line++;
                    if (ts) {
                        px = xd + textsize * (pc % width); // Keep these px and py calculations outside the loop as they are slow
                        py = yd + textsize * (pc / width);
                    }
                    else {
                        px = xd + pc % width; // Keep these px and py calculations outside the loop as they are slow
                        py = yd + pc / width;
                    }
                    while (line--) { // In this case the while(line--) is faster
                        pc++; // This is faster than putting pc+=line before while()?
                        setWindow(px, py, px + ts, py + ts);

                        if (ts) {
                            tnp = np;
                            while (tnp--) {tft_Write_16(textcolor);}
                        }
                        else {tft_Write_16(textcolor);}
                        px += textsize;

                        if (px >= (xd + width * textsize)) {
                            px = xd;
                            py += textsize;
                        }
                    }
                }
                else {
                    line++;
                    pc += line;
                }
            }
        }
        else {
            // Text colour != background and textsize = 1 and character is within viewport area
            // so use faster drawing of characters and background using block write
            if (textcolor != textbgcolor && textsize == 1 && !clip)
            {
                setWindow(xd, yd, xd + width - 1, yd + height - 1);

                // Maximum font size is equivalent to 180x180 pixels in area
                while (w > 0) {
                    line = pgm_read_byte((uint8_t *)flash_address++); // 8 bytes smaller when incrementing here
                    if (line & 0x80) {
                        line &= 0x7F;
                        line++; w -= line;
                        pushBlock(textcolor,line);
                    }
                    else {
                        line++; w -= line;
                        pushBlock(textbgcolor,line);
                    }
                }
            }
            else
            {
                int32_t px = 0, py = 0;    // To hold character pixel coords
                int32_t tx = 0, ty = 0;    // To hold character TFT pixel coords
                int32_t pc = 0;                    // Pixel count
                int32_t pl = 0;                    // Pixel line length
                uint16_t pcol = 0;             // Pixel color
                bool         pf = true;            // Flag for plotting
                while (pc < w) {
                    line = pgm_read_byte((uint8_t *)flash_address);
                    flash_address++;
                    if (line & 0x80) { pcol = textcolor; line &= 0x7F; pf = true;}
                    else { pcol = textbgcolor; if (textcolor == textbgcolor) pf = false;}
                    line++;
                    px = pc % width;
                    tx = x + textsize * px;
                    py = pc / width;
                    ty = y + textsize * py;

                    pl = 0;
                    pc += line;
                    while (line--) {
                        pl++;
                        if ((px+pl) >= width) {
                            if (pf) fillRect(tx, ty, pl * textsize, textsize, pcol);
                            pl = 0;
                            px = 0;
                            tx = x;
                            py ++;
                            ty += textsize;
                        }
                    }
                    if (pl && pf) fillRect(tx, ty, pl * textsize, textsize, pcol);
                }
            }
        }
        inTransaction = lockTransaction;
        end_tft_write();
    }
    // End of RLE font rendering
#endif

#if !defined (LOAD_FONT2) && !defined (LOAD_RLE)
    // Stop warnings
    flash_address = flash_address;
    w = w;
    pX = pX;
    pY = pY;
    line = line;
    clip = clip;
#endif

    return width * tft_dev->textsize;        // x +
}


/***************************************************************************************
** Function name:                     drawString (with or without user defined font)
** Description :                        draw string with padding if it is defined
***************************************************************************************/
// Without font number, uses font set by setTextFont()
static int16_t drawString(struct TFT_eSPI * tft_dev, const char *string, int32_t poX, int32_t poY)
{
    return tft_dev->drawStringFont(tft_dev, string, poX, poY, tft_dev->textfont);
}

// With font number. Note: font number is over-ridden if a smooth font is loaded
static int16_t drawStringFont(struct TFT_eSPI * tft_dev, const char *string, int32_t poX, int32_t poY, uint8_t font)
{
    int16_t sumX = 0;
    uint8_t padding = 1, baseline = 0;
    uint16_t cwidth = tft_dev->textWidthFont(tft_dev, string, font); // Find the pixel width of the string in the font
    uint16_t cheight = 8 * tft_dev->textsize;

#ifdef LOAD_GFXFF
    #ifdef SMOOTH_FONT
        bool freeFont = (font == 1 && gfxFont && !fontLoaded);
    #else
        bool freeFont = (font == 1 && gfxFont);
    #endif

    if (freeFont) {
        cheight = glyph_ab * textsize;
        poY += cheight; // Adjust for baseline datum of free fonts
        baseline = cheight;
        padding =101; // Different padding method used for Free Fonts

        // We need to make an adjustment for the bottom of the string (eg 'y' character)
        if ((textdatum == BL_DATUM) || (textdatum == BC_DATUM) || (textdatum == BR_DATUM)) {
            cheight += glyph_bb * textsize;
        }
    }
#endif


    // If it is not font 1 (GLCD or free font) get the baseline and pixel height of the font
#ifdef SMOOTH_FONT
    if(fontLoaded) {
        baseline = gFont.maxAscent;
        cheight    = fontHeight();
    }
    else
#endif
    if (font!=1) {
        baseline = pgm_read_byte( &fontdata[font].baseline ) * tft_dev->textsize;
        cheight = tft_dev->fontHeight(tft_dev, font);
    }

    if (tft_dev->textdatum || tft_dev->padX) {

        switch(tft_dev->textdatum) {
            case TC_DATUM:
                poX -= cwidth/2;
                padding += 1;
                break;
            case TR_DATUM:
                poX -= cwidth;
                padding += 2;
                break;
            case ML_DATUM:
                poY -= cheight/2;
                //padding += 0;
                break;
            case MC_DATUM:
                poX -= cwidth/2;
                poY -= cheight/2;
                padding += 1;
                break;
            case MR_DATUM:
                poX -= cwidth;
                poY -= cheight/2;
                padding += 2;
                break;
            case BL_DATUM:
                poY -= cheight;
                //padding += 0;
                break;
            case BC_DATUM:
                poX -= cwidth/2;
                poY -= cheight;
                padding += 1;
                break;
            case BR_DATUM:
                poX -= cwidth;
                poY -= cheight;
                padding += 2;
                break;
            case L_BASELINE:
                poY -= baseline;
                //padding += 0;
                break;
            case C_BASELINE:
                poX -= cwidth/2;
                poY -= baseline;
                padding += 1;
                break;
            case R_BASELINE:
                poX -= cwidth;
                poY -= baseline;
                padding += 2;
                break;
        }
    }


    int8_t xo = 0;
#ifdef LOAD_GFXFF
    if (freeFont && (textcolor!=textbgcolor)) {
            cheight = (glyph_ab + glyph_bb) * textsize;
            // Get the offset for the first character only to allow for negative offsets
            uint16_t c2 = 0;
            uint16_t len = strlen(string);
            uint16_t n = 0;

            while (n < len && c2 == 0) c2 = decodeUTF8((uint8_t*)string, &n, len - n);

            if((c2 >= pgm_read_word(&gfxFont->first)) && (c2 <= pgm_read_word(&gfxFont->last) )) {
                c2 -= pgm_read_word(&gfxFont->first);
                GFXglyph *glyph = &(((GFXglyph *)pgm_read_dword(&gfxFont->glyph))[c2]);
                xo = pgm_read_byte(&glyph->xOffset) * textsize;
                // Adjust for negative xOffset
                if (xo > 0) xo = 0;
                else cwidth -= xo;
                // Add 1 pixel of padding all round
                //cheight +=2;
                //fillRect(poX+xo-1, poY - 1 - glyph_ab * textsize, cwidth+2, cheight, textbgcolor);
                fillRect(poX+xo, poY - glyph_ab * textsize, cwidth, cheight, textbgcolor);
            }
            padding -=100;
        }
#endif

    uint16_t len = strlen(string);
    uint16_t n = 0;

#ifdef SMOOTH_FONT
    if(fontLoaded) {
        setCursor(poX, poY);

        bool fillbg = _fillbg;
        // If padding is requested then fill the text background
        if (padX && !_fillbg) _fillbg = true;

        while (n < len) {
            uint16_t uniCode = decodeUTF8((uint8_t*)string, &n, len - n);
            drawGlyph(uniCode);
        }
        _fillbg = fillbg; // restore state
        sumX += cwidth;
        //fontFile.close();
    }
    else
#endif
    {
        while (n < len) {
            uint16_t uniCode = tft_dev->decodeUTF8(tft_dev, (uint8_t*)string, &n, len - n);
            sumX += tft_dev->drawCharUniFont(tft_dev, uniCode, poX+sumX, poY, font);
        }
    }

//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv DEBUG vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// Switch on debugging for the padding areas
//#define PADDING_DEBUG

#ifndef PADDING_DEBUG
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ DEBUG ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    if((tft_dev->padX>cwidth) && (tft_dev->textcolor!=tft_dev->textbgcolor)) {
        int16_t padXc = poX+cwidth+xo;
#ifdef LOAD_GFXFF
        if (freeFont) {
            poX +=xo; // Adjust for negative offset start character
            poY -= glyph_ab * textsize;
            sumX += poX;
        }
#endif
        switch(padding) {
            case 1:
                tft_dev->fillRect(tft_dev, padXc,poY,tft_dev->padX-cwidth,cheight, tft_dev->textbgcolor);
                break;
            case 2:
                tft_dev->fillRect(tft_dev, padXc,poY,(tft_dev->padX-cwidth)>>1,cheight, tft_dev->textbgcolor);
                padXc = poX - ((tft_dev->padX-cwidth)>>1);
                tft_dev->fillRect(tft_dev, padXc,poY,(tft_dev->padX-cwidth)>>1,cheight, tft_dev->textbgcolor);
                break;
            case 3:
                if (padXc>tft_dev->padX) padXc = tft_dev->padX;
                tft_dev->fillRect(tft_dev, poX + cwidth - padXc,poY,padXc-cwidth,cheight, tft_dev->textbgcolor);
                break;
        }
    }


#else

//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv DEBUG vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// This is debug code to show text (green box) and blanked (white box) areas
// It shows that the padding areas are being correctly sized and positioned

    if((padX>sumX) && (textcolor!=textbgcolor)) {
        int16_t padXc = poX+sumX; // Maximum left side padding
#ifdef LOAD_GFXFF
        if ((font == 1) && (gfxFont)) poY -= glyph_ab;
#endif
        drawRect(poX,poY,sumX,cheight, TFT_GREEN);
        switch(padding) {
            case 1:
                drawRect(padXc,poY,padX-sumX,cheight, TFT_WHITE);
                break;
            case 2:
                drawRect(padXc,poY,(padX-sumX)>>1, cheight, TFT_WHITE);
                padXc = (padX-sumX)>>1;
                drawRect(poX - padXc,poY,(padX-sumX)>>1,cheight, TFT_WHITE);
                break;
            case 3:
                if (padXc>padX) padXc = padX;
                drawRect(poX + sumX - padXc,poY,padXc-sumX,cheight, TFT_WHITE);
                break;
        }
    }
#endif
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ DEBUG ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

return sumX;
}


/***************************************************************************************
** Function name:                     drawCentreString (deprecated, use setTextDatum())
** Descriptions:                        draw string centred on dX
***************************************************************************************/
static int16_t drawCentreString(struct TFT_eSPI * tft_dev, const char *string, int32_t dX, int32_t poY, uint8_t font)
{
    uint8_t tempdatum = tft_dev->textdatum;
    int32_t sumX = 0;
    tft_dev->textdatum = TC_DATUM;
    sumX = tft_dev->drawStringFont(tft_dev, string, dX, poY, font);
    tft_dev->textdatum = tempdatum;
    return sumX;
}


/***************************************************************************************
** Function name:                     drawRightString (deprecated, use setTextDatum())
** Descriptions:                        draw string right justified to dX
***************************************************************************************/
static int16_t drawRightString(struct TFT_eSPI * tft_dev, const char *string, int32_t dX, int32_t poY, uint8_t font)
{
    uint8_t tempdatum = tft_dev->textdatum;
    int16_t sumX = 0;
    tft_dev->textdatum = TR_DATUM;
    sumX = tft_dev->drawStringFont(tft_dev, string, dX, poY, font);
    tft_dev->textdatum = tempdatum;
    return sumX;
}


/***************************************************************************************
** Function name:                     drawNumber
** Description:                         draw a long integer
***************************************************************************************/
static int16_t drawNumber(struct TFT_eSPI * tft_dev, long long_num, int32_t poX, int32_t poY)
{
    tft_dev->isDigits = true; // Eliminate jiggle in monospaced fonts
    char str[12];
    utoa(long_num, str, 10);
    return tft_dev->drawStringFont(tft_dev, str, poX, poY, tft_dev->textfont);
}

static int16_t drawNumberFont(struct TFT_eSPI * tft_dev, long long_num, int32_t poX, int32_t poY, uint8_t font)
{
    tft_dev->isDigits = true; // Eliminate jiggle in monospaced fonts
    char str[12];
    utoa(long_num, str, 10);
    return tft_dev->drawStringFont(tft_dev, str, poX, poY, font);
}


/***************************************************************************************
** Function name:                     drawFloat
** Descriptions:                        drawFloat, prints 7 non zero digits maximum
***************************************************************************************/
// Assemble and print a string, this permits alignment relative to a datum
// looks complicated but much more compact and actually faster than using print class
static int16_t drawFloat(struct TFT_eSPI * tft_dev, float floatNumber, uint8_t dp, int32_t poX, int32_t poY)
{
    return tft_dev->drawFloatFont(tft_dev, floatNumber, dp, poX, poY, tft_dev->textfont);
}

int16_t drawFloatFont(struct TFT_eSPI * tft_dev, float floatNumber, uint8_t dp, int32_t poX, int32_t poY, uint8_t font)
{
    tft_dev->isDigits = true;
    char str[14];                             // Array to contain decimal string
    uint8_t ptr = 0;                        // Initialise pointer for array
    int8_t    digits = 1;                 // Count the digits to avoid array overflow
    float rounding = 0.5;             // Round up down delta
    bool negative = false;

    if (dp > 7) dp = 7; // Limit the size of decimal portion

    // Adjust the rounding value
    for (uint8_t i = 0; i < dp; ++i) rounding /= 10.0;

    if (floatNumber < -rounding) {     // add sign, avoid adding - sign to 0.0!
        str[ptr++] = '-'; // Negative number
        str[ptr] = 0; // Put a null in the array as a precaution
        digits = 0;     // Set digits to 0 to compensate so pointer value can be used later
        floatNumber = -floatNumber; // Make positive
        negative = true;
    }

    floatNumber += rounding; // Round up or down

    if (dp == 0) {
        if (negative) floatNumber = -floatNumber;
        return tft_dev->drawNumberFont(tft_dev, (long)floatNumber, poX, poY, font);
    }

    // For error put ... in string and return (all TFT_eSPI library fonts contain . character)
    if (floatNumber >= 2147483647) {
        strcpy(str, "...");
        return tft_dev->drawStringFont(tft_dev, str, poX, poY, font);
    }
    // No chance of overflow from here on

    // Get integer part
    uint32_t temp = (uint32_t)floatNumber;

    // Put integer part into array
    utoa(temp, str + ptr, 10);

    // Find out where the null is to get the digit count loaded
    while ((uint8_t)str[ptr] != 0) ptr++; // Move the pointer along
    digits += ptr;                                    // Count the digits

    str[ptr++] = '.'; // Add decimal point
    str[ptr] = '0';     // Add a dummy zero
    str[ptr + 1] = 0; // Add a null but don't increment pointer so it can be overwritten

    // Get the decimal portion
    floatNumber = floatNumber - temp;

    // Get decimal digits one by one and put in array
    // Limit digit count so we don't get a false sense of resolution
    uint8_t i = 0;
    while ((i < dp) && (digits < 9)) { // while (i < dp) for no limit but array size must be increased
        i++;
        floatNumber *= 10;             // for the next decimal
        temp = floatNumber;            // get the decimal
        utoa(temp, str + ptr, 10);
        ptr++; digits++;                 // Increment pointer and digits count
        floatNumber -= temp;         // Remove that digit
    }

    // Finally we can plot the string and return pixel length
    return tft_dev->drawStringFont(tft_dev, str, poX, poY, font);
}


/***************************************************************************************
** Function name:                     setFreeFont
** Descriptions:                        Sets the GFX free font to use
***************************************************************************************/

#ifdef LOAD_GFXFF

void setFreeFont(struct TFT_eSPI * tft_dev, const GFXfont *f)
{
    if (f == nullptr) { // Fix issue #400 (ESP32 crash)
        setTextFont(1); // Use GLCD font
        return;
    }

    textfont = 1;
    gfxFont = (GFXfont *)f;

    glyph_ab = 0;
    glyph_bb = 0;
    uint16_t numChars = pgm_read_word(&gfxFont->last) - pgm_read_word(&gfxFont->first);

    // Find the biggest above and below baseline offsets
    for (uint16_t c = 0; c < numChars; c++) {
        GFXglyph *glyph1    = &(((GFXglyph *)pgm_read_dword(&gfxFont->glyph))[c]);
        int8_t ab = -pgm_read_byte(&glyph1->yOffset);
        if (ab > glyph_ab) glyph_ab = ab;
        int8_t bb = pgm_read_byte(&glyph1->height) - ab;
        if (bb > glyph_bb) glyph_bb = bb;
    }
}


/***************************************************************************************
** Function name:                     setTextFont
** Description:                         Set the font for the print stream
***************************************************************************************/
void setTextFont(struct TFT_eSPI * tft_dev, uint8_t f)
{
    textfont = (f > 0) ? f : 1; // Don't allow font 0
    gfxFont = NULL;
}

#else


/***************************************************************************************
** Function name:                     setFreeFont
** Descriptions:                        Sets the GFX free font to use
***************************************************************************************/

// Alternative to setTextFont() so we don't need two different named functions
static void setFreeFont(struct TFT_eSPI * tft_dev, uint8_t font)
{
    tft_dev->setTextFont(tft_dev, font);
}


/***************************************************************************************
** Function name:                     setTextFont
** Description:                         Set the font for the print stream
***************************************************************************************/
static void setTextFont(struct TFT_eSPI * tft_dev, uint8_t f)
{
    tft_dev->textfont = (f > 0) ? f : 1; // Don't allow font 0
}
#endif


/***************************************************************************************
** Function name:                     getSPIinstance
** Description:                         Get the instance of the SPI class
***************************************************************************************/
#if 0
#if !defined (TFT_PARALLEL_8_BIT) && ! defined (RP2040_PIO_INTERFACE)
SPIClass& getSPIinstance(struct TFT_eSPI * tft_dev)
{
    return spi;
}
#endif
#endif

/***************************************************************************************
** Function name:                     verifySetupID
** Description:                         Compare the ID if USER_SETUP_ID defined in user setup file
***************************************************************************************/
static bool verifySetupID(struct TFT_eSPI * tft_dev, uint32_t id)
{
#if defined (USER_SETUP_ID)
    if (USER_SETUP_ID == id) return true;
#else
    id = id; // Avoid warning
#endif
    return false;
}

/***************************************************************************************
** Function name:                     getSetup
** Description:                         Get the setup details for diagnostic and sketch access
***************************************************************************************/
static void getSetup(struct TFT_eSPI * tft_dev, setup_t *tft_settings)
{
// tft_settings->version is set in header file

#if defined (USER_SETUP_INFO)
    tft_settings->setup_info = USER_SETUP_INFO;
#else
    tft_settings->setup_info = "NA";
#endif

#if defined (USER_SETUP_ID)
    tft_settings->setup_id = USER_SETUP_ID;
#else
    tft_settings->setup_id = 0;
#endif

#if defined (PROCESSOR_ID)
    tft_settings->esp = PROCESSOR_ID;
#else
    tft_settings->esp = -1;
#endif

#if defined (SUPPORT_TRANSACTIONS)
    tft_settings->trans = true;
#else
    tft_settings->trans = false;
#endif

#if defined (TFT_PARALLEL_8_BIT) || defined(TFT_PARALLEL_16_BIT)
    tft_settings->serial = false;
    tft_settings->tft_spi_freq = 0;
#else
    tft_settings->serial = true;
    tft_settings->tft_spi_freq = SPI_FREQUENCY/100000;
    #ifdef SPI_READ_FREQUENCY
        tft_settings->tft_rd_freq = SPI_READ_FREQUENCY/100000;
    #endif
    #ifdef TFT_SPI_PORT
        tft_settings->port = TFT_SPI_PORT;
    #else
        tft_settings->port = 255;
    #endif
    #ifdef RP2040_PIO_SPI
        tft_settings->interface = 0x10;
    #else
        tft_settings->interface = 0x0;
    #endif
#endif

#if defined(TFT_SPI_OVERLAP)
    tft_settings->overlap = true;
#else
    tft_settings->overlap = false;
#endif

    tft_settings->tft_driver = TFT_DRIVER;
    tft_settings->tft_width  = tft_dev->_init_width;
    tft_settings->tft_height = tft_dev->_init_height;

#ifdef CGRAM_OFFSET
    tft_settings->r0_x_offset = tft_dev->colstart;
    tft_settings->r0_y_offset = tft_dev->rowstart;
    tft_settings->r1_x_offset = 0;
    tft_settings->r1_y_offset = 0;
    tft_settings->r2_x_offset = 0;
    tft_settings->r2_y_offset = 0;
    tft_settings->r3_x_offset = 0;
    tft_settings->r3_y_offset = 0;
#else
    tft_settings->r0_x_offset = 0;
    tft_settings->r0_y_offset = 0;
    tft_settings->r1_x_offset = 0;
    tft_settings->r1_y_offset = 0;
    tft_settings->r2_x_offset = 0;
    tft_settings->r2_y_offset = 0;
    tft_settings->r3_x_offset = 0;
    tft_settings->r3_y_offset = 0;
#endif

#if defined (TFT_MOSI)
    tft_settings->pin_tft_mosi = TFT_MOSI;
#else
    tft_settings->pin_tft_mosi = -1;
#endif

#if defined (TFT_MISO)
    tft_settings->pin_tft_miso = TFT_MISO;
#else
    tft_settings->pin_tft_miso = -1;
#endif

#if defined (TFT_SCLK)
    tft_settings->pin_tft_clk    = TFT_SCLK;
#else
    tft_settings->pin_tft_clk    = -1;
#endif

#if defined (TFT_CS)
    tft_settings->pin_tft_cs     = TFT_CS;
#else
    tft_settings->pin_tft_cs     = -1;
#endif

#if defined (TFT_DC)
    tft_settings->pin_tft_dc    = TFT_DC;
#else
    tft_settings->pin_tft_dc    = -1;
#endif

#if defined (TFT_RD)
    tft_settings->pin_tft_rd    = TFT_RD;
#else
    tft_settings->pin_tft_rd    = -1;
#endif

#if defined (TFT_WR)
    tft_settings->pin_tft_wr    = TFT_WR;
#else
    tft_settings->pin_tft_wr    = -1;
#endif

#if defined (TFT_RST)
    tft_settings->pin_tft_rst = TFT_RST;
#else
    tft_settings->pin_tft_rst = -1;
#endif

#if defined (TFT_PARALLEL_8_BIT) || defined(TFT_PARALLEL_16_BIT)
    tft_settings->pin_tft_d0 = TFT_D0;
    tft_settings->pin_tft_d1 = TFT_D1;
    tft_settings->pin_tft_d2 = TFT_D2;
    tft_settings->pin_tft_d3 = TFT_D3;
    tft_settings->pin_tft_d4 = TFT_D4;
    tft_settings->pin_tft_d5 = TFT_D5;
    tft_settings->pin_tft_d6 = TFT_D6;
    tft_settings->pin_tft_d7 = TFT_D7;
#else
    tft_settings->pin_tft_d0 = -1;
    tft_settings->pin_tft_d1 = -1;
    tft_settings->pin_tft_d2 = -1;
    tft_settings->pin_tft_d3 = -1;
    tft_settings->pin_tft_d4 = -1;
    tft_settings->pin_tft_d5 = -1;
    tft_settings->pin_tft_d6 = -1;
    tft_settings->pin_tft_d7 = -1;
#endif

#if defined (TFT_BL)
    tft_settings->pin_tft_led = TFT_BL;
#endif

#if defined (TFT_BACKLIGHT_ON)
    tft_settings->pin_tft_led_on = TFT_BACKLIGHT_ON;
#endif

#if defined (TOUCH_CS)
    tft_settings->pin_tch_cs     = TOUCH_CS;
    tft_settings->tch_spi_freq = SPI_TOUCH_FREQUENCY/100000;
#else
    tft_settings->pin_tch_cs     = -1;
    tft_settings->tch_spi_freq = 0;
#endif
}

#ifdef TFT_PARALLEL_8_BIT
/***************************************************************************************
** Function name:           GPIO direction control  - supports class functions
** Description:             Set parallel bus to INPUT or OUTPUT
***************************************************************************************/
void busDir(uint32_t mask, uint8_t mode)
{
  // Arduino generic native function
  pinMode(TFT_D0, mode);
  pinMode(TFT_D1, mode);
  pinMode(TFT_D2, mode);
  pinMode(TFT_D3, mode);
  pinMode(TFT_D4, mode);
  pinMode(TFT_D5, mode);
  pinMode(TFT_D6, mode);
  pinMode(TFT_D7, mode);
}

/***************************************************************************************
** Function name:           GPIO direction control  - supports class functions
** Description:             Set ESP32 GPIO pin to input or output (set high) ASAP
***************************************************************************************/
void gpioMode(uint8_t gpio, uint8_t mode)
{
  pinMode(gpio, mode);
  digitalWrite(gpio, HIGH);
}
#endif // #ifdef TFT_PARALLEL_8_BIT

/***************************************************************************************
** Function name:           pushBlock - for STM32
** Description:             Write a block of pixels of the same colour
***************************************************************************************/
static void pushBlock(struct TFT_eSPI *tft_dev, uint16_t color, uint32_t len)
{
    //GB_DEBUGI(DISP_TAG, "PUSH block, color: %04x, len: %d", color, len);
    uint16_t color_buffer[len];

    for (int i = 0; i < len; i++)
        color_buffer[i] = color;
    CHK_LOGE(tft_dev->bus->writeBytes(tft_dev->bus, GB_SPI_DEV_0, 0x00, sizeof(color_buffer), (const uint8_t*)color_buffer), "TFT pushBlock failed\n");
    //GB_DEBUGI(DISP_TAG, "PUSH block ok");
    // while (len--) { tft_Write_16(color); }
}

/***************************************************************************************
** Function name:           pushPixels - for STM32
** Description:             Write a sequence of pixels
***************************************************************************************/
static void pushPixels(struct TFT_eSPI * tft_dev, const void* data_in, uint32_t len){
    uint16_t *data = (uint16_t*)data_in;

    if(tft_dev->_swapBytes) {
        for (uint32_t i = 0; i < len; i++) (data[i] = data[i] << 8 | data[i] >> 8);
    }

    CHK_LOGE(tft_dev->bus->writeBytes(tft_dev->bus, GB_SPI_DEV_0, 0x00, len * 2, data_in), "TFT pushPixels failed\n");
}
