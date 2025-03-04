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
#include "inttypes.h"
#include "stdbool.h"

struct TFT_eSprite;

void TFT_eSprite_init(struct TFT_eSprite* sprite, struct TFT_eSPI *tft);

struct TFT_eSprite{
           // Create a sprite of width x height pixels, return a pointer to the RAM area
           // Sketch can cast returned value to (uint16_t*) for 16 bit depth if needed
           // RAM required is:
           //  - 1 bit per pixel for 1 bit colour depth
           //  - 1 nibble per pixel for 4 bit colour (with palette table)
           //  - 1 byte per pixel for 8 bit colour (332 RGB format)
           //  - 2 bytes per pixel for 16 bit color depth (565 RGB format)
  void*    (*createSprite)(struct TFT_eSprite* sprite, int16_t width, int16_t height, uint8_t frames); //default frames=1

           // Returns a pointer to the sprite or nullptr if not created, user must cast to pointer type
  void*    (*getPointer)(struct TFT_eSprite* sprite);

           // Returns true if sprite has been created
  bool     (*created)(struct TFT_eSprite* sprite);

           // Delete the sprite to free up the RAM
  void     (*deleteSprite)(struct TFT_eSprite* sprite);

           // Select the frame buffer for graphics write (for 2 colour ePaper and DMA toggle buffer)
           // Returns a pointer to the Sprite frame buffer
  void*    (*frameBuffer)(struct TFT_eSprite* sprite, int8_t f);

           // Set or get the colour depth to 1, 4, 8 or 16 bits. Can be used to change depth an existing
           // sprite, but clears it to black, returns a new pointer if sprite is re-created.
  void*    (*setColorDepth)(struct TFT_eSprite* sprite, int8_t b);
  int8_t   (*getColorDepth)(struct TFT_eSprite* sprite);

           // Set the palette for a 4 bit depth sprite.  Only the first 16 colours in the map are used.
  void     (*createPaletteRam)(struct TFT_eSprite* sprite, uint16_t *palette, uint8_t colors);       // Palette in RAM, default: colors=16
  void     (*createPaletteFlash)(struct TFT_eSprite* sprite, const uint16_t *palette, uint8_t colors); // Palette in FLASH, default: colors=16

           // Set a single palette index to the given color
  void     (*setPaletteColor)(struct TFT_eSprite* sprite, uint8_t index, uint16_t color);

           // Get the color at the given palette index
  uint16_t (*getPaletteColor)(struct TFT_eSprite* sprite, uint8_t index);

           // Set foreground and background colours for 1 bit per pixel Sprite
  void     (*setBitmapColor)(struct TFT_eSprite* sprite, uint16_t fg, uint16_t bg);

           // Draw a single pixel at x,y
  void     (*drawPixel)(struct TFT_eSprite* sprite, int32_t x, int32_t y, uint32_t color);

           // Draw a single character in the GLCD or GFXFF font
  void     (*drawChar)(struct TFT_eSprite* sprite, int32_t x, int32_t y, uint16_t c, uint32_t color, uint32_t bg, uint8_t size);

           // Fill Sprite with a colour
  void     (*fillSprite)(struct TFT_eSprite* sprite, uint32_t color);

           // Define a window to push 16 bit colour pixels into in a raster order
           // Colours are converted to the set Sprite colour bit depth
  void     (*setWindow)(struct TFT_eSprite* sprite, int32_t x0, int32_t y0, int32_t x1, int32_t y1);
           // Push a color (aka singe pixel) to the sprite's set window area
  void     (*pushColor)(struct TFT_eSprite* sprite, uint16_t color);
           // Push len colors (pixels) to the sprite's set window area
  void     (*pushColorLen)(struct TFT_eSprite* sprite, uint16_t color, uint32_t len);
           // Push a pixel pre-formatted as a 1, 4, 8 or 16 bit colour (avoids conversion overhead)
  void     (*writeColor)(struct TFT_eSprite* sprite, uint16_t color);

           // Set the scroll zone, top left corner at x,y with defined width and height
           // The colour (optional, black is default) is used to fill the gap after the scroll
  void     (*setScrollRect)(struct TFT_eSprite* sprite, int32_t x, int32_t y, int32_t w, int32_t h, uint16_t color); //default, color=TFT_BLACK
           // Scroll the defined zone dx,dy pixels. Negative values left,up, positive right,down
           // dy is optional (default is 0, so no up/down scroll).
           // The sprite coordinate frame does not move because pixels are moved
  void     (*scroll)(struct TFT_eSprite* sprite, int16_t dx, int16_t dy); //default, dy=0

           // Draw lines
  void     (*drawLine)(struct TFT_eSprite* sprite, int32_t x0, int32_t y0, int32_t x1, int32_t y1, uint32_t color);
  void     (*drawFastVLine)(struct TFT_eSprite* sprite, int32_t x, int32_t y, int32_t h, uint32_t color);
  void     (*drawFastHLine)(struct TFT_eSprite* sprite, int32_t x, int32_t y, int32_t w, uint32_t color);

           // Fill a rectangular area with a color (aka draw a filled rectangle)
  void     (*fillRect)(struct TFT_eSprite* sprite, int32_t x, int32_t y, int32_t w, int32_t h, uint32_t color);

           // Set the coordinate rotation of the Sprite (for 1bpp Sprites only)
           // Note: this uses coordinate rotation and is primarily for ePaper which does not support
           // CGRAM rotation (like TFT drivers do) within the displays internal hardware
  void     (*setRotation)(struct TFT_eSprite* sprite, uint8_t rotation);
  uint8_t  (*getRotation)(struct TFT_eSprite* sprite);

           // Push a rotated copy of Sprite to TFT with optional transparent colour
  bool     (*pushRotated)(struct TFT_eSprite* sprite, int16_t angle, uint32_t transp); //default, transp=0x00FFFFFF
           // Push a rotated copy of Sprite to another different Sprite with optional transparent colour
  bool     (*pushRotatedSprite)(struct TFT_eSprite* sprite, struct TFT_eSprite* des_sprite, int16_t angle, uint32_t transp); //default, transp=0x00FFFFFF

           // Get the TFT bounding box for a rotated copy of this Sprite
  bool     (*getRotatedBounds)(struct TFT_eSprite* sprite, int16_t angle, int16_t *min_x, int16_t *min_y, int16_t *max_x, int16_t *max_y);
           // Get the destination Sprite bounding box for a rotated copy of this Sprite
  bool     (*getRotatedBoundsCopy)(struct TFT_eSprite* sprite, struct TFT_eSprite* des_spr, int16_t angle, int16_t *min_x, int16_t *min_y,
                                                             int16_t *max_x, int16_t *max_y);
           // Bounding box support function
  void     (*getRotatedBoundsBound)(struct TFT_eSprite* sprite, int16_t angle, int16_t w, int16_t h, int16_t xp, int16_t yp,
                            int16_t *min_x, int16_t *min_y, int16_t *max_x, int16_t *max_y);

           // Read the colour of a pixel at x,y and return value in 565 format
  uint16_t (*readPixel)(struct TFT_eSprite* sprite, int32_t x0, int32_t y0);

           // return the numerical value of the pixel at x,y (used when scrolling)
           // 16bpp = colour, 8bpp = byte, 4bpp = colour index, 1bpp = 1 or 0
  uint16_t (*readPixelValue)(struct TFT_eSprite* sprite, int32_t x, int32_t y);

           // Write an image (colour bitmap) to the sprite.
  void     (*pushImageRam)(struct TFT_eSprite* sprite, int32_t x0, int32_t y0, int32_t w, int32_t h, uint16_t *data, uint8_t sbpp); //default, sbpp=0
  void     (*pushImageFlash)(struct TFT_eSprite* sprite, int32_t x0, int32_t y0, int32_t w, int32_t h, const uint16_t *data);

           // Push the sprite to the TFT screen, this fn calls pushImage() in the TFT class.
           // Optionally a "transparent" colour can be defined, pixels of that colour will not be rendered
  void     (*pushSprite)(struct TFT_eSprite* sprite, int32_t x, int32_t y);
  void     (*pushSpriteTrans)(struct TFT_eSprite* sprite, int32_t x, int32_t y, uint16_t transparent);

           // Push a windowed area of the sprite to the TFT at tx, ty
  bool     (*pushSpriteWin)(struct TFT_eSprite* sprite, int32_t tx, int32_t ty, int32_t sx, int32_t sy, int32_t sw, int32_t sh);

           // Push the sprite to another sprite at x,y. This fn calls pushImage() in the destination sprite (dspr) class.
  bool     (*pushToSprite)(struct TFT_eSprite* sprite, struct TFT_eSprite *dspr, int32_t x, int32_t y);
  bool     (*pushToSpriteTrans)(struct TFT_eSprite* sprite, struct TFT_eSprite *dspr, int32_t x, int32_t y, uint16_t transparent);

           // Draw a single character in the selected font
  int16_t  (*drawCharUniFont)(struct TFT_eSprite* sprite, uint16_t uniCode, int32_t x, int32_t y, uint8_t font);
  int16_t  (*drawCharUni)(struct TFT_eSprite* sprite, uint16_t uniCode, int32_t x, int32_t y);

           // Return the width and height of the sprite
  int16_t  (*width)(struct TFT_eSprite* sprite);
  int16_t  (*height)(struct TFT_eSprite* sprite);

           // Functions associated with anti-aliased fonts
           // Draw a single unicode character using the loaded font
  void     (*drawGlyph)(struct TFT_eSprite* sprite, uint16_t code);
           // Print string to sprite using loaded font at cursor position
  //void     (*printToSprite)(struct TFT_eSprite* sprite, String string);
           // Print char array to sprite using loaded font at cursor position
  void     (*printToSprite)(struct TFT_eSprite* sprite, char *cbuffer, uint16_t len);
           // Print indexed glyph to sprite using loaded font at x,y
  int16_t  (*printToSpriteXy)(struct TFT_eSprite* sprite, int16_t x, int16_t y, uint16_t index);

           // Reserve memory for the Sprite and return a pointer
  void*    (*callocSprite)(struct TFT_eSprite* sprite, int16_t width, int16_t height, uint8_t frames); //default frames=1

  struct TFT_eSPI *_tft;

  uint8_t  _bpp;     // bits per pixel (1, 4, 8 or 16)
  uint16_t *_img;    // pointer to 16 bit sprite
  uint8_t  *_img8;   // pointer to  1 and 8 bit sprite frame 1 or frame 2
  uint8_t  *_img4;   // pointer to  4 bit sprite (uses color map)
  uint8_t  *_img8_1; // pointer to frame 1
  uint8_t  *_img8_2; // pointer to frame 2

  uint16_t *_colorMap; // color map pointer: 16 entries, used with 4 bit color map.

  int32_t  _sinra;   // Sine of rotation angle in fixed point
  int32_t  _cosra;   // Cosine of rotation angle in fixed point

  bool     _created; // A Sprite has been created and memory reserved
  bool     _gFont;

  int32_t  _xs, _ys, _xe, _ye, _xptr, _yptr; // for setWindow
  int32_t  _sx, _sy; // x,y for scroll zone
  uint32_t _sw, _sh; // w,h for scroll zone
  uint32_t _scolor;  // gap fill colour for scroll zone

  int32_t  _iwidth, _iheight; // Sprite memory image bit width and height (swapped during rotations)
  int32_t  _dwidth, _dheight; // Real sprite width and height (for <8bpp Sprites)
  int32_t  _bitwidth;         // Sprite image bit width for drawPixel (for <8bpp Sprites, not swapped)
};
