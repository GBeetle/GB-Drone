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

#include "tft_sprite.h"

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

static inline void swap_coord(int32_t *a, int32_t *b)
{ int32_t t = *a; *a = *b; *b = t; }

static void*    createSprite(struct TFT_eSprite* sprite, int16_t width, int16_t height, uint8_t frames); //default frames=1
static void*    getPointer(struct TFT_eSprite* sprite);
static bool     created(struct TFT_eSprite* sprite);
static void     deleteSprite(struct TFT_eSprite* sprite);
static void*    frameBuffer(struct TFT_eSprite* sprite, int8_t f);
static void*    setColorDepth(struct TFT_eSprite* sprite, int8_t b);
static int8_t   getColorDepth(struct TFT_eSprite* sprite);
static void     createPaletteRam(struct TFT_eSprite* sprite, uint16_t *palette, uint8_t colors);       // Palette in RAM, default: colors=16
static void     createPaletteFlash(struct TFT_eSprite* sprite, const uint16_t *palette, uint8_t colors); // Palette in FLASH, default: colors=16
static void     setPaletteColor(struct TFT_eSprite* sprite, uint8_t index, uint16_t color);
static uint16_t getPaletteColor(struct TFT_eSprite* sprite, uint8_t index);
static void     setBitmapColor(struct TFT_eSprite* sprite, uint16_t fg, uint16_t bg);
static void     drawPixel(struct TFT_eSprite* sprite, int32_t x, int32_t y, uint32_t color);
static void     drawChar(struct TFT_eSprite* sprite, int32_t x, int32_t y, uint16_t c, uint32_t color, uint32_t bg, uint8_t size);
static void     fillSprite(struct TFT_eSprite* sprite, uint32_t color);
static void     setWindow(struct TFT_eSprite* sprite, int32_t x0, int32_t y0, int32_t x1, int32_t y1);
static void     pushColor(struct TFT_eSprite* sprite, uint16_t color);
static void     pushColorLen(struct TFT_eSprite* sprite, uint16_t color, uint32_t len);
static void     writeColor(struct TFT_eSprite* sprite, uint16_t color);
static void     setScrollRect(struct TFT_eSprite* sprite, int32_t x, int32_t y, int32_t w, int32_t h, uint16_t color);
static void     scroll(struct TFT_eSprite* sprite, int16_t dx, int16_t dy);
static void     drawLine(struct TFT_eSprite* sprite, int32_t x0, int32_t y0, int32_t x1, int32_t y1, uint32_t color);
static void     drawFastVLine(struct TFT_eSprite* sprite, int32_t x, int32_t y, int32_t h, uint32_t color);
static void     drawFastHLine(struct TFT_eSprite* sprite, int32_t x, int32_t y, int32_t w, uint32_t color);
static void     fillRect(struct TFT_eSprite* sprite, int32_t x, int32_t y, int32_t w, int32_t h, uint32_t color);
static void     setRotation(struct TFT_eSprite* sprite, uint8_t rotation);
static uint8_t  getRotation(struct TFT_eSprite* sprite);
static bool     pushRotated(struct TFT_eSprite* sprite, int16_t angle, uint32_t transp);
static bool     pushRotatedSprite(struct TFT_eSprite* sprite, struct TFT_eSprite* des_sprite, int16_t angle, uint32_t transp);
static bool     getRotatedBounds(struct TFT_eSprite* sprite, int16_t angle, int16_t *min_x, int16_t *min_y, int16_t *max_x, int16_t *max_y);
static bool     getRotatedBoundsCopy(struct TFT_eSprite* sprite, struct TFT_eSprite* des_spr, int16_t angle, int16_t *min_x, int16_t *min_y,
                                                            int16_t *max_x, int16_t *max_y);
static void     getRotatedBoundsBound(struct TFT_eSprite* sprite, int16_t angle, int16_t w, int16_t h, int16_t xp, int16_t yp,
                          int16_t *min_x, int16_t *min_y, int16_t *max_x, int16_t *max_y);

static uint16_t readPixel(struct TFT_eSprite* sprite, int32_t x0, int32_t y0);
static uint16_t readPixelValue(struct TFT_eSprite* sprite, int32_t x, int32_t y);
static void     pushImage(struct TFT_eSprite* sprite, int32_t x0, int32_t y0, int32_t w, int32_t h, uint16_t *data, uint8_t sbpp); //default, sbpp=0
static void     pushImageFlash(struct TFT_eSprite* sprite, int32_t x0, int32_t y0, int32_t w, int32_t h, const uint16_t *data);
static void     pushSprite(struct TFT_eSprite* sprite, int32_t x, int32_t y);
static void     pushSpriteTrans(struct TFT_eSprite* sprite, int32_t x, int32_t y, uint16_t transparent);
static bool     pushSpriteWin(struct TFT_eSprite* sprite, int32_t tx, int32_t ty, int32_t sx, int32_t sy, int32_t sw, int32_t sh);
static bool     pushToSprite(struct TFT_eSprite* sprite, struct TFT_eSprite *dspr, int32_t x, int32_t y);
static bool     pushToSpriteTrans(struct TFT_eSprite* sprite, struct TFT_eSprite *dspr, int32_t x, int32_t y, uint16_t transparent);
static int16_t drawCharUniFont(struct TFT_eSprite* sprite, uint16_t uniCode, int32_t x, int32_t y, uint8_t font);
static int16_t drawCharUni(struct TFT_eSprite* sprite, uint16_t uniCode, int32_t x, int32_t y);
static int16_t width(struct TFT_eSprite* sprite);
static int16_t height(struct TFT_eSprite* sprite);
#ifdef SMOOTH_FONT
static void     drawGlyph(struct TFT_eSprite* sprite, uint16_t code);
static void     printToSprite(struct TFT_eSprite* sprite, char *cbuffer, uint16_t len);
static int16_t printToSpriteXy(struct TFT_eSprite* sprite, int16_t x, int16_t y, uint16_t index);
#endif
static void*    callocSprite(struct TFT_eSprite* sprite, int16_t width, int16_t height, uint8_t frames);


/***************************************************************************************
** Function name:           TFT_eSprite
** Description:             Class constructor
***************************************************************************************/
void TFT_eSprite_init(struct TFT_eSprite *sprite, struct TFT_eSPI *tft)
{
  sprite->_tft = tft;  // Pointer to tft class so we can call member functions

  sprite->_iwidth   = 0;      // Initialise width and height to 0 (it does not exist yet)
  sprite->_iheight  = 0;
  sprite->_bpp      = 16;
  sprite->_created  = false;
  sprite->_xs       = 0;      // window bounds for pushColor
  sprite->_ys       = 0;
  sprite->_xe       = 0;
  sprite->_ye       = 0;
  sprite->_xptr     = 0;      // pushColor coordinate
  sprite->_yptr     = 0;
  sprite->_colorMap = NULL;
  sprite->_tft->_psram_enable = true;

  // Ensure end_tft_write() does nothing in inherited functions.
  sprite->_tft->lockTransaction = true;

  sprite->createSprite          = &createSprite;
  sprite->getPointer            = &getPointer;
  sprite->created               = &created;
  sprite->deleteSprite          = &deleteSprite;
  sprite->frameBuffer           = &frameBuffer;
  sprite->setColorDepth         = &setColorDepth;
  sprite->getColorDepth         = &getColorDepth;
  sprite->createPaletteRam      = &createPaletteRam;
  sprite->createPaletteFlash    = &createPaletteFlash;
  sprite->setPaletteColor       = &setPaletteColor;
  sprite->getPaletteColor       = &getPaletteColor;
  sprite->setBitmapColor        = &setBitmapColor;
  sprite->drawPixel             = &drawPixel;
  sprite->drawChar              = &drawChar;
  sprite->fillSprite            = &fillSprite;
  sprite->setWindow             = &setWindow;
  sprite->pushColor             = &pushColor;
  sprite->pushColorLen          = &pushColorLen;
  sprite->writeColor            = &writeColor;
  sprite->setScrollRect         = &setScrollRect;
  sprite->scroll                = &scroll;
  sprite->drawLine              = &drawLine;
  sprite->drawFastVLine         = &drawFastVLine;
  sprite->drawFastHLine         = &drawFastHLine;
  sprite->fillRect              = &fillRect;
  sprite->setRotation           = &setRotation;
  sprite->getRotation           = &getRotation;
  sprite->pushRotated           = &pushRotated;
  sprite->pushRotatedSprite     = &pushRotatedSprite;
  sprite->getRotatedBounds      = &getRotatedBounds;
  sprite->getRotatedBoundsCopy  = &getRotatedBoundsCopy;
  sprite->getRotatedBoundsBound = &getRotatedBoundsBound;
  sprite->readPixel             = &readPixel;
  sprite->readPixelValue        = &readPixelValue;
  sprite->pushImage          = &pushImage;
  sprite->pushImageFlash        = &pushImageFlash;
  sprite->pushSprite            = &pushSprite;
  sprite->pushSpriteTrans       = &pushSpriteTrans;
  sprite->pushSpriteWin         = &pushSpriteWin;
  sprite->pushToSprite          = &pushToSprite;
  sprite->pushToSpriteTrans     = &pushToSpriteTrans;
  sprite->drawCharUniFont       = &drawCharUniFont;
  sprite->drawCharUni           = &drawCharUni;
  sprite->width                 = &width;
  sprite->height                = &height;
  #ifdef SMOOTH_FONT
  sprite->drawGlyph             = &drawGlyph;
  sprite->printToSprite         = &printToSprite;
  sprite->printToSpriteXy       = &printToSpriteXy;
  #endif
  sprite->callocSprite          = &callocSprite;
}

/***************************************************************************************
** Function name:           createSprite
** Description:             Create a sprite (bitmap) of defined width and height
***************************************************************************************/
// cast returned value to (uint8_t*) for 8 bit or (uint16_t*) for 16 bit colours
static void* createSprite(struct TFT_eSprite* sprite, int16_t w, int16_t h, uint8_t frames)
{

  if ( sprite->_created ) return sprite->_img8_1;

  if ( w < 1 || h < 1 ) return NULL;

  sprite->_iwidth  = sprite->_dwidth  = sprite->_bitwidth = w;
  sprite->_iheight = sprite->_dheight = h;
  // Default scroll rectangle and gap fill colour
  sprite->_sx = 0;
  sprite->_sy = 0;
  sprite->_sw = w;
  sprite->_sh = h;
  sprite->_scolor = TFT_BLACK;

  sprite->_img8   = (uint8_t*) callocSprite(sprite, w, h, frames);
  sprite->_img8_1 = sprite->_img8;
  sprite->_img8_2 = sprite->_img8;
  sprite->_img    = (uint16_t*) sprite->_img8;
  sprite->_img4   = sprite->_img8;

  if ( (sprite->_bpp == 16) && (frames > 1) ) {
    sprite->_img8_2 = sprite->_img8 + (w * h * 2 + 1);
  }

  // ESP32 only 16bpp check
  //if (esp_ptr_dma_capable(_img8_1)) Serial.println("DMA capable Sprite pointer _img8_1");
  //else Serial.println("Not a DMA capable Sprite pointer _img8_1");
  //if (esp_ptr_dma_capable(_img8_2)) Serial.println("DMA capable Sprite pointer _img8_2");
  //else Serial.println("Not a DMA capable Sprite pointer _img8_2");

  if ( (sprite->_bpp == 8) && (frames > 1) ) {
    sprite->_img8_2 = sprite->_img8 + (w * h + 1);
  }

  if ( (sprite->_bpp == 4) && (sprite->_colorMap == NULL)) sprite->createPaletteFlash(sprite, default_4bit_palette, 16);

  // This is to make it clear what pointer size is expected to be used
  // but casting in the user sketch is needed due to the use of void*
  if ( (sprite->_bpp == 1) && (frames > 1) )
  {
    w = (w+7) & 0xFFF8;
    sprite->_img8_2 = sprite->_img8 + ( (w>>3) * h + 1 );
  }

  if (sprite->_img8)
  {
    sprite->_created = true;
    sprite->_tft->setViewport(sprite->_tft, 0, 0, sprite->_dwidth, sprite->_dheight, true);
    sprite->_tft->setPivot(sprite->_tft, sprite->_iwidth/2, sprite->_iheight/2);
    return sprite->_img8_1;
  }

  return NULL;
}


/***************************************************************************************
** Function name:           getPointer
** Description:             Returns pointer to start of sprite memory area
***************************************************************************************/
static void* getPointer(struct TFT_eSprite* sprite)
{
  if (!sprite->_created) return NULL;
  return sprite->_img8_1;
}


/***************************************************************************************
** Function name:           created
** Description:             Returns true if sprite has been created
***************************************************************************************/
static bool created(struct TFT_eSprite* sprite)
{
  return sprite->_created;
}

/***************************************************************************************
** Function name:           callocSprite
** Description:             Allocate a memory area for the Sprite and return pointer
***************************************************************************************/
static void* callocSprite(struct TFT_eSprite* sprite, int16_t w, int16_t h, uint8_t frames)
{
  // Add one extra "off screen" pixel to point out-of-bounds setWindow() coordinates
  // this means push/writeColor functions do not need additional bounds checks and
  // hence will run faster in normal circumstances.
  uint8_t* ptr8 = NULL;

  if (frames > 2) frames = 2; // Currently restricted to 2 frame buffers
  if (frames < 1) frames = 1;

  if (sprite->_bpp == 16)
  {
    //TODO: calloc for SPIRAM
    {
      ptr8 = ( uint8_t*) calloc(frames * w * h + frames, sizeof(uint16_t));
      //Serial.println("Normal RAM");
    }
  }

  else if (sprite->_bpp == 8)
  {
    //TODO: calloc for SPIRAM
    ptr8 = ( uint8_t*) calloc(frames * w * h + frames, sizeof(uint8_t));
  }

  else if (sprite->_bpp == 4)
  {
    w = (w+1) & 0xFFFE; // width needs to be multiple of 2, with an extra "off screen" pixel
    sprite->_iwidth = w;
    //TODO: calloc for SPIRAM
    ptr8 = ( uint8_t*) calloc(((frames * w * h) >> 1) + frames, sizeof(uint8_t));
  }

  else // Must be 1 bpp
  {
    //_dwidth   Display width+height in pixels always in rotation 0 orientation
    //_dheight  Not swapped for sprite rotations
    // Note: for 1bpp _iwidth and _iheight are swapped during Sprite rotations

    w =  (w+7) & 0xFFF8; // width should be the multiple of 8 bits to be compatible with epdpaint
    sprite->_iwidth = w;         // _iwidth is rounded up to be multiple of 8, so might not be = _dwidth
    sprite->_bitwidth = w;       // _bitwidth will not be rotated whereas _iwidth may be

    //TODO: calloc for SPIRAM
    ptr8 = ( uint8_t*) calloc(frames * (w>>3) * h + frames, sizeof(uint8_t));
  }

  return ptr8;
}


/***************************************************************************************
** Function name:           createPalette (from RAM array)
** Description:             Set a palette for a 4-bit per pixel sprite
***************************************************************************************/
static void createPaletteRam(struct TFT_eSprite* sprite, uint16_t colorMap[], uint8_t colors)
{
  if (sprite->_colorMap != NULL)
  {
    free(sprite->_colorMap);
  }

  if (colorMap == NULL)
  {
    // Create a color map using the default FLASH map
    sprite->createPaletteFlash(sprite, default_4bit_palette, 16);
    return;
  }

  // Allocate and clear memory for 16 color map
  sprite->_colorMap = (uint16_t *)calloc(16, sizeof(uint16_t));

  if (colors > 16) colors = 16;

  // Copy map colors
  for (uint8_t i = 0; i < colors; i++)
  {
    sprite->_colorMap[i] = colorMap[i];
  }
}


/***************************************************************************************
** Function name:           createPalette (from FLASH array)
** Description:             Set a palette for a 4-bit per pixel sprite
***************************************************************************************/
static void createPaletteFlash(struct TFT_eSprite* sprite, const uint16_t colorMap[], uint8_t colors)
{
  if (colorMap == NULL)
  {
    // Create a color map using the default FLASH map
    colorMap = default_4bit_palette;
  }

  // Allocate and clear memory for 16 color map
  sprite->_colorMap = (uint16_t *)calloc(16, sizeof(uint16_t));

  if (colors > 16) colors = 16;

  // Copy map colors
  for (uint8_t i = 0; i < colors; i++)
  {
    sprite->_colorMap[i] = pgm_read_word(colorMap++);
  }
}


/***************************************************************************************
** Function name:           frameBuffer
** Description:             For 1 bpp Sprites, select the frame used for graphics
***************************************************************************************/
// Frames are numbered 1 and 2
static void* frameBuffer(struct TFT_eSprite* sprite, int8_t f)
{
  if (!sprite->_created) return NULL;

  if ( f == 2 ) sprite->_img8 = sprite->_img8_2;
  else          sprite->_img8 = sprite->_img8_1;

  if (sprite->_bpp == 16) sprite->_img = (uint16_t*)sprite->_img8;

  //if (_bpp == 8) _img8 = _img8;

  if (sprite->_bpp == 4) sprite->_img4 = sprite->_img8;

  return sprite->_img8;
}


/***************************************************************************************
** Function name:           setColorDepth
** Description:             Set bits per pixel for colour (1, 8 or 16)
***************************************************************************************/
static void* setColorDepth(struct TFT_eSprite* sprite, int8_t b)
{
  // Do not re-create the sprite if the colour depth does not change
  if (sprite->_bpp == b) return sprite->_img8_1;

  // Validate the new colour depth
  if ( b > 8 ) sprite->_bpp = 16;  // Bytes per pixel
  else if ( b > 4 ) sprite->_bpp = 8;
  else if ( b > 1 ) sprite->_bpp = 4;
  else sprite->_bpp = 1;

  // Can't change an existing sprite's colour depth so delete it
  if (sprite->_created) free(sprite->_img8_1);

  // If it existed, re-create the sprite with the new colour depth
  if (sprite->_created)
  {
    sprite->_created = false;
    return sprite->createSprite(sprite, sprite->_dwidth, sprite->_dheight, 1);
  }

  return NULL;
}


/***************************************************************************************
** Function name:           getColorDepth
** Description:             Get bits per pixel for colour (1, 8 or 16)
***************************************************************************************/
static int8_t getColorDepth(struct TFT_eSprite* sprite)
{
  if (sprite->_created) return sprite->_bpp;
  else return 0;
}


/***************************************************************************************
** Function name:           setBitmapColor
** Description:             Set the 1bpp foreground foreground and background colour
***************************************************************************************/
static void setBitmapColor(struct TFT_eSprite* sprite, uint16_t c, uint16_t b)
{
  if (c == b) b = ~c;
  sprite->_tft->bitmap_fg = c;
  sprite->_tft->bitmap_bg = b;
}


/***************************************************************************************
** Function name:           setPaletteColor
** Description:             Set the 4bpp palette color at the given index
***************************************************************************************/
static void setPaletteColor(struct TFT_eSprite* sprite, uint8_t index, uint16_t color)
{
  if (sprite->_colorMap == NULL || index > 15) return; // out of bounds

  sprite->_colorMap[index] = color;
}


/***************************************************************************************
** Function name:           getPaletteColor
** Description:             Return the palette color at 4bpp index, or 0 on error.
***************************************************************************************/
static uint16_t getPaletteColor(struct TFT_eSprite* sprite, uint8_t index)
{
  if (sprite->_colorMap == NULL || index > 15) return 0; // out of bounds

  return sprite->_colorMap[index];
}


/***************************************************************************************
** Function name:           deleteSprite
** Description:             Delete the sprite to free up memory (RAM)
***************************************************************************************/
static void deleteSprite(struct TFT_eSprite* sprite)
{
  if (sprite->_colorMap != NULL)
  {
    free(sprite->_colorMap);
	  sprite->_colorMap = NULL;
  }

  if (sprite->_created)
  {
    free(sprite->_img8_1);
    sprite->_img8        = NULL;
    sprite->_created     = false;
    //sprite->_tft->_vpOoB = true;   // TFT_eSPI class write() uses this to check for valid sprite
  }
}


/***************************************************************************************
** Function name:           pushRotated - Fast fixed point integer maths version
** Description:             Push rotated Sprite to TFT screen
***************************************************************************************/
#define FP_SCALE 10
static bool pushRotated(struct TFT_eSprite* sprite, int16_t angle, uint32_t transp)
{
  if ( !sprite->_created || sprite->_tft->_vpOoB) return false;

  // Bounding box parameters
  int16_t min_x;
  int16_t min_y;
  int16_t max_x;
  int16_t max_y;

  // Get the bounding box of this rotated source Sprite relative to Sprite pivot
  if ( !sprite->getRotatedBounds(sprite, angle, &min_x, &min_y, &max_x, &max_y) ) return false;

  uint16_t sline_buffer[max_x - min_x + 1];

  int32_t xt = min_x - sprite->_tft->_xPivot;
  int32_t yt = min_y - sprite->_tft->_yPivot;
  uint32_t xe = sprite->_dwidth << FP_SCALE;
  uint32_t ye = sprite->_dheight << FP_SCALE;
  uint16_t tpcolor = (uint16_t)transp;

  if (transp != 0x00FFFFFF) {
    if (sprite->_bpp == 4) tpcolor = sprite->_colorMap[transp & 0x0F];
    tpcolor = tpcolor>>8 | tpcolor<<8; // Working with swapped color bytes
  }
  sprite->_tft->startWrite(sprite->_tft); // Avoid transaction overhead for every tft pixel

  // Scan destination bounding box and fetch transformed pixels from source Sprite
  for (int32_t y = min_y; y <= max_y; y++, yt++) {
    int32_t x = min_x;
    uint32_t xs = (sprite->_cosra * xt - (sprite->_sinra * yt - (sprite->_tft->_xPivot << FP_SCALE)) + (1 << (FP_SCALE - 1)));
    uint32_t ys = (sprite->_sinra * xt + (sprite->_cosra * yt + (sprite->_tft->_yPivot << FP_SCALE)) + (1 << (FP_SCALE - 1)));

    while ((xs >= xe || ys >= ye) && x < max_x) { x++; xs += sprite->_cosra; ys += sprite->_sinra; }
    if (x == max_x) continue;

    uint32_t pixel_count = 0;
    do {
      uint32_t rp;
      int32_t xp = xs >> FP_SCALE;
      int32_t yp = ys >> FP_SCALE;
      if (sprite->_bpp == 16) {rp = sprite->_img[xp + yp * sprite->_iwidth]; }
      else { rp = sprite->readPixel(sprite, xp, yp); rp = (uint16_t)(rp>>8 | rp<<8); }
      if (transp != 0x00FFFFFF && tpcolor == rp) {
        if (pixel_count) {
          // TFT window is already clipped, so this is faster than pushImage()
          sprite->_tft->setWindow(sprite->_tft, x - pixel_count, y, x - 1, y);
          sprite->_tft->pushPixels(sprite->_tft, sline_buffer, pixel_count);
          pixel_count = 0;
        }
      }
      else {
        sline_buffer[pixel_count++] = rp;
      }
    } while (++x < max_x && (xs += sprite->_cosra) < xe && (ys += sprite->_sinra) < ye);
    if (pixel_count) {
      // TFT window is already clipped, so this is faster than pushImage()
      sprite->_tft->setWindow(sprite->_tft, x - pixel_count, y, x - 1, y);
      sprite->_tft->pushPixels(sprite->_tft, sline_buffer, pixel_count);
    }
  }

  sprite->_tft->endWrite(sprite->_tft); // End transaction

  return true;
}


/***************************************************************************************
** Function name:           pushRotated - Fast fixed point integer maths version
** Description:             Push a rotated copy of the Sprite to another Sprite
***************************************************************************************/
// Not compatible with 4bpp
static bool pushRotatedSprite(struct TFT_eSprite* sprite, struct TFT_eSprite *spr, int16_t angle, uint32_t transp)
{
  if ( !sprite->_created  || sprite->_bpp == 4) return false; // Check this Sprite is created
  if ( !spr->_created  || spr->_bpp == 4) return false;  // Ckeck destination Sprite is created

  // Bounding box parameters
  int16_t min_x;
  int16_t min_y;
  int16_t max_x;
  int16_t max_y;

  // Get the bounding box of this rotated source Sprite
  if ( !sprite->getRotatedBoundsCopy(sprite, spr, angle, &min_x, &min_y, &max_x, &max_y) ) return false;

  uint16_t sline_buffer[max_x - min_x + 1];

  int32_t xt = min_x - spr->_tft->_xPivot;
  int32_t yt = min_y - spr->_tft->_yPivot;
  uint32_t xe = sprite->_dwidth << FP_SCALE;
  uint32_t ye = sprite->_dheight << FP_SCALE;
  uint16_t tpcolor = (uint16_t)transp;

  if (transp != 0x00FFFFFF) {
    if (sprite->_bpp == 4) tpcolor = sprite->_colorMap[transp & 0x0F];
    tpcolor = tpcolor>>8 | tpcolor<<8; // Working with swapped color bytes
  }

  bool oldSwapBytes = spr->_tft->getSwapBytes(spr->_tft);
  spr->_tft->setSwapBytes(spr->_tft, false);

  // Scan destination bounding box and fetch transformed pixels from source Sprite
  for (int32_t y = min_y; y <= max_y; y++, yt++) {
    int32_t x = min_x;
    uint32_t xs = (sprite->_cosra * xt - (sprite->_sinra * yt - (sprite->_tft->_xPivot << FP_SCALE)) + (1 << (FP_SCALE - 1)));
    uint32_t ys = (sprite->_sinra * xt + (sprite->_cosra * yt + (sprite->_tft->_yPivot << FP_SCALE)) + (1 << (FP_SCALE - 1)));

    while ((xs >= xe || ys >= ye) && x < max_x) { x++; xs += sprite->_cosra; ys += sprite->_sinra; }
    if (x == max_x) continue;

    uint32_t pixel_count = 0;
    do {
      uint32_t rp;
      int32_t xp = xs >> FP_SCALE;
      int32_t yp = ys >> FP_SCALE;
      if (sprite->_bpp == 16) rp = sprite->_img[xp + yp * sprite->_iwidth];
      else { rp = sprite->readPixel(sprite, xp, yp); rp = (uint16_t)(rp>>8 | rp<<8); }
      if (transp != 0x00FFFFFF && tpcolor == rp) {
        if (pixel_count) {
          spr->pushImage(spr, x - pixel_count, y, pixel_count, 1, sline_buffer, 0);
          pixel_count = 0;
        }
      }
      else {
        sline_buffer[pixel_count++] = rp;
      }
    } while (++x < max_x && (xs += sprite->_cosra) < xe && (ys += sprite->_sinra) < ye);
    if (pixel_count) spr->pushImage(spr, x - pixel_count, y, pixel_count, 1, sline_buffer, 0);
  }
  spr->_tft->setSwapBytes(spr->_tft, oldSwapBytes);
  return true;
}


/***************************************************************************************
** Function name:           getRotatedBounds
** Description:             Get TFT bounding box of a rotated Sprite wrt pivot
***************************************************************************************/
static bool getRotatedBounds(struct TFT_eSprite* sprite, int16_t angle, int16_t *min_x, int16_t *min_y,
                                                  int16_t *max_x, int16_t *max_y)
{
  // Get the bounding box of this rotated source Sprite relative to Sprite pivot
  sprite->getRotatedBoundsBound(sprite, angle, sprite->width(sprite), sprite->height(sprite), sprite->_tft->_xPivot, sprite->_tft->_yPivot, min_x, min_y, max_x, max_y);

  // Move bounding box so source Sprite pivot coincides with TFT pivot
  *min_x += sprite->_tft->_xPivot;
  *max_x += sprite->_tft->_xPivot;
  *min_y += sprite->_tft->_yPivot;
  *max_y += sprite->_tft->_yPivot;

  // Return if bounding box is outside of TFT viewport
  if (*min_x > sprite->_tft->_vpW) return false;
  if (*min_y > sprite->_tft->_vpH) return false;
  if (*max_x < sprite->_tft->_vpX) return false;
  if (*max_y < sprite->_tft->_vpY) return false;

  // Clip bounding box to be within TFT viewport
  if (*min_x < sprite->_tft->_vpX) *min_x = sprite->_tft->_vpX;
  if (*min_y < sprite->_tft->_vpY) *min_y = sprite->_tft->_vpY;
  if (*max_x > sprite->_tft->_vpW) *max_x = sprite->_tft->_vpW;
  if (*max_y > sprite->_tft->_vpH) *max_y = sprite->_tft->_vpH;

  return true;
}


/***************************************************************************************
** Function name:           getRotatedBounds
** Description:             Get destination Sprite bounding box of a rotated Sprite wrt pivot
***************************************************************************************/
static bool getRotatedBoundsCopy(struct TFT_eSprite* sprite, struct TFT_eSprite *spr, int16_t angle, int16_t *min_x, int16_t *min_y,
                                                                    int16_t *max_x, int16_t *max_y)
{
  // Get the bounding box of this rotated source Sprite relative to Sprite pivot
  sprite->getRotatedBoundsBound(sprite, angle, sprite->width(sprite), sprite->height(sprite), sprite->_tft->_xPivot, sprite->_tft->_yPivot, min_x, min_y, max_x, max_y);

  // Move bounding box so source Sprite pivot coincides with destination Sprite pivot
  *min_x += spr->_tft->_xPivot;
  *max_x += spr->_tft->_xPivot;
  *min_y += spr->_tft->_yPivot;
  *max_y += spr->_tft->_yPivot;

  // Test only to show bounding box
  // spr->fillSprite(TFT_BLACK);
  // spr->drawRect(min_x, min_y, max_x - min_x + 1, max_y - min_y + 1, TFT_GREEN);

  // Return if bounding box is completely outside of destination Sprite
  if (*min_x > spr->width(spr)) return true;
  if (*min_y > spr->height(spr)) return true;
  if (*max_x < 0) return true;
  if (*max_y < 0) return true;

  // Clip bounding box to Sprite boundaries
  // Clipping to a viewport will be done by destination Sprite pushImage function
  if (*min_x < 0) min_x = 0;
  if (*min_y < 0) min_y = 0;
  if (*max_x > spr->width(spr))  *max_x = spr->width(spr);
  if (*max_y > spr->height(spr)) *max_y = spr->height(spr);

  return true;
}


/***************************************************************************************
** Function name:           rotatedBounds
** Description:             Get bounding box of a rotated Sprite wrt pivot
***************************************************************************************/
static void getRotatedBoundsBound(struct TFT_eSprite* sprite, int16_t angle, int16_t w, int16_t h, int16_t xp, int16_t yp,
                                   int16_t *min_x, int16_t *min_y, int16_t *max_x, int16_t *max_y)
{
  // Trig values for the rotation
  float radAngle = -angle * 0.0174532925; // Convert degrees to radians
  float sina = sin(radAngle);
  float cosa = cos(radAngle);

  w -= xp; // w is now right edge coordinate relative to xp
  h -= yp; // h is now bottom edge coordinate relative to yp

  // Calculate new corner coordinates
  int16_t x0 = -xp * cosa - yp * sina;
  int16_t y0 =  xp * sina - yp * cosa;

  int16_t x1 =  w * cosa - yp * sina;
  int16_t y1 = -w * sina - yp * cosa;

  int16_t x2 =  h * sina + w * cosa;
  int16_t y2 =  h * cosa - w * sina;

  int16_t x3 =  h * sina - xp * cosa;
  int16_t y3 =  h * cosa + xp * sina;

  // Find bounding box extremes, enlarge box to accomodate rounding errors
  *min_x = x0-2;
  if (x1 < *min_x) *min_x = x1-2;
  if (x2 < *min_x) *min_x = x2-2;
  if (x3 < *min_x) *min_x = x3-2;

  *max_x = x0+2;
  if (x1 > *max_x) *max_x = x1+2;
  if (x2 > *max_x) *max_x = x2+2;
  if (x3 > *max_x) *max_x = x3+2;

  *min_y = y0-2;
  if (y1 < *min_y) *min_y = y1-2;
  if (y2 < *min_y) *min_y = y2-2;
  if (y3 < *min_y) *min_y = y3-2;

  *max_y = y0+2;
  if (y1 > *max_y) *max_y = y1+2;
  if (y2 > *max_y) *max_y = y2+2;
  if (y3 > *max_y) *max_y = y3+2;

  sprite->_sinra = round(sina * (1<<FP_SCALE));
  sprite->_cosra = round(cosa * (1<<FP_SCALE));
}


/***************************************************************************************
** Function name:           pushSprite
** Description:             Push the sprite to the TFT at x, y
***************************************************************************************/
static void pushSprite(struct TFT_eSprite* sprite, int32_t x, int32_t y)
{
  if (!sprite->_created) return;

  if (sprite->_bpp == 16)
  {
    bool oldSwapBytes = sprite->_tft->getSwapBytes(sprite->_tft);
    sprite->_tft->setSwapBytes(sprite->_tft, false);
    sprite->_tft->pushImage(sprite->_tft, x, y, sprite->_dwidth, sprite->_dheight, sprite->_img );
    sprite->_tft->setSwapBytes(sprite->_tft, oldSwapBytes);
  }
  else if (sprite->_bpp == 4)
  {
    sprite->_tft->pushImageBpp8(sprite->_tft, x, y, sprite->_dwidth, sprite->_dheight, sprite->_img4, false, sprite->_colorMap);
  }
  else sprite->_tft->pushImageBpp8(sprite->_tft, x, y, sprite->_dwidth, sprite->_dheight, sprite->_img8, (bool)(sprite->_bpp == 8), NULL);
}


/***************************************************************************************
** Function name:           pushSprite
** Description:             Push the sprite to the TFT at x, y with transparent colour
***************************************************************************************/
static void pushSpriteTrans(struct TFT_eSprite* sprite, int32_t x, int32_t y, uint16_t transp)
{
  if (!sprite->_created) return;

  if (sprite->_bpp == 16)
  {
    bool oldSwapBytes = sprite->_tft->getSwapBytes(sprite->_tft);
    sprite->_tft->setSwapBytes(sprite->_tft, false);
    sprite->_tft->pushImageTrans(sprite->_tft, x, y, sprite->_dwidth, sprite->_dheight, sprite->_img, transp);
    sprite->_tft->setSwapBytes(sprite->_tft, oldSwapBytes);
  }
  else if (sprite->_bpp == 8)
  {
    transp = (uint8_t)((transp & 0xE000)>>8 | (transp & 0x0700)>>6 | (transp & 0x0018)>>3);
    sprite->_tft->pushImageBpp8Trans(sprite->_tft, x, y, sprite->_dwidth, sprite->_dheight, sprite->_img8, (uint8_t)transp, (bool)true, NULL);
  }
  else if (sprite->_bpp == 4)
  {
    sprite->_tft->pushImageBpp8Trans(sprite->_tft, x, y, sprite->_dwidth, sprite->_dheight, sprite->_img4, (uint8_t)(transp & 0x0F), false, sprite->_colorMap);
  }
  else sprite->_tft->pushImageBpp8Trans(sprite->_tft, x, y, sprite->_dwidth, sprite-> _dheight, sprite->_img8, 0, (bool)false, NULL);
}


/***************************************************************************************
** Function name:           pushToSprite
** Description:             Push the sprite to another sprite at x, y
***************************************************************************************/
// Note: The following sprite to sprite colour depths are currently supported:
//    Source    Destination
//    16bpp  -> 16bpp
//    16bpp  ->  8bpp
//     8bpp  ->  8bpp
//     4bpp  ->  4bpp (note: color translation depends on the 2 sprites palette colors)
//     1bpp  ->  1bpp (note: color translation depends on the 2 sprites bitmap colors)

static bool pushToSprite(struct TFT_eSprite* sprite, struct TFT_eSprite *dspr, int32_t x, int32_t y)
{
  if (!sprite->_created) return false;
  if (!dspr->created(dspr)) return false;

  // Check destination sprite compatibility
  int8_t ds_bpp = dspr->getColorDepth(dspr);
  if (sprite->_bpp == 16 && ds_bpp != 16 && ds_bpp !=  8) return false;
  if (sprite->_bpp ==  8 && ds_bpp !=  8) return false;
  if (sprite->_bpp ==  4 && ds_bpp !=  4) return false;
  if (sprite->_bpp ==  1 && ds_bpp !=  1) return false;

  bool oldSwapBytes = dspr->_tft->getSwapBytes(dspr->_tft);
  dspr->_tft->setSwapBytes(dspr->_tft, false);
  dspr->pushImage(dspr, x, y, sprite->_dwidth, sprite->_dheight, sprite->_img, sprite->_bpp);
  dspr->_tft->setSwapBytes(dspr->_tft, oldSwapBytes);

  return true;
}


/***************************************************************************************
** Function name:           pushToSprite
** Description:             Push the sprite to another sprite at x, y with transparent colour
***************************************************************************************/
// Note: The following sprite to sprite colour depths are currently supported:
//    Source    Destination
//    16bpp  -> 16bpp
//    16bpp  ->  8bpp
//     8bpp  ->  8bpp
//     1bpp  ->  1bpp

static bool pushToSpriteTrans(struct TFT_eSprite* sprite, struct TFT_eSprite *dspr, int32_t x, int32_t y, uint16_t transp)
{
  if ( !sprite->_created  || !dspr->_created) return false; // Check Sprites exist

  // Check destination sprite compatibility
  int8_t ds_bpp = dspr->getColorDepth(dspr);
  if (sprite->_bpp == 16 && ds_bpp != 16 && ds_bpp !=  8) return false;
  if (sprite->_bpp ==  8 && ds_bpp !=  8) return false;
  if (sprite->_bpp ==  4 || ds_bpp ==  4) return false;
  if (sprite->_bpp ==  1 && ds_bpp !=  1) return false;

  bool oldSwapBytes = dspr->_tft->getSwapBytes(dspr->_tft);
  uint16_t sline_buffer[sprite->width(sprite)];

  transp = transp>>8 | transp<<8;

  // Scan destination bounding box and fetch transformed pixels from source Sprite
  for (int32_t ys = 0; ys < sprite->height(sprite); ys++) {
    int32_t ox = x;
    uint32_t pixel_count = 0;

    for (int32_t xs = 0; xs < sprite->width(sprite); xs++) {
      uint16_t rp = 0;
      if (sprite->_bpp == 16) rp = sprite->_img[xs + ys * sprite->width(sprite)];
      else { rp = sprite->readPixel(sprite, xs, ys); rp = rp>>8 | rp<<8; }
      //dspr->drawPixel(xs, ys, rp);

      if (transp == rp) {
        if (pixel_count) {
          dspr->pushImage(dspr, ox, y, pixel_count, 1, sline_buffer, sprite->_bpp);
          ox += pixel_count;
          pixel_count = 0;
        }
        ox++;
      }
      else {
        sline_buffer[pixel_count++] = rp;
      }
    }
    if (pixel_count) dspr->pushImage(dspr, ox, y, pixel_count, 1, sline_buffer, 0);
    y++;
  }
  dspr->_tft->setSwapBytes(dspr->_tft, oldSwapBytes);
  return true;
}


/***************************************************************************************
** Function name:           pushSprite
** Description:             Push a cropped sprite to the TFT at tx, ty
***************************************************************************************/
static bool pushSpriteWin(struct TFT_eSprite* sprite, int32_t tx, int32_t ty, int32_t sx, int32_t sy, int32_t sw, int32_t sh)
{
  if (!sprite->_created) return false;

  // Perform window boundary checks and crop if needed
  sprite->setWindow(sprite, sx, sy, sx + sw - 1, sy + sh - 1);

  /* These global variables are now populated for the sprite
  _xs = x start coordinate
  _ys = y start coordinate
  _xe = x end coordinate (inclusive)
  _ye = y end coordinate (inclusive)
  */

  // Calculate new sprite window bounding box width and height
  sw = sprite->_xe - sprite->_xs + 1;
  sh = sprite->_ye - sprite->_ys + 1;

  if (sprite->_ys >= sprite->_iheight) return false;

  if (sprite->_bpp == 16)
  {
    bool oldSwapBytes = sprite->_tft->getSwapBytes(sprite->_tft);
    sprite->_tft->setSwapBytes(sprite->_tft, false);

    // Check if a faster block copy to screen is possible
    if ( sx == 0 && sw == sprite->_dwidth)
      sprite->_tft->pushImage(sprite->_tft, tx, ty, sw, sh, sprite->_img +sprite-> _iwidth * sprite->_ys );
    else // Render line by line
      while (sh--)
        sprite->_tft->pushImage(sprite->_tft, tx, ty++, sw, 1, sprite->_img + sprite->_xs + sprite->_iwidth * sprite->_ys++ );

    sprite->_tft->setSwapBytes(sprite->_tft, oldSwapBytes);
  }
  else if (sprite->_bpp == 8)
  {
    // Check if a faster block copy to screen is possible
    if ( sx == 0 && sw == sprite->_dwidth)
      sprite->_tft->pushImageBpp8(sprite->_tft, tx, ty, sw, sh, sprite->_img8 + sprite->_iwidth * sprite->_ys, (bool)true, NULL);
    else // Render line by line
    while (sh--)
      sprite->_tft->pushImageBpp8(sprite->_tft, tx, ty++, sw, 1, sprite->_img8 + sprite->_xs + sprite->_iwidth * sprite->_ys++, (bool)true, NULL);
  }
  else if (sprite->_bpp == 4)
  {
    // Check if a faster block copy to screen is possible
    if ( sx == 0 && sw == sprite->_dwidth)
      sprite->_tft->pushImageBpp8(sprite->_tft, tx, ty, sw, sh, sprite->_img4 + (sprite->_iwidth>>1) * sprite->_ys, false, sprite->_colorMap );
    else // Render line by line
    {
      int32_t ds = sprite->_xs&1; // Odd x start pixel

      int32_t de = 0;     // Odd x end pixel
      if ((sw > ds) && (sprite->_xe&1)) de = 1;

      uint32_t dm = 0;     // Midsection pixel count
      if (sw > (ds+de)) dm = sw - ds - de;
      sw--;

      uint32_t yp = (sprite->_xs + ds + sprite->_iwidth * sprite->_ys)>>1;
      sprite->_tft->startWrite(sprite->_tft);
      while (sh--)
      {
        if (ds) sprite->_tft->drawPixel(sprite->_tft, tx, ty, sprite->readPixel(sprite, sprite->_xs, sprite->_ys) );
        if (dm) sprite->_tft->pushImageBpp8(sprite->_tft, tx + ds, ty, dm, 1, sprite->_img4 + yp, false, sprite->_colorMap );
        if (de) sprite->_tft->drawPixel(sprite->_tft, tx + sw, ty, sprite->readPixel(sprite, sprite->_xe, sprite->_ys) );
        sprite->_ys++;
        ty++;
        yp += (sprite->_iwidth>>1);
      }
      sprite->_tft->endWrite(sprite->_tft);
    }
  }
  else // 1bpp
  {
    // Check if a faster block copy to screen is possible
    if ( sx == 0 && sw == sprite->_dwidth)
      sprite->_tft->pushImageBpp8(sprite->_tft, tx, ty, sw, sh, sprite->_img8 + (sprite->_bitwidth>>3) * sprite->_ys, (bool)false, NULL);
    else // Render line by line
    {
      sprite->_tft->startWrite(sprite->_tft);
      while (sh--)
      {
        sprite->_tft->pushImageBpp8(sprite->_tft, tx, ty++, sw, 1, sprite->_img8 + (sprite->_bitwidth>>3) * sprite->_ys++, (bool)false, NULL);
      }
      sprite->_tft->endWrite(sprite->_tft);
    }
  }

  return true;
}


/***************************************************************************************
** Function name:           readPixelValue
** Description:             Read the color map index of a pixel at defined coordinates
***************************************************************************************/
static uint16_t readPixelValue(struct TFT_eSprite* sprite, int32_t x, int32_t y)
{
  if (sprite->_tft->_vpOoB  || !sprite->_created) return 0xFF;

  x+= sprite->_tft->_xDatum;
  y+= sprite->_tft->_yDatum;

  // Range checking
  if ((x < sprite->_tft->_vpX) || (y < sprite->_tft->_vpY) ||(x >= sprite->_tft->_vpW) || (y >= sprite->_tft->_vpH)) return 0xFF;

  if (sprite->_bpp == 16)
  {
    // Return the pixel colour
    return sprite->readPixel(sprite, x - sprite->_tft->_xDatum, y - sprite->_tft->_yDatum);
  }

  if (sprite->_bpp == 8)
  {
    // Return the pixel byte value
    return sprite->_img8[x + y * sprite->_iwidth];
  }

  if (sprite->_bpp == 4)
  {
    if (x >= sprite->_dwidth) return 0xFF;
    if ((x & 0x01) == 0)
      return sprite->_img4[((x+y*sprite->_iwidth)>>1)] >> 4;   // even index = bits 7 .. 4
    else
      return sprite->_img4[((x+y*sprite->_iwidth)>>1)] & 0x0F; // odd index = bits 3 .. 0.
  }

  if (sprite->_bpp == 1)
  {
    // Note: _dwidth and _dheight bounds not checked (rounded up -iwidth and _iheight used)
    if (sprite->_tft->rotation == 1)
    {
      uint16_t tx = x;
      x = sprite->_dheight - y - 1;
      y = tx;
    }
    else if (sprite->_tft->rotation == 2)
    {
      x = sprite->_dwidth - x - 1;
      y = sprite->_dheight - y - 1;
    }
    else if (sprite->_tft->rotation == 3)
    {
      uint16_t tx = x;
      x = y;
      y = sprite->_dwidth - tx - 1;
    }
    // Return 1 or 0
    return (sprite->_img8[(x + y * sprite->_bitwidth)>>3] >> (7-(x & 0x7))) & 0x01;
  }

  return 0;
}

/***************************************************************************************
** Function name:           readPixel
** Description:             Read 565 colour of a pixel at defined coordinates
***************************************************************************************/
static uint16_t readPixel(struct TFT_eSprite* sprite, int32_t x, int32_t y)
{
  if (sprite->_tft->_vpOoB  || !sprite->_created) return 0xFFFF;

  x+= sprite->_tft->_xDatum;
  y+= sprite->_tft->_yDatum;

  // Range checking
  if ((x < sprite->_tft->_vpX) || (y < sprite->_tft->_vpY) ||(x >= sprite->_tft->_vpW) || (y >= sprite->_tft->_vpH)) return 0xFFFF;

  if (sprite->_bpp == 16)
  {
    uint16_t color = sprite->_img[x + y * sprite->_iwidth];
    return (color >> 8) | (color << 8);
  }

  if (sprite->_bpp == 8)
  {
    uint16_t color = sprite->_img8[x + y * sprite->_iwidth];
    if (color != 0)
    {
    uint8_t  blue[] = {0, 11, 21, 31};
      color =   (color & 0xE0)<<8 | (color & 0xC0)<<5
              | (color & 0x1C)<<6 | (color & 0x1C)<<3
              | blue[color & 0x03];
    }
    return color;
  }

  if (sprite->_bpp == 4)
  {
    if (x >= sprite->_dwidth) return 0xFFFF;
    uint16_t color;
    if ((x & 0x01) == 0)
      color = sprite->_colorMap[sprite->_img4[((x+y*sprite->_iwidth)>>1)] >> 4];   // even index = bits 7 .. 4
    else
      color = sprite->_colorMap[sprite->_img4[((x+y*sprite->_iwidth)>>1)] & 0x0F]; // odd index = bits 3 .. 0.
    return color;
  }

  // Note: Must be 1bpp
  // _dwidth and _dheight bounds not checked (rounded up -iwidth and _iheight used)
  if (sprite->_tft->rotation == 1)
  {
    uint16_t tx = x;
    x = sprite->_dheight - y - 1;
    y = tx;
  }
  else if (sprite->_tft->rotation == 2)
  {
    x = sprite->_dwidth - x - 1;
    y = sprite->_dheight - y - 1;
  }
  else if (sprite->_tft->rotation == 3)
  {
    uint16_t tx = x;
    x = y;
    y = sprite->_dwidth - tx - 1;
  }

  uint16_t color = (sprite->_img8[(x + y * sprite->_bitwidth)>>3] << (x & 0x7)) & 0x80;

  if (color) return sprite->_tft->bitmap_fg;
  else       return sprite->_tft->bitmap_bg;
}


/***************************************************************************************
** Function name:           pushImage
** Description:             push image into a defined area of a sprite
***************************************************************************************/
static void  pushImage(struct TFT_eSprite* sprite, int32_t x, int32_t y, int32_t w, int32_t h, uint16_t *data, uint8_t sbpp)
{
  if (data == NULL || !sprite->_created) return;

  PI_CLIP(sprite->_tft);

  if (sprite->_bpp == 16) // Plot a 16 bpp image into a 16 bpp Sprite
  {
    // Pointer within original image
    uint8_t *ptro = (uint8_t *)data + ((dx + dy * w) << 1);
    // Pointer within sprite image
    uint8_t *ptrs = (uint8_t *)sprite->_img + ((x + y * sprite->_iwidth) << 1);

    if(sprite->_tft->_swapBytes)
    {
      while (dh--)
      {
        // Fast copy with a 1 byte shift
        memcpy(ptrs+1, ptro, (dw<<1) - 1);
        // Now correct just the even numbered bytes
        for (int32_t xp = 0; xp < (dw<<1); xp+=2)
        {
          ptrs[xp] = ptro[xp+1];;
        }
        ptro += w<<1;
        ptrs += sprite->_iwidth<<1;
      }
    }
    else
    {
      while (dh--)
      {
        memcpy(ptrs, ptro, dw<<1);
        ptro += w << 1;
        ptrs += sprite->_iwidth << 1;
      }
    }
  }
  else if (sprite->_bpp == 8 && sbpp == 8) // Plot a 8 bpp image into a 8 bpp Sprite
  {
    // Pointer within original image
    uint8_t *ptro = (uint8_t *)data + (dx + dy * w);
    // Pointer within sprite image
    uint8_t *ptrs = (uint8_t *)sprite->_img + (x + y * sprite->_iwidth);

    while (dh--)
    {
      memcpy(ptrs, ptro, dw);
      ptro += w;
      ptrs += sprite->_iwidth;
    }
  }
  else if (sprite->_bpp == 8) // Plot a 16 bpp image into a 8 bpp Sprite
  {
    uint16_t lastColor = 0;
    uint8_t  color8    = 0;
    for (int32_t yp = dy; yp < dy + dh; yp++)
    {
      int32_t xyw = x + y * sprite->_iwidth;
      int32_t dxypw = dx + yp * w;
      for (int32_t xp = dx; xp < dx + dw; xp++)
      {
        uint16_t color = data[dxypw++];
        if (color != lastColor) {
          // When data source is a sprite, the bytes are already swapped
          if(!sprite->_tft->_swapBytes) color8 = (uint8_t)((color & 0xE0) | (color & 0x07)<<2 | (color & 0x1800)>>11);
          else color8 = (uint8_t)((color & 0xE000)>>8 | (color & 0x0700)>>6 | (color & 0x0018)>>3);
        }
        lastColor = color;
        sprite->_img8[xyw++] = color8;
      }
      y++;
    }
  }
  else if (sprite->_bpp == 4)
  {
    // The image is assumed to be 4 bit, where each byte corresponds to two pixels.
    // much faster when aligned to a byte boundary, because the alternative is slower, requiring
    // tedious bit operations.

    int sWidth = (sprite->_iwidth >> 1);
    uint8_t *ptr = (uint8_t *)data;

    if ((x & 0x01) == 0 && (dx & 0x01) == 0 && (dw & 0x01) == 0)
    {
      x = (x >> 1) + y * sWidth;
      dw = (dw >> 1);
      dx = (dx >> 1) + dy * (w>>1);
      while (dh--)
      {
        memcpy(sprite->_img4 + x, ptr + dx, dw);
        dx += (w >> 1);
        x += sWidth;
      }
    }
    else  // not optimized
    {
      for (int32_t yp = dy; yp < dy + dh; yp++)
      {
        int32_t ox = x;
        for (int32_t xp = dx; xp < dx + dw; xp++)
        {
          uint32_t color;
          if ((xp & 0x01) == 0)
            color = (ptr[((xp+yp*w)>>1)] & 0xF0) >> 4; // even index = bits 7 .. 4
          else
            color = ptr[((xp-1+yp*w)>>1)] & 0x0F;      // odd index = bits 3 .. 0.
          sprite->drawPixel(sprite, ox, y, color);
          ox++;
        }
        y++;
      }
    }
  }

  else // 1bpp
  {
    // Plot a 1bpp image into a 1bpp Sprite
    uint32_t ww =  (w+7)>>3; // Width of source image line in bytes
    uint8_t *ptr = (uint8_t *)data;
    for (int32_t yp = dy;  yp < dy + dh; yp++)
    {
      uint32_t yw = yp * ww;              // Byte starting the line containing source pixel
      int32_t ox = x;
      for (int32_t xp = dx; xp < dx + dw; xp++)
      {
        uint16_t readPixel = (ptr[(xp>>3) + yw] & (0x80 >> (xp & 0x7)) );
        sprite->drawPixel(sprite, ox++, y, readPixel);
      }
      y++;
    }
  }
}


/***************************************************************************************
** Function name:           pushImage
** Description:             push 565 colour FLASH (PROGMEM) image into a defined area
***************************************************************************************/
static void  pushImageFlash(struct TFT_eSprite* sprite, int32_t x, int32_t y, int32_t w, int32_t h, const uint16_t *data)
{
#ifdef ESP32
  pushImage(x, y, w, h, (uint16_t*) data);
#else
  // Partitioned memory FLASH processor
  if (data == NULL || !sprite->_created) return;

  PI_CLIP(sprite->_tft);

  if (sprite->_bpp == 16) // Plot a 16 bpp image into a 16 bpp Sprite
  {
    for (int32_t yp = dy; yp < dy + dh; yp++)
    {
      int32_t ox = x;
      for (int32_t xp = dx; xp < dx + dw; xp++)
      {
        uint16_t color = pgm_read_word(data + xp + yp * w);
        if(sprite->_tft->_swapBytes) color = color<<8 | color>>8;
        sprite->_img[ox + y * sprite->_iwidth] = color;
        ox++;
      }
      y++;
    }
  }

  else if (sprite->_bpp == 8) // Plot a 16 bpp image into a 8 bpp Sprite
  {
    for (int32_t yp = dy; yp < dy + dh; yp++)
    {
      int32_t ox = x;
      for (int32_t xp = dx; xp < dx + dw; xp++)
      {
        uint16_t color = pgm_read_word(data + xp + yp * w);
        if(sprite->_tft->_swapBytes) color = color<<8 | color>>8;
        sprite->_img8[ox + y * sprite->_iwidth] = (uint8_t)((color & 0xE000)>>8 | (color & 0x0700)>>6 | (color & 0x0018)>>3);
        ox++;
      }
      y++;
    }
  }

  else if (sprite->_bpp == 4)
  {
    #ifdef TFT_eSPI_DEBUG
    Serial.println("pushImage(int32_t x, int32_t y, int32_t w, int32_t h, const uint16_t *data) not implemented");
    #endif
    return;
  }

  else // Plot a 1bpp image into a 1bpp Sprite
  {
    x-= sprite->_tft->_xDatum;   // Remove offsets, drawPixel will add
    y-= sprite->_tft->_yDatum;
    uint16_t bsw =  (w+7) >> 3; // Width in bytes of source image line
    uint8_t *ptr = ((uint8_t*)data) + dy * bsw;

    while (dh--) {
      int32_t odx = dx;
      int32_t ox  = x;
      while (odx < dx + dw) {
        uint8_t pbyte = pgm_read_byte(ptr + (odx>>3));
        uint8_t mask = 0x80 >> (odx & 7);
        while (mask) {
          uint8_t p = pbyte & mask;
          mask = mask >> 1;
          sprite->drawPixel(sprite, ox++, y, p);
          odx++;
        }
      }
      ptr += bsw;
      y++;
    }
  }
#endif // if ESP32 check
}


/***************************************************************************************
** Function name:           setWindow
** Description:             Set the bounds of a window in the sprite
***************************************************************************************/
// Intentionally not constrained to viewport area, does not manage 1bpp rotations
static void setWindow(struct TFT_eSprite* sprite, int32_t x0, int32_t y0, int32_t x1, int32_t y1)
{
  if (x0 > x1) swap_coord(&x0, &x1);
  if (y0 > y1) swap_coord(&y0, &y1);

  int32_t w = sprite->width(sprite);
  int32_t h = sprite->height(sprite);

  if ((x0 >= w) || (x1 < 0) || (y0 >= h) || (y1 < 0))
  { // Point to that extra "off screen" pixel
    sprite->_xs = 0;
    sprite->_ys = sprite->_dheight;
    sprite->_xe = 0;
    sprite->_ye = sprite->_dheight;
  }
  else
  {
    if (x0 < 0) x0 = 0;
    if (x1 >= w) x1 = w - 1;
    if (y0 < 0) y0 = 0;
    if (y1 >= h) y1 = h - 1;

    sprite->_xs = x0;
    sprite->_ys = y0;
    sprite->_xe = x1;
    sprite->_ye = y1;
  }

  sprite->_xptr = sprite->_xs;
  sprite->_yptr = sprite->_ys;
}


/***************************************************************************************
** Function name:           pushColor
** Description:             Send a new pixel to the set window
***************************************************************************************/
static void pushColor(struct TFT_eSprite* sprite, uint16_t color)
{
  if (!sprite->_created ) return;

  // Write the colour to RAM in set window
  if (sprite->_bpp == 16)
    sprite->_img [sprite->_xptr + sprite->_yptr * sprite->_iwidth] = (uint16_t) (color >> 8) | (color << 8);

  else  if (sprite->_bpp == 8)
    sprite->_img8[sprite->_xptr + sprite->_yptr * sprite->_iwidth] = (uint8_t )((color & 0xE000)>>8 | (color & 0x0700)>>6 | (color & 0x0018)>>3);

  else if (sprite->_bpp == 4)
  {
    uint8_t c = (uint8_t)color & 0x0F;
    if ((sprite->_xptr & 0x01) == 0) {
      sprite->_img4[(sprite->_xptr + sprite->_yptr * sprite->_iwidth)>>1] = (c << 4) | (sprite->_img4[(sprite->_xptr + sprite->_yptr * sprite->_iwidth)>>1] & 0x0F);  // new color is in bits 7 .. 4
    }
    else {
      sprite->_img4[(sprite->_xptr + sprite->_yptr * sprite->_iwidth)>>1] = (sprite->_img4[(sprite->_xptr + sprite->_yptr * sprite->_iwidth)>>1] & 0xF0) | c; // new color is the low bits
    }
  }

  else sprite->drawPixel(sprite, sprite->_xptr, sprite->_yptr, color);

  // Increment x
  sprite->_xptr++;

  // Wrap on x and y to start, increment y if needed
  if (sprite->_xptr > sprite->_xe)
  {
    sprite->_xptr = sprite->_xs;
    sprite->_yptr++;
    if (sprite->_yptr > sprite->_ye) sprite->_yptr = sprite->_ys;
  }

}


/***************************************************************************************
** Function name:           pushColor
** Description:             Send a "len" new pixels to the set window
***************************************************************************************/
static void pushColorLen(struct TFT_eSprite* sprite, uint16_t color, uint32_t len)
{
  if (!sprite->_created ) return;

  uint16_t pixelColor;

  if (sprite->_bpp == 16)
    pixelColor = (uint16_t) (color >> 8) | (color << 8);

  else  if (sprite->_bpp == 8)
    pixelColor = (color & 0xE000)>>8 | (color & 0x0700)>>6 | (color & 0x0018)>>3;

  else pixelColor = (uint16_t) color; // for 1bpp or 4bpp

  while(len--) sprite->writeColor(sprite, pixelColor);
}


/***************************************************************************************
** Function name:           writeColor
** Description:             Write a pixel with pre-formatted colour to the set window
***************************************************************************************/
static void writeColor(struct TFT_eSprite* sprite, uint16_t color)
{
  if (!sprite->_created ) return;

  // Write 16 bit RGB 565 encoded colour to RAM
  if (sprite->_bpp == 16) sprite->_img [sprite->_xptr + sprite->_yptr * sprite->_iwidth] = color;

  // Write 8 bit RGB 332 encoded colour to RAM
  else if (sprite->_bpp == 8) sprite->_img8[sprite->_xptr + sprite->_yptr * sprite->_iwidth] = (uint8_t) color;

  else if (sprite->_bpp == 4)
  {
    uint8_t c = (uint8_t)color & 0x0F;
    if ((sprite->_xptr & 0x01) == 0)
      sprite->_img4[(sprite->_xptr + sprite->_yptr * sprite->_iwidth)>>1] = (c << 4) | (sprite->_img4[(sprite->_xptr + sprite->_yptr * sprite->_iwidth)>>1] & 0x0F);  // new color is in bits 7 .. 4
    else
      sprite->_img4[(sprite->_xptr + sprite->_yptr * sprite->_iwidth)>>1] = (sprite->_img4[(sprite->_xptr + sprite->_yptr * sprite->_iwidth)>>1] & 0xF0) | c; // new color is the low bits (x is odd)
  }

  else sprite->drawPixel(sprite, sprite->_xptr, sprite->_yptr, color);

  // Increment x
 sprite-> _xptr++;

  // Wrap on x and y to start, increment y if needed
  if (sprite->_xptr > sprite->_xe)
  {
    sprite->_xptr = sprite->_xs;
    sprite->_yptr++;
    if (sprite->_yptr > sprite->_ye) sprite->_yptr = sprite->_ys;
  }
}


/***************************************************************************************
** Function name:           setScrollRect
** Description:             Set scroll area within the sprite and the gap fill colour
***************************************************************************************/
// Intentionally not constrained to viewport area
static void setScrollRect(struct TFT_eSprite* sprite, int32_t x, int32_t y, int32_t w, int32_t h, uint16_t color)
{
  if ((x >= sprite->_iwidth) || (y >= sprite->_iheight) || !sprite->_created ) return;

  if (x < 0) { w += x; x = 0; }
  if (y < 0) { h += y; y = 0; }

  if ((x + w) > sprite->_iwidth ) w = sprite->_iwidth  - x;
  if ((y + h) > sprite->_iheight) h = sprite->_iheight - y;

  if ( w < 1 || h < 1) return;

  sprite->_sx = x;
  sprite->_sy = y;
  sprite->_sw = w;
  sprite->_sh = h;

  sprite->_scolor = color;
}


/***************************************************************************************
** Function name:           scroll
** Description:             Scroll dx,dy pixels, positive right,down, negative left,up
***************************************************************************************/
static void scroll(struct TFT_eSprite* sprite, int16_t dx, int16_t dy)
{
  if (abs(dx) >= sprite->_sw || abs(dy) >= sprite->_sh)
  {
    sprite->fillRect (sprite, sprite->_sx, sprite->_sy, sprite->_sw, sprite->_sh, sprite->_scolor);
    return;
  }

  // Fetch the scroll area width and height set by setScrollRect()
  uint32_t w  = sprite->_sw - abs(dx); // line width to copy
  uint32_t h  = sprite->_sh - abs(dy); // lines to copy
  int32_t iw  = sprite->_iwidth;       // rounded up width of sprite

  // Fetch the x,y origin set by setScrollRect()
  uint32_t tx = sprite->_sx; // to x
  uint32_t fx = sprite->_sx; // from x
  uint32_t ty = sprite->_sy; // to y
  uint32_t fy = sprite->_sy; // from y

  // Adjust for x delta
  if (dx <= 0) fx -= dx;
  else tx += dx;

  // Adjust for y delta
  if (dy <= 0) fy -= dy;
  else
  { // Scrolling down so start copy from bottom
    ty = ty + sprite->_sh - 1; // "To" pointer
    iw = -iw;          // Pointer moves backwards
    fy = ty - dy;      // "From" pointer
  }

  // Calculate "from y" and "to y" pointers in RAM
  uint32_t fyp = fx + fy * sprite->_iwidth;
  uint32_t typ = tx + ty * sprite->_iwidth;

  // Now move the pixels in RAM
  if (sprite->_bpp == 16)
  {
    while (h--)
    { // move pixel lines (to, from, byte count)
      memmove( sprite->_img + typ, sprite->_img + fyp, w<<1);
      typ += iw;
      fyp += iw;
    }
  }
  else if (sprite->_bpp == 8)
  {
    while (h--)
    { // move pixel lines (to, from, byte count)
      memmove( sprite->_img8 + typ, sprite->_img8 + fyp, w);
      typ += iw;
      fyp += iw;
    }
  }
  else if (sprite->_bpp == 4)
  {
    // could optimize for scrolling by even # pixels using memove (later)
    if (dx >  0) { tx += w; fx += w; } // Start from right edge
    while (h--)
    { // move pixels one by one
      for (uint16_t xp = 0; xp < w; xp++)
      {
        if (dx <= 0) sprite->drawPixel(sprite, tx + xp, ty, sprite->readPixelValue(sprite, fx + xp, fy));
        if (dx >  0) sprite->drawPixel(sprite, tx - xp, ty, sprite->readPixelValue(sprite, fx - xp, fy));
      }
      if (dy <= 0)  { ty++; fy++; }
      else  { ty--; fy--; }
    }
  }
  else if (sprite->_bpp == 1 )
  {
    if (dx >  0) { tx += w; fx += w; } // Start from right edge
    while (h--)
    { // move pixels one by one
      for (uint16_t xp = 0; xp < w; xp++)
      {
        if (dx <= 0) sprite->drawPixel(sprite, tx + xp, ty, sprite->readPixelValue(sprite, fx + xp, fy));
        if (dx >  0) sprite->drawPixel(sprite, tx - xp, ty, sprite->readPixelValue(sprite, fx - xp, fy));
      }
      if (dy <= 0)  { ty++; fy++; }
      else  { ty--; fy--; }
    }
  }
  else return; // Not 1, 4, 8 or 16 bpp

  // Fill the gap left by the scrolling
  if (dx > 0) sprite->fillRect(sprite, sprite->_sx, sprite->_sy, dx, sprite->_sh, sprite->_scolor);
  if (dx < 0) sprite->fillRect(sprite, sprite->_sx + sprite->_sw + dx, sprite->_sy, -dx, sprite->_sh, sprite->_scolor);
  if (dy > 0) sprite->fillRect(sprite, sprite->_sx, sprite->_sy, sprite->_sw, dy, sprite->_scolor);
  if (dy < 0) sprite->fillRect(sprite, sprite->_sx, sprite->_sy + sprite->_sh + dy, sprite->_sw, -dy, sprite->_scolor);
}


/***************************************************************************************
** Function name:           fillSprite
** Description:             Fill the whole sprite with defined colour
***************************************************************************************/
static void fillSprite(struct TFT_eSprite* sprite, uint32_t color)
{
  if (!sprite->_created || sprite->_tft->_vpOoB) return;

  // Use memset if possible as it is super fast
  if(sprite->_tft->_xDatum == 0 && sprite->_tft->_yDatum == 0  &&  sprite->_tft->_xWidth == sprite->width(sprite))
  {
    if(sprite->_bpp == 16) {
      if ( (uint8_t)color == (uint8_t)(color>>8) ) {
        memset(sprite->_img,  (uint8_t)color, sprite->_iwidth * sprite->_tft->_yHeight * 2);
      }
      else sprite->fillRect(sprite, sprite->_tft->_vpX, sprite->_tft->_vpY, sprite->_tft->_xWidth, sprite->_tft->_yHeight, color);
    }
    else if (sprite->_bpp == 8)
    {
      color = (color & 0xE000)>>8 | (color & 0x0700)>>6 | (color & 0x0018)>>3;
      memset(sprite->_img8, (uint8_t)color, sprite->_iwidth * sprite->_tft->_yHeight);
    }
    else if (sprite->_bpp == 4)
    {
      uint8_t c = ((color & 0x0F) | (((color & 0x0F) << 4) & 0xF0));
      memset(sprite->_img4, c, (sprite->_iwidth * sprite->_tft->_yHeight) >> 1);
    }
    else if (sprite->_bpp == 1)
    {
      if(color) memset(sprite->_img8, 0xFF, (sprite->_bitwidth>>3) * sprite->_dheight + 1);
      else      memset(sprite->_img8, 0x00, (sprite->_bitwidth>>3) * sprite->_dheight + 1);
    }
  }
  else sprite->fillRect(sprite, sprite->_tft->_vpX - sprite->_tft->_xDatum, sprite->_tft->_vpY - sprite->_tft->_yDatum, sprite->_tft->_xWidth, sprite->_tft->_yHeight, color);
}


/***************************************************************************************
** Function name:           width
** Description:             Return the width of sprite
***************************************************************************************/
// Return the size of the sprite
static int16_t width(struct TFT_eSprite* sprite)
{
  if (!sprite->_created ) return 0;

  if (sprite->_bpp > 1) {
    if (sprite->_tft->_vpDatum) return sprite->_tft->_xWidth;
    return sprite->_dwidth;
  }

  if (sprite->_tft->rotation & 1) {
    if (sprite->_tft->_vpDatum) return sprite->_tft->_xWidth;
    return sprite->_dheight;
  }

  if (sprite->_tft->_vpDatum) return sprite->_tft->_xWidth;
  return sprite->_dwidth;
}


/***************************************************************************************
** Function name:           height
** Description:             Return the height of sprite
***************************************************************************************/
static int16_t height(struct TFT_eSprite* sprite)
{
  if (!sprite->_created ) return 0;

  if (sprite->_bpp > 1) {
    if (sprite->_tft->_vpDatum) return sprite->_tft->_yHeight;
    return sprite->_dheight;
  }

  if (sprite->_tft->rotation & 1) {
    if (sprite->_tft->_vpDatum) return sprite->_tft->_yHeight;
    return sprite->_dwidth;
  }

  if (sprite->_tft->_vpDatum) return sprite->_tft->_yHeight;
  return sprite->_dheight;
}


/***************************************************************************************
** Function name:           setRotation
** Description:             Rotate coordinate frame for 1bpp sprite
***************************************************************************************/
// Does nothing for 4, 8 and 16 bpp sprites.
static void setRotation(struct TFT_eSprite* sprite, uint8_t r)
{
  if (sprite->_bpp != 1) return;

  sprite->_tft->rotation = r;

  if (sprite->_tft->rotation&1) {
    sprite->_tft->resetViewport(sprite->_tft);
  }
  else {
    sprite->_tft->resetViewport(sprite->_tft);
  }
}


/***************************************************************************************
** Function name:           getRotation
** Description:             Get rotation for 1bpp sprite
***************************************************************************************/
static uint8_t getRotation(struct TFT_eSprite* sprite)
{
  return sprite->_tft->rotation;
}


/***************************************************************************************
** Function name:           drawPixel
** Description:             push a single pixel at an arbitrary position
***************************************************************************************/
static void drawPixel(struct TFT_eSprite* sprite, int32_t x, int32_t y, uint32_t color)
{
  if (!sprite->_created || sprite->_tft->_vpOoB) return;

  x+= sprite->_tft->_xDatum;
  y+= sprite->_tft->_yDatum;

  // Range checking
  if ((x < sprite->_tft->_vpX) || (y < sprite->_tft->_vpY) ||(x >= sprite->_tft->_vpW) || (y >= sprite->_tft->_vpH)) return;

  if (sprite->_bpp == 16)
  {
    color = (color >> 8) | (color << 8);
    sprite->_img[x+y*sprite->_iwidth] = (uint16_t) color;
  }
  else if (sprite->_bpp == 8)
  {
    sprite->_img8[x+y*sprite->_iwidth] = (uint8_t)((color & 0xE000)>>8 | (color & 0x0700)>>6 | (color & 0x0018)>>3);
  }
  else if (sprite->_bpp == 4)
  {
    uint8_t c = color & 0x0F;
    int index = (x+y*sprite->_iwidth)>>1;;
    if ((x & 0x01) == 0) {
      sprite->_img4[index] = (uint8_t)((c << 4) | (sprite->_img4[index] & 0x0F));
    }
    else {
      sprite->_img4[index] =  (uint8_t)(c | (sprite->_img4[index] & 0xF0));
    }
  }
  else // 1 bpp
  {
    if (sprite->_tft->rotation == 1)
    {
      uint16_t tx = x;
      x = sprite->_dwidth - y - 1;
      y = tx;
    }
    else if (sprite->_tft->rotation == 2)
    {
      x = sprite->_dwidth - x - 1;
      y = sprite->_dheight - y - 1;
    }
    else if (sprite->_tft->rotation == 3)
    {
      uint16_t tx = x;
      x = y;
      y = sprite->_dheight - tx - 1;
    }

    if (color) sprite->_img8[(x + y * sprite->_bitwidth)>>3] |=  (0x80 >> (x & 0x7));
    else       sprite->_img8[(x + y * sprite->_bitwidth)>>3] &= ~(0x80 >> (x & 0x7));
  }
}


/***************************************************************************************
** Function name:           drawLine
** Description:             draw a line between 2 arbitrary points
***************************************************************************************/
static void drawLine(struct TFT_eSprite* sprite, int32_t x0, int32_t y0, int32_t x1, int32_t y1, uint32_t color)
{
  if (!sprite->_created || sprite->_tft->_vpOoB) return;

  //_xDatum and _yDatum Not added here, it is added by drawPixel & drawFastxLine

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
        err += dx;
        if (dlen == 1) sprite->drawPixel(sprite, y0, xs, color);
        else sprite->drawFastVLine(sprite, y0, xs, dlen, color);
        dlen = 0; y0 += ystep; xs = x0 + 1;
      }
    }
    if (dlen) sprite->drawFastVLine(sprite, y0, xs, dlen, color);
  }
  else
  {
    for (; x0 <= x1; x0++) {
      dlen++;
      err -= dy;
      if (err < 0) {
        err += dx;
        if (dlen == 1) sprite->drawPixel(sprite, xs, y0, color);
        else sprite->drawFastHLine(sprite, xs, y0, dlen, color);
        dlen = 0; y0 += ystep; xs = x0 + 1;
      }
    }
    if (dlen) sprite->drawFastHLine(sprite, xs, y0, dlen, color);
  }
}


/***************************************************************************************
** Function name:           drawFastVLine
** Description:             draw a vertical line
***************************************************************************************/
static void drawFastVLine(struct TFT_eSprite* sprite, int32_t x, int32_t y, int32_t h, uint32_t color)
{
  if (!sprite->_created || sprite->_tft->_vpOoB) return;

  x+= sprite->_tft->_xDatum;
  y+= sprite->_tft->_yDatum;

  // Clipping
  if ((x < sprite->_tft->_vpX) || (x >= sprite->_tft->_vpW) || (y >= sprite->_tft->_vpH)) return;

  if (y < sprite->_tft->_vpY) { h += y - sprite->_tft->_vpY; y = sprite->_tft->_vpY; }

  if ((y + h) > sprite->_tft->_vpH) h = sprite->_tft->_vpH - y;

  if (h < 1) return;

  if (sprite->_bpp == 16)
  {
    color = (color >> 8) | (color << 8);
    int32_t yp = x + sprite->_iwidth * y;
    while (h--) {sprite->_img[yp] = (uint16_t) color; yp += sprite->_iwidth;}
  }
  else if (sprite->_bpp == 8)
  {
    color = (color & 0xE000)>>8 | (color & 0x0700)>>6 | (color & 0x0018)>>3;
    while (h--) sprite->_img8[x + sprite->_iwidth * y++] = (uint8_t) color;
  }
  else if (sprite->_bpp == 4)
  {
    if ((x & 0x01) == 0)
    {
      uint8_t c = (uint8_t) (color & 0xF) << 4;
      while (h--) {
        sprite->_img4[(x + sprite->_iwidth * y)>>1] = (uint8_t) (c | (sprite->_img4[(x + sprite->_iwidth * y)>>1] & 0x0F));
        y++;
      }
    }
    else {
      uint8_t c = (uint8_t)color & 0xF;
      while (h--) {
        sprite->_img4[(x + sprite->_iwidth * y)>>1] = (uint8_t) (c | (sprite->_img4[(x + sprite->_iwidth * y)>>1] & 0xF0)); // x is odd; new color goes into the low bits.
        y++;
      }
    }
  }
  else
  {
    x -= sprite->_tft->_xDatum; // Remove any offset as it will be added by drawPixel
    y -= sprite->_tft->_yDatum;
    while (h--)
    {
      sprite->drawPixel(sprite, x, y++, color);
    }
  }
}


/***************************************************************************************
** Function name:           drawFastHLine
** Description:             draw a horizontal line
***************************************************************************************/
static void drawFastHLine(struct TFT_eSprite* sprite, int32_t x, int32_t y, int32_t w, uint32_t color)
{
  if (!sprite->_created || sprite->_tft->_vpOoB) return;

  x+= sprite->_tft->_xDatum;
  y+= sprite->_tft->_yDatum;

  // Clipping
  if ((y < sprite->_tft->_vpY) || (x >= sprite->_tft->_vpW) || (y >= sprite->_tft->_vpH)) return;

  if (x < sprite->_tft->_vpX) { w += x - sprite->_tft->_vpX; x = sprite->_tft->_vpX; }

  if ((x + w) > sprite->_tft->_vpW) w = sprite->_tft->_vpW - x;

  if (w < 1) return;

  if (sprite->_bpp == 16)
  {
    color = (color >> 8) | (color << 8);
    while (w--) sprite->_img[sprite->_iwidth * y + x++] = (uint16_t) color;
  }
  else if (sprite->_bpp == 8)
  {
    color = (color & 0xE000)>>8 | (color & 0x0700)>>6 | (color & 0x0018)>>3;
    memset(sprite->_img8+sprite->_iwidth * y + x, (uint8_t)color, w);
  }
  else if (sprite->_bpp == 4)
  {
    uint8_t c = (uint8_t)color & 0x0F;
    uint8_t c2 = (c | ((c << 4) & 0xF0));
    if ((x & 0x01) == 1)
    {
      sprite->drawPixel(sprite, x - sprite->_tft->_xDatum, y - sprite->_tft->_yDatum, color);
      x++; w--;
      if (w < 1)
        return;
    }

    if (((w + x) & 0x01) == 1)
    {
      // handle the extra one at the other end
      sprite->drawPixel(sprite, x - sprite->_tft->_xDatum + w - 1, y - sprite->_tft->_yDatum, color);
      w--;
      if (w < 1) return;
    }
    memset(sprite->_img4 + ((sprite->_iwidth * y + x) >> 1), c2, (w >> 1));
  }
  else {
    x -= sprite->_tft->_xDatum; // Remove any offset as it will be added by drawPixel
    y -= sprite->_tft->_yDatum;

    while (w--)
    {
      sprite->drawPixel(sprite, x++, y, color);
    }
  }
}


/***************************************************************************************
** Function name:           fillRect
** Description:             draw a filled rectangle
***************************************************************************************/
static void fillRect(struct TFT_eSprite* sprite, int32_t x, int32_t y, int32_t w, int32_t h, uint32_t color)
{
  if (!sprite->_created || sprite->_tft->_vpOoB) return;

  x+= sprite->_tft->_xDatum;
  y+= sprite->_tft->_yDatum;

  // Clipping
  if ((x >= sprite->_tft->_vpW) || (y >= sprite->_tft->_vpH)) return;

  if (x < sprite->_tft->_vpX) { w += x - sprite->_tft->_vpX; x = sprite->_tft->_vpX; }
  if (y < sprite->_tft->_vpY) { h += y - sprite->_tft->_vpY; y = sprite->_tft->_vpY; }

  if ((x + w) > sprite->_tft->_vpW) w = sprite->_tft->_vpW - x;
  if ((y + h) > sprite->_tft->_vpH) h = sprite->_tft->_vpH - y;

  if ((w < 1) || (h < 1)) return;

  int32_t yp = sprite->_iwidth * y + x;

  if (sprite->_bpp == 16)
  {
    color = (color >> 8) | (color << 8);
    uint32_t iw = w;
    int32_t ys = yp;
    if(h--)  {while (iw--) sprite->_img[yp++] = (uint16_t) color;}
    yp = ys;
    while (h--)
    {
      yp += sprite->_iwidth;
      memcpy( sprite->_img+yp,sprite-> _img+ys, w<<1);
    }
  }
  else if (sprite->_bpp == 8)
  {
    color = (color & 0xE000)>>8 | (color & 0x0700)>>6 | (color & 0x0018)>>3;
    while (h--)
    {
      memset(sprite->_img8 + yp, (uint8_t)color, w);
      yp += sprite->_iwidth;
    }
  }
  else if (sprite->_bpp == 4)
  {
    uint8_t c1 = (uint8_t)color & 0x0F;
    uint8_t c2 = c1 | ((c1 << 4) & 0xF0);
    if ((x & 0x01) == 0 && (w & 0x01) == 0)
    {
      yp = (yp >> 1);
      while (h--)
      {
        memset(sprite->_img4 + yp, c2, (w>>1));
        yp += (sprite->_iwidth >> 1);
      }
    }
    else if ((x & 0x01) == 0)
    {

      // same as above but you have a hangover on the right.
      yp = (yp >> 1);
      while (h--)
      {
        if (w > 1)
          memset(sprite->_img4 + yp, c2, (w-1)>>1);
        // handle the rightmost pixel by calling drawPixel
        sprite->drawPixel(sprite, x+w-1-sprite->_tft->_xDatum, y+h-sprite->_tft->_yDatum, c1);
        yp += (sprite->_iwidth >> 1);
      }
    }
    else if ((w & 0x01) == 1)
    {
      yp = (yp + 1) >> 1;
      while (h--) {
        sprite->drawPixel(sprite, x-sprite->_tft->_xDatum, y+h-sprite->_tft->_yDatum, color & 0x0F);
        if (w > 1)
          memset(sprite->_img4 + yp, c2, (w-1)>>1);
        // same as above but you have a hangover on the left instead
        yp += (sprite->_iwidth >> 1);
      }
    }
    else
    {
      yp = (yp + 1) >> 1;
      while (h--) {
        sprite->drawPixel(sprite, x-sprite->_tft->_xDatum, y+h-sprite->_tft->_yDatum, color & 0x0F);
        if (w > 1) sprite->drawPixel(sprite, x+w-1-sprite->_tft->_xDatum, y+h-sprite->_tft->_yDatum, color & 0x0F);
        if (w > 2)
          memset(sprite->_img4 + yp, c2, (w-2)>>1);
        // maximal hacking, single pixels on left and right.
        yp += (sprite->_iwidth >> 1);
      }
    }
  }
  else
  {
    x -= sprite->_tft->_xDatum;
    y -= sprite->_tft->_yDatum;
    while (h--)
    {
      int32_t ww = w;
      int32_t xx = x;
      while (ww--) sprite->drawPixel(sprite, xx++, y, color);
      y++;
    }
  }
}


/***************************************************************************************
** Function name:           drawChar
** Description:             draw a single character in the Adafruit GLCD or freefont
***************************************************************************************/
static void drawChar(struct TFT_eSprite* sprite, int32_t x, int32_t y, uint16_t c, uint32_t color, uint32_t bg, uint8_t size)
{
  if ( sprite->_tft->_vpOoB || !sprite->_created ) return;

  if ((x >= sprite->_tft->_vpW - sprite->_tft->_xDatum) || // Clip right
      (y >= sprite->_tft->_vpH - sprite->_tft->_yDatum))   // Clip bottom
    return;

  if (c < 32) return;
#ifdef LOAD_GLCD
//>>>>>>>>>>>>>>>>>>
#ifdef LOAD_GFXFF
  if(!gfxFont) { // 'Classic' built-in font
#endif
//>>>>>>>>>>>>>>>>>>

  if (((x + 6 * size - 1) < (sprite->_vpX - sprite->_xDatum)) || // Clip left
      ((y + 8 * size - 1) < (sprite->_vpY - sprite->_yDatum)))   // Clip top
    return;

  bool fillbg = (bg != color);

  if ((size==1) && fillbg)
  {
    uint8_t column[6];
    uint8_t mask = 0x1;

    for (int8_t i = 0; i < 5; i++ ) column[i] = pgm_read_byte(font + (c * 5) + i);
    column[5] = 0;

    int8_t j, k;
    for (j = 0; j < 8; j++) {
      for (k = 0; k < 5; k++ ) {
        if (column[k] & mask) {
          sprite->drawPixel(sprite, x + k, y + j, color);
        }
        else {
          sprite->drawPixel(sprite, x + k, y + j, bg);
        }
      }

      mask <<= 1;

      sprite->drawPixel(sprite, x + k, y + j, bg);
    }
  }
  else
  {
    for (int8_t i = 0; i < 6; i++ ) {
      uint8_t line;
      if (i == 5)
        line = 0x0;
      else
        line = pgm_read_byte(font + (c * 5) + i);

      if (size == 1) // default size
      {
        for (int8_t j = 0; j < 8; j++) {
          if (line & 0x1) sprite->drawPixel(sprite, x + i, y + j, color);
          line >>= 1;
        }
      }
      else {  // big size
        for (int8_t j = 0; j < 8; j++) {
          if (line & 0x1) sprite->fillRect(sprite, x + (i * size), y + (j * size), size, size, color);
          else if (fillbg) sprite->fillRect(sprite, x + i * size, y + j * size, size, size, bg);
          line >>= 1;
        }
      }
    }
  }

//>>>>>>>>>>>>>>>>>>>>>>>>>>>
#ifdef LOAD_GFXFF
  } else { // Custom font
#endif
//>>>>>>>>>>>>>>>>>>>>>>>>>>>
#endif // LOAD_GLCD

#ifdef LOAD_GFXFF
    // Filter out bad characters not present in font
    if ((c >= pgm_read_word(&gfxFont->first)) && (c <= pgm_read_word(&gfxFont->last )))
    {
//>>>>>>>>>>>>>>>>>>>>>>>>>>>

      c -= pgm_read_word(&gfxFont->first);
      GFXglyph *glyph  = &(((GFXglyph *)pgm_read_dword(&gfxFont->glyph))[c]);

      uint8_t  w  = pgm_read_byte(&glyph->width),
               h  = pgm_read_byte(&glyph->height);
      int8_t   xo = pgm_read_byte(&glyph->xOffset),
               yo = pgm_read_byte(&glyph->yOffset);

      if (((x + xo + w * size - 1) < (_vpX - _xDatum)) || // Clip left
          ((y + yo + h * size - 1) < (_vpY - _yDatum)))   // Clip top
        return;

      uint8_t  *bitmap = (uint8_t *)pgm_read_dword(&gfxFont->bitmap);
      uint32_t bo = pgm_read_word(&glyph->bitmapOffset);

      uint8_t  xx, yy, bits=0, bit=0;
      //uint8_t  xa = pgm_read_byte(&glyph->xAdvance);
      int16_t  xo16 = 0, yo16 = 0;

      if(size > 1) {
        xo16 = xo;
        yo16 = yo;
      }

      uint16_t hpc = 0; // Horizontal foreground pixel count
      for(yy=0; yy<h; yy++) {
        for(xx=0; xx<w; xx++) {
          if(bit == 0) {
            bits = pgm_read_byte(&bitmap[bo++]);
            bit  = 0x80;
          }
          if(bits & bit) hpc++;
          else {
            if (hpc) {
              if(size == 1) sprite->drawFastHLine(sprite, x+xo+xx-hpc, y+yo+yy, hpc, color);
              else sprite->fillRect(sprite, x+(xo16+xx-hpc)*size, y+(yo16+yy)*size, size*hpc, size, color);
              hpc=0;
            }
          }
          bit >>= 1;
        }
        // Draw pixels for this line as we are about to increment yy
        if (hpc) {
          if(size == 1) sprite->drawFastHLine(sprite, x+xo+xx-hpc, y+yo+yy, hpc, color);
          else sprite->fillRect(sprite, x+(xo16+xx-hpc)*size, y+(yo16+yy)*size, size*hpc, size, color);
          hpc=0;
        }
      }
    }
#endif


#ifdef LOAD_GLCD
  #ifdef LOAD_GFXFF
  } // End classic vs custom font
  #endif
#else
  #ifndef LOAD_GFXFF
    color = color;
    bg = bg;
    size = size;
  #endif
#endif

}


/***************************************************************************************
** Function name:           drawChar
** Description:             draw a unicode glyph into the sprite
***************************************************************************************/
  // TODO: Rationalise with TFT_eSPI
  // Any UTF-8 decoding must be done before calling drawChar()
static int16_t drawCharUni(struct TFT_eSprite* sprite, uint16_t uniCode, int32_t x, int32_t y)
{
  return sprite->drawCharUniFont(sprite, uniCode, x, y, sprite->_tft->textfont);
}

  // Any UTF-8 decoding must be done before calling drawChar()
static int16_t drawCharUniFont(struct TFT_eSprite* sprite, uint16_t uniCode, int32_t x, int32_t y, uint8_t font)
{
  if (sprite->_tft->_vpOoB || !uniCode) return 0;

  if (font==1) {
#ifdef LOAD_GLCD
  #ifndef LOAD_GFXFF
    sprite->drawChar(sprite, x, y, uniCode, textcolor, textbgcolor, textsize);
    return 6 * textsize;
  #endif
#else
  #ifndef LOAD_GFXFF
    return 0;
  #endif
#endif

#ifdef LOAD_GFXFF
    sprite->drawChar(sprite, x, y, uniCode, textcolor, textbgcolor, textsize);
    if(!gfxFont) { // 'Classic' built-in font
    #ifdef LOAD_GLCD
      return 6 * textsize;
    #else
      return 0;
    #endif
    }
    else {
      if((uniCode >= pgm_read_word(&gfxFont->first)) && (uniCode <= pgm_read_word(&gfxFont->last) )) {
        uint16_t   c2    = uniCode - pgm_read_word(&gfxFont->first);
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

  int32_t width  = 0;
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

  int32_t xd = x + sprite->_tft->_xDatum;
  int32_t yd = y + sprite->_tft->_yDatum;

  if ((xd + width * sprite->_tft->textsize < sprite->_tft->_vpX || xd >= sprite->_tft->_vpW) && (yd + height * sprite->_tft->textsize < sprite->_tft->_vpY || yd >= sprite->_tft->_vpH))
    return width * sprite->_tft->textsize ;

  int32_t w = width;
  int32_t pX      = 0;
  int32_t pY      = y;
  uint8_t line = 0;
  bool clip = xd < sprite->_tft->_vpX || xd + width  * sprite->_tft->textsize >= sprite->_tft->_vpW || yd < sprite->_tft->_vpY || yd + height * sprite->_tft->textsize >= sprite->_tft->_vpH;

#ifdef LOAD_FONT2 // chop out code if we do not need it
  if (font == 2) {
    w = w + 6; // Should be + 7 but we need to compensate for width increment
    w = w / 8;

    for (int32_t i = 0; i < height; i++)
    {
      if (textcolor != textbgcolor) sprite->fillRect(sprite, x, pY, width * textsize, textsize, textbgcolor);

      for (int32_t k = 0; k < w; k++)
      {
        line = pgm_read_byte((uint8_t *)flash_address + w * i + k);
        if (line) {
          if (textsize == 1) {
            pX = x + k * 8;
            if (line & 0x80) sprite->drawPixel(sprite, pX, pY, textcolor);
            if (line & 0x40) sprite->drawPixel(sprite, pX + 1, pY, textcolor);
            if (line & 0x20) sprite->drawPixel(sprite, pX + 2, pY, textcolor);
            if (line & 0x10) sprite->drawPixel(sprite, pX + 3, pY, textcolor);
            if (line & 0x08) sprite->drawPixel(sprite, pX + 4, pY, textcolor);
            if (line & 0x04) sprite->drawPixel(sprite, pX + 5, pY, textcolor);
            if (line & 0x02) sprite->drawPixel(sprite, pX + 6, pY, textcolor);
            if (line & 0x01) sprite->drawPixel(sprite, pX + 7, pY, textcolor);
          }
          else {
            pX = x + k * 8 * textsize;
            if (line & 0x80) sprite->fillRect(sprite, pX, pY, textsize, textsize, textcolor);
            if (line & 0x40) sprite->fillRect(sprite, pX + textsize, pY, textsize, textsize, textcolor);
            if (line & 0x20) sprite->fillRect(sprite, pX + 2 * textsize, pY, textsize, textsize, textcolor);
            if (line & 0x10) sprite->fillRect(sprite, pX + 3 * textsize, pY, textsize, textsize, textcolor);
            if (line & 0x08) sprite->fillRect(sprite, pX + 4 * textsize, pY, textsize, textsize, textcolor);
            if (line & 0x04) sprite->fillRect(sprite, pX + 5 * textsize, pY, textsize, textsize, textcolor);
            if (line & 0x02) sprite->fillRect(sprite, pX + 6 * textsize, pY, textsize, textsize, textcolor);
            if (line & 0x01) sprite->fillRect(sprite, pX + 7 * textsize, pY, textsize, textsize, textcolor);
          }
        }
      }
      pY += textsize;
    }
  }

  #ifdef LOAD_RLE
  else
  #endif
#endif  //FONT2

#ifdef LOAD_RLE  //674 bytes of code
  // Font is not 2 and hence is RLE encoded
  {
    w *= height; // Now w is total number of pixels in the character
    int16_t color = textcolor;
    if (sprite->_bpp == 16) color = (textcolor >> 8) | (textcolor << 8);
    else if (sprite->_bpp == 8) color = ((textcolor & 0xE000)>>8 | (textcolor & 0x0700)>>6 | (textcolor & 0x0018)>>3);

    int16_t bgcolor = textbgcolor;
    if (sprite->_bpp == 16) bgcolor = (textbgcolor >> 8) | (textbgcolor << 8);
    else if (sprite->_bpp == 8) bgcolor = ((textbgcolor & 0xE000)>>8 | (textbgcolor & 0x0700)>>6 | (textbgcolor & 0x0018)>>3);

    if (textcolor == textbgcolor && !clip && sprite->_bpp != 1) {
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
            sprite->setWindow(sprite, px, py, px + ts, py + ts);

            if (ts) {
              tnp = np;
              while (tnp--) sprite->writeColor(sprite, color);
            }
            else sprite->writeColor(sprite, color);

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
      if (textcolor != textbgcolor && textsize == 1 && !clip && _bpp != 1)
      {
        sprite->setWindow(sprite, xd, yd, xd + width - 1, yd + height - 1);

        // Maximum font size is equivalent to 180x180 pixels in area
        while (w > 0) {
          line = pgm_read_byte((uint8_t *)flash_address++); // 8 bytes smaller when incrementing here
          if (line & 0x80) {
            line &= 0x7F;
            line++; w -= line;
            while (line--) sprite->writeColor(sprite, color);
          }
          else {
            line++; w -= line;
            while (line--) sprite->writeColor(sprite, bgcolor);
          }
        }
      }
      else
      {
        int32_t px = 0, py = 0;  // To hold character pixel coords
        int32_t tx = 0, ty = 0;  // To hold character TFT pixel coords
        int32_t pc = 0;          // Pixel count
        int32_t pl = 0;          // Pixel line length
        uint16_t pcol = 0;       // Pixel color
        bool     pf = true;      // Flag for plotting
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
              if (pf) sprite->fillRect(sprite, tx, ty, pl * textsize, textsize, pcol);
              pl = 0;
              px = 0;
              tx = x;
              py ++;
              ty += textsize;
            }
          }
          if (pl && pf) sprite->fillRect(sprite, tx, ty, pl * textsize, textsize, pcol);
        }
      }
    }
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

  return width * sprite->_tft->textsize;    // x +
}


#ifdef SMOOTH_FONT
/***************************************************************************************
** Function name:           drawGlyph
** Description:             Write a character to the sprite cursor position
***************************************************************************************/
//
static void drawGlyph(struct TFT_eSprite* sprite, uint16_t code)
{
  uint16_t fg = textcolor;
  uint16_t bg = textbgcolor;
  bool getBG  = false;
  if (fg == bg) getBG = true;

  // Check if cursor has moved
  if (last_cursor_x != cursor_x)
  {
    bg_cursor_x = cursor_x;
    last_cursor_x = cursor_x;
  }

  if (code < 0x21)
  {
    if (code == 0x20) {
      if (sprite->_fillbg) fillRect(bg_cursor_x, cursor_y, (cursor_x + gFont.spaceWidth) - bg_cursor_x, gFont.yAdvance, bg);
      cursor_x += gFont.spaceWidth;
      bg_cursor_x = cursor_x;
      last_cursor_x = cursor_x;
      return;
    }

    if (code == '\n') {
      cursor_x = 0;
      bg_cursor_x = 0;
      last_cursor_x = 0;
      cursor_y += gFont.yAdvance;
      if (textwrapY && (cursor_y >= height())) cursor_y = 0;
      return;
    }
  }

  uint16_t gNum = 0;
  bool found = getUnicodeIndex(code, &gNum);

  if (found)
  {

    bool newSprite = !_created;

    if (newSprite)
    {
      sprite->createSprite(sprite, gWidth[gNum], gFont.yAdvance);
      if(fg != bg) sprite->fillSprite(sprite, bg);
      cursor_x = -gdX[gNum];
      bg_cursor_x = cursor_x;
      last_cursor_x = cursor_x;
      cursor_y = 0;
    }
    else
    {
      if( textwrapX && ((cursor_x + gWidth[gNum] + gdX[gNum]) > sprite->width(sprite))) {
        cursor_y += gFont.yAdvance;
        cursor_x = 0;
        bg_cursor_x = 0;
        last_cursor_x = 0;
      }

      if( textwrapY && ((cursor_y + gFont.yAdvance) > sprite->height(sprite))) cursor_y = 0;
      if ( cursor_x == 0) cursor_x -= gdX[gNum];
    }

    uint8_t* pbuffer = NULL;
    const uint8_t* gPtr = (const uint8_t*) gFont.gArray;

#ifdef FONT_FS_AVAILABLE
    if (fs_font) {
      fontFile.seek(gBitmap[gNum], fs::SeekSet); // This is slow for a significant position shift!
      pbuffer =  (uint8_t*)malloc(gWidth[gNum]);
    }
#endif

    int16_t cy = cursor_y + gFont.maxAscent - gdY[gNum];
    int16_t cx = cursor_x + gdX[gNum];

    //  if (cx > width() && bg_cursor_x > width()) return;
    //  if (cursor_y > height()) return;

    int16_t  fxs = cx;
    uint32_t fl = 0;
    int16_t  bxs = cx;
    uint32_t bl = 0;
    int16_t  bx = 0;
    uint8_t pixel = 0;

    int16_t fillwidth  = 0;
    int16_t fillheight = 0;

    // Fill area above glyph
    if (_fillbg) {
      fillwidth  = (cursor_x + gxAdvance[gNum]) - bg_cursor_x;
      if (fillwidth > 0) {
        fillheight = gFont.maxAscent - gdY[gNum];
        if (fillheight > 0) {
          sprite->fillRect(sprite, bg_cursor_x, cursor_y, fillwidth, fillheight, textbgcolor);
        }
      }
      else {
        // Could be negative
        fillwidth = 0;
      }

      // Fill any area to left of glyph
      if (bg_cursor_x < cx) sprite->fillRect(sprite, bg_cursor_x, cy, cx - bg_cursor_x, gHeight[gNum], textbgcolor);
      // Set x position in glyph area where background starts
      if (bg_cursor_x > cx) bx = bg_cursor_x - cx;
      // Fill any area to right of glyph
      if (cx + gWidth[gNum] < cursor_x + gxAdvance[gNum]) {
        sprite->fillRect(sprite, cx + gWidth[gNum], cy, (cursor_x + gxAdvance[gNum]) - (cx + gWidth[gNum]), gHeight[gNum], textbgcolor);
      }
    }

    for (int32_t y = 0; y < gHeight[gNum]; y++)
    {
#ifdef FONT_FS_AVAILABLE
      if (fs_font) {
        fontFile.read(pbuffer, gWidth[gNum]);
      }
#endif

      for (int32_t x = 0; x < gWidth[gNum]; x++)
      {
#ifdef FONT_FS_AVAILABLE
        if (fs_font) pixel = pbuffer[x];
        else
#endif
        pixel = pgm_read_byte(gPtr + gBitmap[gNum] + x + gWidth[gNum] * y);

        if (pixel)
        {
          if (bl) { sprite->drawFastHLine(sprite, bxs, y + cy, bl, bg); bl = 0; }
          if (pixel != 0xFF)
          {
            if (fl) {
              if (fl==1) sprite->drawPixel(sprite, fxs, y + cy, fg);
              else sprite->drawFastHLine(sprite, fxs, y + cy, fl, fg);
              fl = 0;
            }
            if (getBG) bg = sprite->readPixel(sprite, x + cx, y + cy);
            sprite->drawPixel(sprite, x + cx, y + cy, alphaBlend(pixel, fg, bg));
          }
          else
          {
            if (fl==0) fxs = x + cx;
            fl++;
          }
        }
        else
        {
          if (fl) { sprite->drawFastHLine(sprite, fxs, y + cy, fl, fg); fl = 0; }
          if (_fillbg) {
            if (x >= bx) {
              if (bl==0) bxs = x + cx;
              bl++;
            }
          }
        }
      }
      if (fl) { sprite->drawFastHLine(sprite, fxs, y + cy, fl, fg); fl = 0; }
      if (bl) { sprite->drawFastHLine(sprite, bxs, y + cy, bl, bg); bl = 0; }
    }

    // Fill area below glyph
    if (fillwidth > 0) {
      fillheight = (cursor_y + gFont.yAdvance) - (cy + gHeight[gNum]);
      if (fillheight > 0) {
        sprite->fillRect(sprite, bg_cursor_x, cy + gHeight[gNum], fillwidth, fillheight, textbgcolor);
      }
    }

    if (pbuffer) free(pbuffer);
    cursor_x += gxAdvance[gNum];

    if (newSprite)
    {
      sprite->pushSprite(, cx, cursor_y);
      sprite->deleteSprite(sprite);
    }
  }
  else
  {
    // Not a Unicode in font so draw a rectangle and move on cursor
    sprite->drawRect(sprite, cursor_x, cursor_y + gFont.maxAscent - gFont.ascent, gFont.spaceWidth, gFont.ascent, fg);
    cursor_x += gFont.spaceWidth + 1;
  }
  bg_cursor_x = cursor_x;
  last_cursor_x = cursor_x;
}


/***************************************************************************************
** Function name:           printToSprite
** Description:             Write a string to the sprite cursor position
***************************************************************************************/
static void printToSprite(struct TFT_eSprite* sprite, String string)
{
  if(!fontLoaded) return;
  sprite->printToSprite(sprite, (char*)string.c_str(), string.length());
}


/***************************************************************************************
** Function name:           printToSprite
** Description:             Write a string to the sprite cursor position
***************************************************************************************/
static void printToSprite(struct TFT_eSprite* sprite, char *cbuffer, uint16_t len) //String string)
{
  if(!fontLoaded) return;

  uint16_t n = 0;
  bool newSprite = !sprite->_created;
  int16_t  cursorX = sprite->_tft->cursor_x;

  if (newSprite)
  {
    int16_t sWidth = 0;
    uint16_t index = 0;
    bool     first = true;
    while (n < len)
    {
      uint16_t unicode = decodeUTF8((uint8_t*)cbuffer, &n, len - n);
      if (getUnicodeIndex(unicode, &index))
      {
        if (first) {
          first = false;
          sWidth -= gdX[index];
          cursorX += gdX[index];
        }
        if (n == len) sWidth += ( gWidth[index] + gdX[index]);
        else sWidth += gxAdvance[index];
      }
      else sWidth += gFont.spaceWidth + 1;
    }

    sprite->createSprite(sprite, sWidth, gFont.yAdvance);

    if (textcolor != textbgcolor) sprite->fillSprite(sprite, textbgcolor);
  }

  n = 0;

  while (n < len)
  {
    uint16_t unicode = decodeUTF8((uint8_t*)cbuffer, &n, len - n);
    //Serial.print("Decoded Unicode = 0x");Serial.println(unicode,HEX);
    //Serial.print("n = ");Serial.println(n);
    sprite->drawGlyph(sprite, unicode);
  }

  if (newSprite)
  { // The sprite had to be created so place at TFT cursor
    sprite->pushSprite(sprite, cursorX, sprite->_tft->cursor_y);
    sprite->deleteSprite(sprite);
  }
}


/***************************************************************************************
** Function name:           printToSprite
** Description:             Print character in a Sprite, create sprite if needed
***************************************************************************************/
static int16_t printToSprite(struct TFT_eSprite* sprite, int16_t x, int16_t y, uint16_t index)
{
  bool newSprite = !_created;
  int16_t sWidth = gWidth[index];

  if (newSprite)
  {
    sprite->createSprite(sprite, sWidth, gFont.yAdvance);

    if (textcolor != textbgcolor) sprite->fillSprite(sprite, textbgcolor);

    sprite->drawGlyph(sprite, gUnicode[index]);

    sprite->pushSprite(sprite, x + gdX[index], y, textbgcolor);
    sprite->deleteSprite(sprite, );
  }

  else sprite->drawGlyph(sprite, gUnicode[index]);

  return gxAdvance[index];
}
#endif
