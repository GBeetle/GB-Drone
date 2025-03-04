#include "st7789_defines.h"
#include "tft_espi.h"

static inline void delay(int32_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

void st7789_init(struct TFT_eSPI * tft_dev, uint8_t tc)
{
    tft_dev->writecommand(tft_dev, ST7789_SWRESET);   // SOFT reset
    delay(200);

    tft_dev->writecommand(tft_dev, ST7789_SLPOUT);   // Sleep out
    delay(200);

    //------------------------------display and color format setting--------------------------------//
    tft_dev->writecommand(tft_dev, ST7789_MADCTL);
    tft_dev->writedata(tft_dev, 0x00);

    tft_dev->writecommand(tft_dev, ST7789_COLMOD);
    tft_dev->writedata(tft_dev, 0x05);

    //--------------------------------ST7789V Frame rate setting----------------------------------//
    tft_dev->writecommand(tft_dev, ST7789_PORCTRL);
    tft_dev->writedata(tft_dev, 0x0c);
    tft_dev->writedata(tft_dev, 0x0c);
    tft_dev->writedata(tft_dev, 0x00);
    tft_dev->writedata(tft_dev, 0x33);
    tft_dev->writedata(tft_dev, 0x33);

    tft_dev->writecommand(tft_dev, ST7789_GCTRL);      // Voltages: VGH / VGL
    tft_dev->writedata(tft_dev, 0x35);

    tft_dev->writecommand(tft_dev, ST7789_VCOMS);
    tft_dev->writedata(tft_dev, 0x37);

    tft_dev->writecommand(tft_dev, ST7789_LCMCTRL);
    tft_dev->writedata(tft_dev, 0x2c);

    tft_dev->writecommand(tft_dev, ST7789_VDVVRHEN);
    tft_dev->writedata(tft_dev, 0x01);

    tft_dev->writecommand(tft_dev, ST7789_VRHS);       // voltage VRHS
    tft_dev->writedata(tft_dev, 0x12);

    tft_dev->writecommand(tft_dev, ST7789_VDVSET);
    tft_dev->writedata(tft_dev, 0x20);

    tft_dev->writecommand(tft_dev, ST7789_FRCTR2);
    tft_dev->writedata(tft_dev, 0x0f);

    tft_dev->writecommand(tft_dev, ST7789_PWCTRL1);
    tft_dev->writedata(tft_dev, 0xa4);
    tft_dev->writedata(tft_dev, 0xa1);

    //--------------------------------ST7789V gamma setting---------------------------------------//
    tft_dev->writecommand(tft_dev, ST7789_PVGAMCTRL);
    tft_dev->writedata(tft_dev, 0xd0);
    tft_dev->writedata(tft_dev, 0x04);
    tft_dev->writedata(tft_dev, 0x0d);
    tft_dev->writedata(tft_dev, 0x11);
    tft_dev->writedata(tft_dev, 0x13);
    tft_dev->writedata(tft_dev, 0x2b);
    tft_dev->writedata(tft_dev, 0x3f);
    tft_dev->writedata(tft_dev, 0x54);
    tft_dev->writedata(tft_dev, 0x4c);
    tft_dev->writedata(tft_dev, 0x18);
    tft_dev->writedata(tft_dev, 0x0d);
    tft_dev->writedata(tft_dev, 0x0b);
    tft_dev->writedata(tft_dev, 0x1f);
    tft_dev->writedata(tft_dev, 0x23);

    tft_dev->writecommand(tft_dev, ST7789_NVGAMCTRL);
    tft_dev->writedata(tft_dev, 0xd0);
    tft_dev->writedata(tft_dev, 0x04);
    tft_dev->writedata(tft_dev, 0x0c);
    tft_dev->writedata(tft_dev, 0x11);
    tft_dev->writedata(tft_dev, 0x13);
    tft_dev->writedata(tft_dev, 0x2c);
    tft_dev->writedata(tft_dev, 0x3f);
    tft_dev->writedata(tft_dev, 0x44);
    tft_dev->writedata(tft_dev, 0x51);
    tft_dev->writedata(tft_dev, 0x2f);
    tft_dev->writedata(tft_dev, 0x1f);
    tft_dev->writedata(tft_dev, 0x1f);
    tft_dev->writedata(tft_dev, 0x20);
    tft_dev->writedata(tft_dev, 0x23);

    tft_dev->writecommand(tft_dev, ST7789_INVON);
    delay(10);

    tft_dev->writecommand(tft_dev, ST7789_NORON);
    delay(10);

    tft_dev->writecommand(tft_dev, ST7789_DISPON);    //Display on
    delay(200);
}

void st7789_rotation(struct TFT_eSPI * tft_dev, uint8_t m)
{
    tft_dev->writecommand(tft_dev, TFT_MADCTL);
    tft_dev->rotation = m % 4;
    switch (tft_dev->rotation) {
    case 0: // Portrait
#ifdef CGRAM_OFFSET
        if (tft_dev->_init_width == 135)
        {
            tft_dev->colstart = 52;
            tft_dev->rowstart = 40;
        }
        else if(tft_dev->_init_height == 280)
        {
            tft_dev->colstart = 0;
            tft_dev->rowstart = 20;
        }
        else if(tft_dev->_init_width == 172)
        {
            tft_dev->colstart = 34;
            tft_dev->rowstart = 0;
        }
        else if(tft_dev->_init_width == 170)
        {
            tft_dev->colstart = 35;
            tft_dev->rowstart = 0;
        }
        else
        {
            tft_dev->colstart = 0;
            tft_dev->rowstart = 0;
        }
#endif
        tft_dev->writedata(tft_dev, TFT_MAD_COLOR_ORDER);

        tft_dev->_width  = tft_dev->_init_width;
        tft_dev->_height = tft_dev->_init_height;
        break;

    case 1: // Landscape (Portrait + 90)
#ifdef CGRAM_OFFSET
        if (tft_dev->_init_width == 135)
        {
            tft_dev->colstart = 40;
            tft_dev->rowstart = 53;
        }
        else if(tft_dev->_init_height == 280)
        {
            tft_dev->colstart = 20;
            tft_dev->rowstart = 0;
        }
        else if(tft_dev->_init_width == 172)
        {
            tft_dev->colstart = 0;
            tft_dev->rowstart = 34;
        }
        else if(tft_dev->_init_width == 170)
        {
            tft_dev->colstart = 0;
            tft_dev->rowstart = 35;
        }
        else
        {
            tft_dev->colstart = 0;
            tft_dev->rowstart = 0;
        }
#endif
        tft_dev->writedata(tft_dev, TFT_MAD_MX | TFT_MAD_MV | TFT_MAD_COLOR_ORDER);

        tft_dev->_width  = tft_dev->_init_height;
        tft_dev->_height = tft_dev->_init_width;
        break;

    case 2: // Inverter portrait
#ifdef CGRAM_OFFSET
        if (tft_dev->_init_width == 135)
        {
            tft_dev->colstart = 53;
            tft_dev->rowstart = 40;
        }
        else if(tft_dev->_init_height == 280)
        {
            tft_dev->colstart = 0;
            tft_dev->rowstart = 20;
        }
        else if(tft_dev->_init_width == 172)
        {
            tft_dev->colstart = 34;
            tft_dev->rowstart = 0;
        }
        else if(tft_dev->_init_width == 170)
        {
            tft_dev->colstart = 35;
            tft_dev->rowstart = 0;
        }
        else
        {
            tft_dev->colstart = 0;
            tft_dev->rowstart = 80;
        }
#endif
        tft_dev->writedata(tft_dev, TFT_MAD_MX | TFT_MAD_MY | TFT_MAD_COLOR_ORDER);

        tft_dev->_width  = tft_dev->_init_width;
        tft_dev->_height = tft_dev->_init_height;
        break;
    case 3: // Inverted landscape
#ifdef CGRAM_OFFSET
        if (tft_dev->_init_width == 135)
        {
            tft_dev->colstart = 40;
            tft_dev->rowstart = 52;
        }
        else if(tft_dev->_init_height == 280)
        {
            tft_dev->colstart = 20;
            tft_dev->rowstart = 0;
        }
        else if(tft_dev->_init_width == 172)
        {
            tft_dev->colstart = 0;
            tft_dev->rowstart = 34;
        }
        else if(tft_dev->_init_width == 170)
        {
            tft_dev->colstart = 0;
            tft_dev->rowstart = 35;
        }
        else
        {
            tft_dev->colstart = 80;
            tft_dev->rowstart = 0;
        }
#endif
        tft_dev->writedata(tft_dev, TFT_MAD_MV | TFT_MAD_MY | TFT_MAD_COLOR_ORDER);

        tft_dev->_width  = tft_dev->_init_height;
        tft_dev->_height = tft_dev->_init_width;
        break;
    }
}

