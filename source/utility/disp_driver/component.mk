# Define sources and include dirs
COMPONENT_SRCDIRS := . disp_tft disp_touch
COMPONENT_ADD_INCLUDEDIRS := .

# TFT display drivers
COMPONENT_ADD_INCLUDEDIRS += disp_tft

$(call compile_only_if,$(CONFIG_TFT_DISPLAY_CONTROLLER_ILI9341),disp_tft/ili9341.o)
$(call compile_only_if,$(CONFIG_TFT_DISPLAY_CONTROLLER_ILI9481),disp_tft/ili9481.o)
$(call compile_only_if,$(CONFIG_TFT_DISPLAY_CONTROLLER_ILI9486),disp_tft/ili9486.o)
$(call compile_only_if,$(CONFIG_TFT_DISPLAY_CONTROLLER_ILI9488),disp_tft/ili9488.o)
$(call compile_only_if,$(CONFIG_TFT_DISPLAY_CONTROLLER_ST7789),disp_tft/st7789.o)
$(call compile_only_if,$(CONFIG_TFT_DISPLAY_CONTROLLER_ST7735S),disp_tft/st7735s.o)
$(call compile_only_if,$(CONFIG_TFT_DISPLAY_CONTROLLER_ST7796S),disp_tft/st7796s.o)
$(call compile_only_if,$(CONFIG_TFT_DISPLAY_CONTROLLER_HX8357),disp_tft/hx8357.o)
$(call compile_only_if,$(CONFIG_TFT_DISPLAY_CONTROLLER_SH1107),disp_tft/sh1107.o)
$(call compile_only_if,$(CONFIG_TFT_DISPLAY_CONTROLLER_SSD1306),disp_tft/ssd1306.o)
$(call compile_only_if,$(CONFIG_TFT_DISPLAY_CONTROLLER_FT81X),disp_tft/EVE_commands.o)
$(call compile_only_if,$(CONFIG_TFT_DISPLAY_CONTROLLER_FT81X),disp_tft/FT81x.o)
$(call compile_only_if,$(CONFIG_TFT_DISPLAY_CONTROLLER_IL3820),disp_tft/il3820.o)
$(call compile_only_if,$(CONFIG_TFT_DISPLAY_CONTROLLER_JD79653A),disp_tft/jd79653a.o)
$(call compile_only_if,$(CONFIG_TFT_DISPLAY_CONTROLLER_UC8151D),disp_tft/uc8151d.o)
$(call compile_only_if,$(CONFIG_TFT_DISPLAY_CONTROLLER_RA8875),disp_tft/ra8875.o)
$(call compile_only_if,$(CONFIG_TFT_DISPLAY_CONTROLLER_GC9A01),disp_tft/GC9A01.o)

$(call compile_only_if,$(CONFIG_TFT_DISPLAY_PROTOCOL_SPI),disp_tft/disp_spi.o)

# Touch controller drivers
COMPONENT_ADD_INCLUDEDIRS += disp_touch

$(call compile_only_if,$(CONFIG_DISP_TOUCH_CONTROLLER),disp_touch/touch_driver.o)
$(call compile_only_if,$(and $(CONFIG_DISP_TOUCH_CONTROLLER),$(CONFIG_DISP_TOUCH_CONTROLLER_XPT2046)), disp_touch/xpt2046.o)
$(call compile_only_if,$(and $(CONFIG_DISP_TOUCH_CONTROLLER),$(CONFIG_DISP_TOUCH_CONTROLLER_FT6X06)), disp_touch/ft6x36.o)
$(call compile_only_if,$(and $(CONFIG_DISP_TOUCH_CONTROLLER),$(CONFIG_DISP_TOUCH_CONTROLLER_STMPE610)), disp_touch/stmpe610.o)
$(call compile_only_if,$(and $(CONFIG_DISP_TOUCH_CONTROLLER),$(CONFIG_DISP_TOUCH_CONTROLLER_ADCRAW)), disp_touch/adcraw.o)
$(call compile_only_if,$(and $(CONFIG_DISP_TOUCH_CONTROLLER),$(CONFIG_DISP_TOUCH_CONTROLLER_FT81X)), disp_touch/FT81x.o)
$(call compile_only_if,$(and $(CONFIG_DISP_TOUCH_CONTROLLER),$(CONFIG_DISP_TOUCH_CONTROLLER_RA8875)), disp_touch/ra8875_touch.o)

$(call compile_only_if,$(and $(CONFIG_DISP_TOUCH_CONTROLLER),$(CONFIG_DISP_TOUCH_DRIVER_PROTOCOL_SPI)), disp_touch/tp_spi.o)
$(call compile_only_if,$(and $(CONFIG_DISP_TOUCH_CONTROLLER),$(CONFIG_DISP_TOUCH_DRIVER_PROTOCOL_I2C)), disp_touch/tp_i2c.o)
