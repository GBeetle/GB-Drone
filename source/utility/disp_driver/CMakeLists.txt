if(ESP_PLATFORM)

file(GLOB SOURCES *.c)
set(DISP_INCLUDE_DIRS . disp_tft)
list(APPEND SOURCES "disp_driver.c")

if(CONFIG_TFT_DISPLAY_PROTOCOL_SPI)
    list(APPEND SOURCES "disp_spi.c")
endif()

#@todo add SimleInclude macro here

# Include only the source file of the selected
# display controller.
if(CONFIG_TFT_DISPLAY_CONTROLLER_ILI9341)
    list(APPEND SOURCES "disp_tft/ili9341.c")
elseif(CONFIG_TFT_DISPLAY_CONTROLLER_ILI9481)
    list(APPEND SOURCES "disp_tft/ili9481.c")
elseif(CONFIG_TFT_DISPLAY_CONTROLLER_ILI9486)
    list(APPEND SOURCES "disp_tft/ili9486.c")
elseif(CONFIG_TFT_DISPLAY_CONTROLLER_ILI9488)
    list(APPEND SOURCES "disp_tft/ili9488.c")
elseif(CONFIG_TFT_DISPLAY_CONTROLLER_ST7789)
    list(APPEND SOURCES "disp_tft/st7789.c")
elseif(CONFIG_TFT_DISPLAY_CONTROLLER_ST7735S)
    list(APPEND SOURCES "disp_tft/st7735s.c")
elseif(CONFIG_TFT_DISPLAY_CONTROLLER_ST7796S)
    list(APPEND SOURCES "disp_tft/st7796s.c")
elseif(CONFIG_TFT_DISPLAY_CONTROLLER_HX8357)
    list(APPEND SOURCES "disp_tft/hx8357.c")
elseif(CONFIG_TFT_DISPLAY_CONTROLLER_SH1107)
    list(APPEND SOURCES "disp_tft/sh1107.c")
elseif(CONFIG_TFT_DISPLAY_CONTROLLER_SSD1306)
    list(APPEND SOURCES "disp_tft/ssd1306.c")
elseif(CONFIG_TFT_DISPLAY_CONTROLLER_FT81X)
    list(APPEND SOURCES "disp_tft/EVE_commands.c")
    list(APPEND SOURCES "disp_tft/FT81x.c")
elseif(CONFIG_TFT_DISPLAY_CONTROLLER_IL3820)
    list(APPEND SOURCES "disp_tft/il3820.c")
elseif(CONFIG_TFT_DISPLAY_CONTROLLER_JD79653A)
    list(APPEND SOURCES "disp_tft/jd79653a.c")
elseif(CONFIG_TFT_DISPLAY_CONTROLLER_UC8151D)
    list(APPEND SOURCES "disp_tft/uc8151d.c")
elseif(CONFIG_TFT_DISPLAY_CONTROLLER_RA8875)
    list(APPEND SOURCES "disp_tft/ra8875.c")
elseif(CONFIG_TFT_DISPLAY_CONTROLLER_GC9A01)
    list(APPEND SOURCES "disp_tft/GC9A01.c")
else()
    message(WARNING "Disp drivers: Display controller not defined.")
endif()

# Add touch driver to compilation only if it is selected in menuconfig
if(CONFIG_DISP_TOUCH_CONTROLLER)
    list(APPEND SOURCES "disp_touch/touch_driver.c")
    list(APPEND DISP_INCLUDE_DIRS disp_touch)

    # Include only the source file of the selected
    # touch controller.
    if(CONFIG_DISP_TOUCH_CONTROLLER_XPT2046)
        list(APPEND SOURCES "disp_touch/xpt2046.c")
    elseif(CONFIG_DISP_TOUCH_CONTROLLER_FT6X06)
        list(APPEND SOURCES "disp_touch/ft6x36.c")
    elseif(CONFIG_DISP_TOUCH_CONTROLLER_STMPE610)
        list(APPEND SOURCES "disp_touch/stmpe610.c")
    elseif(CONFIG_DISP_TOUCH_CONTROLLER_ADCRAW)
        list(APPEND SOURCES "disp_touch/adcraw.c")
    elseif(CONFIG_DISP_TOUCH_CONTROLLER_FT81X)
        list(APPEND SOURCES "disp_touch/FT81x.c")
    elseif(CONFIG_DISP_TOUCH_CONTROLLER_RA8875)
        list(APPEND SOURCES "disp_touch/ra8875_touch.c")
    endif()

    if(CONFIG_DISP_TOUCH_DRIVER_PROTOCOL_SPI)
        list(APPEND SOURCES "disp_touch/tp_spi.c")
    elseif(CONFIG_DISP_TOUCH_DRIVER_PROTOCOL_I2C)
        list(APPEND SOURCES "disp_touch/tp_i2c.c")
    endif()
endif()

idf_component_register(SRCS ${SOURCES}
                       INCLUDE_DIRS ${DISP_INCLUDE_DIRS}
                       REQUIRES log_sys error_handle io_define spi_bus i2c_bus gpio_setting gb_timer)

else()
    message(FATAL_ERROR "Disp drivers: ESP_PLATFORM is not defined. Try reinstalling ESP-IDF.")
endif()
