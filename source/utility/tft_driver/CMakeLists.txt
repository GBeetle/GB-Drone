set(COMPONENT_SRCS "st7735.c" "tft_espi.c"
                   "tft_sprite.c"
                   "st7789.c")

set(COMPONENT_REQUIRES wpa_supplicant spi_bus error_handle io_define log_sys)
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
