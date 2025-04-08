set(COMPONENT_SRCS "tft_espi.c"
                   "tft_sprite.c")

set(COMPONENT_REQUIRES wpa_supplicant spi_bus error_handle io_define log_sys disp_driver gpio_setting)
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
