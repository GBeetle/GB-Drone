set(COMPONENT_SRCS "log_sys.c")

set(COMPONENT_REQUIRES esp_driver_uart esp_tinyusb)
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
