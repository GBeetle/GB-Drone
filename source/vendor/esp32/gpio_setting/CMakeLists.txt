set(COMPONENT_SRCS "gpio_setting.c")

set(COMPONENT_REQUIRES log_sys esp_driver_gpio)
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
