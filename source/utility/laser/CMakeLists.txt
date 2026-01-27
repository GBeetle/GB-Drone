file(GLOB_RECURSE SOURCES vl53l1x/API/core/*.c vl53l1x/API/platform/*.c)
list(APPEND SOURCES "laser_ranging.c")

set(COMPONENT_SRCS ${SOURCES})

set(COMPONENT_REQUIRES log_sys error_handle i2c_bus gb_timer io_define gpio_setting)
set(COMPONENT_ADD_INCLUDEDIRS
    "include"
    "vl53l1x/API/core/inc"
    "vl53l1x/API/platform/inc")

register_component()
