list(APPEND SOURCES "vl53l1x.c")

set(COMPONENT_SRCS ${SOURCES})

set(COMPONENT_REQUIRES log_sys error_handle i2c_bus io_define gb_timer gpio_setting)
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
