set(COMPONENT_SRCS "i2c_bus.c")

set(COMPONENT_REQUIRES esp_driver_i2c log_sys error_handle)

set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
