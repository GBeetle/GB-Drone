set(COMPONENT_SRCS "i2c_bus.c")

set(COMPONENT_REQUIRES driver log_sys error_handle)

set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
