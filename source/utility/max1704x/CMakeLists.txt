set(COMPONENT_SRCS "max1704x.c")

set(COMPONENT_REQUIRES i2c_bus log_sys error_handle io_define)
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
