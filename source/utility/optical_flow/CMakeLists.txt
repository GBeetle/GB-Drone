list(APPEND SOURCES "pmw3901.c")

set(COMPONENT_SRCS ${SOURCES})

set(COMPONENT_REQUIRES log_sys error_handle spi_bus io_define gb_timer gpio_setting)
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
