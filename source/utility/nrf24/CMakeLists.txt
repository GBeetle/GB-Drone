set(COMPONENT_SRCS "nrf24_interface.c")

set(COMPONENT_ADD_INCLUDEDIRS "include")

set(COMPONENT_REQUIRES spi_bus log_sys error_handle io_define gpio_setting gb_timer)

register_component()
