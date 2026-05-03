set(COMPONENT_SRCS "spi_bus.c")

set(COMPONENT_REQUIRES esp_driver_spi log_sys error_handle)
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
