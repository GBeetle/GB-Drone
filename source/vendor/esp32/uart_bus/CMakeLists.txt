set(COMPONENT_SRCS "uart_bus.c")

set(COMPONENT_REQUIRES esp_driver_uart log_sys error_handle)

set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
