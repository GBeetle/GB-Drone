set(COMPONENT_SRCS "gps_driver.c")

set(COMPONENT_REQUIRES log_sys error_handle uart_bus io_define gb_timer minmea)
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
