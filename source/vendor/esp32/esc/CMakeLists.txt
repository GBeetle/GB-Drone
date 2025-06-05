set(COMPONENT_SRCS "esc_system.c")

set(COMPONENT_REQUIRES log_sys error_handle io_define driver gb_timer)
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
