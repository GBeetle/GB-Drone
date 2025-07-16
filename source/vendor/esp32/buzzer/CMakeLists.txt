set(COMPONENT_SRCS "buzzer.c")

set(COMPONENT_REQUIRES log_sys error_handle io_define driver gb_timer)
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
