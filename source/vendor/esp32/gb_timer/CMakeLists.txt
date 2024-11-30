set(COMPONENT_SRCS "gb_timer.c")

set(COMPONENT_REQUIRES esp_timer log_sys)
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
