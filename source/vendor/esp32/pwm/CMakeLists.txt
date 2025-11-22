set(COMPONENT_SRCS "pwm.c")

set(COMPONENT_REQUIRES esp_driver_ledc log_sys error_handle)
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
