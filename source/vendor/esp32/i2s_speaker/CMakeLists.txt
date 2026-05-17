set(COMPONENT_SRCS "i2s_speaker.c")

set(COMPONENT_REQUIRES esp_driver_i2s log_sys error_handle io_define)
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
