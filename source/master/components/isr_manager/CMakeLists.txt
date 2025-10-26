set(COMPONENT_SRCS "isr_manager.c")

set(COMPONENT_REQUIRES esp_rom io_define imu_driver log_sys)
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
