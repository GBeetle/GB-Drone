set(COMPONENT_SRCS "task_manager.c")

set(COMPONENT_REQUIRES esp_timer io_define isr_manager mpu_driver anotic_debug Fusion)
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
