set(COMPONENT_SRCS "task_manager.c")

set(COMPONENT_REQUIRES io_define isr_manager mpu_driver anotic_debug Fusion gb_timer)
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
