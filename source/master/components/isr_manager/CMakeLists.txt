set(COMPONENT_SRCS "isr_manager.c")

set(COMPONENT_REQUIRES io_define mpu_driver)
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
