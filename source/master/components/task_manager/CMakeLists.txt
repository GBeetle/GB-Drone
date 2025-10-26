set(COMPONENT_SRCS "task_manager.c")

set(COMPONENT_REQUIRES esp_driver_uart io_define isr_manager imu_driver anotic_debug Fusion gb_timer nrf24 esc max1704x)
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
