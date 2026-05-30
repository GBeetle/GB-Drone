set(COMPONENT_SRCS "quad_3d.c" "controller.c" "task_manager.c" "lvgl_task.c" "dashboard_widgets.c")

set(COMPONENT_REQUIRES io_define gb_timer tinygl lvgl disp_driver tft_driver nrf24 esp_adc file_system max1704x)
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
