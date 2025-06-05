set(COMPONENT_SRCS "quad_3d.c" "controller.c" "task_manager.c" "lvgl_task.c")

set(COMPONENT_REQUIRES io_define gb_timer tinygl lvgl lv_examples disp_driver tft_driver nrf24)
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
