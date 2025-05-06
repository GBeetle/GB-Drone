if(ESP_PLATFORM)

file(GLOB_RECURSE SOURCES lv_examples/*.c)
set(LVGL_EXP . include)

idf_component_register(SRCS ${SOURCES}
                       INCLUDE_DIRS ${LVGL_EXP}
                       REQUIRES lvgl)

else()
    message(FATAL_ERROR "LVGL LV examples: ESP_PLATFORM is not defined. Try reinstalling ESP-IDF.")
endif()
