if(ESP_PLATFORM)

file(GLOB SOURCES src/*.c)
set(TINYGL_INCLUDE_DIRS src include examples)
#list(APPEND SOURCES "disp_driver.c")

idf_component_register(SRCS ${SOURCES}
                       INCLUDE_DIRS ${TINYGL_INCLUDE_DIRS}
                       REQUIRES log_sys)

else()
    message(FATAL_ERROR "tinygl: ESP_PLATFORM is not defined. Try reinstalling ESP-IDF.")
endif()
