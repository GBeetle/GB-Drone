file(GLOB_RECURSE SOURCES
    "vl53l1x/API/core/*.c"
    "vl53l1x/API/platform/*.c"
)

list(APPEND SOURCES "laser_ranging.c")

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS
        "include"
        "vl53l1x/API/core/inc"
        "vl53l1x/API/platform/inc"
    REQUIRES
        log_sys error_handle i2c_bus gb_timer io_define gpio_setting
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -mlongcalls
)

target_compile_definitions(${COMPONENT_LIB} PRIVATE
    #VL53L1_LOG_ENABLE
)
