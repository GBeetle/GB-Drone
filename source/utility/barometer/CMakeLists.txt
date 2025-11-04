if(CONFIG_MS5611)
    list(APPEND SOURCES "ms5611.c")
elseif(CONFIG_BMP280)
    list(APPEND SOURCES "bmp280.c")
else()
    message(WARNING "Barometer drivers not defined.")
endif()
list(APPEND SOURCES "baro_math.c")

set(COMPONENT_SRCS ${SOURCES})

set(COMPONENT_REQUIRES log_sys error_handle spi_bus i2c_bus io_define gb_timer)
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
