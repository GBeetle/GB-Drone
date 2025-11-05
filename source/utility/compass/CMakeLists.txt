if(CONFIG_QMC5883L)
    list(APPEND SOURCES "qmc5883l.c")
elseif(CONFIG_LIS3MDL)
    list(APPEND SOURCES "lis3mdl.c")
else()
    message(WARNING "Compass drivers not defined.")
endif()

set(COMPONENT_SRCS ${SOURCES})

set(COMPONENT_REQUIRES log_sys error_handle spi_bus i2c_bus io_define gb_timer)

set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
