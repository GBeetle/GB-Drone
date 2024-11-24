set(COMPONENT_SRCS "mpu_driver.c"
                   "mpu_math.c"
                  )
set(COMPONENT_REQUIRES spi_bus i2c_bus log_sys error_handle io_define lis3mdl bmp280)

set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
