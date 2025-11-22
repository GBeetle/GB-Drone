#file(GLOB SOURCES *.c)

if(CONFIG_MPU6050)
    list(APPEND SOURCES "mpu_driver.c")
elseif(CONFIG_MPU6500)
    list(APPEND SOURCES "mpu_driver.c")
elseif(CONFIG_ICM42688)
    list(APPEND SOURCES "imu_driver.c")
else()
    message(WARNING "IMU drivers not defined.")
endif()

list(APPEND SOURCES "filter.c")
set(COMPONENT_SRCS ${SOURCES})

set(COMPONENT_REQUIRES spi_bus i2c_bus log_sys error_handle io_define compass barometer isr_manager gb_timer gpio_setting pwm)

set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
