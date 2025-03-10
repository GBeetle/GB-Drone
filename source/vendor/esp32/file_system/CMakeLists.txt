set(COMPONENT_SRCS "file_system.c")

set(COMPONENT_REQUIRES spi_flash log_sys io_define spi_flash fatfs error_handle)
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
