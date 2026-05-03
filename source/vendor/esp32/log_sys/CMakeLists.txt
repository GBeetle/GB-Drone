set(COMPONENT_SRCS "log_sys.c")

set(COMPONENT_REQUIRES esp_driver_uart esp_tinyusb esp_driver_usb_serial_jtag)

set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
