if(CONFIG_QMC5883L)
    list(APPEND SOURCES)
elseif(CONFIG_LIS3MDL)
    list(APPEND SOURCES "lis3mdl.c")
else()
    message(WARNING "Compass drivers not defined.")
endif()

set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
