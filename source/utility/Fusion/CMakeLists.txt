set(COMPONENT_SRCS "FusionAhrs.c"
                   "FusionCompass.c"
                   "FusionOffset.c")

set(COMPONENT_REQUIRES log_sys)

set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
