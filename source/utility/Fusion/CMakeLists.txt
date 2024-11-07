set(component_srcs "Fusion/FusionAhrs.c"
                   "Fusion/FusionCompass.c"
                   "Fusion/FusionOffset.c")

#set(COMPONENT_REQUIRES driver)

set(COMPONENT_ADD_INCLUDEDIRS "Fusion")

register_component()
