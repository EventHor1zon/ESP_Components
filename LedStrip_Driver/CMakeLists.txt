set(srcs
    "LedStrip_Driver.c"
    "LedEffects.c"
)

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "inc/"
                    REQUIRES "PeripheralManager" "driver" "Utils"
)
