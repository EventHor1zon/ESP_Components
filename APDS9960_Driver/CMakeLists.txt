set(srcs
    "APDS9960_Driver.c"
)

set(includes ".")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS ${includes}
                    REQUIRES "driver" "GenericCommsDriver" "Utils" "PeripheralManager"
        )
