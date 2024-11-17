set(srcs
    "BME280_Driver.c"   
)

set(includes "../../main/inc")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS ${includes}
                    REQUIRES "GenericCommsDriver" "CBuffer" "PeripheralManager"
                    )
