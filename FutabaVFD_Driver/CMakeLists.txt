set(srcs
    "FutabaVFD_Driver.c"
)


idf_component_register(SRCS "${srcs}"
                    REQUIRES "driver" "GenericCommsDriver" "Utils" "PeripheralManager"
                    )
