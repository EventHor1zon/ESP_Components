set(srcs
    "genericCommsDriver.c"
    "Utilities.c"
    "SystemInterface.c"
    "DS2321_Driver.c"
    )

set(includes "include")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS ${includes}
                    ) 