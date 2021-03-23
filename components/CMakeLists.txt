set(srcs

    "genericCommsDriver.c"
    "Utilities.c"
    "LSM_Driver.c"
    "SystemInterface.c"

    )

set(includes "include")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS ${includes}
                    ) 