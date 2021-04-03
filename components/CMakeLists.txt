set(srcs
    "genericCommsDriver.c"
    "Utilities.c"
    "SystemInterface.c"
<<<<<<< HEAD
    "APDS9960_Driver.c"
=======

    "HMC5883_Driver.c"
>>>>>>> HMC5883
    )

set(includes "include")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS ${includes}
                    ) 