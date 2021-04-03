set(srcs
<<<<<<< HEAD
    "genericCommsDriver.c"
    "Utilities.c"
    "SystemInterface.c"
<<<<<<< HEAD
<<<<<<< HEAD
    "APDS9960_Driver.c"
=======

    "HMC5883_Driver.c"
>>>>>>> HMC5883
=======
    "BME280_Driver.c"
    "WS2812_Driver.c"
    "LedEffects.c"
    "RotaryEncoder_Driver.c"
    "PushButton_Driver.c"
    "APA102_Driver.c"
    "Utilities.c"
>>>>>>> apa102Driver
=======
    "DS2321_Driver.c"
>>>>>>> ds3231
    )

set(includes "include")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS ${includes}
                    ) 