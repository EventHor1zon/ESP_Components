set(srcs
    "BME280_Driver.c"
    "WS2812_Driver.c"
    "LedEffects.c"
    "RotaryEncoder_Driver.c"
    "PushButton_Driver.c"
    "APA102_Driver.c"
    )

set(includes "include")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS ${includes}
                    ) 