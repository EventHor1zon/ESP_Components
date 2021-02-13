set(srcs
    "BME280_Driver.c"
    "WS2812_Driver.c"
    "RotaryEncoder_Driver.c"
    "PushButton_Driver.c"
    "MSGEQ7_Driver.c"
    )

set(includes "include")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS ${includes}
                    ) 