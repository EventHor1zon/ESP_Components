set(srcs
    "genericCommsDriver.c"
    "Utilities.c"
    # "LedEffects.c"
    # "SystemComponent.c"
    # "APA102_Driver.c"
    # "APDS9960_Driver.c"
    # "BME280_Driver.c"
    # "CCS811_Driver.c"
    # "DS2321_Driver.c"
    # "HMC5883_Driver.c"
    # "HPDL1414_Driver.c"
    # "Lora_SX1276_Driver.c"
    # "LSM_Driver.c"
    # "Max30102_Driver.c"
    # "MSGEQ7_Driver.c"
    # "PushButton_Driver.c"
    # "RotaryEncoder_Driver.c"
    # "SSD1306_Driver.c"
    # "ST7735_Screen.c"
    # "VL53L0X_Driver.c"
    # "WS2812_Driver.c"
    "A4988_Driver.c"
    )

set(includes "include")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS ${includes}
                    )