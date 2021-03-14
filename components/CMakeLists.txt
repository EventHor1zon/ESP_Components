set(srcs
    "BME280_Driver.c"
    "WS2812_Driver.c"
    "LedEffects.c"
    "RotaryEncoder_Driver.c"
    "PushButton_Driver.c"
    "genericCommsDriver.c"
    "Utilities.c"
    "LSM_Driver.c"
    "SystemInterface.c"
    "HPDL1414_Driver.c"
    "Max30102_Driver.c"
    "Lora_SX1276_Driver.c"
    "MSGEQ7_Driver.c"
    "vl53l0x_api.c"
    "vl53l0x_api_calibration.c"
    "vl53l0x_api_core.c"
    "vl53l0x_api_ranging.c"
    "vl53l0x_api_strings.c"
    "vl53l0x_platform.c"
    "vl53l0x_platform_i2c.c"
    "vl53l0x_platform_log.c"
    "VL53L0X_Driver.c"
    )

set(includes "include")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS ${includes}
                    ) 