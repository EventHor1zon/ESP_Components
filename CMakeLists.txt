
set(CAM_PRIV_REQUIRES freertos nvs_flash)

set(CAM_SRCS
  OV2640_Driver/conversions/yuv.c
  OV2640_Driver/conversions/to_jpg.cpp
  OV2640_Driver/conversions/to_bmp.c
  OV2640_Driver/conversions/jpge.cpp
  OV2640_Driver/conversions/esp_jpg_decode.c
)

set(CAM_PRIV_INCLUDES
  OV2640_Driver/conversions/private_include
)

set(CAM_ADD_INCLUDES
  OV2640_Driver/driver/include
  OV2640_Driver/conversions/include
)

list(APPEND CAM_SRCS
  OV2640_Driver/driver/esp_camera.c
  OV2640_Driver/driver/cam_hal.c 
  OV2640_Driver/driver/sccb.c
  OV2640_Driver/sensors/ov2640.c
  )

list(APPEND CAM_PRIV_INCLUDES
  OV2640_Driver/driver/private_include
  OV2640_Driver/sensors/private_include
  OV2640_Driver/target/private_include
)

list(APPEND CAM_SRCS
  OV2640_Driver/target/xclk.c
  #OV2640_Driver/target/esp32s2/ll_cam.c
  OV2640_Driver/target/tjpgd.c 
)

list(APPEND CAM_PRIV_INCLUDES
  OV2640_Driver/target/esp32s2/private_include
)


set(min_version_for_esp_timer "4.2")
if (idf_version VERSION_GREATER_EQUAL min_version_for_esp_timer)
  list(APPEND COMPONENT_PRIV_REQUIRES esp_timer)
endif()


set(srcs
     "Utils/Utilities.c"
    # "LedEffects.c"
    # "SystemComponent.c"
    # "APA102_Driver/APA102_Driver.c"
    # "APDS9960_Driver.c"
    # "BME280_Driver.c"
    # "CCS811_Driver.c"
    # "DS2321_Driver.c"
    # "HMC5883_Driver.c"
    # "HPDL1414_Driver.c"
    # "LedStrip_Driver/LedStrip_Driver.c"
    # "LoraSX1276_Driver/Lora_SX1276_Driver.c"
    # "LSM_Driver/LSM_Driver.c"
    # "Max30102_Driver/Max30102_Driver.c"
    # "MSGEQ7_Driver/MSGEQ7_Driver.c"
    # "Pushbutton_Driver/PushButton_Driver.c"
    # "RotaryEncoder_Driver.c"
    # "SSD1306_Driver/SSD1306_Driver.c"
    # "ST7735_Screen.c"
    # "VL53L0X_Driver.c"
    # "WS2812B_Driver/WS2812_Driver.c"
    # "A4988_Driver/A4988_Driver.c"
    # "VEML6070_Driver.c"
    # "RGB_Driver/RGB_Driver.c"
    # "CBuffer/CircularBuffer.c"
    # "FutabaVFD_Driver/FutabaVFD_Driver.c"
    # "MFRC522_Driver/MFRC522_Driver.c"
    # "TestComponent/TestComponent.c"
    # "LedStrip_Driver/LedEffects.c"
    "OV2640_Driver/OV2640_Driver.c"
)

set(includes "../main/inc"
             "/home/rich/Tools/ToolChains/ESP/esp-idf/components/esp_http_server/include"
             "/home/rich/Tools/ToolChains/ESP/esp-idf/components/nghttp/port/include"
             "/home/rich/Tools/ToolChains/ESP/esp-idf/components/nvs_flash/include"
             "/home/rich/Tools/ToolChains/ESP/esp-idf/components/spi_flash/include"
             "/home/rich/Tools/ToolChains/ESP/esp-idf/components/soc/soc/esp32s2/include"
)

idf_component_register(SRCS "${srcs}" "${CAM_SRCS}" 
  INCLUDE_DIRS ${includes} ${CAM_ADD_INCLUDES} ${CAM_PRIV_INCLUDES}
)
