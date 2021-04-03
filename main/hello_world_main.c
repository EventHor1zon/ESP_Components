/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "../inc/main.h"
#include "../inc/WifiDriver.h"
#include "genericCommsDriver.h"
#include "SystemInterface.h"
#include "APDS9960_Driver.h"

#include "HMC5883_Driver.h"

#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"


const char *MAIN_TAG = "main";


void envSensor(void *args)
{

    // bm_initData_t initData = {0};

    // initData.addressPinState = 0;
    // initData.i2cChannel = 0;
    // initData.devType = BME_280_DEVICE;
    // initData.sampleMode = BM_FORCE_MODE;
    // initData.sampleType = BM_MODE_TEMP_PRESSURE_HUMIDITY;

    // bm_controlData_t *bmeHandle = NULL;
    // bmeHandle = bm280_init(&initData);
    // if (bmeHandle == NULL)
    // {
    //     ESP_LOGE(MAIN_TAG, "Error initialising bme");
    //     while (1)
    //     {
    //         vTaskDelay(2000);
    //     }
    // }


    while (1)
    {

        // bm280_updateMeasurements(bmeHandle);

        // printf("Humidity: %f (calib: %ul) (raw %ul) \n", bmeHandle->sensorData.realHumidity, bmeHandle->sensorData.calibratedHumidity, bmeHandle->sensorData.rawHumidity);
        // printf("Temp: %f (calib: %ul) (raw %ul) \n", bmeHandle->sensorData.realTemperature, bmeHandle->sensorData.calibratedTemperature, bmeHandle->sensorData.rawTemperature);
        // printf("Pressure: %f (calib: %ul) (raw %ul) \n", bmeHandle->sensorData.realPressure, bmeHandle->sensorData.calibratedPressure, bmeHandle->sensorData.rawPressure);

        vTaskDelay(1000);
    }
}

// void reTask(void *args)
// {

//     uint32_t notify = 0;
//     TaskHandle_t reTaskHandle = xTaskGetCurrentTaskHandle();
//     rotaryEncoderInit(gpio_num_13, gpio_num_12, true, reTaskHandle);
//     while (1)
//     {
//         ESP_LOGI(MAIN_TAG, "pong");

//         xTaskNotifyWait(0, 0, &notify, portMAX_DELAY);
//         if (notify & RE_NOTIFY_CW_STEP)
//         {
//             ESP_LOGI(MAIN_TAG, "Got clockwise step!");
//         }
//         else if (notify & RE_NOTIFY_CC_STEP)
//         {
//             ESP_LOGI(MAIN_TAG, "Got counter-clockwise step!");
//         }
//         else if (notify & RE_NOTIFY_BTN_UP)
//         {
//             ESP_LOGI(MAIN_TAG, "Got button press!");
//         }
//         else
//         {
//             ESP_LOGI(MAIN_TAG, "The value didn't match :( %ul", notify);
//         }
//         vTaskDelay(10);
//     }
// }

#include "WS2812_Driver.h"
#include "APA102_Driver.h"


void app_main(void)
{
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU cores, WiFi%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Free heap: %d\n", esp_get_free_heap_size());

    if(gcd_spi_init(5, 27, 19, SPI2_HOST, false) != ESP_OK ||
       gcd_i2c_init(18, 19, 200000, 0, false) != ESP_OK) {
        ESP_LOGE("MAIN", "Error starting comms");
        while(1) {
            ESP_LOGI("MAIN", "Chillin...");
            vTaskDelay(1000);
        }
    }

    apds_init_t ini = {0};
    ini.i2c_bus = 0;
    APDS_DEV d = apds_init(&ini);

    printf("Starting LEDSs\n");
    uint16_t numLeds = 16;
    // gpio_num_t pin = GPIO_NUM_5;
    // WS2812_init(1, &numLeds, &pin);


    apa102_init_t ini = {0};
    ini.clock_pin = 32;
    ini.data_pin = 33;
    ini.numleds = 16;
    ini.spi_bus = 2;
    ini.init_spi = 1;

    StrandData_t *handle = APA102_init(&ini);

    while (1)
    {
        ESP_LOGI("MAIN", "ping");
        vTaskDelay(2000);
    }
}
