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
#include "WS2812_Driver.h"
#include "BME280_Driver.h"
#include "LSM_Driver.h"
#include "genericCommsDriver.h"
#include "SystemInterface.h"
#include "HPDL1414_Driver.h"
#include "Max30102_Driver.h"
#include "RotaryEncoder_Driver.h"
#include "MSGEQ7_Driver.h"

#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
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

void reTask(void *args)
{

    uint32_t notify = 0;
    TaskHandle_t reTaskHandle = xTaskGetCurrentTaskHandle();
    rotaryEncoderInit(GPIO_NUM_17, GPIO_NUM_16, true, reTaskHandle);

    while (1)
    {
        ESP_LOGI(MAIN_TAG, "pong");

        xTaskNotifyWait(0, 0, &notify, portMAX_DELAY);
        if (notify & RE_NOTIFY_CW_STEP)
        {
            ESP_LOGI(MAIN_TAG, "Got clockwise step!");
        }
        else if (notify & RE_NOTIFY_CC_STEP)
        {
            ESP_LOGI(MAIN_TAG, "Got counter-clockwise step!");
        }
        else if (notify & RE_NOTIFY_BTN_UP)
        {
            ESP_LOGI(MAIN_TAG, "Got button press!");
        }
        else
        {
            ESP_LOGI(MAIN_TAG, "The value didn't match :( %ul", notify);
        }
        vTaskDelay(10);
    }
}

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
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // printf("\n\nSetting up a rotary encoder!\n");

    // xTaskCreate(reTask, "reTask", 5012, NULL, 4, NULL);

    // printf("\n\nSetting up a BME 280");

    // xTaskCreate(envSensor, "envSensor", 5012, NULL, 3, NULL);


    while (1)
    {

        //dump_system_info();
        vTaskDelay(1000);
    }
}
