/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "../inc/main.h"
#include "WS2812_Driver.h"
#include "BME280_Driver.h"
#include "RotaryEncoder_Driver.h"

const char *MAIN_TAG = "main";

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

    printf("\n\nSetting up a rotary encoder!\n");

    xTaskCreate(reTask, "reTask", 5012, NULL, 4, NULL);

    while (1)
    {
        ESP_LOGI(MAIN_TAG, "ping");
        vTaskDelay(2000);
    }
}