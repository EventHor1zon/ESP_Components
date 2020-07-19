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
#include "esp_spi_flash.h"
#include "../inc/main.h"
#include "../inc/WS2812_Driver.h"
#include "../inc/BME280_Driver.h"

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

    printf("\n\n[BME_DRIVER:] Initialising BME or BMP - we'll soon see...\n");

    bm_initData_t initData = {0};
    initData.sampleMode = BM_FORCE_MODE;
    initData.sampleType = BM_MODE_TEMP;
    initData.addressPinState = 0;
    initData.i2cChannel = 0;

    bm_controlData_t *handle = bm280_init(&initData);

    float temp = 0;
    esp_err_t status = ESP_OK;
    status = bm280_updateMeasurements(handle);
    if (status)
    {
        printf("Err: status %u", status);
    }
    status = bm280_getTemperature(handle, &temp);

    printf("Getting the temperature...\n");

    if (status == ESP_OK)
    {
        printf("Got temp! %f\n", temp);
    }
    else
    {
        printf("Aww... an error... %u", status);
    }

    for (int i = 10; i >= 0; i--)
    {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}
