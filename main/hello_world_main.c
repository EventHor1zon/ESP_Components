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

#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"


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

    // esp_err_t ret = nvs_flash_init();
    // if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    // {
    //     ESP_ERROR_CHECK(nvs_flash_erase());
    //     ret = nvs_flash_init();
    // }
    // ESP_ERROR_CHECK(ret);

    // ESP_LOGI("MAIN", "ESP_WIFI_MODE_STA");
    //wifi_init_sta();

    // gcd_i2c_init(17, 16, 100000, 0);

    if(gcd_spi_init(5, 27, 19, SPI2_HOST) != ESP_OK ||
       gcd_i2c_init(4, 15, 200000, 0, false) != ESP_OK) {
        ESP_LOGE("MAIN", "Error starting comms");
        while(1) {
            ESP_LOGI("MAIN", "Chillin...");
            vTaskDelay(1000);
        }
    }

    


    while (1)
    {
        //dump_system_info();
        vTaskDelay(1000);
    }
}
