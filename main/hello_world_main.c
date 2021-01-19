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

#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

void app_main(void)
{
    printf("Hello world!\n");

    // /* Print chip information */
    // esp_chip_info_t chip_info;
    // esp_chip_info(&chip_info);
    // printf("This is %s chip with %d CPU cores, WiFi%s%s, ",
    //        CONFIG_IDF_TARGET,
    //        chip_info.cores,
    //        (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
    //        (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    // printf("silicon revision %d, ", chip_info.revision);

    // printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
    //        (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    // printf("Free heap: %d\n", esp_get_free_heap_size());

    // esp_err_t ret = nvs_flash_init();
    // if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    // {
    //     ESP_ERROR_CHECK(nvs_flash_erase());
    //     ret = nvs_flash_init();
    // }
    // ESP_ERROR_CHECK(ret);

    // ESP_LOGI("MAIN", "ESP_WIFI_MODE_STA");
    //wifi_init_sta();

    // genericI2Cinit(17, 16, 100000, 0);

    printf("Starting LEDSs\n");
    uint16_t numLeds = 6;
    gpio_num_t pin = GPIO_NUM_16;
    WS2812_init(1, &numLeds, &pin);

    while (1)
    {

        
        //dump_system_info();
        vTaskDelay(1000);
    }
}
