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
#include "DS2321_Driver.h"

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

    // esp_err_t ret = nvs_flash_init();
    // if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    // {
    //     ESP_ERROR_CHECK(nvs_flash_erase());
    //     ret = nvs_flash_init();
    // }
    // ESP_ERROR_CHECK(ret);

    // ESP_LOGI("MAIN", "ESP_WIFI_MODE_STA");
    //wifi_init_sta();


    if(gcd_spi_init(5, 27, 19, SPI2_HOST, false) != ESP_OK ||
       gcd_i2c_init(18, 19, 200000, 0, false) != ESP_OK) {
        ESP_LOGE("MAIN", "Error starting comms");
        while(1) {
            ESP_LOGI("MAIN", "Chillin...");
            vTaskDelay(1000);
        }
    }

    ds2321_init_t i = {0};
    i.i2c_bus = 0;

    uint8_t t_m = 2;
    uint8_t t_h = 21;
    uint8_t t_d = 22;
    uint8_t t_mnt = 3;
    uint8_t t_y = 21;
    DS2321_DEV ds = ds2321_init(&i);
    ds2321_set_minutes(ds, &t_m);
    ds2321_set_hours(ds, &t_h);
    ds2321_set_date(ds, &t_d);
    ds2321_set_month(ds, &t_mnt);
    ds2321_set_year(ds, &t_y);


    while (1)
    {
        //dump_system_info();
        vTaskDelay(1000);
    }
}
