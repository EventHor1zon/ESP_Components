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

    // bm_initData_t ini;
    // ini.addressPinState = 0;
    // ini.devType = 1;
    // ini.i2cChannel = 0;
    // ini.sampleMode = BM_NORMAL_MODE;
    // ini.sampleType = BM_MODE_TEMP_PRESSURE_HUMIDITY;
    
    // bm_controlData_t *handle = bm280_init(&ini);

    hpdl_initdata_t init = {0};
    init.write = 26;
    init.D0 = 15;
    init.D1 = 16;
    init.D2 = 17;
    init.D3 = 18;
    init.D4 = 25;
    init.D5 = 19;
    init.D6 = 32;
    init.A0 = 14;
    init.A1 = 27;

    char *a = "F";
    char *b = "U";
    char *c = "C";
    char *d = "K";

    uint8_t a0 = 0;
    uint8_t a1 = 1;
    uint8_t a2 = 2;
    uint8_t a3 = 3;

    hpdl_driver_t *handle = hdpl1414_init(&init);
    hpdl_set_led(handle, &a0);
    hpdl_set_char(handle, (uint8_t *)a);
    hpdl_set_led(handle, &a1);
    hpdl_set_char(handle, (uint8_t *)b);
    hpdl_set_led(handle, &a2);
    hpdl_set_char(handle, (uint8_t *)c);
    hpdl_set_led(handle, &a3);
    hpdl_set_char(handle, (uint8_t *)d);

    while (1)
    {

        
        //dump_system_info();
        vTaskDelay(1000);
    }
}
