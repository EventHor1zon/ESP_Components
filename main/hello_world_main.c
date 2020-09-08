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
#include "WS2812_Driver.h"
#include "BME280_Driver.h"
#include "LSM_Driver.h"
#include "genericCommsDriver.h"

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

    // printf("Starting LEDSs\n");
    // uint16_t numLeds = 6;
    // gpio_num_t pin = GPIO_NUM_5;
    // WS2812_init(1, &numLeds, &pin);

    // uint8_t whoami = 0;
    // esp_err_t status = genericI2CReadFromAddress(0, LSM_I2C_ADDR, LSM_WHOAMI_REG, 1, &whoami);

    // printf("Read from device whoami = %08x", whoami);
    uint8_t i2cChannel = 1;
    genericI2Cinit(17, 16, 200000, i2cChannel);
    bm_initData_t id = {0};
    id.addressPinState = 0;
    id.devType = BME_280_DEVICE;
    id.i2cChannel = i2cChannel;
    id.sampleMode = BM_FORCE_MODE;
    id.sampleType = BM_MODE_TEMP_PRESSURE_HUMIDITY;

    bm_controlData_t *bmHandle = bm280_init(&id);
    uint8_t whoami = 0;
    uint8_t cal = 0;
    uint8_t set = 0;
    uint8_t other = 0;
    while (1)
    {

        // printf("Whoami = %02x\r\n", whoami);
        vTaskDelay(1000);
        // esp_err_t st = bm280_getDeviceID(bmHandle, &whoami);
        // printf("st = %02x\r\n", st);
        // genericI2CReadFromAddress(1, 0x76, 0xD0, 1, &whoami);
        // ESP_LOGI("MAIN_TAG", "ping");
        // vTaskDelay(2000);
        // bm280_updateMeasurements(bmHandle);
        // float temp = 0, pressure = 0, hum = 0;
        // bm280_getTemperature(bmHandle, &temp);
        // bm280_getPressure(bmHandle, &pressure);
        // bm280_getHumidity(bmHandle, &hum);
        // printf("Temp:\t\t%f", temp);
        // printf("Pressure: \t\t%f", pressure);
        // printf("Humidity:\t\t %f", hum);
    }
}
