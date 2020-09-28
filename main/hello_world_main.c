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

    LSM_initData_t init = {0};
    init.int1Pin = GPIO_NUM_14;
    init.int2Pin = GPIO_NUM_12;
    init.accelRate = LSM_ACCODR_104_HZ;
    init.gyroRate = LSM_GYRO_ODR_104_HZ;
    init.commMode = LSM_DEVICE_COMM_MODE_I2C;
    init.commsChannel = 1;
    init.opMode = LSM_OPMODE_GYRO_ACCEL;
    init.assignFifoBuffer = 1;
    init.addrPinState = 0;

    LSM_DriverSettings_t *handle = LSM_init(&init);

    uint8_t who = 0, blank = 3, a = 0;
    LSM_getWhoAmI(handle, &who);

    while (1)
    {
        vTaskDelay(100);
    }
}
