/******************************************************/ /**
 *  @file main.c
 *  
 *  @brief main application file for ESPHome
 * 
 *********************************************************/

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "genericCommsDriver.h"

#include "main.h"
#include "esp_log.h"
#include "WifiDriver.h"
#include "device_config.h"
#include "PeripheralManager.h"
#include "Utilities.h"
#include "LSM_Driver.h"
#include "APDS9960_Driver.h"
#include "driver/uart.h"
#include <time.h>

/************************************************
 * Private Functions 
 ************************************************/

void app_main(void)
{

        //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);


#ifdef CONFIG_USE_PERIPH_MANAGER
    ESP_LOGI("MAIN", "Config use periph manager is enabled");

    pm_init_t pm_init = {0};
    pm_init.spi_channel = VSPI_HOST;
    pm_init.init_gpio_isr = true;
    pm_init.init_i2c = true;
    pm_init.init_spi = false;
    pm_init.mosi = 12;
    pm_init.miso = -1;
    pm_init.sck = 13;
    pm_init.i2c_speed = 200000;
    pm_init.i2c_channel = 1;
    pm_init.scl = 18;
    pm_init.sda = 23;
    pm_init.init_uart = false;
    pm_init.uart_channel = UART_NUM_1;
    pm_init.uart_rx = 25;
    pm_init.uart_tx = 26;
    pm_init.uart_cts = -1;
    pm_init.uart_rts = -1;
    pm_init.uart_baud = 115200;

    ret = peripheral_manager_init(&pm_init);

    if(ret != ESP_OK) {
        ESP_LOGE("MAIN", "Error starting PM");
    }
#endif


    apds_init_t apds_init_data = {
        .i2c_bus = 1,
        .intr_pin = 19,
    };

    APDS_DEV apds = apds_init(&apds_init_data);

    /** set the prx en & als en **/

    uint8_t byte = 0; 
    uint16_t val = 0;
    uint8_t  prx = 0;
    uint8_t crgbraw[8] = {0};


    /** set the AGAIN to 64* **/
    byte = APDS_PRX_GAIN_64;
    apds_set_als_gain(apds, &byte);

    /** set the wait time to ~100ms **/
    byte = 200;
    apds_set_wait_time(apds, &byte);
    
    /** set the als persistence to some reasonable value **/
    byte = 5;
    apds_set_als_intr_persistence(apds, &byte);

    /** set the als high and low threshold to pretty wide values
     *  this way round triggers with unable to read
     *  switch them for measurements of interest!
     *  **/
    val = 400;
    apds_set_alsintr_low_thr(apds, &val);

    val = 30000;
    apds_set_alsintr_hi_thr(apds, &val);

    byte = 220;
    apds_set_adc_time(apds, &byte);

    /** set the als enabled, power on bit, wait enable and als interrupt enable **/
    byte = (APDS_REGBIT_ALS_EN | APDS_REGBIT_PWR_ON | APDS_REGBIT_WAIT_EN | APDS_REGBIT_ALS_INT_EN);
    byte = 1;
    apds_set_als_status(apds, &apds);


    // if(ret == ESP_OK) {

    //     wifi_init_sta();

    // }

    while (1)
    {

        //ESP_ERROR_CHECK(gcd_i2c_read_address(apds->bus, apds->addr, APDS_REGADDR_PROX_DATA, 1, &prx));
        ESP_ERROR_CHECK(gcd_i2c_read_address(apds->bus, apds->addr, APDS_REGADDR_CLRCHAN_DATA_LSB, 8, crgbraw));
        uint16_t c = (crgbraw[0] | crgbraw[1] << 8);
        uint16_t r = (crgbraw[2] | crgbraw[3] << 8);
        uint16_t g = (crgbraw[4] | crgbraw[5] << 8);
        uint16_t b = (crgbraw[6] | crgbraw[7] << 8);

        ESP_LOGI("MAIN", "Results: PRX: %u  C: %u R: %u G: %u B: %u ", prx, c, r, g, b);
        vTaskDelay(pdMS_TO_TICKS(100));

    }
    /* here be dragons */
}
