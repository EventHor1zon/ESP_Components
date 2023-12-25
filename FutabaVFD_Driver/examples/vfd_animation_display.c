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
// #include "../components/TestComponent/TestComponent.h"
// #include "../components/SSD1306_Driver/SSD1306_Driver.h"
// #include "../components/LedStrip_Driver/LedStrip_Driver.h"
// #include "../components/LedStrip_Driver/LedEffects.h"
// #include "../components/Max30102_Driver/Max30102_Driver.h"
#include "../components/FutabaVFD_Driver/FutabaVFD_Driver.h"
#include "main.h"
#include "esp_log.h"
// #include "WifiDriver.h"
#include "PeripheralManager.h"
#include "Utilities.h"


void app_main(void)
{

        //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }

    // wifi_init_sta();

    ESP_ERROR_CHECK(ret);


#ifdef CONFIG_USE_PERIPH_MANAGER
    ESP_LOGI("MAIN", "Config use periph manager is enabled");

    pm_init_t pm_init = {0};
    pm_init.spi_channel = VSPI_HOST;
    pm_init.init_gpio_isr = true;
    pm_init.init_i2c = false;
    pm_init.init_spi = true;
    pm_init.mosi = 15;
    pm_init.miso = -1;
    pm_init.sck = 13;
    pm_init.i2c_speed = 200000;
    pm_init.i2c_channel = 1;
    pm_init.scl = 22;
    pm_init.sda = 21;
    pm_init.init_uart = false;
    pm_init.uart_channel = 0;
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

    vfd_init_t init = {
        .clock_speed = 100000,
        .cs_pin = 12,
        .rst_pin = 14,
        .spi_bus = VSPI_HOST
    };

    vfd_handle_t dev;

    VFD_HANDLE dev_ptr = vfd_init(&dev, &init);

    uint8_t counter = 0, sub = 0;

    uint8_t frames[5][5] = {
        {0b1010101,0,0,0,0},
        {0,0b1010101,0,0,0},
        {0,0,0b1010101,0,0},
        {0,0,0,0b1010101,0},
        {0,0,0,0,0b1010101}
    };
    
    for(uint8_t i=0; i<5; i++) {
        vfd_set_current_cgram_page(dev_ptr, &i);        
        vfd_load_custom_segment(dev_ptr, &frames[i][0]);
        vfd_display_custom_segment(dev_ptr, &i);
    }

    vTaskDelay(pdMS_TO_TICKS(1000));

    vfd_copy_string(dev_ptr, "Hello");
    vfd_write_all_segments(dev_ptr);

    vTaskDelay(pdMS_TO_TICKS(1000));

    vfd_copy_string(dev_ptr, "World!");
    vfd_write_all_segments(dev_ptr);

    vTaskDelay(pdMS_TO_TICKS(1000));

    while (1)
    {

        vfd_clear_all_segments(dev_ptr);

        if(sub < 5) {
            vfd_set_current_cgram_page(dev_ptr, &sub);
            vfd_display_custom_segment(dev_ptr, &counter);
        }
        sub++;

        if(sub > 5) {
            sub=0;
            counter++;
        }

        if(counter > 7) {
            counter = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(20));

    }
    /* here be dragons */
}
