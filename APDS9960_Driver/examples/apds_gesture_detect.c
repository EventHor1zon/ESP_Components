/******************************************************/ /**
                                                          *  @file main.c
                                                          *
                                                          *  @brief main application file for
                                                          *ESPHome
                                                          *
                                                          *********************************************************/

#include "APDS9960_Driver.h"
#include "LSM_Driver.h"
#include "PeripheralManager.h"
#include "Utilities.h"
#include "WifiDriver.h"
#include "device_config.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "genericCommsDriver.h"
#include "main.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

/************************************************
 * Private Functions
 ************************************************/

void app_main(void)
{
    // Initialize NVS
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

    if (ret != ESP_OK) {
        ESP_LOGE("MAIN", "Error starting PM");
    }
#endif

    apds_init_t apds_init_data = {
        .i2c_bus = 1,
        .intr_pin = 19,
    };

    APDS_DEV dev = apds_init(&apds_init_data);

    /** try to work out the gesture sensor ... **/

    uint8_t byte = 0;
    uint16_t val = 0;

    // byte = 2;
    // apds_set_gst_ext_persist(dev, &byte);

    /** set to gst ext tp zero to stay in gesture detect mode **/
    byte = 20;
    apds_set_gst_proximity_ext_thr(dev, &byte);

    byte = 6;
    apds_set_prx_intr_persistence(dev, &byte);

    /** set the entr threshold to ~5/10ths **/
    byte = 20;
    apds_set_gst_proximity_ent_thr(dev, &byte);

    /** set the fifo threshold to 16 **/
    byte = 3;
    apds_set_fifo_thresh(dev, &byte);

    /** enable gesture interrupts - not sure if needed for fifo threshold but
     * probably **/
    byte = 1;
    apds_set_gst_intr(dev, &byte);

    apds_set_prox_intr(dev, &byte);

    /** set the gesture direction sampling to all  **/
    byte = 0;
    apds_set_gst_direction_mode(dev, &byte);

    /** set the led to be full current **/
    apds_set_gst_led_drive(dev, &byte);

    /**  set wait time between gestures to ~40ms **/
    byte = APDS_GST_WAIT_T_39_2MS;
    apds_set_gst_wait(dev, &byte);

    /** reset/clear the fifo **/
    apds_gst_clr_fifo(dev);

    /** set Gst En, pwr on and GMODE **/
    byte = 1;
    apds_set_gst_gmode(dev, &byte);

    /** try turning on proximity to get the entry threshold **/
    apds_set_proximity_status(dev, &byte);

    apds_set_gesture_status(dev, &byte);

    apds_set_pwr_on_status(dev, &byte);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    /* here be dragons */
}
