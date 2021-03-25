/***************************************
* \file     ADPS9960_Driver.c
* \brief    Driver for the RGB, Gesture & proximity detector
*           Going to cover most of the device's features
*           TBH not going to do the offsets unless needed.
*           (Aint nobody got time for that)
*           
* \date     March 2021
* \author   RJAM
****************************************/

/********* Includes *******************/
#include "esp_err.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "genericCommsDriver.h"
#include "APDS9960_Driver.h"


const char *APDS_TAG = "APDS Driver";

/****** Function Prototypes ***********/

/************ ISR *********************/

static esp_err_t apds_initialise_device(APDS_DEV dev) {

    uint8_t pwr_on = 1;
    uint8_t id = 0;
    esp_err_t err = gcd_i2c_write_address(dev->bus, dev->addr, APDS_REGADDR_ENABLE, 1, &pwr_on);

    if(!err) {
        err = gcd_i2c_read_address(dev->bus, dev->addr, APDS_REGADDR_ID, 1, &id);
        if(!err) {
            ESP_LOGI(APDS_TAG, "Got ID: 0x%02x", id);
        }
    }

    /** set some default values **/


    return err;

}

/****** Private Data ******************/

/****** Private Functions *************/

/****** Global Data *******************/

/****** Global Functions *************/

APDS_DEV apds_init(apds_init_t *init) {

    esp_err_t err = ESP_OK;
    APDS_DEV dev = NULL;


    if(!gdc_i2c_check_bus(init->i2c_bus)) {
        err = ESP_ERR_INVALID_ARG;
        ESP_LOGE(APDS_TAG, "Error - invalid i2c bus");
    }

    if(!err) {
        dev = heap_caps_calloc(1, sizeof(adps_handle_t), MALLOC_CAP_DEFAULT);
        if(dev == NULL) {
            ESP_LOGE(APDS_TAG, "Error: Error assigning heap mem [%u]", err);
            err = ESP_ERR_NO_MEM;
        }
        else {
            dev->addr = APDS_I2C_ADDRESS;
            dev->bus = init->i2c_bus;
        }
    }

    if(!err && apds_initialise_device(dev)) {
        ESP_LOGE(APDS_TAG, "Error: Error assigning heap mem [%u]", err);
        err = ESP_ERR_TIMEOUT;
    }

    return dev;
}


