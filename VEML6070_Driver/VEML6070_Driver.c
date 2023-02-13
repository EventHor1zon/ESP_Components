/***************************************
* \file     VEML6070_Driver.c
* \brief    A driver for the VEML 6070 UV sensor
*           seems like a simple device - let's see....
*
* \date     Jun 21
* \author   RJAM
****************************************/

/********* Includes *******************/

#include "VEML6070_Driver.h"

#include "esp_types.h"
#include "genericCommsDriver.h"
#include "esp_log.h"
#include "esp_err.h"

const char *VEML_TAG = "VEML Driver";

/****** Function Prototypes ***********/

/************ ISR *********************/

/****** Private Data ******************/

/****** Private Functions *************/

static void veml_driver_task(void *args) {
 
    VEML_DEV dev = (veml_driver_t *)args;

   while(1) {



       vTaskDelay(pdMS_TO_TICKS(10));
   }
   /** here be dragons **/
}

/****** Global Data *******************/

/****** Global Functions *************/

#ifdef CONFIG_DRIVERS_USE_HEAP
VEML_DEV veml_init(veml_init_t *init) 
#else
VEML_DEV veml_init(VEML_DEV dev, veml_init_t *init) 
#endif
{

    esp_err_t err = ESP_OK;
    TaskHandle_t thandle = NULL;

    if(!gcd_i2c_check_bus(init->i2c_bus)) {
        ESP_LOGE(VEML_TAG, "Error invalid i2c bus");
        err = ESP_ERR_INVALID_ARG;
    }

#ifdef CONFIG_DRIVERS_USE_HEAP
    VEML_DEV dev = heap_caps_calloc(1, sizeof(veml_driver_t), MALLOC_CAP_DEFAULT);
    if(dev == NULL) {
        ESP_LOGE(VEML_TAG, "Error initialising device memory");
        err = ESP_ERR_NO_MEM;
    }
#else
    memset(dev, 0, sizeof(veml_driver_t));
#endif
    if(!err) {
        dev->bus = init->i2c_bus;
    }


    if(!err && init->ack_pin > 0) {
        gpio_config_t conf = {
            .intr_type = GPIO_INTR_NEGEDGE,
            .mode = GPIO_MODE_INPUT,
            .pin_bit_mask = (1 << init->ack_pin),
            .pull_down_en = false,
            .pull_up_en = true
        };

        err = gpio_config(&conf);
        if(err) {
            ESP_LOGE(VEML_TAG, "Error initialising gpio [%u]", err);
        }
    }


    if(!err) {

        if(xTaskCreate(veml_driver_task, "veml_driver_task", 5012, dev, 3, &thandle) != pdTRUE) {
            ESP_LOGE(VEML_TAG, "Error initialising driver task");
            err = ESP_ERR_NO_MEM;
        }
        else {
            dev->task = thandle;
        }
    }


    if(!err) {
        ESP_LOGI(VEML_TAG, "VEML Driver succesfully initialised!");
    }
    else {
        ESP_LOGE(VEML_TAG, "VEML Driver initialisation failed");
#ifdef CONFIG_DRIVERS_USE_HEAP
        if(dev != NULL) {
            heap_caps_free(dev);
        }
#endif
    }

    return dev;
}


esp_err_t veml_get_measurement(VEML_DEV dev, uint16_t *val) {
    esp_err_t err = ESP_OK;
    *val = dev->last_value;
    return err;
}


esp_err_t veml_sample(VEML_DEV dev) {

    uint8_t data[2] = {0};
    uint16_t value = 0;
    esp_err_t err = gcd_i2c_short_slave_read(dev->bus, VEML_REGADDR_DATA_MSB, 1, &data[1]);
    if(!err) {
        err = gcd_i2c_short_slave_read(dev->bus, VEML_REGADDR_DATA_LSB, 1, data);
    }
    if(!err) {
        value = (((uint16_t) data[1]) << 8) | ((uint16_t)data[0]);

        dev->last_value = value;
    }

    return err;
}


