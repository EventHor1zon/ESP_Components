/***************************************
* \file     HMC5883_Driver.c
* \brief    Driver for the HMC5883L Magneto-meter! 
*           Follows standard pattern of other drivers
*           - task
*           - gets/sets
*           - modes, etc
*           Hahaha little gotcha... Couldn't understand why the chip ID wasn't matching - 
*               went searching & found there's 2 different types of HMC5883L 
*               A honeywell version & a QST version
* \date     March 2021
* \author   RJAM
****************************************/

/********* Includes *******************/
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_types.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "genericCommsDriver.h"
#include "HMC5883_Driver.h"


const char *HMC_TAG = "HMC5883";



/****** Function Prototypes ***********/


static esp_err_t perform_self_test(HMC_DEV dev) {
    return ESP_ERR_NOT_SUPPORTED;
}

static void hmc_driver_task(void *args);



/************ ISR *********************/

/****** Private Data ******************/

/****** Private Functions *************/

static void hmc_driver_task(void *args) {
 
   while(1) {
       vTaskDelay(pdMS_TO_TICKS(10));
   }
   /** here be dragons **/
}

/****** Global Data *******************/

/****** Global Functions *************/


HMC_DEV hmc_init(hmc_init_t *ini) {

    esp_err_t err = ESP_OK;
    HMC_DEV dev = NULL;
    TaskHandle_t t_handle = NULL;

    /** EC the i2c bus **/
    if(!gcd_i2c_check_bus(ini->i2c_bus)) {
        ESP_LOGE(HMC_TAG, "Error: Invalid i2c Bus");
        err = ESP_ERR_INVALID_ARG;
    }
    /** EC/Config the GPIO pin **/
    if(!err) {
        if(ini->drdy_pin > 0) {
            gpio_config_t conf = {0};
            conf.intr_type = GPIO_INTR_NEGEDGE;
            conf.mode = GPIO_MODE_INPUT;
            conf.pin_bit_mask = (1 << ini->drdy_pin);
            err = gpio_config(&conf);

            if(err != ESP_OK) {
               ESP_LOGE(HMC_TAG, "Error: in configuring DRDY pin [%u]", err);
            }
        } 
    }
    /** create driver handle **/
    if(!err) {
        dev = heap_caps_calloc(1, sizeof(hmc_driver_t), MALLOC_CAP_DEFAULT);
        if(dev == NULL) {
            err= ESP_ERR_NO_MEM;
            ESP_LOGE(HMC_TAG, "Error: Could not create driver handle!");
        }
        else {
            /** set default values in the handle **/
            dev->drdy_pin = ini->drdy_pin;
            dev->i2c_bus = ini->i2c_bus;
            dev->i2c_address = HMC_I2C_ADDR;
            dev->mode = HMC_MODE_SINGLE_MEASURE;
        }
    }
    /** create the driver task ***/
    if(!err && xTaskCreate(hmc_driver_task, "hmc_driver_task", configMINIMAL_STACK_SIZE, dev, 3, &t_handle) != pdTRUE) {
        ESP_LOGE(HMC_TAG, "Error: Creating driver task!");
        err = ESP_ERR_NO_MEM;
    } 
    else {
        dev->t_handle = t_handle;
    }

    /** if setup fails and heap has been allocated, free now **/
    if(err){
        ESP_LOGE(HMC_TAG, "Error: Failed to initialise HMC device! [%u]", err);
        if(dev != NULL ){
            heap_caps_free(dev);
        }
    }
    else {
        ESP_LOGI(HMC_TAG, "Succesfully started the HMC Driver!");
    }    

    return dev;

}


esp_err_t hmc_get_mode(HMC_DEV dev, uint8_t *val) {

    esp_err_t err = ESP_OK;
    *val = dev->mode;
}

esp_err_t hmc_set_mode(HMC_DEV dev, uint8_t *val) {
    
    esp_err_t err = ESP_OK;
    uint8_t byte = *val;
    uint8_t regval = 0;
    if(byte > 1) {
        ESP_LOGE(HMC_TAG, "Invalid mode");
        err = ESP_ERR_INVALID_ARG;
    }
    else {
        err = gcd_i2c_read_address(dev->i2c_bus, dev->i2c_address, HMC_REGADDR_CTRL_A, 1, regval);
        if(!err) {
            regval &= ~(0b11); // clear the lowest 2 bits
            regval |= byte;
            err = gcd_i2c_write_address(dev->i2c_bus, dev->i2c_address, HMC_REGADDR_CTRL_A, 1, &regval);
        }
    }

    return err;
}

esp_err_t hmc_get_avg_samples(HMC_DEV dev, uint8_t *val);

esp_err_t hmc_set_avg_samples(HMC_DEV dev, uint8_t *val);





esp_err_t hmc_update_measurements(HMC_DEV dev);