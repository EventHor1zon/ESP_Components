/***************************************
* \file     HMC5883_Driver.c
* \brief    Driver for the HMC5883L Magneto-meter! 
*           Follows standard pattern of other drivers
*           - task
*           - gets/sets
*           - modes, etc
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

const char *hmc_id_bytes[3] = { "H", "C", "4" };


/****** Function Prototypes ***********/


static esp_err_t perform_self_test(HMC_DEV dev);

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
    if(!gcd_check_i2c_bus(ini->i2c_bus)) {
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

    /** better check the device is actually on the bus... **/
    if(!err) {
        uint8_t recv[3] = {0};
        err = gcd_i2c_read_address(dev->i2c_bus, HMC_I2C_ADDR, HMC_REGADDR_ID_A, 3, recv);
        if(!err 
            && recv[0] == hmc_id_bytes[0]
            && recv[1] == hmc_id_bytes[1]
            && recv[2] == hmc_id_bytes[2]
        ) {
            ESP_LOGI(HMC_TAG, "Performed succesful ID check!");
        } 
        else {
            ESP_LOGI(HMC_TAG, "ID Check was unsuccesful! [ %02x %02x %02x]", recv[0], recv[1], recv[2]);
            /** dont fail at this point, out of curiosity **/
        }
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


esp_err_t hmc_get_mode(HMC_DEV dev, uint8_t *val);

esp_err_t hmc_set_mode(HMC_DEV dev, uint8_t *val);

esp_err_t hmc_get_avg_samples(HMC_DEV dev, uint8_t *val);

esp_err_t hmc_set_avg_samples(HMC_DEV dev, uint8_t *val);

esp_err_t hmc_get_mode(HMC_DEV dev, uint8_t *val);

esp_err_t hmc_set_mode(HMC_DEV dev, uint8_t *val);

esp_err_t hmc_get_mode(HMC_DEV dev, uint8_t *val);

esp_err_t hmc_set_mode(HMC_DEV dev, uint8_t *val);

esp_err_t hmc_update_measurements(HMC_DEV dev);