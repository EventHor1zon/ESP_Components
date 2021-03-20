/***************************************
* \file     APDS9960_Driver.c
* \brief    Driver for the APDS Gesture/RGB/Distancce/Light-level sensor
*
* \date     March 2021
* \author   RJAM
****************************************/

/********* Includes *******************/
#include "esp_types.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "genericCommsDriver.h"
#include "APDS9960_Driver.h"

/****** Function Prototypes ***********/

/************ ISR *********************/

/****** Private Data ******************/

static void APSD9960_driver_task(void *args) {
 
   while(1) {
       vTaskDelay(pdMS_TO_TICKS(10));
   }
   /** here be dragons **/
}

/****** Private Functions *************/

/****** Global Data *******************/

/****** Global Functions *************/

APDS_DEV apds_init(apds_init_t *ini) {
    esp_err_t status = ESP_OK;
    

    if(!gcd_check_i2c_bus(ini->i2c_bus)) {
        ESP_LOGE();
    }


    return status;
}

