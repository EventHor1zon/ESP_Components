/***************************************
* \file     DS2321_Driver.c
* \brief    A driver for the RTC device
*
* \date     March 2021
* \author   RJAM
****************************************/

/********* Includes *******************/
#include "esp_err.h"
#include "esp_log.h"
#include "esp_types.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "genericCommsDriver.h"


/****** Function Prototypes ***********/

/************ ISR *********************/

/****** Private Data ******************/

/****** Private Functions *************/

/****** Global Data *******************/

/****** Global Functions *************/

DS2321_DEV ds2321_init(ds2321_init_t *ini) {

    esp_err_t err = ESP_OK;

    if(!gcd)


}

