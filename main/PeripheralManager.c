/***************************************
* \file      PeripheralManager.c
* \brief     This API mangs the components, and communicates with the 
*            API_Manager. 
* \date     Sept 2020
* \author   RJAM
****************************************/

/********* Includes *******************/
#include <stdint.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"

/**
 *      The plan - 
 *          Can pass in components from a config file?
 *          An init phase - initialise all compoonents
 *          initialise the PM - 
 *                         incomming message queue
 *                         outgoing message queue
 *                         task
 *           task - wait on incomming messages              
 *                - call approriate function      
 *                - return the requested data
 *                - wait for more messages
 *          
 *          problems - how to capture functionality from peripherals?
 *                   - how to expose functionality to interface?
 *                   - simple get/set interface?    
 ***/

/****** Function Prototypes ***********/

/************ ISR *********************/

/****** Private Data ******************/

/****** Private Functions *************/

/****** Global Data *******************/

/****** Global Functions *************/

esp_err_t peripheral_manager_init()
{
    esp_err_t initStatus = ESP_OK;



    return initStatus;
}