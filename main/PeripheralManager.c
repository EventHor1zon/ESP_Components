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

#include "../inc/PeripheralManager.h"

#include "BME280_Driver.h"
#include "../components/include/APIComponentMap.h"

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

static peripheral_t *peripherals[PM_MAX_PERIPHERALS];
static uint8_t peripheral_num = 0;
/****** Private Functions *************/

/****** Global Data *******************/

/****** Global Functions *************/

esp_err_t peripheral_manager_init()
{
    esp_err_t initStatus = ESP_OK;

    /** peripheral init code goes here **/
    /** use a random peripheral to check process **/

    bm_initData_t bme = {0};
    bme.devType = BME_280_DEVICE;
    bme.addressPinState = 0;
    bme.sampleMode = BM_FORCE_MODE;
    bme.sampleType = BM_MODE_TEMP_PRESSURE_HUMIDITY;
    bme.i2cChannel = PM_I2C_BUS_PRIMARY;

    bm_controlData_t *bmHandle = bm280_init(&bme);

    peripheral_t *bmeP = heap_caps_calloc(1, sizeof(peripheral_t), MALLOC_CAP_DEFAULT);
    bmeP->handle = bmHandle;
    bmeP->ptype = PTYPE_ENVIRO_SENSOR;
    bmeP->stype = STYPE_ENVIRO_SENSOR_BME_290;
    bmeP->actions = bm_action_mappings;
    bmeP->actions_len = bm_action_len;
    bmeP->params = bm_param_mappings;
    bmeP->param_len = bm_param_len;
    bmeP->peripheral_id = (uint32_t)(bmeP->ptype << 16) | (uint32_t)(bmeP->stype << 8) | (uint32_t)peripheral_num;

    
    return initStatus;
}