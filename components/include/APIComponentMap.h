/****************************************
* \file     APIComponentMap.h
* \brief    A quick header file which maps the functionality of 
*           a component to a peripheral manager-style interface
* \date 
* \author
****************************************/

#ifndef API_COMPONENT_MAP_H
#define API_COMPONENT_MAP_H

/********* Includes ********************/

#include "BME280_Driver.h"
#include "../../inc/PeripheralManager.h"

/********* Definitions *****************/

uint8_t bm_param_len = 11;
parameter_t bm_param_mappings[bm_param_len] = {
    {"Sample Interval", 0, &bm280_getSampleInterval, &bm280_setSampleInterval, PARAMTYPE_UINT8},
    {"Filter Setting", 1, &bm280_getFilterSetting, &bm280_setFilterSetting, PARAMTYPE_UINT8},
    {"Device ID", 2, &bm280_getDeviceID, NULL, PARAMTYPE_UINT8},
    {"Temperature", 3, &bm280_getTemperature, NULL, PARAMTYPE_FLOAT},
    {"Pressure", 4, &bm280_getPressure, NULL, PARAMTYPE_FLOAT},
    {"Humidity", 5, &bm280_getHumidity, NULL, PARAMTYPE_FLOAT},
    {"Temperature OSample", 6, &bm280_getTemperatureOS, &bm280_setTemperatureOS, PARAMTYPE_UINT8},
    {"Pressure OSample", 7, &bm280_getPressureOS, &bm280_setPressureOS, PARAMTYPE_UINT8},
    {"Humidity OSample", 8, &bm280_getHumidityOS, &bm280_setHumidityOS, PARAMTYPE_UINT8},
    {"Sample Mode", 9, &bm280_getSampleMode, &bm280_setSampleMode, PARAMTYPE_UINT8},
    {"Sample Type", 10, &bm280_getSampleType, NULL, PARAMTYPE_UINT8},
};

uint8_t bm_action_len = 1;
action_t bm_action_mappings[bm_action_len] = {
    {"Update Measurements", 0, &bm280_updateMeasurements},
};

/********** Types **********************/

/******** Function Definitions *********/

#endif /* API_COMPONENT_MAP_H */
