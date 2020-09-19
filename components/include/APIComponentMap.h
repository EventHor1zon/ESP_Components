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
#include <stdint.h>
#include "esp_err.h"

/********* Definitions *****************/

typedef void(*handle_t); /** < TRY: casting generic handle pointer as void pointer
                                        They should both be same size **/

/** Probably don't need these **/
// typedef esp_err_t (*getParam8)(handle_t, uint8_t *);
// typedef esp_err_t (*getParam16)(handle_t, uint16_t *);
// typedef esp_err_t (*getParam32)(handle_t, uint32_t *);
// typedef esp_err_t (*getParamFloat)(handle_t, float *);

// typedef esp_err_t (*setParam8)(handle_t, uint8_t);
// typedef esp_err_t (*setParam16)(handle_t, uint16_t);
// typedef esp_err_t (*setParam32)(handle_t, uint32_t);
// typedef esp_err_t (*setParamFloat)(handle_t, float);

/** TRY: not sure if this will work... want to have each parameter_t 
 *  to have a generic get/set func pointer, regardless of arg size.
 *  Can try typecasting later?
 *  Try - void * for get (size of memory address, 32bit)
 *        uint32_t - not sure if space allocated or whatever, but 32bit is largest var
 *          except maybe float.
 * */
typedef esp_err_t (*getFunc)(handle_t, void *); /** for ease of conversion, pass in gets & sets as pointers **/
typedef esp_err_t (*setFunc)(handle_t, void *); /** cast depending on param_type_t                          **/

typedef esp_err_t (*actionFunc)(handle_t);

/** \brief parameter value enum
 *          is also size of value in bytes
 * **/
typedef enum param_type
{
    PARAMTYPE_INT8 = 0x01,
    PARAMTYPE_UINT8 = 0x01,
    PARAMTYPE_INT16 = 0x02,
    PARAMTYPE_UINT16 = 0x02,
    PARAMTYPE_INT32 = 0x04,
    PARAMTYPE_UINT32 = 0x04,
    PARAMTYPE_FLOAT = 0x04,
    PARAMTYPE_DOUBLE = 0x08,

    PARAMTYPE_INVALID = 0xFF
} param_type_t;

/** \brief Detail struct for each parameter
 * **/
typedef struct parameter
{
    char param_name[32];    /** < parameter name **/
    uint32_t param_id;      /** < parameter unique id **/
    getFunc get;            /** < get function pointer **/
    setFunc set;            /** < set function pointer **/
    param_type_t valueType; /** < size of parameter in bytes **/
    uint32_t maxValid;      /** < maximum valid value **/
} parameter_t;

/** \brief detail struct for each action
 * **/
typedef struct action
{
    char action_name[32]; /** < action name **/
    uint32_t action_id;   /** < action unique id **/
    actionFunc action;    /** < pointer to action function **/
} action_t;

#define bm_param_len 11
parameter_t bm_param_mappings[bm_param_len] = {
    {"Sample Interval", 0, &bm280_getSampleInterval, &bm280_setSampleInterval, PARAMTYPE_UINT8, 7},
    {"Filter Setting", 1, &bm280_getFilterSetting, &bm280_setFilterSetting, PARAMTYPE_UINT8, 4},
    {"Device ID", 2, bm280_getDeviceID, NULL, PARAMTYPE_UINT8, 0},
    {"Temperature", 3, &bm280_getTemperature, NULL, PARAMTYPE_FLOAT, 0},
    {"Pressure", 4, &bm280_getPressure, NULL, PARAMTYPE_FLOAT, 0},
    {"Humidity", 5, &bm280_getHumidity, NULL, PARAMTYPE_FLOAT, 0},
    {"Temperature OSample", 6, &bm280_getTemperatureOS, &bm280_setTemperatureOS, PARAMTYPE_UINT8, 5},
    {"Pressure OSample", 7, &bm280_getPressureOS, &bm280_setPressureOS, PARAMTYPE_UINT8, 5},
    {"Humidity OSample", 8, &bm280_getHumidityOS, &bm280_setHumidityOS, PARAMTYPE_UINT8, 5},
    {"Sample Mode", 9, &bm280_getSampleMode, &bm280_setSampleMode, PARAMTYPE_UINT8, 3},
    {"Sample Type", 10, &bm280_getSampleType, NULL, PARAMTYPE_UINT8, 7},
};

#define bm_action_len 1
action_t bm_action_mappings[bm_action_len] = {
    {"Update Measurements", 0, &bm280_updateMeasurements},
};

/********** Types **********************/

/******** Function Definitions *********/

#endif /* API_COMPONENT_MAP_H */
