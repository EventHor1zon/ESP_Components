/****************************************
* \file     PeripheralManager.h
* \brief    Header file for the Peripheral manager.c
* \date     Sept 2020
* \author   RJAM
****************************************/

#ifndef PERIPHERAL_MANAGER_H
#define PERIPHERAL_MANAGER_H

/********* Includes ********************/
#include <stdint.h>
#include "esp_err.h"

#include "driver/i2c.h"
/********* Definitions *****************/

#define PM_MAX_PERIPHERALS 30

#define PM_I2C_BUS_PRIMARY I2C_NUM_0

typedef void *peripheral_handle_t;

typedef esp_err_t (*getParam8)(uint8_t *);
typedef esp_err_t (*getParam16)(uint16_t *);
typedef esp_err_t (*getParam32)(uint32_t *);
typedef esp_err_t (*getParamFloat)(float *);

typedef esp_err_t (*setParam8)(uint8_t);
typedef esp_err_t (*setParam16)(uint16_t);
typedef esp_err_t (*setParam32)(uint32_t);
typedef esp_err_t (*setParamFloat)(float);

/** not sure if this will work... want to have each parameter_t 
 *  to have a generic get/set func pointer, regardless of arg size.
 *  Can try typecasting later?
 *  Try - void * for get (size of memory address, 32bit)
 *        uint32_t - not sure if space allocated or whatever, but 32bit is largest var
 *          except maybe float.
 * */
typedef esp_err_t (*getFunc)(void *);
typedef esp_err_t (*setFunc)(uint32_t);

typedef esp_err_t (*actionFunc)();

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

/** \brief  Peripheral_type
 *          Type of peripheral
 * **/
typedef enum peripheral_type
{
    PTYPE_ADDR_LEDS = 0x01, /** < leds, addressable */
    PTYPE_STD_LED,          /** < leds regular */
    PTYPE_ACCEL_SENSOR,     /** < accelerometer/gyroscope/g-sensors */
    PTYPE_ENVIRO_SENSOR,    /** < environment, temp, humid, pressure sensors */
    PTYPE_DISTANCE_SENSOR,  /** < distance/movement/etc sensors */
    PTYPE_POWER_SENSOR,     /** < voltage/current sensor */
    PTYPE_ADC,              /** < adc periperal (on board) */
    PTYPE_IO,               /** < basic io function */
    PTYPE_DISPLAY,          /** < display oled/led/epaper */
    PTYPE_COMMS,            /** < a comms/bluetooth/radio */
    PTYPE_NONE = 0xFF       /** < blank **/
} peripheral_type_t;

/**
 *  \brief enum of peripheral subtypes - increase as drivers developed
 *  TODO: split these up
*/
typedef enum peripheral_subtype
{
    STYPE_NONE,
    STYPE_LEDS_SINGLE_LED,
    STYPE_ADDR_LEDS_APA102,
    STYPE_ADDR_LEDS_WS2812,
    STYPE_ACCEL_SENSOR_LSM,
    STYPE_ACCEL_SENSOR_MPU6050,
    STYPE_ENVIRO_SENSOR_BME_280,
    STYPE_ENVIRO_SENSOR_BME_290,
    STYPE_ENVIRO_SENSOR_DHT11,
    STYPE_DISTANCE_SENSOR_ECHO,
    STYPE_DISTANCE_SENSOR_VL0X,
    STYPE_ADC_ONBOARD,
    STYPE_ADC_EXTERNAL,
    STYPE_IO_SIMPLE,
    STYPE_IO_ARRAY,
    STYPE_DISPLAY_SD1306,
    STYPE_LCD_64_2,
    STYPE_COMMS_BT_ONBOARD,

    STYPE_END = 0xFF
} peripheral_subtype_t;

/** \brief Detail struct for each parameter
 * **/
typedef struct parameter
{
    char param_name[16];    /** < parameter name **/
    uint32_t param_id;      /** < parameter unique id **/
    getFunc get;            /** < get function pointer **/
    setFunc set;            /** < set function pointer **/
    param_type_t valueType; /** < size of parameter in bytes **/
} parameter_t;

/** \brief detail struct for each action
 * **/
typedef struct action
{
    char action_name[32]; /** < action name **/
    uint32_t action_id;   /** < action unique id **/
    actionFunc action;    /** < pointer to action function **/
} action_t;

/** \brief Peripheral type struct. Peripheral manager 
 *         keeps a master list of these
 * 
 * **/
typedef struct peripheral
{
    peripheral_type_t ptype;
    peripheral_subtype_t stype;
    peripheral_handle_t handle;
    parameter_t *params;
    uint8_t param_len;
    action_t *actions;
    uint8_t actions_len;
    uint32_t peripheral_id;
} peripheral_t;

/********** Types **********************/

/******** Function Definitions *********/

#endif /* PERIPHERAL_MANAGER_H */
