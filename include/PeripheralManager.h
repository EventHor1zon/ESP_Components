/****************************************
* \file     PeripheralManager.h
* \brief    Header file for the Peripheral manager.c
*
*   PeripheralManager is one of the main components of ESP_Home system
*   It acts as a bridge btween the connected peripherals and the API manager
*   It creates the main command queue and the control task is blocked waiting for 
*   an incomming command from the API manager
*
*
*
* \date     Sept 2020
* \author   RJAM
****************************************/

#ifndef PERIPHERAL_MANAGER_H
#define PERIPHERAL_MANAGER_H

/********* Includes ********************/
#include <stdint.h>
#include "esp_err.h"
#include "APIComponentMap.h"

#include "driver/i2c.h"
/********* Definitions *****************/

#define PM_MAX_PERIPHERALS 10
#define PM_MAX_QUEUE_LEN 10
#define PM_QUEUE_SEND_TIMEOUT 1000
#define PM_PIFO_NUM_TYPE 0xFF

#define PM_ERR_INVALID_ID 0x80
#define PM_ERR_INVALID_ARG 0x81
#define PM_ERR_INVALID_TYPE 0x82
#define PM_ERR_INVALID_CMD 0x83

#define PM_I2C_BUS_PRIMARY I2C_NUM_0

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

/** \brief Peripheral type struct. Peripheral manager 
 *         keeps a master list of these
 * 
 * **/
typedef struct peripheral
{
    peripheral_type_t ptype;
    peripheral_subtype_t stype;
    handle_t handle;
    parameter_t *params;
    uint8_t param_len;
    action_t *actions;
    uint8_t actions_len;
    uint32_t peripheral_id;
} peripheral_t;

typedef struct peripheral_summary
{
    peripheral_t *peripherals;
    uint8_t perip_num;
} peripheral_summary_t;

/********** Types **********************/

/******** Function Definitions *********/

/** peripheral_manager_init()
 *  
 *  initialises the peripheral manager
 * 
 *  \return ESP_OK or error 
 **/
esp_err_t peripheral_manager_init(void);

/** pm_process_incoming_request
 * 
 * 
 **/
cmd_rsp_t pm_process_incoming_request(cmd_request_t *request);

#endif /* PERIPHERAL_MANAGER_H */
