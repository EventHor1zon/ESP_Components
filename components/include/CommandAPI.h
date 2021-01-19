/****************************************
* \file     CommandAPI.h
* \brief    Contains command api struct definitions
* \date     Sept 2020
* \author   RJAM
****************************************/

#ifndef COMMAND_API_H
#define COMMAND_API_H

/********* Includes ********************/

#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_http_server.h"

/********* Definitions *****************/
/** TRY: not sure if this will work... want to have each parameter_t 
 *  to have a generic get/set func pointer, regardless of arg size.
 *  Can try typecasting later?
 *  Try - void * for get (size of memory address, 32bit)
 *        uint32_t - not sure if space allocated or whatever, but 32bit is largest var
 *          except maybe float.
 * */
typedef void(*handle_t); /** < TRY: casting generic handle pointer as void pointer
                                        They should both be same size **/

typedef esp_err_t (*getFunc)(handle_t, void *); /** for ease of conversion, pass in gets & sets as pointers **/
typedef esp_err_t (*setFunc)(handle_t, void *); /** cast depending on param_type_t                          **/

typedef esp_err_t (*actionFunc)(handle_t);

#define CMD_API_ERR_MAX_LEN 64

QueueHandle_t commandq; /** < Handle for the main command queue **/
QueueHandle_t respondq; /** < Handle for the main respond queue **/

/********** Types **********************/


// uint8_t DEV_MANUFACTURER_ESPRESSIF =  0x30;
// uint8_t DEV_TYPE_ESP8266  =  0x01;
// uint8_t DEV_TYPE_ESP32_S1  =  0x02;
// uint8_t DEV_TYPE_ESP32_S2  =  0x03;
// uint8_t DEV_TYPE_ESP_01  =  0x04;
// uint8_t DEV_TYPE_ESP32_LOLINBOARD  =  0x05;
// uint8_t DEV_TYPE_ESP32_LORABOARD  =  0x06;
// uint8_t DEV_TYPE_ESP32_NODEBOARD  =  0x07;
// uint8_t PERIPH_TYPE_LED = 0x01;
// uint8_t PERIPH_TYPE_LEDADDR = 0x02;
// uint8_t PERIPH_TYPE_GPIO = 0x03;
// uint8_t PERIPH_TYPE_DIGITAL_SENSOR = 0x04;
// uint8_t PERIPH_TYPE_ANALOG_SENSOR = 0x05;
// uint8_t PERIPH_TYPE_COMMS_WIFI = 0x06;
// uint8_t PERIPH_TYPE_COMMS_LORA = 0x07;
// uint8_t PERIPH_TYPE_COMMS_RADIO = 0x08;
// uint8_t PERIPH_TYPE_I2C_BASE = 0x09;
// uint8_t PERIPH_TYPE_SPI_BASE = 0x0A;
// uint8_t PERIPH_MEAS_TEMP = 0x01;
// uint8_t PERIPH_MEAS_PRESS = 0x02;
// uint8_t PERIPH_MEAS_HUMID = 0x04;


/** \brief parameter value enum
 *          is also size of value in bytes
 * **/
typedef enum param_type
{
    PARAMTYPE_NONE = 0x00,
    PARAMTYPE_INT8 = 0x01,
    PARAMTYPE_UINT8 = 0x01,
    PARAMTYPE_INT16 = 0x02,
    PARAMTYPE_UINT16 = 0x02,
    PARAMTYPE_INT32 = 0x04,
    PARAMTYPE_UINT32 = 0x04,
    PARAMTYPE_FLOAT = 0x05,
    PARAMTYPE_DOUBLE = 0x08,

    PARAMTYPE_INVALID = 0xFF
} param_type_t;

typedef enum param_flags {
    GET_FLAG = 1,
    SET_FLAG = 2,
    ACT_FLAG = 4,
} param_flag_t;

typedef enum rsp_type
{
    RSP_TYPE_INFO = 0,
    RSP_TYPE_OK = 0x01, /** < generic ok response **/
    RSP_TYPE_ERR,       /** < error response **/
    RSP_TYPE_DATA,      /** < ok response with data **/
    RSP_TYPE_STREAM,    /** < ok response with stream data **/
    RSP_TYPE_END = 0xFF
} rsp_type_t;

typedef struct rsp_err
{
    uint32_t error_code;
    char err_message[CMD_API_ERR_MAX_LEN];
} rsp_err_t;

typedef struct rsp_stream
{
    void *data_mem;
    void *semaphore;
} rsp_stream_t;

typedef enum cmd_type
{
    CMD_TYPE_INFO = 0,
    CMD_TYPE_GET = 1,
    CMD_TYPE_SET = 2,
    CMD_TYPE_GETSET = 3,
    CMD_TYPE_ACT = 4,
    CMD_TYPE_SYSINFO = 5,
} cmd_type_t;

/** Peripheral info packet 
 *      - contains jsonable info for the api manager to send out
 *      - enumerate once per peripheral
**/
typedef struct periph_info
{
    char name[16];
    uint8_t param_num; /** < number of parameters **/
    uint32_t periph_type;
    uint32_t periph_id;
} periph_info_t;

/** Param info packet 
 *      - contains jsonable info for api manager to dispatch
 *      - for type string, max can be used for max length
 *      - for base_type == action, zero both paramtype and max
 **/
typedef struct param_info
{
    char prm_name[32];
    uint8_t p_index;
    cmd_type_t base_type;
    param_type_t param_type;
    uint32_t max;
} param_info_t;


/** Peripheral command packet
 *      - contains info for a peripheral command
 *      - if cmd_type == action, data and ptype can be 0
 *      - if pcmd = info send 
 **/
typedef struct periph_cmd
{
    uint8_t periph_id;
    uint8_t param_id;
    cmd_type_t cmd_type;
    uint32_t data;
    param_type_t ptype;
} periph_cmd_t;

typedef struct rsp_data
{
    char ustring[32];
    uint8_t periph_id; 
    uint8_t param_id;
    rsp_type_t rsp_type;
    uint32_t max;
    uint32_t data;
    uint8_t list[32];
    float float_data;
    param_type_t data_t;
} rsp_data_t;



/** internal use - Command Q and Response Q objects **/

/** command request **/
typedef struct command_request
{
    uint32_t cmd_uid;    /** < command unique id **/
    httpd_req_t *origin;
    periph_cmd_t pcmd_data;    /** < the command data **/
    
} cmd_request_t;

/** command response **/
typedef struct command_response
{
    uint32_t rsp_uid;
    httpd_req_t *origin;
    rsp_data_t rsp_data;
} cmd_rsp_t;



/** \brief Detail struct for each parameter
 * **/
typedef struct parameter
{
    char param_name[32]; /** < parameter name **/
    uint32_t param_id;   /** < parameter unique id **/
    getFunc get;         /** < get function pointer **/
    setFunc set;         /** < set function pointer **/
    uint8_t valueType;   /** < size of parameter in bytes **/
    uint32_t maxValid;   /** < maximum valid value **/
    uint8_t types;       /** < get/set/act flags **/
} parameter_t;

/** \brief detail struct for each action
 * **/

/** TODO: remove action, integrate into parameter **/
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
    char peripheral_name[32];
    handle_t handle;
    parameter_t *params;
    uint8_t param_len;
    action_t *actions;
    uint8_t actions_len;
    uint32_t peripheral_id;
} peripheral_t;




/******** Function Definitions *********/

#endif /* COMMAND_API_H */
