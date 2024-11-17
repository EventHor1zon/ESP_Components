/****************************************
* \file     CommandAPI.h
* \brief    Contains command api struct definitions
* \date     Sept 2020
* \author   RJAM
****************************************/

#ifndef COMMAND_API_H
#define COMMAND_API_H

/********* Includes ********************/

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_event.h"

/********* Definitions *****************/
/** TRY: not sure if this will work... want to have each parameter_t 
 *  to have a generic get/set func pointer, regardless of arg size.
 *  Can try typecasting later?
 *  Try - void * for get (size of memory address, 32bit)
 *        uint32_t - not sure if space allocated or whatever, but 32bit is largest var
 *          except maybe float.
 * 
 * 
 * Refactor:
 *      Command API 
 *      
 *      Request Types:
 *     
 *      Command Type |    cmd value     |   periph id   |   param id    |    value     |    value_type 
 *      Device Info          0                  0               0
 *      Periph Info          0                  N               0     
 *      Param Info           0                  N               M
 *      Get                  1                  N               M
 *      Set                  2                  N               M               X            X_type
 *      Act                  4                  N               M 
 * 
 *      Response Types:
 * 
 *      Response Type   |    rsp value  |    name    |   periph_id    |   n periphs  |   periph_ids
 *      Device Info              0         char 16       ui8         ui8            ui8 array[]
 *      Periph Info     |        1      |  char 16   |   ui8   |   n_params    |   param_ids
 *      Param Info      |        2      |  char 16   |   ui8   |   data_t   |   max     |   param_type 
 *                                          periph_id
 *      Get             |       4       |  
 * 
 * */
typedef void(*handle_t); /** < TRY: casting generic handle pointer as void pointer
                                        They should both be same size **/

typedef esp_err_t (*getFunc)(handle_t, void *); /** for ease of conversion, pass in gets & sets as pointers **/
typedef esp_err_t (*setFunc)(handle_t, void *); /** cast depending on datatype_t                          **/

typedef esp_err_t (*actFunc)(handle_t);

#define CMD_API_ERR_MAX_LEN 32
#define API_HOST_MAX_LEN 32

#define DEVICE_MAX_PERIPHERALS 8
#define PERIPHERAL_MANAGER_MAX_STR_LEN 32
#define PERIPHERAL_MANAGER_MAX_PARAMETERS    64
#define PM_MAX_PERIPH_NAME_LENGTH 16
#define PM_MAX_PARAM_NAME_LENGTH 32
#define PM_MAX_DEVICE_NAME_LENGTH 16


/********** Types **********************/


#ifdef CONFIG_USE_EVENTS

ESP_EVENT_DECLARE_BASE(PM_EVENT_BASE);


typedef struct event_map_init {
    uint32_t event_id;
    handle_t handle;
    getFunc get;
    setFunc set;
    actFunc act;
    void *args;
} event_map_init_t;

/** Event map - make a linked-list kinda thing for events,
 * allowing user to register and unregister events 
 * & allowing different events to be linked to different actions
 **/

typedef struct event_map {
    uint32_t event_id;      /**< the event id which triggers the action **/
    getFunc get;            /**< pointer to get function **/
    setFunc set;            /**< pointer to set function **/
    actFunc act;            /**< pointer to act function - should only use one of these! **/
    handle_t handle;        /**< the device handle for function **/
    void *args;             /**< args for function **/
    void *next;      /**< next event map **/ 
    void *prev;      /**< previous event map **/
} event_map_t;


#endif

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

// typedef enum {
//     RSP_PKT_T_INFO,
//     RSP_PKT_T_DATA,
// } rsp_pkt_t;

/** \brief parameter value enum
 *          is also size of value in bytes
 *  TODO: This is bad, fix. Don't duplicate
 * **/
typedef enum datatype
{
    DATATYPE_NONE = 0x00,
    DATATYPE_INT8 = 0x01,
    DATATYPE_UINT8 = 0x02,
    DATATYPE_INT16 = 0x03,
    DATATYPE_UINT16 = 0x04,
    DATATYPE_INT32 = 0x05,
    DATATYPE_UINT32 = 0x06,
    DATATYPE_FLOAT = 0x07,
    DATATYPE_DOUBLE = 0x08,
    DATATYPE_STRING = 0x0A,
    DATATYPE_BOOL = 0x0B,
    DATATYPE_INVALID = 0xFF
} datatype_t;

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


typedef enum param_flags {
    GET_FLAG = 1,
    SET_FLAG = 2,
    ACT_FLAG = 4,
    STREAM_FLAG = 8
} param_flag_t;

typedef enum rsp_type
{
    RSP_TYPE_DEV_INFO   = 0,        /** device info response **/
    RSP_TYPE_PERIPH_INFO = 1,       /** periph info response **/
    RSP_TYPE_PARAM_INFO = 2,        /** param info response  **/
    RSP_TYPE_DATA       = 3,        /** < ok response with data **/
    RSP_TYPE_ACK        = 4,        /** acknowledge response **/
    RSP_TYPE_ERR        = 5,        /** < error response **/
    RSP_TYPE_STREAM     = 6,        /** < ok response with stream data **/
    RSP_TYPE_MAX
} rsp_type_t;

typedef enum {
    REQ_PKT_TYPE_PERIPH_CMD = 1,
    REQ_PKT_TYPE_STREAM = 2,
} req_pkt_t; 


typedef enum cmd_type
{
    CMD_TYPE_INFO = 0,
    CMD_TYPE_GET = 1,
    CMD_TYPE_SET = 2,
    CMD_TYPE_GETSET = 3,
    CMD_TYPE_ACT = 4,
    CMD_TYPE_STREAM = 5,
} cmd_type_t;



/**< 
 *      These are responses given by the PeripheralManager Task
 *      
 *      device_info_rsp_t - response to device_info_req
 *      peripheral_info_t - response to peripheral_info_req
 *      param_info_rsp_t - response to param_info_req
 *      data_rsp_t       - Data response to get request
 *      rsp_ack_t        - Acknowledged response to set/invoke
 *      rsp_err_t        - Error response
 ***/

/** \struct device_info_rsp_t 
 *  \brief device info response struct
 */
typedef struct device_info {
    char name[PM_MAX_DEVICE_NAME_LENGTH];      /** device name **/
    uint8_t device_id;  /** device id **/
    uint8_t num_periphs;    /** number of peripheral **/
    uint8_t periph_ids[DEVICE_MAX_PERIPHERALS]; /** list of peripheral IDs **/
} device_info_rsp_t;

/** Peripheral info packet 
 *      - contains jsonable info for the api manager to send out
 *      - enumerate once per peripheral
**/
typedef struct periph_info
{
    char name[PM_MAX_PERIPH_NAME_LENGTH];          /** peripheral name **/
    uint8_t periph_id;      /** peripheral id **/
    uint8_t param_num;      /** number of parameters **/
    uint8_t param_ids[PERIPHERAL_MANAGER_MAX_PARAMETERS];
    uint8_t periph_type;    /** peripheral type **/
} periph_info_rsp_t;

/** Param info packet 
 *      - contains jsonable info for api manager to dispatch
 *      - for type string, max can be used for max length
 *      - for base_type == action, zero both paramtype and max
 **/
typedef struct param_info
{
    char name[PM_MAX_PARAM_NAME_LENGTH];  /** parameter name **/
    uint8_t param_id;           /** parameter id **/
    uint8_t periph_id;          /** parent peripheral id **/
    uint8_t cmd_type;        /** the valid commands for this parameter
                                *   valid command types are a mask of 
                                *   cmd_type_t. 
                                **/
    datatype_t value_type;    /** the value type 
                                 *  floats are 32-bit 
                                 **/
    uint32_t max_value;         /** the maximum value this parameter can
                                 *  be set to. For string values, this is the
                                 *  maximum length of the string. Set to 0
                                 *  when SET command not supported
                                **/
} param_info_rsp_t;


/** \struct data_rsp_t
 *  \brief  data response, response to a get request 
 * 
 */
typedef struct data_rsp {
    uint8_t param_id;
    uint8_t periph_id;
    union {
        int32_t sint_data;
        uint32_t uint_data;
        float float_data;
        char str_data[PERIPHERAL_MANAGER_MAX_STR_LEN];
    } data;
    datatype_t data_t;       /** the data type **/
} data_rsp_t;


/** \struct ack_rsp_t
 *  \brief The most simple response packet - response to a set or
 *          invoke function which succeeds
 * 
*/
typedef struct rsp_ack {
    uint8_t opt;
    uint8_t periph_id;
    uint8_t param_id;
} ack_rsp_t;


/**  \struct Error response packet
 *   \brief contains an error code and
 *           a 64-char error message
*/
typedef struct rsp_err
{
    uint32_t error_code;
    char err_message[CMD_API_ERR_MAX_LEN];
} err_rsp_t;




typedef struct rsp_stream
{
    void *data_mem;
    void *semaphore;
} rsp_stream_t;


/**
 * ~~~~~~~~~ Peripheral Manager Requests ~~~~~~~~~
 *  Peripheral command - currently a do-all struct
 *                       data set depending on cmd type
 *  TODO: stream response
*/

/** Peripheral command packet
 *      - contains info for a peripheral command
 *      - if cmd_type == action, data and data_t can be 0
 *      - if pcmd = info send 
 **/
typedef struct periph_cmd
{
    cmd_type_t cmd_type;    /** the command type  **/
    uint8_t periph_id;      /** the peripheral id **/
    uint8_t param_id;       /** the parameter id  **/
    union {
        int32_t sint_data;
        uint32_t uint_data;
        float float_data;
        char str_data[PERIPHERAL_MANAGER_MAX_STR_LEN];
    } data;                 /** the data (for SET only)
                             *  this is a union depending on 
                             *  data type
                             **/
    datatype_t data_t;       /** the data type **/
} periph_cmd_t;


/** stream command type
 * 
 **/
typedef struct stream_cmd 
{
    uint8_t param_ids[5];       /**< IDs of parameters to stream **/
    uint8_t periph_id;          /**< Peripheral id **/
    uint8_t rate;               /**< the websocket data rate **/
    uint8_t param_num;          /**< the number of parameters to stream **/
    uint8_t stream_type;        /**< type of stream **/
    void *stream_handle;        /**< pointer to the stream handle (created by API) **/
} stream_cmd_t;


/** internal use - Command Q and Response Q objects **/

/** command request **/
typedef struct command_request
{
    uint16_t cmd_uid;
    req_pkt_t cmd_type;
    union CommandAPI
    {
        /* data */
        periph_cmd_t cmd_data;    /** < the command data **/
        stream_cmd_t strm_data;
    } data;

    void *rsp_args;
} cmd_request_t;


/** command response **/
typedef struct cmd_rsp {
    rsp_type_t rsp_type;

    union {
        device_info_rsp_t dev_info;
        periph_info_rsp_t periph_info;
        param_info_rsp_t param_info;
        data_rsp_t data;
        ack_rsp_t ack;
        err_rsp_t error;
    } rsp_data;

    uint16_t rsp_uid;
    void *rsp_args;
} cmd_rsp_t;



/** \brief Detail struct for each parameter
 * **/
typedef struct parameter
{
    char param_name[32]; /** < parameter name **/
    uint8_t param_id;   /** < parameter unique id **/
    getFunc get;         /** < get function pointer **/
    setFunc set;         /** < set function pointer **/
    actFunc act;
    uint8_t value_type;   /** < size of parameter in bytes **/
    uint32_t max_valid;   /** < maximum valid value **/
    uint8_t types;       /** < get/set/act flags **/
} parameter_t;



/** \brief Peripheral type struct. Peripheral manager 
 *         keeps a master list of these
 * 
 * **/
typedef struct peripheral
{
    char peripheral_name[32];
    handle_t handle;
    const parameter_t *params;
    uint8_t param_len;
    uint32_t peripheral_id;
    peripheral_type_t periph_type;
} peripheral_t;




/******** Function Definitions *********/


/** returns the datatype size in bytes **/
uint8_t get_param_data_size(datatype_t t);


#endif /* COMMAND_API_H */
