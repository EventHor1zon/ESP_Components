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

/********* Definitions *****************/

#define CMD_API_ERR_MAX_LEN 64

QueueHandle_t commandq; /** < Handle for the main command queue **/
QueueHandle_t respondq; /** < Handle for the main respond queue **/

/********** Types **********************/

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

typedef enum cmd_type
{
    CMD_TYPE_PINFO = 0x01, /** < command peripheral info **/
    CMD_TYPE_PRMINFO,      /** < command parameter type **/
    CMD_TYPE_SINFO,        /** < command system info **/
    CMD_TYPE_PCMD,         /** < peripheral command **/
    CMD_TYPE_DIRECT,       /** < system command **/

    CMD_TYPE_END = 0xFF
} cmd_type_t;

typedef enum rsp_type
{
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

typedef struct rsp_data
{
    uint32_t param_id;
    uint8_t type;
    uint32_t data;
    char name[16];
} rsp_data_t;

typedef struct rsp_stream
{
    void *data_mem;
    void *semaphore;
} rsp_stream_t;

typedef enum pcmd_type
{
    PCMD_TYPE_GET,
    PCMD_TYPE_SET,
    PCMD_TYPE_ACT
} pcmd_type_t;

/** Peripheral info packet 
 *      - contains jsonable info for the api manager to send out
 *      - enumerate once per peripheral
**/
typedef struct periph_info
{
    char name[16];
    uint8_t paramNums; /** < number of parameters **/
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
    char prm_name[16];
    pcmd_type_t base_type;
    param_type_t param_type;
    uint32_t max;
} param_info_t;

/** Peripheral command packet
 *      - contains info for a peripheral command
 *      - if pcmd_type == action, data and ptype can be 0
 **/
typedef struct periph_cmd
{
    uint32_t periph_id;
    uint32_t param_id;
    pcmd_type_t pcmd_type;
    uint32_t data;
    param_type_t ptype;
} periph_cmd_t;

typedef struct command_request
{
    cmd_type_t cmd_type; /** < command type (one of cmd_type_t) **/
    uint32_t cmd_uid;    /** < command unique id **/
    union
    {
        periph_info_t lcmd_data;
        periph_cmd_t pcmd_data;
    } cmd_data;    /** < the command data **/
    uint32_t auth; /** < reserved **/
} cmd_request_t;

typedef struct command_response
{
    rsp_type_t rsp_type;
    uint32_t rsp_uid;
    union
    {
        rsp_err_t ersp_data;
        rsp_data_t drsp_data;
        rsp_stream_t srsp_data;
        uint32_t reserved;
    } rsp_data;
    uint32_t auth; /** < reserved **/
} cmd_rsp_t;

/******** Function Definitions *********/

#endif /* COMMAND_API_H */
