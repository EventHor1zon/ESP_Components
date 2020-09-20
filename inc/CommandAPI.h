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

/********* Definitions *****************/

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
    PARAMTYPE_FLOAT = 0x05,
    PARAMTYPE_DOUBLE = 0x08,

    PARAMTYPE_INVALID = 0xFF
} param_type_t;

typedef enum cmd_type
{
    CMD_TYPE_PINFO = 0x01, /** < command peripheral info **/
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
    char err_message[64];
} rsp_err_t;

typedef struct rsp_data
{
    uint32_t param_id;
    uint8_t type;
    uint32_t data;
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

typedef struct periph_info
{
    uint32_t periph_id;
} periph_info_t;

typedef struct sys_info
{
    uint32_t sys_id;
} sys_info_t;

typedef struct periph_cmd
{
    uint32_t periph_id;
    uint32_t param_id;
    pcmd_type_t pcmd_type;
    uint32_t data;
    param_type_t ptype;
} periph_cmd_t;

typedef struct direct_cmd
{
    uint32_t dcmd_id;
    uint32_t data;
} direct_cmd_t;

typedef struct command_request
{
    cmd_type_t cmd_type;
    uint32_t cmd_uid;
    union
    {
        periph_info_t lcmd_data;
        sys_info_t icmd_data;
        periph_cmd_t pcmd_data;
        direct_cmd_t dcmd_data;
    } cmd_data;
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

/********** Types **********************/

/******** Function Definitions *********/

#endif /* COMMAND_API_H */
