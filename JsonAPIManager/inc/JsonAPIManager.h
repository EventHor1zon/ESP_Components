/****************************************
* \file     JsonAPIManager.h
* \brief    Header file for the JsonAPIManager
* \date     Nov 2020
* \author   RJAM
****************************************/

#ifndef API_MANAGER_H
#define API_MANAGER_H

/********* Includes ********************/
#include "esp_http_server.h"
#include "esp_err.h"
/********* Definitions *****************/

#define API_MAN_QUEUE_LEN               10  /** < number of consecutive requests **/
#define API_MAN_QUEUE_ITEM_SIZE         20

#define API_MAX_HTTP_REQUEST_SIZE       1024
#define API_MANAGER_MAX_POST_SIZE       256
#define API_MANAGER_MAX_WS_FRAME_SIZE   128
#define API_MAX_HDR_RESPONSE_LEN        127
#define API_MAX_RESPONSE_LEN            256

#define API_JSON_GET_TAGS               2
#define API_JSON_INFO_TAGS              2
#define API_JSON_SET_TAGS               4
#define API_JSON_SYSINFO_TAGS           2
#define API_JSON_STREAM_TAGS            4

#define API_ERR_MISSING_JSON_FIELD      0x70
#define API_ERR_BAD_JSON                0x71
#define API_ERR_FIELD_NOT_NUMBER        0x72
#define API_ERR_FIELD_NOT_STRING        0x73
#define API_ERR_INVALID_DATA_TYPE       0x74
#define API_ERR_INVALID_JSON_LENGTH     0x75
#define API_ERR_CMD_TIMEOUT             0x76

#define API_JSON_MAX_TAGLEN 16

#define CONFIG_API_MANAGER_TASK_STACK 5012
#define CONFIG_API_MANAGER_TASK_PRIORITY 8

#define API_MANAGER_QUEUE_PUT_TIMEOUT 1000
#define API_MANAGER_QUEUE_GET_TIMEOUT pdMS_TO_TICKS(5000)


#define WS_MAX_LEN 512

/********** Types **********************/


typedef struct async_resp_arg {
    void* hd;
    int fd;
} async_resp_arg_t;


/******** Function Definitions *********/
esp_err_t api_manager_init(void);

esp_err_t cmd_post_handler(httpd_req_t *req);

#endif /* API_MANAGER_H */
