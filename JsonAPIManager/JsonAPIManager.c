/***************************************
* \file     API_Manager.c
* \brief    api manager for the ESP home project
*           hosts the webserver and opens up a restish api for 
*           external communication
* \date     AUG 2020
* \author   RJAM
****************************************/

/** TODO:
 *  -   start the http server
 *  -   add json handling
 *  -   create an interface with the peripheral manager
 *  -   create a message-pump fueled by event group
 *  -   use task blocking to keep activity to a minimum
 *  -   implement some kind of token system for auth?
 *  -   add support for websockets/stream
 **/

/** Stream thoughts:  
 ** the first stream packet should contain these json tags 
 ** the ESP responds with an 'OK' json packet
 ** then the esp sets up a timer for simple auto-get function
 ** or for fast mode, sets up a circular buffer & inits a stream function in the 
 ** streamable driver. This bit would be better separated out into stream component
 ** Individual FIFO setups will be a pain...
 ** Maybe get the device to tell about its streaming component capabilities?
 ** Maybe have a standard data rate for different component fifos, then
 ** sorta try to match them? 
 **/

/**
    Refactor Time! 

    TODO: 
        Decouple send/receive tasks 
        Parse json in a more cleverer manner

    How should we handle http packets?


    
    ~~~ TxTask ~~~



*/


/********* Includes *******************/


#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "cJSON.h"

#include "string.h"

#include "freertos/task.h"
#include "freertos/queue.h"
#include "JsonAPIManager.h"
#if CONFIG_ENABLE_STREAM
#include "StreamComponent.h"
#endif /* CONFIG_ENABLE_STREAM */
#include "Utilities.h"
#include "CommandAPI.h"
#include "PeripheralManager.h"

#define DEBUG_MODE 1

/****** Global Data *******************/


const char *API_TAG = "API-MNGR";
const char *info_json_tags[API_JSON_INFO_TAGS] = { "periph_id", "param_id"}; 
const char *get_json_tags[API_JSON_GET_TAGS] = { "periph_id", "param_id" };
const char *set_json_tags[API_JSON_SET_TAGS] = { "periph_id", "param_id", "data_t", "data"};
const char *stream_json_tags[API_JSON_STREAM_TAGS] = {"periph_id", "param_ids", "rate", "type"};

const char *http_ok_hdr_str = "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: %d\r\n";
const char *http_server_err_str = "HTTP/1.1 500\r\n";
const char *cr_lf_seperator = "\r\n";



static const httpd_uri_t cmdhandler = {
    .uri = "/cmd",
    .method = HTTP_POST,
    .handler = cmd_post_handler, 
    .user_ctx = NULL 
};

#ifdef CONFIG_ENABLE_STREAM

const char *stream_json_tags[API_JSON_STREAM_TAGS] = { "periph_id", "param_ids", "chunks_per_packet", "stream_type" };

static const httpd_uri_t streamhandler = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL,
    .is_websocket = true
};

#endif

/****** Static Function Prototypes ***********/

/** \brief - returns a json string response to device info cmd
 *  \param - rsp_data  - data returned from the PeripheralManager
 *  \param - strbuffer - a buffer to store the json string in 
 *  \param - buffer_len - length of the storage buffer 
 *  \return ESP_OK or Error
 */
static esp_err_t device_info_to_json_string(device_info_rsp_t *rsp_data, char *strbuffer, uint16_t buffer_len);

/** \brief - returns a json string resposne to param info cmd
 *  \param - rsp_data  - data returned from the PeripheralManager
 *  \param - strbuffer - a buffer to store the json string in 
 *  \param - buffer_len - length of the storage buffer 
 *  \return ESP_OK or Error
 */
static esp_err_t param_info_to_json_string(param_info_rsp_t *rsp_data, char *strbuffer, uint16_t buffer_len);

/** \brief - returns a json string resposne to periph info cmd
 *  \param - rsp_data  - data returned from the PeripheralManager
 *  \param - strbuffer - a buffer to store the json string in 
 *  \param - buffer_len - length of the storage buffer 
 *  \return ESP_OK or Error
 */
static esp_err_t periph_info_to_json_string(periph_info_rsp_t *rsp_data, char *strbuffer, uint16_t buffer_len);

/** \brief - returns a json string resposne to GET cmd
 *  \param - rsp_data  - data returned from the PeripheralManager
 *  \param - strbuffer - a buffer to store the json string in 
 *  \param - buffer_len - length of the storage buffer 
 *  \return ESP_OK or Error
 */
static esp_err_t get_response_to_json_string(data_rsp_t *rsp_data, char *strbuffer, uint16_t buffer_len);

/** \brief - returns a json string respone to SET cmd
 *  \param - rsp_data  - data returned from the PeripheralManager
 *  \param - strbuffer - a buffer to store the json string in 
 *  \param - buffer_len - length of the storage buffer 
 * \return ESP_OK or Error
 */
static esp_err_t ok_rsp_to_json_string(ack_rsp_t *rsp_data, char *strbuffer, uint16_t buffer_len);

/** \brief - returns a json string error response
 *  \param - err_msg - error message
 *  \param - strbuffer - a buffer to store the json string in 
 *  \param - buffer_len - length of the storage buffer 
 *  \return ESP_OK or Error
 */
static esp_err_t error_rsp_to_json_string(const char *err_msg, uint8_t err_code, char *strbuffer, uint16_t buffer_len);

/** \brief - converts an info json http request into
 *           a peripheral manager command request  
 *  \param json    - cJSON parsed json data
 *  \param command - pointer to a cmd_request_t struct to populate
 *  \return ESP_OK or Error
 */
static esp_err_t handle_info_request(cJSON *json, cmd_request_t *command);

/** \brief - converts an get json http request into
 *           a peripheral manager command request  
 *  \param json    - cJSON parsed json data
 *  \param command - pointer to a cmd_request_t struct to populate
 *  \return ESP_OK or Error
 */
static esp_err_t handle_get_request(cJSON *json, cmd_request_t *command);

/** \brief - converts a set json http request into
 *           a peripheral manager command request  
 *  \param json    - cJSON parsed json data
 *  \param command - pointer to a cmd_request_t struct to populate
 *  \return ESP_OK or Error
 **/
static esp_err_t handle_set_request(cJSON *json, cmd_request_t *command);

/** \brief - converts an invoke (action) json http request into
 *           a peripheral manager command request  
 *  \param json    - cJSON parsed json data
 *  \param command - pointer to a cmd_request_t struct to populate
 *  \return ESP_OK or Error
 */
static esp_err_t handle_invoke_request(cJSON *json, cmd_request_t *command);

/**
 *  \brief This is the api response task. 
 *          It waits forever for an item to arrive in
 *          the command response queue, then assembles the json 
 *          response and sends it asyncronously.
 *  \param args - unused
 */
static void api_response_task(void *args);



/************** RESPONSE TO JSON FUNCTIONS *********************/

static esp_err_t device_info_to_json_string(device_info_rsp_t *rsp_data, char *strbuffer, uint16_t buffer_len) {


    esp_err_t err = ESP_OK;
    /** object to add to **/
    cJSON *json = NULL;
    char *json_str = NULL;
    cJSON *name = NULL;
    cJSON *periph_n = NULL;
    cJSON *plist = NULL;
    cJSON *dev_id = NULL;
    cJSON *rsp_type = NULL;

    json = cJSON_CreateObject();

    if(json == NULL) {
        err = ESP_ERR_NO_MEM;
    }

    if(!err) {
        plist = cJSON_CreateArray();
        if(plist != NULL) {
            cJSON_AddItemToObject(json, "periph_ids", plist);

            for(uint8_t i=0; i< rsp_data->num_periphs && err == ESP_OK; i++) {
                /** add param id's **/
                cJSON *id = cJSON_CreateNumber((double)rsp_data->periph_ids[i]);
                if(id != NULL) {
                    cJSON_AddItemToArray(plist, id);
                } else {
                    err = ESP_ERR_NO_MEM;
                }
            }
        }
        else {
            err = ESP_ERR_NO_MEM;
        }
    }

    if(!err) {
        rsp_type = cJSON_CreateNumber((double )RSP_TYPE_DEV_INFO);
        if(rsp_type != NULL) {
            cJSON_AddItemToObject(json, "rsp_type", rsp_type);
        } else {
            err = ESP_ERR_NO_MEM;
        }
    }

    if(!err) {
        name = cJSON_CreateString(rsp_data->name);
        if(name != NULL) {
            cJSON_AddItemToObject(json, "name", name);
        } else {
            err = ESP_ERR_NO_MEM;
        }
    }

    if(!err) {
        periph_n = cJSON_CreateNumber((double)rsp_data->num_periphs);
        if(periph_n != NULL) {
            cJSON_AddItemToObject(json, "periph_num", periph_n);
        } else {
            err = ESP_ERR_NO_MEM;
        }
    }

    if(!err) {
        dev_id = cJSON_CreateNumber((double)rsp_data->device_id);
        if(dev_id != NULL) {
            cJSON_AddItemToObject(json, "dev_id", dev_id);
        } else {
            err = ESP_ERR_NO_MEM;
        }
    }

    if(err != ESP_OK) {
        ESP_LOGE(API_TAG, "Error creating Device info JSON");
    } else {
        json_str = cJSON_Print(json);
        if(json_str != NULL) {
            if(strlen(json_str) > buffer_len) {
                ESP_LOGE(API_TAG, "Not enough buffer space to hold response!");
                err = ESP_ERR_NO_MEM;
            } 
            else {
                memcpy(strbuffer, json_str, strlen(json_str));
            }
        } else {
            ESP_LOGE(API_TAG, "Error in making json string");
            err = ESP_ERR_NO_MEM;
        }
    }

    cJSON_Delete(json);

    return err;

}


static esp_err_t param_info_to_json_string(param_info_rsp_t *rsp_data, char *strbuffer, uint16_t buffer_len) {

    esp_err_t err = ESP_OK;

    cJSON *json = NULL;
    char *json_str = NULL;
    cJSON *name = NULL;
    cJSON *periph_id = NULL;
    cJSON *param_id = NULL;
    cJSON *p_max = NULL;
    cJSON *methods = NULL;
    cJSON *d_type = NULL;
    cJSON *rsp_type= NULL;

    json = cJSON_CreateObject();

    if(json == NULL) {
        err = ESP_ERR_NO_MEM;
    } 

    if(!err) {
        /** add param name **/
        name = cJSON_CreateString(rsp_data->name);
        if(name != NULL) {
            cJSON_AddItemToObject(json, "param_name", name);
        } else {
            err = ESP_ERR_NO_MEM;
        }
    }

    if(!err) {

        rsp_type = cJSON_CreateNumber((double )RSP_TYPE_PARAM_INFO);
        if(rsp_type != NULL) {
            cJSON_AddItemToObject(json, "rsp_type", rsp_type);
        } else {
            err = ESP_ERR_NO_MEM;
        }
    }

    if(!err) {
        /** add periph id **/
        periph_id = cJSON_CreateNumber((double)rsp_data->periph_id);
        if(periph_id != NULL) {
            cJSON_AddItemToObject(json, "periph_id", periph_id);
        } else {
            err = ESP_ERR_NO_MEM;
        }
    }

    if(!err) {
        /** add param id **/
        param_id = cJSON_CreateNumber((double)rsp_data->param_id);
        if(param_id != NULL) {
            cJSON_AddItemToObject(json, "param_id", param_id);
        } else {
            err = ESP_ERR_NO_MEM;
        }
    }

    if(!err) {
        /** add param max **/
        p_max = cJSON_CreateNumber((double)rsp_data->max_value);
        if(p_max != NULL) {
            cJSON_AddItemToObject(json, "param_max", p_max);
        } else {
            err = ESP_ERR_NO_MEM;
        }
    }

    if(!err) {
        /** add methods (flags) **/
        methods = cJSON_CreateNumber((double)rsp_data->cmd_type);
        if(methods != NULL && err == ESP_OK) {
            cJSON_AddItemToObject(json, "methods", methods);
        } else {
            err = ESP_ERR_NO_MEM;
        }
    }

    if(!err) {
        /** add data_type **/
        d_type = cJSON_CreateNumber((double)rsp_data->value_type);
        if(d_type != NULL) {
            cJSON_AddItemToObject(json, "data_type", d_type);
        } else {
            err = ESP_ERR_NO_MEM;
        }
    }

    if(err != ESP_OK) {
        ESP_LOGE(API_TAG, "Error creating Param JSON");
    } else {
        json_str = cJSON_Print(json);
        if(json_str != NULL) {
            if(strlen(json_str) > buffer_len) {
                ESP_LOGE(API_TAG, "Not enough buffer space to hold response!");
                err = ESP_ERR_NO_MEM;
            } 
            else {
                memcpy(strbuffer, json_str, strlen(json_str));
            }
        } else {
            ESP_LOGE(API_TAG, "Error in making json string");
            err = ESP_ERR_NO_MEM;
        }
    }

    if(json != NULL) {
        cJSON_Delete(json);
    }

    return err;
}


static esp_err_t periph_info_to_json_string(periph_info_rsp_t *rsp_data, char *strbuffer, uint16_t buffer_len) {

    esp_err_t err = ESP_OK;
    /** object to add to **/
    cJSON *json = NULL;
    char *json_str = NULL;
    cJSON *name = NULL;
    cJSON *param_n = NULL;
    cJSON *plist = NULL;
    cJSON *periph_id = NULL;
    cJSON *periph_t = NULL;
    cJSON *rsp_type = NULL;

    json = cJSON_CreateObject();

    if(json == NULL) {
        err = ESP_FAIL;
    }

    if(!err) {
        plist = cJSON_CreateArray();
        if(plist != NULL) {
            cJSON_AddItemToObject(json, "param_ids", plist);

            for(uint8_t i=0; i< rsp_data->param_num && err == ESP_OK; i++) {
                /** add param id's **/
                cJSON *id = cJSON_CreateNumber((double)rsp_data->param_ids[i]);
                if(id != NULL) {
                    cJSON_AddItemToArray(plist, id);
                } else {
                    err = ESP_ERR_NO_MEM;
                }
            }
        }
        else {
            err = ESP_ERR_NO_MEM;
        }
    }

    if(!err) {
        rsp_type = cJSON_CreateNumber((double )RSP_TYPE_PERIPH_INFO);
        if(rsp_type != NULL) {
            cJSON_AddItemToObject(json, "rsp_type", rsp_type);
        } else {
            err = ESP_ERR_NO_MEM;
        }
    }

    if(!err) {
        name = cJSON_CreateString(rsp_data->name);
        if(name != NULL && err == ESP_OK) {
            cJSON_AddItemToObject(json, "name", name);
        } else {
            err = ESP_ERR_NO_MEM;
        }
    }

    if(!err) {
        param_n = cJSON_CreateNumber((double)rsp_data->param_num);
        if(param_n != NULL) {
            cJSON_AddItemToObject(json, "param_num", param_n);
        } else {
            err = ESP_ERR_NO_MEM;
        }
    }

    if(!err) {
        periph_t = cJSON_CreateNumber((double)rsp_data->periph_type);
        if(periph_t != NULL) {
            cJSON_AddItemToObject(json, "periph_type", periph_t);
        } else {
            err = ESP_ERR_NO_MEM;
        }
    }
    
    if(!err) {
        periph_id = cJSON_CreateNumber((double)rsp_data->periph_id);
        if(periph_id != NULL) {
            cJSON_AddItemToObject(json, "periph_id", periph_id);
        } else {
            err = ESP_ERR_NO_MEM;
        }
    }

    if(err != ESP_OK) {
        ESP_LOGE(API_TAG, "Error creating Peripheral JSON");
    } else {
        json_str = cJSON_Print(json);
        if(json_str != NULL) {
            if(strlen(json_str) > buffer_len) {
                ESP_LOGE(API_TAG, "Not enough buffer space to hold response!");
                err = ESP_ERR_NO_MEM;
            } 
            else {
                memcpy(strbuffer, json_str, strlen(json_str));
            }
        } else {
            ESP_LOGE(API_TAG, "Error in making json string");
            err = ESP_ERR_NO_MEM;
        }
    }

    if(json != NULL) {
        cJSON_Delete(json);
    }

    return err;
}


static esp_err_t ok_rsp_to_json_string(ack_rsp_t *rsp_data, char *strbuffer, uint16_t buffer_len) {

    esp_err_t err = ESP_OK;
    /** object to add to **/
    cJSON *json = NULL;
    char *json_str = NULL;
    cJSON *param_id = NULL;
    cJSON *periph_id = NULL;
    cJSON *rsp_type = NULL;
    cJSON *opt = NULL;

    json = cJSON_CreateObject();

    if(json == NULL) {
        err = ESP_FAIL;
    }

    if(!err) {
        rsp_type = cJSON_CreateNumber((double )RSP_TYPE_ACK);
        if(rsp_type != NULL) {
            cJSON_AddItemToObject(json, "rsp_type", rsp_type);
        } else {
            err = ESP_ERR_NO_MEM;
        }
    }

    if(!err) {
        param_id = cJSON_CreateNumber((double)rsp_data->param_id);
        if(param_id != NULL) {
            cJSON_AddItemToObject(json, "param_id", param_id);
        } else {
            err = ESP_ERR_NO_MEM;
        }
    }

    if(!err) {
        periph_id = cJSON_CreateNumber((double)rsp_data->periph_id);
        if(periph_id != NULL) {
            cJSON_AddItemToObject(json, "periph_id", periph_id);
        } else {
            err = ESP_ERR_NO_MEM;
        }
    }

    if(!err) {
        opt = cJSON_CreateNumber((double)rsp_data->opt);
        if(opt != NULL) {
            cJSON_AddItemToObject(json, "opt", opt);
        } else {
            err = ESP_ERR_NO_MEM;
        }
    }

    if(err != ESP_OK) {
        ESP_LOGE(API_TAG, "Error creating Peripheral JSON");
    } else {
        json_str = cJSON_Print(json);
        if(json_str != NULL) {
            if(strlen(json_str) > buffer_len) {
                ESP_LOGE(API_TAG, "Not enough buffer space to hold response!");
                err = ESP_ERR_NO_MEM;
            } 
            else {
                memcpy(strbuffer, json_str, strlen(json_str));
            }
        } else {
            ESP_LOGE(API_TAG, "Error in making json string");
            err = ESP_ERR_NO_MEM;
        }
    }

    cJSON_Delete(json);

    return err;
}


static esp_err_t error_rsp_to_json_string(const char *errmsg, uint8_t error_code, char *strbuffer, uint16_t bufflen) {

    esp_err_t err = ESP_OK;
    /** object to add to **/
    cJSON *json = NULL;
    char *json_str = NULL;
    cJSON *rsp_type = NULL;
    cJSON *err_msg = NULL;
    cJSON *err_code = NULL;

    json = cJSON_CreateObject();

    if(json == NULL) {
        err = ESP_ERR_NO_MEM;
    }

    if(!err) {
        rsp_type = cJSON_CreateNumber((double )RSP_TYPE_ERR);
        if(rsp_type != NULL) {
            cJSON_AddItemToObject(json, "rsp_type", rsp_type);
        } else {
            err = ESP_ERR_NO_MEM;
        }
    }

    if(!err) {
        err_msg = cJSON_CreateString(errmsg);
        if(err_msg != NULL) {
            cJSON_AddItemToObject(json, "error", err_msg);
        } else {
            err = ESP_ERR_NO_MEM;
        }
    }

    if(!err) {
        err_code = cJSON_CreateNumber((double)error_code);
        if(err_code != NULL) {
            cJSON_AddItemToObject(json, "err_code", err_code);
        } else {
            err = ESP_ERR_NO_MEM;
        }
    }

    if(err != ESP_OK) {
        ESP_LOGE(API_TAG, "Error creating Peripheral JSON");
    } else {
        json_str = cJSON_Print(json);
        if(json_str != NULL) {
            if(strlen(json_str) > bufflen) {
                ESP_LOGE(API_TAG, "Not enough buffer space to hold response!");
                err = ESP_ERR_NO_MEM;
            } else {
                memcpy(strbuffer, json_str, strlen(json_str));
            }
        } else {
            ESP_LOGE(API_TAG, "Error in making json string");
            err = ESP_ERR_NO_MEM;
        }
    }

    if(json != NULL) {
        cJSON_Delete(json);
    }


    return err;

}
 

static esp_err_t get_response_to_json_string(data_rsp_t *rsp_data, char *strbuffer, uint16_t buffer_len) {

    esp_err_t err = ESP_OK;
    /** object to add to **/
    cJSON *json = NULL;
    char *json_str = NULL;
    cJSON *rsp_type = NULL;
    cJSON *param_id = NULL;
    cJSON *periph_id = NULL;
    cJSON *data = NULL;
    cJSON *t_data = NULL;

    json = cJSON_CreateObject();

    if(json == NULL) {
        err = ESP_FAIL;
    }

    if(!err) {
        periph_id = cJSON_CreateNumber((double)rsp_data->periph_id);
        if(periph_id != NULL) {
            cJSON_AddItemToObject(json, "periph_id", periph_id);
        } else {
            err = ESP_ERR_NO_MEM;
        }
    }

    if(!err) {
        param_id = cJSON_CreateNumber((double)rsp_data->param_id);
        if(param_id != NULL) {
            cJSON_AddItemToObject(json, "param_id", param_id);
        } else {
            err = ESP_ERR_NO_MEM;
        }
    }

    if(!err) {
        rsp_type = cJSON_CreateNumber((double)RSP_TYPE_DATA);
        if(rsp_type != NULL) {
            cJSON_AddItemToObject(json, "rsp_type", rsp_type);
        } else {
            err = ESP_ERR_NO_MEM;
        }
    }

    if(!err) {
        switch (rsp_data->data_t)
        {
            case DATATYPE_BOOL:
            case DATATYPE_UINT8:
            case DATATYPE_UINT16:
            case DATATYPE_UINT32:
                data = cJSON_CreateNumber((double)rsp_data->data.uint_data);
                if(data != NULL) {
                    cJSON_AddItemToObject(json, "data", data);
                } else {
                    err = ESP_ERR_NO_MEM;
                }
                break;
            case DATATYPE_INT8:
            case DATATYPE_INT16:
            case DATATYPE_INT32:
                data = cJSON_CreateNumber((double)rsp_data->data.sint_data);
                if(data != NULL) {
                    cJSON_AddItemToObject(json, "data", data);
                } else {
                    err = ESP_ERR_NO_MEM;
                }
                break;
            case DATATYPE_FLOAT:
            case DATATYPE_DOUBLE:
                data = cJSON_CreateNumber((double)rsp_data->data.float_data);
                if(data != NULL) {
                    cJSON_AddItemToObject(json, "data", data);
                } else {
                    err = ESP_ERR_NO_MEM;
                }
                break;
            case DATATYPE_STRING:
                data = cJSON_CreateString(&rsp_data->data.str_data[0]);
                if(data != NULL) {
                    cJSON_AddItemToObject(json, "data", data);
                } else {
                    err = ESP_ERR_NO_MEM;
                }
                break;
            default:
                break;
        }
    }

    if(!err) {
        t_data = cJSON_CreateNumber((double)rsp_data->data_t);
        if(t_data != NULL) {
            cJSON_AddItemToObject(json, "data_type", t_data);
        } else {
            err = ESP_FAIL;
        }
    }

    if(err != ESP_OK) {
        ESP_LOGE(API_TAG, "Error creating Peripheral JSON");
    } 
    else {
        /** convert the assembled json to a string */
        json_str = cJSON_Print(json);
        if(json_str != NULL) {
            if(strlen(json_str) > buffer_len) {
                ESP_LOGE(API_TAG, "Not enough buffer space to hold response!");
                err = ESP_FAIL;
            } else {
                memcpy(strbuffer, json_str, strlen(json_str));
            }
        } else {
            ESP_LOGE(API_TAG, "Error in making json string");
            err = ESP_FAIL;
        }
    }

    /** this deletes all objects attached **/
    if(json != NULL) {
        cJSON_Delete(json);
    }

    return err;

}


/************ JSON REQUEST HANDLER FUNCTIONS *********************/


static esp_err_t handle_info_request(cJSON *json, cmd_request_t *command) {

    esp_err_t err = ESP_OK;
    char *fieldname;
    cJSON *periph_id = NULL;
    cJSON *param_id = NULL;

    fieldname = info_json_tags[0];
    ESP_LOGI(API_TAG, "Looking for %s...", fieldname);
    periph_id = cJSON_GetObjectItemCaseSensitive(json, fieldname);

    if(periph_id == NULL) {
        ESP_LOGE(API_TAG, "Error field %s missing", fieldname);
        err = API_ERR_MISSING_JSON_FIELD;
    }
    else if (!cJSON_IsNumber(periph_id)) {
        err = API_ERR_FIELD_NOT_NUMBER;
    }


    if(!err) {
        fieldname = info_json_tags[1];
        param_id = cJSON_GetObjectItemCaseSensitive(json, fieldname);

        if(param_id == NULL) {
            ESP_LOGE(API_TAG, "Error field %s missing", fieldname);
            err = API_ERR_MISSING_JSON_FIELD;
        }
        else if (!cJSON_IsNumber(param_id)) {
            err = API_ERR_FIELD_NOT_NUMBER;
        }
    }

    if(!err) {
        command->data.cmd_data.periph_id = (uint8_t)periph_id->valueint;
        command->data.cmd_data.param_id = (uint8_t)param_id->valueint;
        command->data.cmd_data.cmd_type = CMD_TYPE_INFO;
        command->data.cmd_data.data_t = DATATYPE_NONE;
        command->cmd_type = REQ_PKT_TYPE_PERIPH_CMD;
        /** TODO: Sort out a proper command UID system **/
        command->cmd_uid = 0;
    }

    return err;
}


static esp_err_t handle_get_request(cJSON *json, cmd_request_t *command) {


    esp_err_t err = ESP_OK;
    char *fieldname;
    cJSON *periph_id = NULL;
    cJSON *param_id = NULL;

    fieldname = get_json_tags[0];
    ESP_LOGI(API_TAG, "Looking for %s...", fieldname);
    periph_id = cJSON_GetObjectItemCaseSensitive(json, fieldname);

    if(periph_id == NULL) {
        ESP_LOGE(API_TAG, "Error field %s missing", fieldname);
        err = API_ERR_MISSING_JSON_FIELD;
    }
    else if (!cJSON_IsNumber(periph_id)) {
        err = API_ERR_FIELD_NOT_NUMBER;
    }


    if(!err) {
        fieldname = get_json_tags[1];
        param_id = cJSON_GetObjectItemCaseSensitive(json, fieldname);

        if(param_id == NULL) {
            ESP_LOGE(API_TAG, "Error field %s missing", fieldname);
            err = API_ERR_MISSING_JSON_FIELD;
        }
        else if (!cJSON_IsNumber(param_id)) {
            err = API_ERR_FIELD_NOT_NUMBER;
        }
    }

    if(!err) {
        command->data.cmd_data.periph_id = (uint8_t)periph_id->valueint;
        command->data.cmd_data.param_id = (uint8_t)param_id->valueint;
        command->data.cmd_data.cmd_type = CMD_TYPE_GET;
        command->data.cmd_data.data_t = DATATYPE_NONE;
        command->cmd_type = REQ_PKT_TYPE_PERIPH_CMD;
        /** TODO: Sort out a proper command UID system **/
        command->cmd_uid = 0;
    }

    return err;

}


static esp_err_t handle_set_request(cJSON *json, cmd_request_t *command) {
    esp_err_t err = ESP_OK;
    char *fieldname;
    cJSON *periph_id = NULL;
    cJSON *param_id = NULL;
    cJSON *data = NULL;
    cJSON *data_t = NULL;
    uint32_t copy_len;

    fieldname = info_json_tags[0];
    ESP_LOGI(API_TAG, "Looking for %s...", fieldname);
    periph_id = cJSON_GetObjectItemCaseSensitive(json, fieldname);

    if(periph_id == NULL) {
        ESP_LOGE(API_TAG, "Error field %s missing", fieldname);
        err = API_ERR_MISSING_JSON_FIELD;
    }
    else if (!cJSON_IsNumber(periph_id)) {
        err = API_ERR_FIELD_NOT_NUMBER;
    }


    if(!err) {
        fieldname = info_json_tags[1];
        param_id = cJSON_GetObjectItemCaseSensitive(json, fieldname);

        if(param_id == NULL) {
            ESP_LOGE(API_TAG, "Error field %s missing", fieldname);
            err = API_ERR_MISSING_JSON_FIELD;
        }
        else if (!cJSON_IsNumber(param_id)) {
            err = API_ERR_FIELD_NOT_NUMBER;
        }
    }

    if(!err) {
        fieldname = set_json_tags[2];
        data_t = cJSON_GetObjectItemCaseSensitive(json, fieldname);
        if(data_t == NULL) {
            ESP_LOGE(API_TAG, "Error field %s missing", fieldname);
            err = API_ERR_MISSING_JSON_FIELD;
        }

        else if (!cJSON_IsNumber(data_t)) {
            err = API_ERR_FIELD_NOT_NUMBER;
        }
    }

    if(!err) {
        fieldname = set_json_tags[3];
        data = cJSON_GetObjectItemCaseSensitive(json, fieldname);

        if(data == NULL) {
            ESP_LOGE(API_TAG, "Error field %s missing", fieldname);
            err = API_ERR_MISSING_JSON_FIELD;
        }
    }
    /** check the data_t field matches the data type */
    if(!err) {
        switch (data_t->valueint)
        {
            case DATATYPE_BOOL:
            case DATATYPE_UINT8:
            case DATATYPE_UINT16:
            case DATATYPE_UINT32:
            case DATATYPE_INT8:
            case DATATYPE_INT16:
            case DATATYPE_INT32:
            case DATATYPE_FLOAT:
            case DATATYPE_DOUBLE:
                if(!cJSON_IsNumber(data)) {
                    err = API_ERR_FIELD_NOT_NUMBER;
                }
                break;
            case DATATYPE_STRING:
                if(!cJSON_IsString(data)) {
                    err = API_ERR_FIELD_NOT_STRING;
                }
                break;
            default:
                err = API_ERR_INVALID_DATA_TYPE;
                break;
        }
    }

    if(!err) {
        /** set the data field depending on type **/
        switch (data_t->valueint)
        {
        case DATATYPE_FLOAT:
        case DATATYPE_DOUBLE:
            command->data.cmd_data.data.float_data = (float)data->valuedouble;
            break;
        case DATATYPE_STRING:
            copy_len = strlen(data->valuestring);
            copy_len = (copy_len > PERIPHERAL_MANAGER_MAX_STR_LEN-1 ?
                                   PERIPHERAL_MANAGER_MAX_STR_LEN-1 : copy_len);
            stpncpy(&command->data.cmd_data.data.str_data[0], data->valuestring, copy_len);
            break;
        case DATATYPE_INT8:
        case DATATYPE_INT16:
        case DATATYPE_INT32:
            command->data.cmd_data.data.sint_data = (int32_t)data->valueint;
            break;
        default:
            command->data.cmd_data.data.uint_data = (uint32_t)data->valueint;
            break;
        }
        command->data.cmd_data.periph_id = (uint8_t)periph_id->valueint;
        command->data.cmd_data.param_id = (uint8_t)param_id->valueint;
        command->data.cmd_data.data_t = (uint8_t)data_t->valueint;
        command->data.cmd_data.cmd_type = CMD_TYPE_SET;
        command->cmd_type = REQ_PKT_TYPE_PERIPH_CMD;
        /** TODO: Sort out a proper command UID system **/
        command->cmd_uid = 0;
    }

    return err;  
}


static esp_err_t handle_invoke_request(cJSON *json, cmd_request_t *command) {

    esp_err_t err = ESP_OK;
    char *fieldname;
    cJSON *periph_id = NULL;
    cJSON *param_id = NULL;

    fieldname = info_json_tags[0];
    ESP_LOGI(API_TAG, "Looking for %s...", fieldname);
    periph_id = cJSON_GetObjectItemCaseSensitive(json, fieldname);

    if(periph_id == NULL) {
        ESP_LOGE(API_TAG, "Error field %s missing", fieldname);
        err = API_ERR_MISSING_JSON_FIELD;
    }
    else if (!cJSON_IsNumber(periph_id)) {
        err = API_ERR_FIELD_NOT_NUMBER;
    }


    if(!err) {
        fieldname = info_json_tags[1];
        param_id = cJSON_GetObjectItemCaseSensitive(json, fieldname);

        if(param_id == NULL) {
            ESP_LOGE(API_TAG, "Error field %s missing", fieldname);
            err = API_ERR_MISSING_JSON_FIELD;
        }
        else if (!cJSON_IsNumber(param_id)) {
            err = API_ERR_FIELD_NOT_NUMBER;
        }
    }

    if(!err) {
        command->data.cmd_data.periph_id = (uint8_t)periph_id->valueint;
        command->data.cmd_data.param_id = (uint8_t)param_id->valueint;
        command->data.cmd_data.cmd_type = CMD_TYPE_ACT;
        command->data.cmd_data.data_t = DATATYPE_NONE;
        command->cmd_type = REQ_PKT_TYPE_PERIPH_CMD;
        /** TODO: Sort out a proper command UID system **/
        command->cmd_uid = 0;
    }

    if(periph_id != NULL) {
        cJSON_Delete(periph_id);
    }
    if(param_id != NULL) {
        cJSON_Delete(param_id);
    }
    return err;

}


/************* Response Handler Task ***************************/

static void api_response_task(void *args) {

    /**
     *  This task waits for an item from the command response queue
     *  and transmits a json response to the http client, before closing the 
     *  session.
     * 
     *  Yay, async http works!
     *  http_server raises an error when it expects the connection but it
     *  has already been closed. I don't think httpd_sess_delete_invalid 
     *  closes anything. No debug anyway. 
     * 
     *  This is hacky but it shouldn't have to be.
     * 
     *  TODO: Investigate httpd error further
     * 
     ***/

    esp_err_t err = ESP_OK;
    cmd_rsp_t command_response = {0};
    struct async_resp_arg *resp_arg = NULL;
    struct sock_db *sess = NULL;
    httpd_handle_t hd = 0;
    int fd = 0;
    httpd_req_t *req = NULL;

    char header_buff[API_MAX_HDR_RESPONSE_LEN] = {0};
    char response[API_MAX_RESPONSE_LEN] = {0};

    /** wait for queue to be created **/
    while(respondq == NULL) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    while(1) {

        /** reset the error status **/
        err = ESP_OK;

        /** wait forever for item from response queue **/
        if(xQueueReceive(respondq, &command_response, portMAX_DELAY) != pdTRUE) {
            ESP_LOGE(API_TAG, "Error in reading from queue");
            err = API_ERR_CMD_TIMEOUT;
        }

#ifdef DEBUG_MODE
        printf("Got PM Queue item...\n");
#endif /** DEBUG_MODE **/

        if(!err)
        {
            /** create the json response from command response **/
            switch (command_response.rsp_type)
            {
            case RSP_TYPE_ERR:
                err = error_rsp_to_json_string(&command_response.rsp_data.error.err_message[0],
                                         command_response.rsp_data.error.error_code,
                                         response,
                                         API_MAX_RESPONSE_LEN-1
                                        );
                break;
            
            case RSP_TYPE_ACK:
                err = ok_rsp_to_json_string(&command_response.rsp_data.ack, response, API_MAX_RESPONSE_LEN-1);
                break;
            case RSP_TYPE_DATA:
                err = get_response_to_json_string(&command_response.rsp_data.data, response, API_MAX_RESPONSE_LEN-1);
                break;
            case RSP_TYPE_DEV_INFO:
                err = device_info_to_json_string(&command_response.rsp_data.dev_info, response, API_MAX_RESPONSE_LEN-1);
                break;
            case RSP_TYPE_PERIPH_INFO:
                err = periph_info_to_json_string(&command_response.rsp_data.periph_info, response, API_MAX_RESPONSE_LEN-1);
                break;
            case RSP_TYPE_PARAM_INFO:
                err = param_info_to_json_string(&command_response.rsp_data.param_info, response, API_MAX_RESPONSE_LEN-1);
                break;
            default:
                ESP_LOGE(API_TAG, "Error: Invalid RSP type");
                err = error_rsp_to_json_string("PeriphManager Error",
                                                88,
                                                response,
                                                API_MAX_RESPONSE_LEN-1
                                                );
                break;

            if(err) {
                /** failed to assemble json response or add/receive from queue **/
                ESP_LOGE(API_TAG, "Error creating JSON response!!");
            }
        }
    }

        /** 
         * Prepare the response & header buffers
         **/
    if(!err) {

        resp_arg = command_response.rsp_args;
        hd = resp_arg->hd;
        fd = resp_arg->fd;

#ifdef DEBUG_MODE
            ESP_LOGI(API_TAG, "Sending response");
#endif /* DEBUG */
            
        req = (httpd_req_t *)httpd_sess_get_ctx(hd, fd);

        if (!req) {
            ESP_LOGE(API_TAG, "Error getting session");
            err = ESP_ERR_INVALID_ARG;
        }
        else {
            ESP_LOGI(API_TAG, "Got session!");
        }

        // /** copy the headers into the header buffer **/
        // if(snprintf(header_buff, API_MAX_HDR_RESPONSE_LEN, http_ok_hdr_str, strlen(response)) > API_MAX_HDR_RESPONSE_LEN) {
        //     ESP_LOGE(API_TAG, "Error http header too long");
        //     err = ESP_ERR_INVALID_ARG;
        // }
#ifdef DEBUG_MODE
        // ESP_LOGI(API_TAG, "Sending headers");
        // ESP_LOGI(API_TAG, "headers: %s", header_buff);
#endif // DEBUG_MODE
    }

    if(!err) {
        err = httpd_resp_send(req, response, strlen(response));
    
        if(err) {
            ESP_LOGE(API_TAG, "Error sending response! [%u]", err);
        }
    }
    

        /*********** Return Response ************/
        /** At this point we should have a response ready in all situations 
         *  set the http response type to json
         *  and send the response with the json data
         **/
//         if(!err) {
//             /** send the http headers **/
//             httpd_socket_send(hd, fd, (const char *)header_buff, strlen(header_buff), 0) < 0; etc...
//              if (sess->send_fn(hd, fd, (const char *)header_buff, strlen(header_buff), 0) < 0) {
//                 ESP_LOGE(API_TAG, "Failed to send HTTP response header");
//             }
//             /** send crlf sep **/
//             if(sess->send_fn(hd, fd, cr_lf_seperator, strlen(cr_lf_seperator), 0) < 0) {
//                 ESP_LOGE(API_TAG, "Failed to send cr lf data");
//             }
            
// #ifdef DEBUG_MODE 
//             printf("Content length %u\n", strlen(response));
//             ESP_LOGI(API_TAG, "Content: %s", response);
//             ESP_LOGI(API_TAG, "Sending Content");
// #endif // DEBUG_MODE
//             /** send the response data **/
//             if(sess->send_fn(hd, fd, (const char *)response, strlen(response), 0) < 0) {
//                 ESP_LOGE(API_TAG, "Failed to send HTTP response data");
//             }

//             /** Close the connection **/
//             httpd_sess_trigger_close(hd, fd);

//             httpd_sess_delete(hd, req);


        /** clear the message buffers **/
        memset(response, 0, sizeof(uint8_t) * API_MAX_RESPONSE_LEN);
        memset(header_buff, 0, sizeof(uint8_t) * API_MAX_HDR_RESPONSE_LEN);

        /** free the async resources **/
        if(resp_arg) {
            ESP_LOGI(API_TAG, "Freeing args");
            heap_caps_free(resp_arg);
        }
    }

    /** Here be Dragons **/
}


#ifdef CONFIG_ENABLE_STREAM

/** Stream handler **/
esp_err_t stream_handler(httpd_req_t *req) {

    // httpd_ws_frame_t ws_pkt = {0};
    // httpd_ws_frame_t ws_rsp = {0};
    // cJSON *cmd_json = NULL;
    // cJSON *cmd_element = NULL;
    // cJSON *array_val = NULL;
    // uint8_t wsbuffer[API_MANAGER_MAX_WS_FRAME_SIZE] = {0};

    // /** this handles the initial websocket request **/
    // if (req->method == HTTP_GET) {
    //     ESP_LOGI(API_TAG, "Handshake done, the new connection was opened");
    //     return ESP_OK;
    // }

    // ws_pkt.payload = wsbuffer;

    // esp_err_t err = httpd_ws_recv_frame(req, &ws_pkt, API_MANAGER_MAX_WS_FRAME_SIZE);
    // if (err != ESP_OK) {
    //     ESP_LOGE(API_TAG, "httpd_ws_recv_frame failed to get frame");
    //     return err;
    // }
    // else {
    //     ESP_LOGI(API_TAG, "Got Packet of len %u with type: %d", ws_pkt.len, ws_pkt.type);
    // }

    // if(!err) {
    //     /** CONTINUE PACKET **/
    //     if (ws_pkt.type == HTTPD_WS_TYPE_CONTINUE && stream_is_active()) {
    //         stream_kick_timer();
    //         return ESP_OK;
    //     }
    //     /** CLOSE PACKET **/
    //     else if(ws_pkt.type == HTTPD_WS_TYPE_CLOSE && stream_is_active()) {
    //         stream_signal_stop();
    //         return ESP_OK;
    //     }
    //     /** PING PACKET **/
    //     else if(ws_pkt.type == HTTPD_WS_TYPE_PING && stream_is_active()) {
    //         stream_send_pong();
    //         return ESP_OK;
    //     }
    //     /** NEW STREAM !!! **/
    //     else if(!err && ws_pkt.type == HTTPD_WS_TYPE_TEXT && !stream_is_active()) {
    //         /** UNPACK PAYLOAD **/
    //         ESP_LOGI(API_TAG, "Starting new stream....");
    //         uint8_t rspbuffer[API_MAX_RESPONSE_LEN] = {0};
    //         cmd_rsp_t rsp = {0};
    //         cmd_request_t strm_req = {0};
    //         stream_cmd_t strm_data = {0};


    //         cmd_json = cJSON_Parse((char *)wsbuffer);
            
    //         if(cmd_json == NULL) {
    //             err = ESP_ERR_INVALID_RESPONSE;
    //             ESP_LOGE(API_TAG, "Error parsing json!");
    //         }
    //         /** GO THROUGH TAGS **/
    //         if(!err) {
    //             for(uint8_t i=0; i < API_JSON_STREAM_TAGS; i++) {
    //                 cmd_element = cJSON_GetObjectItemCaseSensitive(cmd_json, stream_json_tags[i]);
    //                 if(cmd_element == NULL) {
    //                     ESP_LOGE(API_TAG, "Error: Missing tag - %s", stream_json_tags[i]);
    //                     err = ESP_ERR_INVALID_ARG;
    //                     error_rsp_to_json_string("Error: missing tag", 100, (char *)rspbuffer, API_MAX_RESPONSE_LEN-1);
    //                     break;
    //                 }

    //                 else {
    //                     switch (i)
    //                     {
    //                     case 0:
    //                         strm_data.periph_id = cmd_element->valueint;
    //                         break;
    //                     case 1:
    //                         if(!cJSON_IsArray(cmd_element)) {
    //                             ESP_LOGE(API_TAG, "Error in json");
    //                             error_rsp_to_json_string("Error: param_ids should be an array", 100, (char *)rspbuffer, API_MAX_RESPONSE_LEN-1);                  
    //                             err = ESP_FAIL;
    //                         } 
    //                         else if(cJSON_GetArraySize(cmd_element) > 6 || cJSON_GetArraySize(cmd_element) < 1) 
    //                         {
    //                             ESP_LOGE(API_TAG, "Error in json");
    //                             error_rsp_to_json_string("Error: param_ids invalid length (1-6)", 100, (char *)rspbuffer, API_MAX_RESPONSE_LEN-1);                  
    //                             err = ESP_FAIL;
    //                         }
    //                         else {
    //                             uint8_t arr_len = cJSON_GetArraySize(cmd_element);
    //                             ESP_LOGI(API_TAG,"Length of param list is %u", arr_len);
    //                             strm_data.param_num = arr_len;
    //                             for(uint8_t i=0; i< arr_len; i++) {
    //                                 array_val = cJSON_GetArrayItem(cmd_element, i);
    //                                 if(array_val == NULL) {
    //                                     ESP_LOGE(API_TAG, "Error in unpacking param ids");
    //                                     error_rsp_to_json_string("Error decoding JSON!", 100, (char *)rspbuffer, API_MAX_RESPONSE_LEN-1);                  
    //                                     err = ESP_FAIL;                                    
    //                                     break;
    //                                 }
    //                                 else {
    //                                     strm_data.param_ids[i] = (uint8_t)array_val->valuedouble;
    //                                     ESP_LOGI(API_TAG,"Set element %u to %u", i, strm_data.param_ids[i]);
    //                                 }
    //                             }
    //                         }
    //                         break;
    //                     case 2:
    //                         strm_data.rate = cmd_element->valueint;
    //                         break;
    //                     case 3:
    //                         strm_data.stream_type = cmd_element->valueint;
    //                         break;
    //                     default:
    //                         break;
    //                     }
    //                 }
    //             }
    //             /** CLEAN UP THE MEM **/
    //             if(cmd_json != NULL) {
    //                 cJSON_free(cmd_json);
    //             }
    //         }
            
    //         /** Send the stream request to the PM **/
    //         if(err == ESP_OK) {

    //             /** add the info needed to use async send **/
    //             stream->sockfd = httpd_req_to_sockfd(req);
    //             stream->server = req->handle;
    //             stream->stream_type = strm_data.stream_type;
    //             strm_req.data.strm_data = strm_data;
    //             strm_req.type = REQ_PKT_TYPE_STREAM;
    //             strm_req.cmd_uid = 100;

    //             if(xQueueSend(command_queue, &strm_req, API_MANAGER_QUEUE_PUT_TIMEOUT) != pdTRUE) {
    //                 ESP_LOGI(API_TAG, "Error in writing to queue");            
    //                 err = ESP_FAIL;
    //             }
    //             if(xQueueReceive(respondq, &rsp, API_MANAGER_QUEUE_GET_TIMEOUT) != pdTRUE) {
    //                 ESP_LOGE(API_TAG, "Error in reading from queue");
    //                 err = ESP_FAIL;
    //             }
    //             else 
    //             {
    //                 /** json the response & send **/
    //                 if(rsp.rsp_data.rsp_type == RSP_TYPE_ERR) {
    //                     // error_rsp_to_json_string(rsp.rsp_data.ustring, rsp.rsp_data.data, response, API_MAX_RESPONSE_LEN-1);
    //                     ESP_LOGI(API_TAG, "Periph manager says no...");
    //                     err = ESP_FAIL;
    //                     error_rsp_to_json_string(rsp.rsp_data.ustring, 100, (char *)rspbuffer, API_MAX_RESPONSE_LEN);
    //                 }
    //                 else {

    //                     /** TODO: Make sure OK response
    //                      *        Get the timer value from rsp.data     
    //                      *        Store the req->handle, sock_id = get_socket_from_req()    x
    //                      *        Timer value, timer handle, circular wsbuffer pointer/handle x
    //                      *        etc.
    //                      *        Start the timer, send an ok response? Optional
    //                      *        Make timer callback 
    //                      *        Pass pointer to stream info as argument 
    //                      *        read data from circular wsbuffer
    //                      *        add ws packet to httpd work queue 
    //                      *              (how to deal with lag/slowdown?)
    //                      **/
    //                     ESP_LOGI("Stream", "Stream established!");
    //                 }
    //             }
    //         }
    //         /** SEND THE WS RESPONSE **/
    //         if(err ) {
    //             ws_rsp.final = true;
    //             ws_rsp.type = HTTPD_WS_TYPE_TEXT;
    //             ws_rsp.payload = rspbuffer;
    //             ws_rsp.len = strlen((char *)rspbuffer);

    //             httpd_ws_send_frame(req, &ws_rsp);
    //             return ESP_OK;
    //         }
    //         else {
    //             ok_rsp_to_json_string(&rsp.rsp_data.ack, (char *)rspbuffer, API_MAX_RESPONSE_LEN);
                
    //             ws_rsp.final = true;
    //             ws_rsp.type = HTTPD_WS_TYPE_TEXT;
    //             ws_rsp.payload = rspbuffer;
    //             ws_rsp.len = strlen((char *)rspbuffer);

    //             httpd_ws_send_frame(req, &ws_rsp);
    //             return ESP_OK;
    //         }
    //     }
    // }

    return ESP_OK;
}

#endif


httpd_free_ctx_fn_t free_ctx(void *ctx) {
    heap_caps_free(ctx);
    return NULL;
}

/********************** URL HANDLERS *****************/

esp_err_t cmd_post_handler(httpd_req_t *req) {
    // HTTP Receive packet

    // ~~~ RxTask ~~~
    // Does Packet contain json data?
    //     - Y: Continue
    //     - N: Break
    // Is Json data valid?
    //     - Y: Cont.
    //     - N: Invalid request response
    // Extract cmd_type
    //     - Y: Process request
    //     - N: Missing field
    // Send to handler
    //      - Ok: Build command request
    //      - Err: error rsp
    // Pass Request to PM Queue

    esp_err_t err = ESP_OK;
    int16_t bytes_rcvd = 0;
    cJSON *cmd_json = NULL;
    cJSON *cmd_type = NULL;
    cmd_request_t command_request = {0};

    char content[API_MANAGER_MAX_POST_SIZE] = {0};
    char response[API_MAX_RESPONSE_LEN] = {0};

    /******* Sanity Checks *******/
    if(req->content_len > API_MANAGER_MAX_POST_SIZE-1 || req->content_len < 1) {
        /** ERROR RSP Request invalid length */
        err = API_ERR_INVALID_JSON_LENGTH;
    }

    if(!err) {
        bytes_rcvd = httpd_req_recv(req, content, req->content_len);
    
        if(bytes_rcvd < 0) {
            /** Something went wrong! */
            httpd_resp_send_500(req);
            return (bytes_rcvd == HTTPD_SOCK_ERR_TIMEOUT ? ESP_ERR_TIMEOUT : ESP_ERR_INVALID_STATE);
        }
    }

    /******* parse data *********/
    if(!err) {
        cmd_json = cJSON_Parse((const char *)content);
        
        if(cmd_json == NULL) {
            ESP_LOGI(API_TAG, "Error unpacking json");
            err = API_ERR_BAD_JSON;
        }
    }

    ESP_LOGI(API_TAG, "Err 1: %u", err);

    if(!err) {
        cmd_type = cJSON_GetObjectItemCaseSensitive(cmd_json, "cmd_type");

        if(cmd_type == NULL) {
            err = API_ERR_MISSING_JSON_FIELD;
        }
        else if (!cJSON_IsNumber(cmd_type))
        {
            err = API_ERR_FIELD_NOT_NUMBER;
        }
    }

    ESP_LOGI(API_TAG, "Err 2: %u", err);


    /********** Parse request *********/
    if(!err) {
        switch(cmd_type->valueint) {
            case CMD_TYPE_INFO:
                err = handle_info_request(cmd_json, &command_request);
                break;
            case CMD_TYPE_GET:
                err = handle_get_request(cmd_json, &command_request);
                break;
            case CMD_TYPE_SET:
                err = handle_set_request(cmd_json, &command_request);
                break;
            case CMD_TYPE_ACT:
                err = handle_invoke_request(cmd_json, &command_request);
                break;
            case CMD_TYPE_STREAM:
#ifdef CONFIG_STREAM_ENABLED
            ESP_LOGE(API_TAG, "Stream not developed yet");
#else
            ESP_LOGE(API_TAG, "Stream not enabled");
#endif /** CONFIG_STREAM_ENABLED **/
                break;
            default:
                break;
        }
    }


    if(cmd_json != NULL) {
        /* we're done with the cmd json, free it */
        cJSON_Delete(cmd_json);
    }

    /********** handle Errors **************/
    if(err) {
        switch (err)
        {
            case API_ERR_BAD_JSON:
                err = error_rsp_to_json_string("Invalid JSON", err, response, API_MAX_RESPONSE_LEN);
                break;
            case API_ERR_FIELD_NOT_NUMBER:
                err = error_rsp_to_json_string("Field is not a number", err, response, API_MAX_RESPONSE_LEN);
                break;
            case API_ERR_FIELD_NOT_STRING:
                err = error_rsp_to_json_string("String data not string", err, response, API_MAX_RESPONSE_LEN);
                break;
            case API_ERR_INVALID_DATA_TYPE:
                err = error_rsp_to_json_string("Invalid data_t value", err, response, API_MAX_RESPONSE_LEN);
                break;
            case API_ERR_MISSING_JSON_FIELD:
                err = error_rsp_to_json_string("Missing JSON field", err, response, API_MAX_RESPONSE_LEN);
                break;
            case API_ERR_INVALID_JSON_LENGTH:
                err = error_rsp_to_json_string("Invalid JSON length", err, response, API_MAX_RESPONSE_LEN);
                break;            
            default:
                err = error_rsp_to_json_string("Undefined Error", err, response, API_MAX_RESPONSE_LEN);
                break;
        }

        if(err) {
            /** error crafting the response, send internal server error response **/
            return httpd_resp_send_500(req);
        }
        else {
            /** send the json error response **/
            httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
        }

        /** set invalid state so the request doesn't get passed to the queue */
        err = ESP_ERR_INVALID_STATE;
    }

    if(!err) {
        /** copy the http information to the command request **/
        async_resp_arg_t *resp_arg = heap_caps_calloc(1, sizeof(async_resp_arg_t), MALLOC_CAP_8BIT);
        resp_arg->hd = req->handle;
        resp_arg->fd = httpd_req_to_sockfd(req);
        command_request.rsp_args = resp_arg;
        req->sess_ctx = resp_arg;
        req->free_ctx = free_ctx;
#ifdef DEBUG_MODE
        printf("Address of request Pre-Queue API side: [%p]\n", req);
        showmem((uint8_t *)req, 16);
#endif
        /************  Submit Request  *********/
        if(xQueueSend(command_queue, &command_request, API_MANAGER_QUEUE_PUT_TIMEOUT) != pdTRUE) {
            ESP_LOGI(API_TAG, "Error in writing to queue");            
            err = API_ERR_CMD_TIMEOUT;
        }
    }
    
    return err;
}





/************ ISR *********************/

/****** Private Data ******************/






/** Start the http server **/
static httpd_handle_t http_server_start()
{

    httpd_config_t httpdConf = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t serverHandle = NULL;

    if (httpd_start(&serverHandle, &httpdConf) == ESP_OK)
    {
        httpd_register_uri_handler(serverHandle, &cmdhandler);
#ifdef CONFIG_ENABLE_STREAM
        httpd_register_uri_handler(serverHandle, &streamhandler);
#endif
    }

    return serverHandle;
}

    /****** Global Functions *************/


esp_err_t api_manager_init()
{

    esp_err_t init_status = ESP_OK;
    httpd_handle_t server = http_server_start();
    TaskHandle_t apiTaskHandle = NULL;
    command_queue = NULL;

    command_queue = xQueueCreate(API_MAN_QUEUE_LEN, sizeof(cmd_request_t));

    if (command_queue == NULL)
    {
        ESP_LOGE(API_TAG, "Error creating command Queue!");
        init_status = ESP_ERR_INVALID_STATE;
    }
    else if (xTaskCreatePinnedToCore(api_response_task,
                                     "api_response_task",
                                     CONFIG_API_MANAGER_TASK_STACK,
                                     NULL,
                                     CONFIG_API_MANAGER_TASK_PRIORITY,
                                     &apiTaskHandle,
                                     1
                                    ) != pdTRUE)
    {
        ESP_LOGE(API_TAG, "Error creating api task");
        init_status = ESP_ERR_NO_MEM;
    }

    return init_status;
}
