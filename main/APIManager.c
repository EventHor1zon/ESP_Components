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

/********* Includes *******************/

#include "esp_err.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"

#include "freertos/task.h"
#include "freertos/queue.h"
#include "../inc/APIManager.h"
#include "../inc/PeripheralManager.h"
#include "../inc/CommandAPI.h"

/****** Function Prototypes ***********/

/************ ISR *********************/

/****** Private Data ******************/

/****** Global Data *******************/

const char *API_TAG = "API_Manager";

static const httpd_uri_t index_get = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = basic_get_handler,
    .user_ctx = NULL};

/****** Private Functions *************/
static void apiTask(void *args)
{
    ESP_LOGI("API MANAGER", "API Manager task begun");
    while (1)
    {
        /**  code  here **/
        //xTaskNotifyWait()
        vTaskDelay(1000);
    }
}

static httpd_handle_t http_server_start()
{

    httpd_config_t httpdConf = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t serverHandle = NULL;

    if (httpd_start(&serverHandle, &httpdConf) == ESP_OK)
    {
        httpd_register_uri_handler(serverHandle, &index_get);
    }

    return serverHandle;
}

/****** Global Functions *************/

esp_err_t basic_get_handler(httpd_req_t *req)
{
    esp_err_t status = ESP_OK;

    const char resp[] = "Hello World!";
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return status;
}

// esp_err_t api_manager_register_peripheral_routes(peripheral_t *periphs, uint8_t periph_num)
// {

//     esp_err_t status = ESP_OK;

//     for (uint8_t i = 0; i < periph_num; i++)
//     {
//     }

//     return status;
// }

esp_err_t api_manager_init()
{

    esp_err_t init_status = ESP_OK;
    httpd_handle_t server = http_server_start();
    TaskHandle_t apiTaskHandle = NULL;
    commandq = NULL;
    commandq = xQueueCreate(API_MAN_QUEUE_LEN, sizeof(cmd_request_t));

    if (commandq == NULL)
    {
        ESP_LOGE(API_TAG, "Error creating command Queue!");
        init_status = ESP_ERR_INVALID_STATE;
    }
    else if (xTaskCreate(apiTask, "apiTask", 5012, NULL, 2, &apiTaskHandle) != pdTRUE)
    {
        ESP_LOGE(API_TAG, "Error creating api task");
        init_status = ESP_ERR_NO_MEM;
    }
    // else if (api_manager_register_peripheral_routes(peripherals, periph_num) != ESP_OK)
    // {
    //     ESP_LOGE(API_TAG, "Error creating api task");
    //     init_status = ESP_ERR_NO_MEM;
    // }

    /** send basic commands to the PM here, test communication **/
    cmd_request_t periph_num_rq = {0};

    return init_status;
}