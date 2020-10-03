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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "http_server.h"

/****** Function Prototypes ***********/

/************ ISR *********************/

/****** Private Data ******************/

/****** Private Functions *************/

/****** Global Data *******************/

httpd_uri_t index_get = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = get_handler,
    .user_ctx = NULL};
/****** Global Functions *************/

httpd_handle_t httpServer_init()
{

    httpd_config_t httpdConf = HTTPD_DEFAULT_CONFIG();
    http_handle_t serverHandle = NULL;

    if (httpd_start(&serverHandle, &httpdConf) == ESP_OK)
    {
        httpd_register_uri_handler(server, &index_get);
    }
}
