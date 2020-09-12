/****************************************
* \file     apaManager.h
* \brief    header for the api manager driver 
* \date     Aug 2020    
* \author   RJAM
****************************************/

#ifndef APA_MANAGER_H
#define APA_MANAGER_H

/********* Includes ********************/

#include "http_server.h"

/********* Definitions *****************/
/* URI handler structure for POST /uri */
httpd_uri_t index_get = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = get_handler,
    .user_ctx = NULL};
/********** Types **********************/

/******** Function Definitions *********/

httpd_handle_t httpServer_init()
{

    httpd_config_t httpdConf = HTTPD_DEFAULT_CONFIG();
    http_handle_t serverHandle = NULL;

    if (httpd_start(&serverHandle, &httpdConf) == ESP_OK)
    {
        httpd_register_uri_handler(server, &index_get);
    }
}
#endif /* APA_MANAGER_H */
