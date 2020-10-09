/****************************************
* \file
* \brief
* \date
* \author
****************************************/

#ifndef API_MANAGER_H
#define API_MANAGER_H

/********* Includes ********************/
#include "esp_http_server.h"
#include "esp_err.h"
/********* Definitions *****************/

#define API_MAN_QUEUE_LEN 10 /** < number of consecutive requests **/
#define API_MAN_QUEUE_ITEM_SIZE 20

/********** Types **********************/

/******** Function Definitions *********/
esp_err_t api_manager_init(void);

esp_err_t basic_get_handler(httpd_req_t *req);

#endif /* API_MANAGER_H */
