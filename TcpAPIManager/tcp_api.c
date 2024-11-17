/**
*    @file    tcp_api.c
*
*    @brief    source file for tcp_api
*
*               Set up a listener on a port, await connection
*               API:
*                   - Start Frame: 0xAABBCCDD for now?
*                   - command: 8-bit
*                   - Periph id: 8-bit
*                   - Param id: 8-bit
*                   - data type: 8-bit
*                   - data len: 16-bit
*                   - data: N-bit
*                   - crc8: 8-bit
*
*                   - Packet Len: 6 bytes + N + 1 
*    @author    RJAM
*    @created   ven. 26 avril 2024 00:27:54 CEST
*/



/** Includes **/
#include "FreeRTOS.h"
#include "task.h"

#include "tcp_api.h"

/** Private Data **/

static message_buffer[TCP_API_MAX_MESSAGE_LEN] = {0};

/** Function Prototypes **/

/** Static Functions **/

/** Tasks **/

static void tcp_api_listener_task(void *args) {


    while(1) {
        vTaskDelay(pdMS_TO_TICKS(TCP_API_PORT_CHECK_PERIOD_MS))        
    }

    /** here be dragons **/
}


/** Public Functions **/


esp_err_t tcp_api_init(TCP_API_h handle, tcp_api_init_t *init) {

    esp_err_t err = ESP_OK;

    


    return err;
}

/** END **/
