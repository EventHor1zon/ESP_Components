/***************************************
* \file     Sim800L_Driver.c
* \brief    Driver for the Sim800L GPS/GPSR Data chip
*           Going to use uart on the esp for the first time, yay!
*           let's see how this goes
* \date     Aug 2021
* \author   RJAM
****************************************/

/********* Includes *******************/
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_heap_caps_init.h"

#include "SIM800L_Driver.h"

const char *SIM_TAG = "Sim800L Driver";

/****** Function Prototypes ***********/

/************ ISR *********************/

/****** Private Data ******************/

/****** Private Functions *************/

/****** Global Data *******************/

/****** Global Functions *************/


SIM800L_t sim800_init(sim800_init_t *init) {

    esp_err_t err = ESP_OK;
    uart_config_t ua_cfg = {0};
    SIM800L_t handle = NULL;

    if(init->port > 2) {
        ESP_LOGE(SIM_TAG, "Error: Invalid uart port");
        err = ESP_ERR_INVALID_ARG;
    }

    if(!err) {
        handle = heap_caps_calloc(1, sizeof(simdriver_t), MALLOC_CAP_DEFAULT);
        if(handle == NULL) {
            ESP_LOGE(SIM_TAG, "Error, unable to assign handle memory");
            err = ESP_ERR_NO_MEM;
        }
        else {
            handle->port = init->port;
            handle->tx = init->tx;
            handle->rx = init->rx;
        }
    }

    if(!err) {
        ua_cfg.baud_rate = 115200;
        ua_cfg.data_bits = UART_DATA_8_BITS;
        ua_cfg.flow_ctrl = 0;
        ua_cfg.parity = 0;
    }


    return handle;
}
