/***************************************
* \file    CBuffer_Utils.c
* \brief    Utility functions for cbuffer - 
*           want to be able to assign large cbuffers as data storage
*           either over short or long periods of time. UIseful to have functions
*           to dump packets of accumulated data to console, SD card or other memory 
*           TODO: think about how to dump info - formatting, keep it generic to save 
*                   rewriting large amounts of awkward code
*               
* \date     Aug 2021
* \author   RJAM
****************************************/

/********* Includes *******************/
#include "esp_err.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "CircularBuffer.h"

/****** Function Prototypes ***********/

/************ ISR *********************/

/****** Private Data ******************/

/****** Private Functions *************/

/****** Global Data *******************/

/****** Global Functions *************/


esp_err_t dump_cbuffer_packets_string(CBuff handle) {

    esp_err_t err = ESP_OK;

    if(handle->use_packets) {


    }
    return err;
}
