/***************************************
* \file     TouchSenseDriver.c
* \brief    A driver for a touch-sensor manager
*           Idea: Bool HW/SW, implement both for comparison
*           Hook up events for long & short presses
*           Really need to make some touch sensors first!
* \date     July 2021
* \author   RJAM
****************************************/

/********* Includes *******************/
#include "driver/touch_sensor.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"

#include "TouchSenseDriver.h"


const char *TSENSE_TAG = "TSENSE";

/****** Function Prototypes ***********/

/************ ISR *********************/

/****** Private Data ******************/

/****** Private Functions *************/

/****** Global Data *******************/

/****** Global Functions *************/


TSENSE_HANDLE touchsense_init(touchsense_init_t *ts_init) {

    gpio_num_t to_init[TOUCHSENSE_MAX_PINS] = {0};
    uint8_t numpins = 0;
    esp_err_t err = ESP_OK;

    err = touch_pad_init();

    if(err) {
        ESP_LOGE(TSENSE_TAG, "Error during touchpad init (%u)", err);
    }

    if(!err) {    
        for(int i=0; i<TOUCHSENSE_MAX_PINS; i++) {

            if(ts_init->touch_pins[i] > 0) {
                to_init[i] = ts_init->touch_pins[i];
                numpins++;
            }
        }
    
        for(int j=0; j<numpins && err == ESP_OK; j++) {

        }
    }
}