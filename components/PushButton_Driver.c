/***************************************
* \file .c
* \brief
*
* \date
* \author
****************************************/

/********* Includes *******************/
#include <stdio.h>
#include "driver/gpio.h"

/****** Function Prototypes ***********/

/************ ISR *********************/

/****** Private Data ******************/

/****** Private Functions *************/

/****** Global Data *******************/

/****** Global Functions *************/
esp_err_t PushButtonInit(gpio_num_t btnPin, bool pullUp)
{

    esp_err_t initStatus;
    gpio_config_t pinConfig;

    pinConfig.mode = GPIO_MODE_INPUT;
    pinConfig.pin_bit_mask = (1UL << btnPin); /** set unused pins to 0 **/
    pinConfig.intr_type = GPIO_INTR_ANYEDGE;
    pinConfig.pull_down_en = 0;
    pinConfig.pull_up_en = (pullUp) ? 1 : 0;

    initStatus = gpio_config(&pinConfig);

    if (initStatus != ESP_OK)
    {
        ESP_LOGE(BTN_TAG, "Error in initialising GPIO : %u", initStatus);
    }

}