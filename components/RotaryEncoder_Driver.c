/***************************************
* \file RotaryEncoder_Driver.c
* \brief    An ESP32 IDF / FreeRTOS driver for a standard incremental Rotary encoder
*           Driver is based on GPIO driver, ESP interrupts and (maybe) freertos timer
*           Also includes support for central button
* \date     July 2020
* \author   RJAM
****************************************/

/********* Includes *******************/
#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "esp_err.h"
#include "esp_log.h"
#include "RotaryEncoder_Driver.h"

/****** Function Prototypes ***********/

/************ ISR *********************/

static void IRAM_ATTR RE_DataInterrupt(void *args){

};

static void IRAM_ATTR RE_BtnInterrupt(void *args){

};

/****** Private Data ******************/

static rotaryEncoder_t reControl;

const char *RE_TAG = "RotaryEncoder:";

/****** Private Functions *************/

/****** Global Data *******************/

/****** Global Functions *************/
/**
 *  init function - initialise the pins & interrupts, 
 *                  populate the device struct
 **/

esp_err_t rotaryEncoderInit(gpio_num_t dataPin, gpio_num_t clockPin, gpio_num_t btnPin, bool installISR, bool enableButton)
{

    esp_err_t initStatus = ESP_OK;

    gpio_config_t pinConfig;
    pinConfig.mode = GPIO_MODE_INPUT;
    pinConfig.pin_bit_mask = dataPin;
    pinConfig.intr_type = GPIO_INTR_ANYEDGE;
    pinConfig.pull_down_en = 0;
    pinConfig.pull_up_en = 0;

    initStatus = gpio_config(&pinConfig);
    if (initStatus == ESP_OK)
    {
        pinConfig.intr_type = GPIO_INTR_DISABLE;
        pinConfig.pin_bit_mask = clockPin;
        initStatus = gpio_config(&pinConfig);
    }

    if (initStatus == ESP_OK)
    {
        if (installISR)
        {
            /** driver will init the gpio isr service **/
            initStatus = gpio_install_isr_service(ESP_INTR_FLAG_LOWMED);
            if (initStatus != ESP_OK)
            {
                ESP_LOGE(RE_TAG, "Error installing ISR service: 0x%x", initStatus);
            }
        }
        if (initStatus == ESP_OK)
        {

            initStatus = gpio_isr_handler_add(dataPin, &RE_DataInterrupt, NULL);
        }
        if (initStatus != ESP_OK)
        {
            ESP_LOGE(RE_TAG, "Error adding data gpio isr handler: 0x%x", initStatus);
        }
        if (initStatus == ESP_OK && enableButton)
        {
            initStatus = gpio_isr_handler_add(btnPin, &RE_BtnInterrupt, NULL);
        }
    }

    memset(&reControl, 0, sizeof(rotaryEncoder_t));
    reControl.clockPinNum = clockPin;
    reControl.dataPinNum = dataPin;
    reControl.buttonEnabled = enableButton;
    reControl.stepSize = 1;
    reControl.count.uValue = (UINT16_MAX / 2); /** start the counter in middle of value range **/

    return initStatus;
}