/***************************************
* \file     PushButton_Driver.c
* \brief    A simple freeRtos / esp-idf component for a pushbutton interface
*           Uses FreeRTOS notifications with bits set as a lightweight way to
*           interact with a parent task. Note: these bits must be unique!
*           Also uses freertos software timers to debounce the button, leaving 
*           hardware timers for better use.
*           This driver uses esp-idf gpio interrupts and requires the isr service to be installed
*           in order to work correctly.
*
* \date     July 2020
* \author   RJAM
****************************************/

/********* Includes *******************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "PushButton_Driver.h"
#include "Utilities.h"

/****** Function Prototypes ***********/

/****** Private Data ******************/

static buttonData_t btnData;

const char *BTN_TAG = "Push Button Driver";

/************ ISR *********************/

static void IRAM_ATTR pushBtn_Isr(void *args)
{

    BaseType_t pdHigherPrioWoken;
    uint32_t notification = 0;
    uint8_t pinLevel = 0;

    pinLevel = gpio_get_level(btnData.btnPin);
    btnData.btnState = pinLevel;

    /** check debounce **/
    if (btnData.btnDebounceEnable && !(btnData.btnDebounceState))
    {
        if (pinLevel)
        { /** found a rising edge **/
            if (btnData.alertBtn)
            {
                notification |= (NOTIFY_BTN_UP);
                xTaskNotifyFromISR(btnData.parentTask, notification, eSetValueWithOverwrite, &pdHigherPrioWoken);
            }
        }
        else
        {
            if (btnData.alertBtn && btnData.halfBtnInterrupt)
            {
                notification |= (NOTIFY_BTN_DWN);
                xTaskNotifyFromISR(btnData.parentTask, notification, eSetValueWithOverwrite, &pdHigherPrioWoken);
            }
        }

        /* enable the debounce */
        if (btnData.btnDebounceEnable)
        {
            /** start timer, set debounce state **/
            xTimerStartFromISR(btnData.debounceTimer, &pdHigherPrioWoken);
            btnData.btnDebounceState = 1;
        }

        btnData.btnCount = INCREMENT_TO_MAX(btnData.btnCount, UINT16_MAX);
    }

    portYIELD_FROM_ISR();
};

void debounceExpireCallback(TimerHandle_t xTimer)
{
    btnData.btnDebounceState = 0;
}

/****** Private Functions *************/

/****** Global Data *******************/

/****** Global Functions *************/
esp_err_t pushBtn_Init(gpio_num_t btnPin, btn_config_t btnConfig, TaskHandle_t parentTask)
{

    esp_err_t initStatus;
    gpio_config_t pinConfig;

    switch (btnConfig)
    {
    case BTN_CONFIG_ACTIVELOW:
    case BTN_CONFIG_ACTIVEHIGH:
        pinConfig.pull_down_en = 0;
        pinConfig.pull_up_en = 0;
        break;

    case BTN_CONFIG_ACTIVELOW_PULLUP:
        pinConfig.pull_down_en = 0;
        pinConfig.pull_up_en = 1;
        break;

    case BTN_CONFIG_ACTIVEHIGH_PULLDOWN:
        pinConfig.pull_down_en = 1;
        pinConfig.pull_up_en = 0;
        break;
    default:
        ESP_LOGE(BTN_TAG, "Error: Invalid button config");
        initStatus = ESP_ERR_INVALID_ARG;
    }
    pinConfig.mode = GPIO_MODE_INPUT;
    pinConfig.pin_bit_mask = (1UL << btnPin); /** set unused pins to 0 **/
    pinConfig.intr_type = GPIO_INTR_ANYEDGE;

    if (initStatus == ESP_OK)
    {
        initStatus = gpio_config(&pinConfig);
    }

    if (initStatus == ESP_OK)
    {
        initStatus = gpio_isr_handler_add(btnPin, pushBtn_Isr, NULL);
    }
    if (initStatus != ESP_OK)
    {
        ESP_LOGE(BTN_TAG, "Error adding isr handler");
    }

    if (initStatus == ESP_OK)
    {
        /** initialise the btnData struct **/
        memset(&btnData, 0, sizeof(btnData));

        if (parentTask)
        {
            btnData.parentTask = parentTask;
            btnData.alertBtn = 1;
            btnData.halfBtnInterrupt = 0;
        }

        TimerHandle_t timerHandle = xTimerCreate("btnDebounceTmr", pdMS_TO_TICKS(20), pdFALSE, NULL, &debounceExpireCallback);

        btnData.debounceTimer = timerHandle;
        btnData.btnDebounceEnable = 1;
        btnData.tDebounce = BTN_DEFAULT_DEBOUNCE_T;
        btnData.btnPin = btnPin;
    }
    else
    {
        ESP_LOGE(BTN_TAG, "Error in initialising pushbutton : %u", initStatus);
    }

    return initStatus;
}

esp_err_t pushBtn_getButtonState(uint8_t *state)
{
    *state = (uint8_t)btnData.btnState;
    return ESP_OK;
}

esp_err_t pushBtn_getButtonPressT(uint32_t *btnPressT)
{
    *btnPressT = btnData.tBtnPress;
    return ESP_OK;
}

esp_err_t pushBtn_setDebounceTime(uint16_t dbTime)
{
    btnData.tDebounce = dbTime;
    return ESP_OK;
}

esp_err_t pushBtn_setHalfPressNotify(bool state)
{
    btnData.halfBtnInterrupt = state;
    return ESP_OK;
}
