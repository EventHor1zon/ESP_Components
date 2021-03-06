/***************************************
* \file     RotaryEncoder_Driver.c
* \brief    An ESP32 IDF / FreeRTOS driver for a standard incremental Rotary encoder
*           Driver is based on GPIO driver, ESP interrupts and (maybe) freertos timer
*           Also includes support for central button
* \date     July 2020
* \author   RJAM
****************************************/

/**
 *  TODO: Maybe add a short debounce to the rotary encoder?
 *        Do function Prototypes
 *        Maybe do button as separate component?
 *          
 **/

/********* Includes *******************/
#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "RotaryEncoder_Driver.h"
#include "Utilities.h"

/****** Function Prototypes ***********/

/****** Private Data ******************/

static rotaryEncoder_t reControl;

const char *RE_TAG = "RotaryEncoder:";

/************ ISR *********************/

static void IRAM_ATTR RE_DataInterrupt(void *args)
{

    BaseType_t pdHigherPrioWoken = pdFALSE;

    /** check debounce **/
    if (!(reControl.debounceState))
    {
        uint8_t direction = gpio_get_level(reControl.clockPinNum);
        uint8_t rising = gpio_get_level(reControl.dataPinNum);

        uint8_t rotation = direction + rising;
        uint32_t notification = 0;

        switch (rotation)
        {
        case 0:
        case 2:
            /** this was a counter-clockwise rotation */
            if (reControl.signedCounter)
            {
                reControl.count.Value = INCREMENT_TO_MAX(reControl.count.Value, reControl.counterMax);
            }
            else
            {
                reControl.count.uValue = INCREMENT_TO_MAX(reControl.count.uValue, reControl.counterMax);
            }

            if (reControl.alertStep && reControl.parentTask != NULL)
            {
                notification |= RE_NOTIFY_CC_STEP;
                xTaskNotifyFromISR(reControl.parentTask, notification, eSetValueWithOverwrite, &pdHigherPrioWoken);
            }
            break;

        case 1:
            if (reControl.signedCounter)
            {
                reControl.count.Value = DECREMENT_TO_MIN(reControl.count.Value, reControl.counterMin);
            }
            else
            {
                reControl.count.uValue = DECREMENT_TO_MIN(reControl.count.uValue, reControl.counterMin);
            }

            if (reControl.alertStep && reControl.parentTask != NULL)
            {
                notification |= RE_NOTIFY_CW_STEP;
                xTaskNotifyFromISR(reControl.parentTask, notification, eSetValueWithOverwrite, &pdHigherPrioWoken);
            }
            break;

        default:
            break;
        }

        /** start the debounce timer **/
        xTimerStartFromISR(reControl.debounceTimer, &pdHigherPrioWoken);
        reControl.debounceState = 1;
    }

    portYIELD_FROM_ISR();
};

void debounceExpireCallback(TimerHandle_t xTimer)
{
    reControl.debounceState = 0;
}

/****** Private Functions *************/

/****** Global Data *******************/

/****** Global Functions *************/
/**
 *  init function - initialise the pins & interrupts, 
 *                  populate the device struct
 **/

esp_err_t rotaryEncoderInit(gpio_num_t dataPin, gpio_num_t clockPin, bool installISR, TaskHandle_t parentTask)
{

    esp_err_t initStatus = ESP_OK;

    /** configure the pins **/
    gpio_config_t pinConfig;

    pinConfig.mode = GPIO_MODE_INPUT;
    pinConfig.pin_bit_mask = (1UL << dataPin); /** set unused pins to 0 **/
    pinConfig.intr_type = GPIO_INTR_POSEDGE;
    pinConfig.pull_down_en = 0;
    pinConfig.pull_up_en = 0;

    ESP_ERROR_CHECK(gpio_config(&pinConfig));
    if (initStatus == ESP_OK)
    {
        pinConfig.intr_type = GPIO_INTR_DISABLE;
        pinConfig.pin_bit_mask = clockPin;
        initStatus = gpio_config(&pinConfig);
    }
    else
    {
        ESP_LOGE(RE_TAG, "Error configuring data pin: 0x%x", initStatus);
    }

    /** configure the interrupts **/
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
            initStatus = gpio_isr_handler_add(dataPin, RE_DataInterrupt, &reControl);
        }
        if (initStatus != ESP_OK)
        {
            ESP_LOGE(RE_TAG, "Error adding data gpio isr handler: 0x%x", initStatus);
        }
    }

    TimerHandle_t timerHandle = xTimerCreate("reDebounceTmr", pdMS_TO_TICKS(20), pdFALSE, NULL, &debounceExpireCallback);

    /* build the control struct */
    if (initStatus == ESP_OK)
    {
        memset(&reControl, 0, sizeof(rotaryEncoder_t));
        reControl.clockPinNum = clockPin;
        reControl.dataPinNum = dataPin;
        reControl.stepSize = 1;
        reControl.counterMax = UINT16_MAX;
        reControl.count.uValue = (reControl.counterMax / 2); /** start the counter in middle of value range **/
        reControl.counterMin = 0;
        reControl.debounceTimer = timerHandle;

        if (parentTask)
        {
            reControl.alertStep = 1;
            reControl.parentTask = parentTask;
        }
    }

    if (initStatus)
    {
        ESP_LOGE(RE_TAG, "Error in initialising Rotary Encoder Driver");
    }
    else
    {
        ESP_LOGI(RE_TAG, "Succesfully intialised Rotary Encoder Driver!");
    }

    return initStatus;
}

esp_err_t rotaryEncoder_attachToTask(TaskHandle_t parentTask, bool alertStep)
{

    esp_err_t status = ESP_OK;

    if (reControl.parentTask != NULL)
    {
        ESP_LOGI(RE_TAG, "Replacing task handle...");
    }

    reControl.parentTask = parentTask;
    if (alertStep)
    {
        reControl.alertStep = true;
    }

    return status;
}

esp_err_t rotaryEncoder_setSignedCounter()
{

    esp_err_t status = ESP_OK;

    if (!(reControl.signedCounter))
    {
        reControl.signedCounter = true;
        reControl.count.Value = 0;
    }

    return status;
}

esp_err_t rotaryEncoder_getValue(uint16_t *value)
{
    if (reControl.signedCounter)
    {
        *value = reControl.count.Value;
    }
    else
    {
        *value = reControl.count.uValue;
    }

    return ESP_OK;
}

esp_err_t rotaryEncoder_getLastDir(uint8_t *lastDir)
{
    *lastDir = reControl.dirLast;
    return ESP_OK;
}

esp_err_t rotaryEncoder_setCounterMax(uint16_t countMax)
{

    esp_err_t setStatus = ESP_OK;

    if (reControl.signedCounter && countMax > INT16_MAX)
    {
        ESP_LOGE(RE_TAG, "Error: In signed mode, max value is %d", INT16_MAX);
        setStatus = ESP_ERR_INVALID_SIZE;
    }
    else
    {
        reControl.counterMax = countMax;
    }

    return setStatus;
}

esp_err_t rotaryEncoder_resetCounter()
{
    if (reControl.signedCounter)
    {
        reControl.count.Value = 0;
    }
    else
    {
        reControl.count.uValue = (reControl.counterMax / 2);
    }

    return ESP_OK;
}