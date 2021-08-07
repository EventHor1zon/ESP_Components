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

const char *RE_TAG = "RotaryEncoder:";

/************ ISR *********************/

static void IRAM_ATTR RE_DataInterrupt(void *args)
{

    BaseType_t pdHigherPrioWoken = pdFALSE;
    rotaryEncoder_t *reControl = (rotaryEncoder_t *)args;
    /** check debounce **/
    if (!(reControl->debounceState))
    {
        uint8_t direction = gpio_get_level(reControl->clockPinNum);
        uint8_t rising = gpio_get_level(reControl->dataPinNum);

        uint8_t rotation = direction + rising;
        uint32_t notification = 0;

        if(rotation) {
            notification = RE_EVENT_CW_STEP;
        }
        else {
            notification = RE_EVENT_CC_STEP;
        }
        xTaskNotifyFromISR(reControl->parentTask, notification, eSetValueWithOverwrite, &pdHigherPrioWoken);

        /** start the debounce timer **/
        xTimerStartFromISR(reControl->debounceTimer, &pdHigherPrioWoken);
        reControl->debounceState = 1;
    }

    portYIELD_FROM_ISR();
};

void debounceExpireCallback(TimerHandle_t xTimer)
{
    rotaryEncoder_t *re = (rotaryEncoder_t *)pvTimerGetTimerID(xTimer);
    // uint32_t notify = RE_NOTIFY_DEBOUNCE_EXPIRED;
    re->debounceState = 0;
    /** need to do this? **/
    // xTaskNotify(re->parentTask, )
}

/****** Private Functions *************/

static void re_driver_task(void *args) {
 
    /** keep this driver event-based - interrupt pins raise a notify to 
     * this task, then task handles sending events depending on driver settings
     */
    rotaryEncoder_t *re = (rotaryEncoder_t *) args;

    uint32_t notify_value = 0;
#ifdef CONFIG_USE_EVENTS    
    uint32_t event_value = 0;
#endif

    while(1) {

        xTaskNotifyWait(0x00, 0x00, &notify_value, portMAX_DELAY);

        if(notify_value & RE_NOTIFY_CW_STEP) {
#ifdef CONFIG_USE_EVENTS    
            event_value = re->use_events ? RE_EVENT_CW_STEP : event_value;
#endif
            if(re->signedCounter) {
                re->count.Value = INCREMENT_TO_MAX(re->count.Value, re->counterMax);
            }
            else {
                re->count.uValue = INCREMENT_TO_MAX(re->count.uValue, re->counterMax);
            }
            ESP_LOGI(RE_TAG, "Got CW step - {%u}", re->count.Value);
        }
        
        else if(notify_value & RE_NOTIFY_CW_STEP) {
#ifdef CONFIG_USE_EVENTS    
            event_value = re->use_events ? RE_EVENT_CC_STEP : event_value;
#endif
            if(re->signedCounter) {
                re->count.Value = DECREMENT_TO_MIN(re->count.Value, re->counterMin);
            }
            else {
                re->count.uValue = DECREMENT_TO_MIN(re->count.uValue, re->counterMin);
            }
            ESP_LOGI(RE_TAG, "Got CC step - {%u}", re->count.Value);
        }
    
#ifdef CONFIG_USE_EVENTS    
        if(re->use_events && event_value) {
            esp_event_post_to(re->loop, PM_EVENT_BASE, event_value, NULL, 0, pdMS_TO_TICKS(100));
        }
#endif    
    }
    /** here be dragons **/
}



/****** Global Data *******************/

/****** Global Functions *************/
/**
 *  init function - initialise the pins & interrupts, 
 *                  populate the device struct
 **/

esp_err_t rotaryEncoderInit(gpio_num_t dataPin, gpio_num_t clockPin, bool installISR)
{

    esp_err_t initStatus = ESP_OK;

    /** configure the pins **/
    gpio_config_t pinConfig;
    TimerHandle_t timerHandle = NULL;
    TaskHandle_t parentTask;
    rotaryEncoder_t *reControl = NULL; 

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
            if (initStatus != ESP_OK)
            {
                ESP_LOGE(RE_TAG, "Error adding data gpio isr handler: 0x%x", initStatus);
            }
        }
    }

    /* build the control struct */
    if (initStatus == ESP_OK)
    {
        reControl = heap_caps_calloc(1, sizeof(rotaryEncoder_t), MALLOC_CAP_DEFAULT);
        reControl->clockPinNum = clockPin;
        reControl->dataPinNum = dataPin;
        reControl->stepSize = 1;
        reControl->counterMax = UINT16_MAX;
        reControl->count.uValue = (reControl->counterMax / 2); /** start the counter in middle of value range **/
        reControl->counterMin = 0;

        if(xTaskCreate("re_driver_task", re_driver_task, 2048, reControl, 4, &parentTask) == pdTRUE) {
            reControl->parentTask = parentTask;
        }
        else {
            ESP_LOGE(RE_TAG, "Error starting task");
            initStatus = ESP_ERR_NO_MEM;
        }
    }

    if(initStatus == ESP_OK) {
        timerHandle = xTimerCreate("reDebounceTmr", pdMS_TO_TICKS(RE_DEBOUNCE_TIME_MS), pdFALSE, reControl, &debounceExpireCallback);
        if(timerHandle == NULL) {
            ESP_LOGE(RE_TAG, "Error creating timer");
            initStatus = ESP_ERR_NO_MEM;
        }
        else {
            reControl->debounceTimer = timerHandle;
        }
    }

    if (initStatus)
    {
        ESP_LOGE(RE_TAG, "Error in initialising Rotary Encoder Driver");
        if(reControl != NULL) {
            heap_caps_free(reControl);
        }
    }
    else
    {
        ESP_LOGI(RE_TAG, "Succesfully intialised Rotary Encoder Driver!");
    }

    return reControl;
}


esp_err_t rotaryEncoder_setSignedCounter(rotaryEncoder_t *re, int16_t *set)
{

    esp_err_t status = ESP_OK;

    if (!(re->signedCounter))
    {
        re->signedCounter = true;
        re->count.Value = 0;
    }

    return status;
}

esp_err_t rotaryEncoder_getValue(rotaryEncoder_t *re, int16_t *get)
{
    if (re->signedCounter)
    {
        *get = re->count.Value;
    }
    else
    {
        *get = re->count.uValue;
    }

    return ESP_OK;
}

esp_err_t rotaryEncoder_getLastDir(rotaryEncoder_t *re, uint8_t *lastDir)
{
    *lastDir = re->dirLast;
    return ESP_OK;
}

esp_err_t rotaryEncoder_setCounterMax(rotaryEncoder_t *re, uint16_t countMax)
{

    esp_err_t setStatus = ESP_OK;

    if (re->signedCounter && countMax > INT16_MAX)
    {
        ESP_LOGE(RE_TAG, "Error: In signed mode, max value is %d", INT16_MAX);
        setStatus = ESP_ERR_INVALID_SIZE;
    }
    else
    {
        re->counterMax = countMax;
    }

    return setStatus;
}

esp_err_t rotaryEncoder_resetCounter(rotaryEncoder_t *re)
{
    if (re->signedCounter)
    {
        re->count.Value = 0;
    }
    else
    {
        re->count.uValue = (re->counterMax / 2);
    }

    return ESP_OK;
}