/***************************************
* \file     RotaryEncoder_Driver.c
* \brief    An ESP32 IDF / FreeRTOS driver for a standard incremental Rotary encoder
*           Driver is based on GPIO driver, ESP interrupts and (maybe) freertos timer
* \date     July 2020
* \author   RJAM
****************************************/

/**
 *  TODO: Maybe add a short debounce to the rotary encoder?
 *        Do function Prototypes
 *        Maybe do button as separate component?
 * 
 * 
 *  REFACTOR:
 *      - Design for multi-device support
 *      - Single task
 *      - Move gpio reads to task
 *      - simplifiy initialiser, etc
 *      - more consistent naming
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

static bool task_running = false;

/************ ISR *********************/

static void IRAM_ATTR RE_DataInterrupt(void *args)
{

    BaseType_t pdHigherPrioWoken = pdFALSE;
    RE_h reControl = (RE_h )args;
    /** check debounce **/
    if (!(reControl->debounceState))
    {
        /** New approach - send the handle as the notify value
         *  check direction in the task 
         ***/
        xTaskNotifyFromISR(reControl->parentTask, args, eSetValueWithOverwrite, &pdHigherPrioWoken);

        /** start the debounce timer **/
        xTimerStartFromISR(reControl->debounceTimer, &pdHigherPrioWoken);
        reControl->debounceState = 1;
    }

    portYIELD_FROM_ISR();
};

static uint8_t re_get_direction(RE_h dev) {

    uint8_t direction = gpio_get_level(dev->clock_pin);
    uint8_t rising = gpio_get_level(dev->data_pin);
    
    return direction + rising;
}


void debounceExpireCallback(TimerHandle_t xTimer)
{
    RE_h re = (RE_h )pvTimerGetTimerID(xTimer);
    // uint32_t notify = RE_NOTIFY_DEBOUNCE_EXPIRED;
    re->debounceState = 0;
}

/****** Private Functions *************/

static void re_driver_task(void *args) {
 
    /** keep this driver event-based - interrupt pins raise a notify to 
     * this task, then task handles sending events depending on driver settings
     */
    RE_h re = (RE_h ) args;

    uint32_t rotary_value = 0;
    uint32_t handle_ptr;
    RE_h dev;
#ifdef CONFIG_USE_EVENTS    
    uint32_t event_value = 0;
#endif

    while(1) {

        xTaskNotifyWait(0x00, 0x00, &handle_ptr, portMAX_DELAY);

        dev = (rotaryEncoder_t *)handle_ptr;

        rotary_value = re_get_direction(dev);

        if(rotary_value == RE_NOTIFY_CW_STEP) {
#ifdef CONFIG_USE_EVENTS    
            event_value = re->use_events ? RE_EVENT_INCREMENT : event_value;
#endif
            re->count.uValue = INCREMENT_TO_MAX(re->count.uValue, re->counterMax);
            ESP_LOGI(RE_TAG, "Got CW step - {%u}", re->count.Value);
        }
        
        else if(rotary_value == RE_NOTIFY_CW_STEP) {
#ifdef CONFIG_USE_EVENTS    
            event_value = re->use_events ? RE_EVENT_DECREMENT : event_value;
#endif
            re->count.uValue = DECREMENT_TO_MIN(re->count.uValue, re->counterMin);
            ESP_LOGI(RE_TAG, "Got CC step - {%u}", re->count.Value);
        }

        else {
            ESP_LOGE(RE_TAG, "Unknown rotary state [%u]", rotary_value);
        }
    
#ifdef CONFIG_USE_EVENTS    
        if(re->use_events && event_value) {
            esp_event_post_to(re->loop, 0, event_value, NULL, 0, pdMS_TO_TICKS(100));
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

#ifdef CONFIG_DRIVERS_USE_HEAP
RE_h rotaryEncoderInit(RE_h handle, rotary_encoder_init_t *init)
#else
RE_h rotaryEncoderInit(RE_h handle, rotary_encoder_init_t *init)
#endif
{

    esp_err_t initStatus = ESP_OK;

    /** configure the pins **/
    gpio_config_t pinConfig;
    TimerHandle_t timerHandle = NULL;
    TaskHandle_t parentTask;
    RE_h reControl = NULL; 

    pinConfig.mode = GPIO_MODE_INPUT;
    pinConfig.pin_bit_mask = (1UL << init->data_pin); /** set unused pins to 0 **/
    pinConfig.intr_type = GPIO_INTR_POSEDGE;
    pinConfig.pull_down_en = 0;
    pinConfig.pull_up_en = 0;

    ESP_ERROR_CHECK(gpio_config(&pinConfig));
    if (initStatus == ESP_OK)
    {
        pinConfig.intr_type = GPIO_INTR_DISABLE;
        pinConfig.pin_bit_mask = init->clock_pin;
        initStatus = gpio_config(&pinConfig);
    }
    else
    {
        ESP_LOGE(RE_TAG, "Error configuring data pin: 0x%x", initStatus);
    }

    /** configure the interrupts **/
    if (initStatus == ESP_OK)
    {
        initStatus = gpio_isr_handler_add(init->data_pin, RE_DataInterrupt, &reControl);
        if (initStatus != ESP_OK)
        {
            ESP_LOGE(RE_TAG, "Error adding data gpio isr handler: 0x%x", initStatus);
        }
    }

    /* build the control struct */
    if (initStatus == ESP_OK)
    {
#ifdef CONFIG_DRIVERS_USE_HEAP
        reControl = heap_caps_calloc(1, sizeof(rotaryEncoder_t), MALLOC_CAP_DEFAULT);
#else
        memset(reControl, 0, sizeof(rotaryEncoder_t));
#endif
        reControl->clock_pin = init->clock_pin;
        reControl->data_pin = init->data_pin;
        reControl->stepSize = 1;
        reControl->counterMax = UINT16_MAX;
        reControl->count.uValue = (reControl->counterMax / 2); /** start the counter in middle of value range **/
        reControl->counterMin = 0;

        if(!task_running) {
            if(xTaskCreate("re_driver_task", re_driver_task, 2048, reControl, 4, &parentTask) == pdTRUE) {
                reControl->parentTask = parentTask;
                task_running = true;
            }
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


esp_err_t rotaryEncoder_setSignedCounter(RE_h re, int16_t *set)
{

    esp_err_t status = ESP_OK;

    if (!(re->signedCounter))
    {
        re->signedCounter = true;
        re->count.Value = 0;
    }

    return status;
}

esp_err_t rotaryEncoder_getValue(RE_h re, int16_t *get)
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

esp_err_t rotaryEncoder_getLastDir(RE_h re, uint8_t *lastDir)
{
    *lastDir = re->dirLast;
    return ESP_OK;
}

esp_err_t rotaryEncoder_setCounterMax(RE_h re, uint16_t countMax)
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

esp_err_t rotaryEncoder_resetCounter(RE_h re)
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