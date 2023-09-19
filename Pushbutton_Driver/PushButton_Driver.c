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
#include "esp_event.h"
#include "esp_heap_caps.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "PushButton_Driver.h"
#include "Utilities.h"

/****** Function Prototypes ***********/

/****** Private Data ******************/

static buttonData_t btnData;

const char *BTN_TAG = "Push Button Driver";
static void btn_driver_task(void *args);

/************ ISR *********************/

void pushBtn_Isr(void *args)
{
    BTN_DEV btnData = (buttonData_t *)args;
    BaseType_t pdHigherPrioWoken;
    uint32_t notification = 0;
    uint8_t pinLevel = 0;

    pinLevel = gpio_get_level(btnData->btnPin);
    btnData->btnState = pinLevel;

    /** check debounce **/
    if (btnData->btnDebounceEnable && btnData->btnDebounceState == 0)
    {
        if (pinLevel)
        { /** found a rising edge **/
            notification |= (NOTIFY_BTN_UP);
            xTaskNotifyFromISR(btnData->parentTask, notification, eSetValueWithOverwrite, &pdHigherPrioWoken);
        }
        else
        {
            notification |= (NOTIFY_BTN_DWN);
            xTaskNotifyFromISR(btnData->parentTask, notification, eSetValueWithOverwrite, &pdHigherPrioWoken);
        }

        /* enable the debounce */
        if (btnData->btnDebounceEnable)
        {
            /** start timer, set debounce state **/
            xTimerStartFromISR(btnData->debounceTimer, &pdHigherPrioWoken);
            btnData->btnDebounceState = 1;
        }

        btnData->btnCount = INCREMENT_TO_MAX(btnData->btnCount, UINT16_MAX);
    }

    portYIELD_FROM_ISR();
};


static void btn_driver_task(void *args) {
 
    BTN_DEV btn = (buttonData_t *)args;
    uint32_t notify = 0;

    while(1) {
        /** wait forever for a notification **/
        xTaskNotifyWait(ULONG_MAX, ULONG_MAX, &notify, portMAX_DELAY);

        ESP_LOGI("BTN Driver", "Received a task notification - value [0x%08x] (db state: %u)", notify, btn->btnDebounceState);
    
        if(notify & NOTIFY_BTN_DWN) {
            if(btn->btn_setting == BTN_CONFIG_ACTIVELOW || 
                btn->btn_setting == BTN_CONFIG_ACTIVELOW_PULLUP) {
#ifdef CONFIG_USE_EVENTS
                if(btn->loop != NULL) {
                    uint32_t id = BTN_EVENT_BTNDOWN;
                    ESP_LOGI("BTN", "Sending a btn down event (%u | 0x%08x)", id, id);
                    esp_event_post_to(btn->loop, PM_EVENT_BASE, id, NULL, 0, pdMS_TO_TICKS(10));
                }
#endif /** CONFIG_USE_EVENTS **/
            }
        }
        else if (notify & NOTIFY_BTN_UP) {
            if(btn->btn_setting == BTN_CONFIG_ACTIVEHIGH || 
                btn->btn_setting == BTN_CONFIG_ACTIVEHIGH_PULLDOWN) {
                    

#ifdef CONFIG_USE_EVENTS
                if(btn->loop != NULL) {
                    uint32_t id = BTN_EVENT_BTNUP;
                    ESP_LOGI("BTN", "Sending a btn up event  (%u | 0x%08x)", id, id);
                    esp_event_post_to(btn->loop, PM_EVENT_BASE, id, NULL, 0, pdMS_TO_TICKS(10));
                }
#endif /** CONFIG_USE_EVENTS **/
            }
        }
    }
   /** here be dragons **/

}


void debounceExpireCallback(TimerHandle_t xTimer)
{
    printf("debounce timer expired\n");

    BTN_DEV btn = (buttonData_t *)pvTimerGetTimerID(xTimer);
    if(btn == NULL) {
        ESP_LOGE("BTN", "That didn't work!");
    }
    else {
        btn->btnDebounceState = false;
    }
}

/****** Private Functions *************/

/****** Global Data *******************/

/****** Global Functions *************/
BTN_DEV pushBtn_Init(gpio_num_t btnPin, btn_config_t btnConfig, esp_event_loop_handle_t event_loop)
{

    esp_err_t initStatus = 0;
    gpio_config_t pinConfig = {0};

    BTN_DEV btn = heap_caps_calloc(1, sizeof(buttonData_t), MALLOC_CAP_DEFAULT);
    if(btn == NULL) {
        ESP_LOGE("BTN DRIVER", "Error assigning struct memory");
        initStatus = ESP_ERR_NO_MEM;
    }

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
    pinConfig.pin_bit_mask = (1 << btnPin); /** set unused pins to 0 **/
    pinConfig.intr_type = GPIO_INTR_ANYEDGE;

    if (initStatus == ESP_OK)
    {
        initStatus = gpio_config(&pinConfig);
        if(initStatus)
        {
            ESP_LOGE(BTN_TAG, "Error configuring pin %08x", initStatus);
        }
    }

    if (initStatus == ESP_OK)
    {
        initStatus = gpio_isr_handler_add(btnPin, pushBtn_Isr, btn);
        if(initStatus)
        {
            ESP_LOGE(BTN_TAG, "Error asssigning isr handler %08x", initStatus);
        }
    }


    if (initStatus == ESP_OK)
    {
        /** initialise the btnData struct **/

        TimerHandle_t timerHandle = xTimerCreate("btnDebounceTmr", pdMS_TO_TICKS(BTN_DEFAULT_DEBOUNCE_T), pdFALSE, btn, &debounceExpireCallback);
        btn->debounceTimer = timerHandle;
        btn->btnDebounceEnable = true;
        btn->tDebounce = BTN_DEFAULT_DEBOUNCE_T;
        btn->btnPin = btnPin;
        btn->loop = event_loop;
    }

    if(initStatus == ESP_OK) {
        if(xTaskCreate(btn_driver_task, "btn_driver_task", 2048, btn, 3, &(btn->parentTask)) != pdTRUE) {
            ESP_LOGE("BTN Driver", "Error starting driver task");
            initStatus = ESP_ERR_NO_MEM;
        }
    }

    if(initStatus == ESP_OK) {
        ESP_LOGI("BTN Driver", "Succesfully started the button driver!");
    }
    else {
        ESP_LOGE("BTN Driver", "Failed to start button driver!");
        if(btn != NULL) {
            heap_caps_free(btn);
        }
    }

    return btn;
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
