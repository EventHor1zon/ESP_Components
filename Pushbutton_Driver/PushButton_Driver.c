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
static TaskHandle_t btn_task_handle = NULL;

const char *BTN_TAG = "Push Button Driver";
static void btn_driver_task(void *args);

/************ ISR *********************/

void pushbutton_isr(void *args)
{
    BTN_DEV btnData = (buttonData_t *)args;
    BaseType_t pdHigherPrioWoken;

    /** check debounce **/
    if (btnData->btn_debounce_en && btnData->btnDebounceState == 0)
    {
        /** restart debounce timer & set debounce state **/
        xTimerStartFromISR(btnData->debounceTimer, &pdHigherPrioWoken);
        btnData->btnDebounceState = 1;

        /** update the button details **/
        btnData->btnCount = INCREMENT_TO_MAX(btnData->btnCount, UINT16_MAX);

        /** send the notify using the button handle as the argument **/
        xTaskNotifyFromISR(btn_task_handle, args, eSetValueWithOverwrite, &pdHigherPrioWoken);
    }

    portYIELD_FROM_ISR();
};


void debounceExpireCallback(TimerHandle_t xTimer)
{
    BTN_DEV btn = (buttonData_t *)pvTimerGetTimerID(xTimer);
    if(btn == NULL) {
        ESP_LOGE("BTN", "That didn't work!");
    }
    else {
        btn->btnDebounceState = false;
    }
}


static void btn_driver_task(void *args) {
 
    uint32_t source = 0;

    BTN_DEV btn;
    uint8_t pinLevel = 0;


    while(1) {
        /** wait forever for a notification **/
        xTaskNotifyWait(0, ULONG_MAX, &source, portMAX_DELAY);

        btn = (BTN_DEV)source;

        if(btn == NULL) {
            ESP_LOGE("BTN", "Error, invalid source value 0x%08x", source);
        }
        else {
            /** get button state **/
            pinLevel = gpio_get_level(btn->btn_pin);
            btn->btn_state = pinLevel;
            
            /** falling interrupt **/
            if(btn->btn_state == 0) {
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
            /** rising interrupt **/
            else if (btn->btn_state == 1) {
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
    }
   /** here be dragons **/

}




/****** Private Functions *************/

/****** Global Data *******************/

/****** Global Functions *************/
#ifdef CONFIG_DRIVERS_USE_HEAP
BTN_DEV pushbutton_init(pushbtn_init_t *init)
#else
BTN_DEV pushbutton_init(BTN_DEV btn, pushbtn_init_t *init)
#endif
{

    esp_err_t initStatus = 0;
    gpio_config_t pinConfig = {0};

#ifdef CONFIG_DRIVERS_USE_HEAP
    BTN_DEV btn = heap_caps_calloc(1, sizeof(buttonData_t), MALLOC_CAP_DEFAULT);
    if(btn == NULL) {
        ESP_LOGE("BTN DRIVER", "Error assigning struct memory");
        initStatus = ESP_ERR_NO_MEM;
    }
#else
    memset(btn, 0, sizeof(buttonData_t));
#endif

    if(initStatus == ESP_OK) {
        btn->btn_debounce_en = true;
        btn->debounce_time = BTN_DEFAULT_DEBOUNCE_T;
        btn->btn_pin = init->btn_pin;
#ifdef CONFIG_USE_EVENTS
        btn->loop = init->event_loop;
#endif
    }

    if(initStatus == ESP_OK) {
        switch (init->btn_config)
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
    }

    if(initStatus == ESP_OK) {
        pinConfig.mode = GPIO_MODE_INPUT;
        pinConfig.pin_bit_mask = (1 << init->btn_pin);
        pinConfig.intr_type = GPIO_INTR_ANYEDGE;
        initStatus = gpio_config(&pinConfig);
        if(initStatus)
        {
            ESP_LOGE(BTN_TAG, "Error configuring pin %08x", initStatus);
        }
    }

    if (initStatus == ESP_OK)
    {
        initStatus = gpio_isr_handler_add(btn->btn_pin, pushbutton_isr, btn);
        if(initStatus)
        {
            ESP_LOGE(BTN_TAG, "Error asssigning isr handler %08x", initStatus);
        }
    }


    if (initStatus == ESP_OK)
    {
        /** initialise the btnData struct **/
        btn->debounceTimer = xTimerCreate("btnDebounceTmr", pdMS_TO_TICKS(BTN_DEFAULT_DEBOUNCE_T), pdFALSE, btn, &debounceExpireCallback);
        if(btn->debounceTimer == NULL)
        {
            initStatus = ESP_ERR_NO_MEM;
            ESP_LOGE(BTN_TAG, "Error asssigning button timer %08x", initStatus);
        }
    }

    if(initStatus == ESP_OK && btn_task_handle == NULL) {
        /** only start one instance of the task **/
        if(xTaskCreate(btn_driver_task, "btn_driver_task", 2048, NULL, 3, &btn_task_handle) != pdTRUE) {
            ESP_LOGE("BTN Driver", "Error starting driver task");
            initStatus = ESP_ERR_NO_MEM;
        }
    }

    if(initStatus == ESP_OK) {
        ESP_LOGI("BTN Driver", "Succesfully started the button driver!");
    }
    else {
        ESP_LOGE("BTN Driver", "Failed to start button driver!");
#ifdef CONFIG_DRIVERS_USE_HEAP
        if(btn != NULL) {
            heap_caps_free(btn);
        }
#endif /**CONFIG_DRIVERS_USE_HEAP **/
    }

    return btn;
}


esp_err_t pushBtn_getButtonState(BTN_DEV btn, uint8_t *state)
{
    *state = btn->btn_state;
    return ESP_OK;
}

esp_err_t pushBtn_getButtonPressT(BTN_DEV btn, uint32_t *btnPressT)
{
    *btnPressT = btn->tBtnPress;
    return ESP_OK;
}

esp_err_t pushbutton_set_debounce_time(BTN_DEV btn, uint16_t dbTime)
{
    btn->debounce_time = dbTime;
    return ESP_OK;
}

esp_err_t pushbutton_set_half_interrupt(BTN_DEV btn, bool state)
{
    btn->halfBtnInterrupt = state;
    return ESP_OK;
}
