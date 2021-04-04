/***************************************
* \file     RGB_Driver.c
* \brief    A simple 3-channel PWM controller
*           Designed for controlling med/high-powered 
*           RGB leds by way of pulse width modulation
*           Use Ledctrl module for easyness. Is that a word?
* \date
* \author
****************************************/

/********* Includes *******************/
#include "esp_err.h"
#include "esp_types.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "RGB_Driver.h"

const char *RGB_TAG = "RGB_Driver";

/****** Function Prototypes ***********/

/************ ISR *********************/

/****** Private Data ******************/

/****** Private Functions *************/

static void rgb_driver_task(void *args) {
 
    RGB_HANDLE handle = (RGB_HANDLE)args;

   while(1) {
       vTaskDelay(pdMS_TO_TICKS(10));
   }
   /** here be dragons **/
}

/****** Global Data *******************/

/****** Global Functions *************/


RGB_HANDLE rgb_driver_init(rgb_init_t *init) {

    esp_err_t err = ESP_OK;
    ledc_channel_config_t cfg = {0};
    RGB_HANDLE handle = NULL;
    TaskHandle_t t_handle = NULL;

    if(init->freq > 100000) {
        err = ESP_ERR_INVALID_ARG;
        ESP_LOGE(RGB_TAG, "Error, frequency is too damn high!");
    }

    ledc_timer_config_t tmrcfg = {0};
    tmrcfg.freq_hz = init->freq;
    tmrcfg.clk_cfg = LEDC_USE_APB_CLK;
    tmrcfg.timer_num = LEDC_TIMER_0;
    tmrcfg.speed_mode = LEDC_HIGH_SPEED_MODE;
    tmrcfg.duty_resolution = LEDC_TIMER_12_BIT;

    err = ledc_timer_config(&tmrcfg);

    if(err) {
        ESP_LOGE(RGB_TAG, "Error configuring timer");        
    }
    else {
        /** config the ledcontrol on first 3 channels **/
        cfg.channel = 0;
        cfg.duty = 0;
        cfg.gpio_num = init->r_pin;
        cfg.speed_mode = LEDC_HIGH_SPEED_MODE;
        cfg.hpoint = 0;
        cfg.timer_sel = LEDC_TIMER_0; /** TODO: see if better to use different channels **/
        err = ledc_channel_config(&cfg);
        
        cfg.channel = 1;
        cfg.gpio_num = init->g_pin;
        err += ledc_channel_config(&cfg);

        cfg.channel = 2;
        cfg.gpio_num = init->b_pin;
        err += ledc_channel_config(&cfg);
    
        if(err) {
            ESP_LOGE(RGB_TAG, "Error configuring channel!");        
        }
    }

    if(!err) {
        handle = heap_caps_calloc(1, sizeof(rgb_driver_t), MALLOC_CAP_DEFAULT);
        if(handle == NULL) {
            ESP_LOGE(RGB_TAG, "Error allocating heap for driver structure!");
            err = ESP_ERR_NO_MEM;
        }
        else {
            handle->active_level = init->active_level;
            handle->b_duty = 0;
            handle->r_duty = 0;
            handle->g_duty = 0;
            handle->frequency = init->freq;
            handle->r_channel = 0;
            handle->g_channel = 1;
            handle->b_channel = 2;
            handle->resolution = 12;        /** TODO: don't hardwire values! **/
            handle->max_duty = 4095;
        }
    }

    if(!err && xTaskCreate(rgb_driver_task, "rgb_driver_task", 5012, handle, 3, t_handle) != pdTRUE) {
        ESP_LOGE(RGB_TAG, "Error creating driver task!");
        err = ESP_ERR_NO_MEM;
    }
    else {
        handle->t_handle = t_handle;
    }


    if(!err) {
        ESP_LOGI(RGB_TAG, "Succesfully started the RGB driver!");
    }
    else {
        ESP_LOGE(RGB_TAG, "Failed to start RGB driver!");
        if(handle != NULL) {
            heap_caps_free(handle);
        }
    }

    return handle;

}

esp_err_t set_r_duty(RGB_HANDLE *handle, uint32_t *val) {
    esp_err_t status = ESP_OK;

    

    return status;
}

esp_err_t set_g_duty(RGB_HANDLE *handle, uint32_t *val) {

}

esp_err_t set_b_duty(RGB_HANDLE *handle, uint32_t *val) {

}