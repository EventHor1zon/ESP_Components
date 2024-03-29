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


#include "RGB_Driver.h"

#include "esp_err.h"
#include "esp_types.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


const char *RGB_TAG = "RGB_Driver";


#ifdef CONFIG_USE_PERIPH_MANAGER


const parameter_t rgb_param_map[rgb_param_len] = {
    {"Red Duty", 1, &rgb_get_r_duty, &rgb_set_r_duty, NULL, DATATYPE_UINT32, 100000, (GET_FLAG | SET_FLAG)},
    {"Green Duty", 2, &rgb_get_g_duty, &rgb_set_g_duty, NULL, DATATYPE_UINT32, 100000, (GET_FLAG | SET_FLAG)},
    {"Blue Duty", 3, &rgb_get_b_duty, &rgb_set_b_duty, NULL, DATATYPE_UINT32, 100000, (GET_FLAG | SET_FLAG)},
    {"Red (percent)", 4, NULL, &rgb_set_r_duty_percent, NULL, DATATYPE_UINT32, 100000, (GET_FLAG | SET_FLAG)},
    {"Green (percent)", 5, NULL, &rgb_set_g_duty_percent,NULL,  DATATYPE_UINT32, 100000, (GET_FLAG | SET_FLAG)},
    {"Blue (percent)", 6, NULL, &rgb_set_b_duty_percent, NULL, DATATYPE_UINT32, 100000, (GET_FLAG | SET_FLAG)},
    {"Fade time", 7, &rgb_get_fade_time, &rgb_set_fade_time, NULL, DATATYPE_UINT32, 10000, (GET_FLAG | SET_FLAG)},
};

const peripheral_t rgb_periph_template = {
    .handle = NULL,
    .param_len = rgb_param_len,
    .params = rgb_param_map,
    .peripheral_name = "RGB",
    .peripheral_id = 0,
    .periph_type = PTYPE_IO,
};

#endif /* CONFIG_USE_PERIPH_MANAGER */

/****** Function Prototypes ***********/


static esp_err_t update_duty(uint32_t duty, uint8_t channel, uint32_t fade_t);

static uint32_t percent_to_duty(uint8_t percent, uint32_t max_duty);





/************ ISR *********************/

/****** Private Data ******************/

/****** Private Functions *************/


static esp_err_t update_duty(uint32_t duty, uint8_t channel, uint32_t fade_t) {
    esp_err_t err = ESP_OK;

    ESP_LOGI(RGB_TAG, "Setting channel %u duty to %u", channel, duty);

    switch (channel)
    {
    /** TODO: This,better - dont pass in a handle, pass a duty? **/
    case 0:
        err = ledc_set_fade_time_and_start(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)channel, duty, fade_t, LEDC_FADE_NO_WAIT);
        break;
    case 1:
        err = ledc_set_fade_time_and_start(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)channel, duty, fade_t, LEDC_FADE_NO_WAIT);
        break;
    case 2:
        err = ledc_set_fade_time_and_start(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)channel, duty, fade_t, LEDC_FADE_NO_WAIT);
        break;
    default:
        err = ESP_ERR_INVALID_ARG;
        break;
    }

    return err;
}



static uint32_t percent_to_duty(uint8_t percent, uint32_t max_duty) {

    uint32_t p = max_duty / 100;
    p *= percent;
    return p;
}




static void rgb_driver_task(void *args) {
 
    RGB_HANDLE handle = (RGB_HANDLE)args;


    while(1) {
        ESP_LOGI(RGB_TAG, "RGB Task");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    /** here be dragons **/
}

/****** Global Data *******************/

/****** Global Functions *************/

#ifdef CONFIG_DRIVERS_USE_HEAP
RGB_HANDLE rgb_driver_init(rgb_init_t *init) 
#else
RGB_HANDLE rgb_driver_init(RGB_HANDLE handle, rgb_init_t *init) 
#endif
{

    esp_err_t err = ESP_OK;
    ledc_channel_config_t cfg = {0};
    TaskHandle_t t_handle;

    ESP_LOGI(RGB_TAG, "Starting RGB driver");

    if(init->freq > 100000) {
        err = ESP_ERR_INVALID_ARG;
        ESP_LOGE(RGB_TAG, "Error, frequency is too damn high!");
    }


    if(!err) {
#ifdef CONFIG_DRIVERS_USE_HEAP
        RGB_HANDLE handle = heap_caps_calloc(1, sizeof(rgb_driver_t), MALLOC_CAP_DEFAULT);
        if(handle == NULL) {
            ESP_LOGE(RGB_TAG, "Error allocating heap for driver structure!");
            err = ESP_ERR_NO_MEM;
        }
#else
        memset(handle, 0, sizeof(rgb_driver_t));
#endif
    }

    if(!err) {
        handle->fade_time = 1000;
        handle->active_level = init->active_level;
        handle->b_duty = 0;
        handle->r_duty = 0;
        handle->g_duty = 0;
        handle->frequency = init->freq;
        handle->r_channel = 0;
        handle->g_channel = 1;
        handle->b_channel = 2;
        handle->resolution = 12;        /** TODO: don't hardwire values! **/
        handle->max_duty = RGB_LEDC_DUTY;
    }


    ledc_timer_config_t tmrcfg = {0};
    if(!err) {
        tmrcfg.freq_hz = init->freq;
        tmrcfg.clk_cfg = LEDC_USE_APB_CLK;
        tmrcfg.timer_num = LEDC_TIMER_0;
        tmrcfg.speed_mode = LEDC_HIGH_SPEED_MODE;
        tmrcfg.duty_resolution = LEDC_TIMER_12_BIT;
        ESP_LOGI(RGB_TAG, "Setting up LedCtrl timer at freq %u", tmrcfg.freq_hz);

        err = ledc_timer_config(&tmrcfg);
    }

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


    if(!err && xTaskCreate(rgb_driver_task, "rgb_driver_task", 5012, handle, 3, &t_handle) != pdTRUE) {
        ESP_LOGE(RGB_TAG, "Error creating driver task!");
        err = ESP_ERR_NO_MEM;
    }
    else {
        handle->t_handle = t_handle;
    }

    if(!err) {
        err = ledc_fade_func_install(ESP_INTR_FLAG_LOWMED);
    }

    if(!err) {
        ESP_LOGI(RGB_TAG, "Succesfully started the RGB driver!");
    }
    else {
        ESP_LOGE(RGB_TAG, "Failed to start RGB driver!");
#ifdef CONFIG_DRIVERS_USE_HEAP
        if(handle != NULL) {
            heap_caps_free(handle);
        }
#endif
    }

    return handle;

}



esp_err_t rgb_get_fade_time(RGB_HANDLE handle, uint32_t *val) {

    esp_err_t err = ESP_OK;

    *val = handle->fade_time;
    return err;
}

esp_err_t rgb_set_fade_time(RGB_HANDLE handle, uint32_t *val) {
    esp_err_t err = ESP_OK;
    uint32_t t = *val; 
    if(t > RGB_MAX_FADE_TIME || t < RGB_MIN_FADE_TIME) {
        handle->fade_time = t;
        err = ESP_ERR_INVALID_ARG;
    }
    else {
        handle->fade_time = t;
    }
    return err;
}


esp_err_t rgb_set_r_duty(RGB_HANDLE handle, uint32_t *val) {
    esp_err_t status = ESP_OK;
    uint32_t d = *val;
    if(d > handle->max_duty) {
        status = ESP_ERR_INVALID_ARG;
        ESP_LOGE(RGB_TAG, "Duty %u too high!", d);
    }
    else {
        handle->r_duty = d;
        update_duty(d, RGB_RED_CHANNEL, handle->fade_time);
    }

    return status;
}

esp_err_t rgb_set_g_duty(RGB_HANDLE handle, uint32_t *val) {
    esp_err_t status = ESP_OK;
    uint32_t d = *val;
    if(d > handle->max_duty) {
        status = ESP_ERR_INVALID_ARG;
        ESP_LOGE(RGB_TAG, "Duty %u too high!", d);
    }
    else {
        handle->g_duty = d;
        update_duty(d, RGB_GRN_CHANNEL, handle->fade_time);
    }

    return status;
}

esp_err_t rgb_set_b_duty(RGB_HANDLE handle, uint32_t *val) {
    esp_err_t status = ESP_OK;
    uint32_t d = *val;
    if(d > handle->max_duty) {
        status = ESP_ERR_INVALID_ARG;
        ESP_LOGE(RGB_TAG, "Duty %u too high!", d);
    }
    else {
        handle->b_duty = d;
        update_duty(d, RGB_BLU_CHANNEL, handle->fade_time);
    }

    return status;
}



esp_err_t rgb_get_r_duty(RGB_HANDLE handle, uint32_t *val) {
   esp_err_t status = ESP_OK;
   *val = handle->r_duty;
   return status;
}

esp_err_t rgb_get_g_duty(RGB_HANDLE handle, uint32_t *val) {
   esp_err_t status = ESP_OK;
   *val = handle->g_duty;
   return status;
}

esp_err_t rgb_get_b_duty(RGB_HANDLE handle, uint32_t *val) {
   esp_err_t status = ESP_OK;
   *val = handle->b_duty;
   return status;
}




esp_err_t rgb_set_r_duty_percent(RGB_HANDLE handle, uint32_t *val) {
    esp_err_t status = ESP_OK;
    uint32_t d = *val;
    if(d > 100) {
        status = ESP_ERR_INVALID_ARG;
        ESP_LOGE(RGB_TAG, "Duty %u too high!", d);
    }
    else {
        handle->r_duty = percent_to_duty(d, handle->max_duty);
        update_duty(handle->r_duty, RGB_RED_CHANNEL, handle->fade_time);
    }

    return status;
}

esp_err_t rgb_set_g_duty_percent(RGB_HANDLE handle, uint32_t *val) {
    esp_err_t status = ESP_OK;
    uint32_t d = *val;
    if(d > 100) {
        status = ESP_ERR_INVALID_ARG;
        ESP_LOGE(RGB_TAG, "Duty %u too high!", d);
    }
    else {
        handle->g_duty =  percent_to_duty(d, handle->max_duty);
        update_duty(handle->g_duty, RGB_GRN_CHANNEL, handle->fade_time);
    }

    return status;
}

esp_err_t rgb_set_b_duty_percent(RGB_HANDLE handle, uint32_t *val) {
    esp_err_t status = ESP_OK;
    uint32_t d = *val;
    if(d > 100) {
        status = ESP_ERR_INVALID_ARG;
        ESP_LOGE(RGB_TAG, "Duty %u too high!", d);
    }
    else {
        handle->b_duty = percent_to_duty(d, handle->max_duty);
        update_duty(handle->b_duty, RGB_BLU_CHANNEL, handle->fade_time);
    }

    return status;
}