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




#ifdef CONFIG_USE_PERIPH_MANAGER

const parameter_t rgb_param_map[rgb_param_len] = {
    {"Red Duty", 1, &rgb_get_r_duty, &rgb_set_r_duty, PARAMTYPE_UINT32, 100000, (GET_FLAG | SET_FLAG)},
    {"Green Duty", 2, &rgb_get_g_duty, &rgb_set_g_duty, PARAMTYPE_UINT32, 100000, (GET_FLAG | SET_FLAG)},
    {"Blue Duty", 3, &rgb_get_b_duty, &rgb_set_b_duty, PARAMTYPE_UINT32, 100000, (GET_FLAG | SET_FLAG)},
    {"Red (percent)", 4, NULL, &rgb_set_r_duty_percent, PARAMTYPE_UINT32, 100000, (GET_FLAG | SET_FLAG)},
    {"Green (percent)", 5, NULL, &rgb_set_g_duty_percent, PARAMTYPE_UINT32, 100000, (GET_FLAG | SET_FLAG)},
    {"Blue (percent)", 6, NULL, &rgb_set_b_duty_percent, PARAMTYPE_UINT32, 100000, (GET_FLAG | SET_FLAG)},
};

const peripheral_t rgb_periph_template = {
    .handle = NULL,
    .param_len = rgb_param_len,
    .params = rgb_parameter_map,
    .peripheral_name = "RGB",
    .peripheral_id = 0,
    .periph_type = PTYPE_IO,
};

#endif /* CONFIG_USE_PERIPH_MANAGER */

/****** Function Prototypes ***********/


static esp_err_t update_duty(uint32_t duty, uint8_t channel);

static uint32_t percent_to_duty(uint8_t percent, uint32_t max_duty);


/************ ISR *********************/

/****** Private Data ******************/

/****** Private Functions *************/


static esp_err_t update_duty(uint32_t duty, uint8_t channel) {
    esp_err_t err = ESP_OK;

    switch (channel)
    {
    /** TODO: This,better - dont pass in a handle, pass a duty? **/
    case 0:
        err = ledc_set_duty_and_update(LEDC_HIGH_SPEED_MODE, (ledc_channel_t )channel, duty, 0);
        break;
    case 1:
        err = ledc_set_duty_and_update(LEDC_HIGH_SPEED_MODE, (ledc_channel_t )channel, duty, 0);
        break;
    case 2:
        err = ledc_set_duty_and_update(LEDC_HIGH_SPEED_MODE, (ledc_channel_t )channel, duty, 0);
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

    static uint32_t duty = 100; 
    static int8_t dir = 1;
    while(1) {
        if(duty < 3999 && duty > 99) {
            duty = duty + (100 * dir);
            ledc_set_duty_and_update(LEDC_HIGH_SPEED_MODE, 0, duty, 0);
        }
        else {
            dir *= -1;
            duty = duty + (100 * dir);
            ledc_set_duty_and_update(LEDC_HIGH_SPEED_MODE, 0, duty, 0);
        }

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

esp_err_t rgb_set_r_duty(RGB_HANDLE handle, uint32_t *val) {
    esp_err_t status = ESP_OK;
    uint32_t d = *val;
    if(d > handle->max_duty) {
        status = ESP_ERR_INVALID_ARG;
        ESP_LOGE(RGB_TAG, "Duty too high!");
    }
    else {
        handle->r_duty = d;
        update_duty(handle, RGB_RED_CHANNEL);
    }

    return status;
}

esp_err_t rgb_set_g_duty(RGB_HANDLE handle, uint32_t *val) {
    esp_err_t status = ESP_OK;
    uint32_t d = *val;
    if(d > handle->max_duty) {
        status = ESP_ERR_INVALID_ARG;
        ESP_LOGE(RGB_TAG, "Duty too high!");
    }
    else {
        handle->g_duty = d;
        update_duty(handle, RGB_RED_CHANNEL);
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


esp_err_t rgb_set_b_duty(RGB_HANDLE handle, uint32_t *val) {
    esp_err_t status = ESP_OK;
    uint32_t d = *val;
    if(d > handle->max_duty) {
        status = ESP_ERR_INVALID_ARG;
        ESP_LOGE(RGB_TAG, "Duty too high!");
    }
    else {
        handle->b_duty = d;
        update_duty(handle, RGB_RED_CHANNEL);
    }

    return status;
}


esp_err_t rgb_set_r_duty_percent(RGB_HANDLE handle, uint32_t *val) {
    esp_err_t status = ESP_OK;
    uint32_t d = *val;
    if(d > 100) {
        status = ESP_ERR_INVALID_ARG;
        ESP_LOGE(RGB_TAG, "Duty too high!");
    }
    else {
        handle->r_duty = percent_to_duty(d, handle->max_duty);
        update_duty(handle, RGB_RED_CHANNEL);
    }

    return status;
}

esp_err_t rgb_set_g_duty_percent(RGB_HANDLE handle, uint32_t *val) {
    esp_err_t status = ESP_OK;
    uint32_t d = *val;
    if(d > 100) {
        status = ESP_ERR_INVALID_ARG;
        ESP_LOGE(RGB_TAG, "Duty too high!");
    }
    else {
        handle->g_duty =  percent_to_duty(d, handle->max_duty);
        update_duty(handle, RGB_RED_CHANNEL);
    }

    return status;
}

esp_err_t rgb_set_b_duty_percent(RGB_HANDLE handle, uint32_t *val) {
    esp_err_t status = ESP_OK;
    uint32_t d = *val;
    if(d > 100) {
        status = ESP_ERR_INVALID_ARG;
        ESP_LOGE(RGB_TAG, "Duty too high!");
    }
    else {
        handle->b_duty = percent_to_duty(d, handle->max_duty);
        update_duty(handle, RGB_RED_CHANNEL);
    }

    return status;
}