/***************************************
* \file .c
* \brief
*
* \date
* \author
****************************************/

/********* Includes *******************/
#include "A4988_Driver.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_types.h"

#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

const char *DEV_TAG = "A4988 Driver";

/****** Function Prototypes ***********/

/************ ISR *********************/

/****** Private Data ******************/

/****** Private Functions *************/


/****** Global Data *******************/

/****** Global Functions *************/


A4988_DEV a4988_init(a4988_init_t *init) {

    A4988_DEV dev = NULL;
    esp_err_t err = ESP_OK;

    gpio_config_t pins = {0};


    /** initialise the handle **/
    dev = heap_caps_calloc(1, sizeof(a4988_handle_t), MALLOC_CAP_DEFAULT);
    if(dev == NULL) {
        ESP_LOGE(DEV_TAG, "Error assigning memory for the handle");
        err = ESP_ERR_NO_MEM;
    }
    else {
        dev->step_wait = A4988_DEFAULT_STEP_DELAY;
        dev->is_enabled = true;
        dev->is_sleeping = false;
        dev->step_size = init->step_size;
        dev->rst = init->rst;
        dev->sleep = init->sleep;
        dev->enable = init->enable;
        dev->step = init->step;
        dev->dir = init->dir;
        dev->ms1 = init->ms1;
        dev->ms2 = init->ms2;
        dev->ms3 = init->ms3;
    }

    /** check & initialise the mandatory pins **/

    if(init->rst == 0 || init->dir == 0 || init->step == 0) {
        ESP_LOGE(DEV_TAG, "Error - missing important pins!");
        err = ESP_ERR_INVALID_ARG;
    }

    if(!err) {
        pins.intr_type = GPIO_INTR_DISABLE;
        pins.mode = GPIO_MODE_OUTPUT;
        pins.pin_bit_mask = ((1 << init->rst) | (1 << init->step) | (1 << init->dir));
        err = gpio_config(&pins);

        if(err) {
            ESP_LOGE(DEV_TAG, "error setting up GPIO pins");
        }
        else {
            /** set pin states **/
            gpio_set_level(dev->rst, 1);
            gpio_set_level(dev->dir, 1);
            gpio_set_level(dev->step, 0);
        }
    }

    if(!err) {
        uint32_t pinmask = 0;
        /** check if the driver controls the enable pin **/
        if(init->enable) {
            pinmask |= (1 << init->enable);
            dev->_en = true;
        }
        /** check if the driver controls the sleep pin **/
        if(init->sleep) {
            pinmask |= (1 << init->sleep);
            dev->_sleep = true;
        }
        /** check if the driver controls all 3 ms pins **/
        if(init->ms1 && init->ms2 && init->ms3) {
            pinmask |= (1 << init->ms1);
            pinmask |= (1 << init->ms2);
            pinmask |= (1 << init->ms3);
            dev-> _ms = true;
        }

        /** initialise these as outputs **/
        if(pinmask > 0) {
            pins.pin_bit_mask = pinmask;
            err = gpio_config(&pins);
            if(err) {
                ESP_LOGE(DEV_TAG, "error setting up secondary GPIO pins");
            }
            else {
                /** set pin levels **/
                if(dev->_en) {
                    gpio_set_level(dev->enable, 0); 
                }
                if(dev->_sleep) {
                    gpio_set_level(dev->sleep, 1);
                }
                if(dev->_ms) {
                    gpio_set_level(dev->ms1, 0);
                    gpio_set_level(dev->ms2, 0);
                    gpio_set_level(dev->ms3, 0);
                }
            }
        }
    }


    if(!err) {
        ESP_LOGI(DEV_TAG, "Succesfully started the A4988 Driver!");
    }

    return dev;
}


esp_err_t a4988_step(A4988_DEV dev) {

    esp_err_t err = ESP_OK;

    err = gpio_set_level(dev->step, 1);
    vTaskDelay(pdMS_TO_TICKS(dev->step_wait));
    err = gpio_set_level(dev->step, 0);
    return err;
}

esp_err_t a5988_reset(A4988_DEV dev) {
    esp_err_t err = ESP_OK;

    err = gpio_set_level(dev->rst, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    err = gpio_set_level(dev->rst, 1);
    return err;  
}

esp_err_t a4988_clear_step_queue(A4988_DEV dev) {
    dev->steps_queued = 0;
    return ESP_OK;
}

esp_err_t a4988_queue_steps(A4988_DEV dev, uint16_t *steps) {
    dev->steps_queued = *steps;
    return ESP_OK;
}

esp_err_t a4988_get_stepsize(A4988_DEV dev, uint8_t *sz) {
    *sz = dev->step_size;
    return ESP_OK;
}

esp_err_t a4988_set_stepsize(A4988_DEV dev, uint8_t *sz) {

    uint8_t val = *sz;

    if(!(dev->_ms)) {
        ESP_LOGE(DEV_TAG, "Driver does not control step size!");
        return ESP_ERR_NOT_SUPPORTED;
    }
    if(val > SXTN_STEP_T) {
        return ESP_ERR_INVALID_ARG;
    }
    else {
        /** write the pins **/
        gpio_set_level(dev->ms1, (val & 0b001));
        gpio_set_level(dev->ms1, (val & 0b010));
        gpio_set_level(dev->ms1, (val & 0b100));
    }
}

esp_err_t a4988_get_sleepstate(A4988_DEV dev, bool *slp) {
    *slp = dev->is_sleeping;
    return ESP_OK;
}

esp_err_t a4988_set_sleepstate(A4988_DEV dev, bool *slp) {
    
    bool sleep = *slp;
    if(!(dev->_sleep)) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    else if(sleep && dev->is_sleeping) {
        return ESP_OK;
    }
    else if(!sleep && !(dev->is_sleeping)) {
        return ESP_OK;
    }
    else {
        gpio_set_level(dev->sleep, (uint32_t )sleep);
    }
    return ESP_OK;
}

esp_err_t a4988_get_enable(A4988_DEV dev, bool *en) {

}

esp_err_t a4988_set_enable(A4988_DEV dev, bool *en);

esp_err_t a4988_get_step_delay(A4988_DEV dev, uint8_t *sz);

esp_err_t a4988_set_step_delay(A4988_DEV dev, uint8_t *sz);

esp_err_t a4988_get_microstep_delay(A4988_DEV dev, uint8_t *sz);

esp_err_t a4988_set_microstep_delay(A4988_DEV dev, uint8_t *sz);

