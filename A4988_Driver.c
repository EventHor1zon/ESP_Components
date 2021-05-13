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
static TaskHandle_t toNotify = NULL;


#ifdef CONFIG_USE_PERIPH_MANAGER


const parameter_t a4988_param_map[a4988_param_len] = {
    {"step size", 1, &a4988_get_stepsize, &a4988_set_stepsize, PARAMTYPE_UINT8, 7, (GET_FLAG | SET_FLAG)},
    {"step wait", 2, &a4988_get_step_delay, &a4988_set_step_delay, PARAMTYPE_UINT16, 0xFFFF, (GET_FLAG | SET_FLAG)},
    {"queued steps", 3, &a4988_get_queued_steps, &a4988_set_queued_steps, PARAMTYPE_UINT16, 0xFFFF, (GET_FLAG | SET_FLAG)},
    {"sleep state", 4, &a4988_get_sleepstate, &a4988_set_sleepstate, PARAMTYPE_BOOL, 1, (GET_FLAG | SET_FLAG)},
    {"enable", 5, &a4988_get_enable, &a4988_set_enable, PARAMTYPE_BOOL, 1, (GET_FLAG | SET_FLAG)},
    {"direction", 6, &a4988_get_direction, &a4988_set_direction, PARAMTYPE_BOOL, 1, (GET_FLAG | SET_FLAG)},
    {"clear queue", 7, &a4988_clear_step_queue, NULL, PARAMTYPE_NONE, 0, (ACT_FLAG)},
    {"step", 8, &a4988_step, NULL, PARAMTYPE_NONE, 0, (ACT_FLAG)},
    {"reset", 9, &a4988_reset, NULL, PARAMTYPE_NONE, 0, (ACT_FLAG)},
};

const peripheral_t a4988_periph_template = {
    .actions = NULL,
    .actions_len = 0,
    .handle = NULL,
    .param_len = a4988_param_len,
    .params = a4988_param_map,
    .peripheral_name = "A4988",
    .peripheral_id = 0,
    .periph_type = PTYPE_IO,
};

#endif /** CONFIG_USE_PERIPH_MANAGER **/



/****** Function Prototypes ***********/

/************ ISR *********************/

/****** Private Data ******************/

/****** Private Functions *************/


void a4988_timer_callback(TimerHandle_t tmr) {

    A4988_DEV dev = (a4988_handle_t *)tmr;
    BaseType_t higherPrioWoken = pdFALSE;
    xTaskNotifyGive(toNotify);
    return;
} 


static void a4988_driver_task(void *args) {
 

    A4988_DEV dev = args;
    uint16_t last_t = dev->step_wait;

    toNotify = xTaskGetCurrentTaskHandle();

    while(1) {

        if(dev->steps_queued > 0) {
            /** take the queued step, then start the timer & wait for notify **/
            ESP_LOGI(DEV_TAG, "Stepping... (steps in queue: %u)", dev->steps_queued);
            a4988_step(dev);
            if(last_t != dev->step_wait) {
                if(xTimerChangePeriod(dev->timer, pdMS_TO_TICKS(dev->step_wait), portMAX_DELAY) == pdFAIL) {
                    ESP_LOGE(DEV_TAG, "Error changing timer freq");
                }
                last_t = dev->step_wait;
            }
            if(xTimerStart(dev->timer, 0) != pdPASS) {
                ESP_LOGE(DEV_TAG, "Error starting timer");
            }
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }
        else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    /** here be dragons **/
}


/****** Global Data *******************/

/****** Global Functions *************/


A4988_DEV a4988_init(a4988_init_t *init) {

    A4988_DEV dev = NULL;
    esp_err_t err = ESP_OK;
    TaskHandle_t t_handle = NULL;
    TimerHandle_t timer = NULL;
    gpio_config_t pins = {0};


    /** initialise the handle **/
    dev = heap_caps_calloc(1, sizeof(a4988_handle_t), MALLOC_CAP_DEFAULT);
    if(dev == NULL) {
        ESP_LOGE(DEV_TAG, "Error assigning memory for the handle");
        err = ESP_ERR_NO_MEM;
    }
    else {
        dev->step_pulse_len = A4988_DEFAULT_STEP_PULSE_LEN;
        dev->step_wait = A4988_DEFAULT_STEP_DELAY;
        dev->is_enabled = true;
        dev->is_sleeping = false;
        dev->direction = false;
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
            gpio_set_level(dev->dir, 0);
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

    /** Try something - make the timer ID = pointer to our dev struct **/
    if(!err && xTaskCreate(a4988_driver_task, "a4988_driver_task", 5012, (void *)dev, 3, &t_handle) != pdTRUE) {
        ESP_LOGE(DEV_TAG, "Error starting dev task!");
        err = ESP_ERR_NO_MEM;
    }
    else {
        dev->t_handle = t_handle;
    }

    if(!err ) {
        timer = xTimerCreate("a4988_timer", pdMS_TO_TICKS(dev->step_wait), true, (void *)dev, a4988_timer_callback);
        if(timer == NULL) {
            ESP_LOGE(DEV_TAG, "Error creating timer!");
            err = ESP_ERR_NO_MEM;
        }
        else {
            dev->timer = timer;
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
    vTaskDelay(pdMS_TO_TICKS(dev->step_pulse_len));
    err = gpio_set_level(dev->step, 0);
    if(dev->steps_queued > 0) {
        dev->steps_queued--;
    }
    return err;
}

esp_err_t a4988_reset(A4988_DEV dev) {
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

esp_err_t a4988_set_queued_steps(A4988_DEV dev, uint16_t *steps) {
    dev->steps_queued = *steps;
    return ESP_OK;
}

esp_err_t a4988_get_queued_steps(A4988_DEV dev, uint16_t *steps) {
    *steps = dev->steps_queued;
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
    
    if( val != FULL_STEP_T &&
        val != HALF_STEP_T &&
        val != QURT_STEP_T &&
        val != EGTH_STEP_T &&
        val != SXTN_STEP_T 
    ){
        return ESP_ERR_INVALID_ARG;
    }

    /** write the pins **/
    gpio_set_level(dev->ms1, (val & 0b001));
    gpio_set_level(dev->ms1, (val & 0b010));
    gpio_set_level(dev->ms1, (val & 0b100));

    return ESP_OK;
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
    *en = dev->is_enabled;
    return ESP_OK;
}

esp_err_t a4988_set_enable(A4988_DEV dev, bool *en) {
    if(!(dev->_en)) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    else {
        uint32_t e = (uint32_t)*en;
        gpio_set_level(dev->enable, e);
    }
    return ESP_OK;
}

esp_err_t a4988_get_step_delay(A4988_DEV dev, uint16_t *t) {
    *t = dev->step_wait;
    return ESP_OK;
}

esp_err_t a4988_set_step_delay(A4988_DEV dev, uint16_t *t) {

    dev->step_wait = *t;
    xTaskNotifyGive(dev->t_handle);
    return ESP_OK;
}


esp_err_t a4988_get_direction(A4988_DEV dev, bool *dir) {
    *dir = dev->direction;
    return ESP_OK;
}

esp_err_t a4988_set_direction(A4988_DEV dev, bool *dir) {

    uint8_t val = *dir;
    if((val && !(dev->direction)) || (!(val) && dev->direction)) {
        gpio_set_level(dev->dir, (uint32_t )val);
        dev->direction = val;
    }
    return ESP_OK;
}


