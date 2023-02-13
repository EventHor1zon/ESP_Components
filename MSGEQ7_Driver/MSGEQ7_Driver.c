/***************************************
* \file  MSGEQ7_Driver.c
* \brief   Driver for the MSGEq7 audio spectrum chip
*           
* \date     FEB 2021
* \author   RJAM
****************************************/

/********* Includes *******************/
#include "esp_err.h"
#include "driver/adc_common.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "string.h"

#include "soc/adc_channel.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "MSGEQ7_Driver.h"
#include "Utilities.h"

#define DEBUG

const char *MSG_TAG = "MSGEQ7";

/****** Function Prototypes ***********/

/************ ISR *********************/

/****** Private Data ******************/

/****** Private Functions *************/



static void pulse_rst(msg_handle_t *handle) {
    
    /** pulse the reset pin **/
    gpio_set_level(handle->rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(handle->rst_pin, 0);

}

static void pulse_strobe(msg_handle_t *handle) {

    /** pulse the strobe pin **/
    gpio_set_level(handle->strobe_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(handle->strobe_pin, 0);

}


static esp_err_t sample_all_channels(msg_handle_t *handle) {


    uint8_t channel = 0;
    int16_t adc_val = 0;

    pulse_rst(handle);

    for(channel=0; channel < MSG_CHANNELS; channel++) {
        pulse_strobe(handle);
        adc_val = adc1_get_raw(handle->adc_channel);
        /** so dirty **/
        *(&handle->data.band_1 + (sizeof(int16_t) * channel)) = adc_val;
    }

    return ESP_OK;
}



static void test_mode(msg_handle_t *handle) {

    int read = 0;
    /** both pins should be low **/
    gpio_set_level(handle->strobe_pin, 0);
   
    /** pulse the reset pin **/
    gpio_set_level(handle->rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(handle->rst_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(1));



    uint16_t arr[7] = {0};

    int ch = 0;

    gpio_set_level(handle->rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(handle->rst_pin, 0);

    for(uint16_t i=0; i < 7; i++) {
        ch = i+1;
        gpio_set_level(handle->strobe_pin, 1);
        vTaskDelay(pdMS_TO_TICKS(1));
        gpio_set_level(handle->strobe_pin, 0);
        vTaskDelay(pdMS_TO_TICKS(5));
        read = adc1_get_raw((adc1_channel_t)handle->adc_channel);
        arr[i] = read;
        vTaskDelay(5);
    }
    ESP_LOGI(MSG_TAG, "%u %u %u %u %u %u %u", arr[0], arr[1], arr[2], arr[3], arr[4], arr[5], arr[6]);
    /** try to read the ADC **/
    return;
}


static void msg_task(void *arg) {

    msg_handle_t *handle = (msg_handle_t *)arg;
    

    uint8_t verbose = 1;



    while (1)
    {
        // /* code */
        // if (handle->autosample) {
        //     ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        //     ESP_LOGI(MSG_TAG, "Got notified...");
        

        // }
        // else {
        //     /** just do nothing **/
        //     vTaskDelay(1000);
        // }

        test_mode(handle);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    /** here be dragons **/
}

/****** Global Data *******************/


TimerCallbackFunction_t msgeq7_timer_callback(TimerHandle_t timer) {

    BaseType_t higherPrio = pdFALSE;
    msg_handle_t *handle = (msg_handle_t *)pvTimerGetTimerID(timer);
    vTaskNotifyGiveFromISR(handle->task, &higherPrio);
    return NULL;
}


/****** Global Functions *************/

#ifdef CONFIG_DRIVERS_USE_HEAP
msg_handle_t *msg_init(msg_init_t *init) 
#else
msg_handle_t *msg_init(msg_handle_t *handle, msg_init_t *init) 
#endif
{

    esp_err_t istat = ESP_OK;
    gpio_config_t conf = {0};
    uint8_t channel = 0;

    if(init->data > 39 || init->data < 32) {
        ESP_LOGE(MSG_TAG, "Error, invalid GPIO for ADC1: Use 32-39 inclusive");
        istat = ESP_ERR_INVALID_ARG;
    }
    else {
        /** Get the channel from pin **/
        /** TODO: Make this a utility function **/
        uint32_t pin = init->data;
        channel = ADC1_CHANNEL_4;
    }

    /** init adc **/
    if(istat == ESP_OK) {
        istat = adc_gpio_init(ADC_UNIT_1, (adc_channel_t)channel);
    }

    if (istat == ESP_OK) {
        if (init->adc_width >= ADC_WIDTH_MAX) {
            istat = ESP_ERR_INVALID_ARG;
            ESP_LOGE(MSG_TAG, "Error, invalid ADC width: Use 0-3(4 for S2)");
        }
        else {
            istat = adc1_config_width(init->adc_width);
            adc1_config_channel_atten((adc_channel_t)channel, ADC_ATTEN_DB_11);
        }
    }

    /** Config GPIOs **/
    if(istat == ESP_OK) {
        conf.intr_type = GPIO_INTR_DISABLE;
        conf.mode = GPIO_MODE_OUTPUT;
        conf.pin_bit_mask = ((1 << init->rst ) | (1 << init->strobe));

        istat = gpio_config(&conf);
        gpio_set_level(init->rst, 0);
        gpio_set_level(init->strobe, 0);

    }

    /** Init handle **/
    if (istat == ESP_OK) {
#ifdef CONFIG_DRIVERS_USE_HEAP
        msg_handle_t *handle = (msg_handle_t *)heap_caps_calloc(1, sizeof(msg_handle_t), MALLOC_CAP_8BIT);
        if(handle == NULL) {
            ESP_LOGE(MSG_TAG, "Error alocating memory for handle!");
            istat = ESP_ERR_NO_MEM;
        }
#else
        memset(handle, 0, sizeof(msg_handle_t));
#endif
    }

    if(istat == ESP_OK) {
        handle->adc_bits = ADC_WIDTH_10Bit; /** TODO: make configurable **/
        handle->adc_channel = (adc_channel_t)channel;      
        handle->data_pin = init->data;
        handle->rst_pin = init->rst;
        handle->strobe_pin = init->strobe;
    }

    /** Init Task **/
    if (istat == ESP_OK) {
        TaskHandle_t th;
        if(xTaskCreate(msg_task, "msg_task", 2048, handle, 3, &th) != pdTRUE) {
            ESP_LOGE(MSG_TAG, "Error creating task!");
            istat = ESP_ERR_NO_MEM;
        }
        else {
            handle->task = th;
        }
    }

    if (istat == ESP_OK) {
        TimerHandle_t timer_handle = xTimerCreate("msgeq7_timer", pdMS_TO_TICKS(100), pdTRUE, handle, msgeq7_timer_callback);
        if(timer_handle == NULL) {
            ESP_LOGE(MSG_TAG, "Error creating the timer");
            istat = ESP_ERR_NO_MEM;
        }
        else {
            handle->timer = timer_handle;
        }
    }

#ifdef CONFIG_DRIVERS_USE_HEAP
    /** If init fails, free handle **/
    if(istat != ESP_OK && handle != NULL) {
        heap_caps_free(handle);
    }
#endif

    if(istat == ESP_OK) {
        ESP_LOGI(MSG_TAG, "Succesfully started MSG_EQ7 Driver!");
    } else {
        ESP_LOGI(MSG_TAG, "Failed to started MSG_EQ7 Driver!");
    }

// #ifdef DEBUG
//     test_mode(handle);
// #endif

    return handle;
}



esp_err_t msg_update_channels(msg_handle_t *handle) {

    esp_err_t err = ESP_OK;
    sample_all_channels(handle);
    return err;
}


esp_err_t msg_get_channel_1(msg_handle_t *handle, int16_t *val) {
    esp_err_t status = ESP_OK;
    *val = handle->data.band_1;        
    return status;
}

esp_err_t msg_get_channel_2(msg_handle_t *handle, int16_t *val) {
    esp_err_t status = ESP_OK;
    *val = handle->data.band_2;        
    return status;
}

esp_err_t msg_get_channel_3(msg_handle_t *handle, int16_t *val) {
    esp_err_t status = ESP_OK;
    *val = handle->data.band_3;        
    return status;
}

esp_err_t msg_get_channel_4(msg_handle_t *handle, int16_t *val) {
    esp_err_t status = ESP_OK;
    *val = handle->data.band_4;        
    return status;
}

esp_err_t msg_get_channel_5(msg_handle_t *handle, int16_t *val) {
    esp_err_t status = ESP_OK;
    *val = handle->data.band_5;        
    return status;
}

esp_err_t msg_get_channel_6(msg_handle_t *handle, int16_t *val) {
    esp_err_t status = ESP_OK;
    *val = handle->data.band_6;        
    return status;
}

esp_err_t msg_get_channel_7(msg_handle_t *handle, int16_t *val) {
    esp_err_t status = ESP_OK;
    *val = handle->data.band_7;        
    return status;
}
