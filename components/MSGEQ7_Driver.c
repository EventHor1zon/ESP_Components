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
#include "/home/rich/Tools/ToolChains/ESP/esp-idf/components/esp_adc_cal/include/esp_adc_cal.h"
#include "/home/rich/Tools/ToolChains/ESP/esp-idf/components/soc/soc/esp32/include/soc/adc_channel.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "MSGEQ7_Driver.h"

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
    vTaskDelay(pdMS_TO_TICKS(1));

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
    for(uint16_t j=0; j < 1000; j++) {
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
    }
    /** try to read the ADC **/
    return;
}


static void msg_task(void *arg) {

    msg_handle_t *handle = (msg_handle_t *)arg;
    

    while (1)
    {
        /* code */
        ESP_LOGI(MSG_TAG, "In task");
        vTaskDelay(1000);

    }
    /** here be dragons **/
}

/****** Global Data *******************/

/****** Global Functions *************/


msg_handle_t *msg_init(msg_init_t *init) {

    esp_err_t istat = ESP_OK;
    gpio_config_t conf = {0};

    msg_handle_t *handle = NULL;

    uint8_t channel = 0;

    if(init->data > 39 || init->data < 32) {
        ESP_LOGE(MSG_TAG, "Error, invalid GPIO for ADC1: Use 32-39 inclusive");
        istat = ESP_ERR_INVALID_ARG;
    }
    else {
        /** Get the channel from pin **/
        /** TODO: Make this a utility function **/
        uint8_t pin = init->data;

        switch(pin) {
        case 32:
                channel = ADC1_GPIO32_CHANNEL;
                break;
        case 33:
                channel = ADC1_GPIO33_CHANNEL;
                break;
        case 34:
                channel = ADC1_GPIO34_CHANNEL;
                break;
        case 35:
                channel = ADC1_GPIO35_CHANNEL;
                break;
        case 36:
                channel = ADC1_GPIO36_CHANNEL;
                break;
        case 37:
                channel = ADC1_GPIO37_CHANNEL;
                break;
        case 38:
                channel = ADC1_GPIO38_CHANNEL;
                break;
        case 39:
                channel = ADC1_GPIO39_CHANNEL;
                break;
        default:
                istat = ESP_ERR_INVALID_ARG;
                break;
        }
    }

    /** init adc **/
    if(istat == ESP_OK) {
        istat = adc_gpio_init(ADC_UNIT_1, (adc_channel_t)channel);
    }

    if (istat == ESP_OK) {
        adc1_config_width(ADC_WIDTH_10Bit);
        adc1_config_channel_atten((adc_channel_t)channel, ADC_ATTEN_DB_11);
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
        handle = (msg_handle_t *)heap_caps_calloc(1, sizeof(msg_handle_t), MALLOC_CAP_8BIT);
        if(handle == NULL) {
            ESP_LOGE(MSG_TAG, "Error alocating memory for handle!");
            istat = ESP_ERR_NO_MEM;
        }
        else {
            handle->adc_bits = ADC_WIDTH_10Bit; /** TODO: make configurable **/
            handle->adc_channel = (adc_channel_t)channel;      
            handle->data_pin = init->data;
            handle->rst_pin = init->rst;
            handle->strobe_pin = init->strobe;
        }
    }

    /** Init Task **/
    if (istat == ESP_OK) {
        TaskHandle_t th;
        if(xTaskCreate(msg_task, "msg_task", 2048, handle, 3, &th) != pdTRUE) {
            ESP_LOGE(MSG_TAG, "Error creating task!");
            istat = ESP_ERR_NO_MEM;
        }
    }


    /** If init fails, free handle **/
    if(istat != ESP_OK && handle != NULL) {
        heap_caps_free(handle);
    }

    if(istat == ESP_OK) {
        ESP_LOGI(MSG_TAG, "Succesfully started MSG_EQ7 Driver!");
    } else {
        ESP_LOGI(MSG_TAG, "Failed to started MSG_EQ7 Driver!");
    }

#ifdef DEBUG
    test_mode(handle);
#endif

    return handle;
}