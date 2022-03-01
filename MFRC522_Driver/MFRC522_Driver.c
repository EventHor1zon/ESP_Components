/***************************************
* \file     MFRC522_Driver.c
* \brief    A driver for the MFRC RFID read/write IC
*
* \date     November 2021
* \author   RJAM
****************************************/

/********* Includes *******************/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_types.h"
#include "esp_err.h"
#include "esp_log.h"
#include "Utilities.h"
#include "genericCommsDriver.h"

#include "MFRC522_Driver.h"

/****** Global Data *******************/

const char *MFRC_TAG = "MFRC Driver";

/****** Function Prototypes ***********/

/************ ISR *********************/

void mfrc_irq_function(void *args) {


}

/****** Private Data ******************/

/****** Private Functions *************/

static esp_err_t mfrc_read_from_address(MFRC_DEV dev, uint8_t address, uint8_t len, uint8_t *buffer) {

    esp_err_t err = ESP_OK;

    if(dev->comms_mode == MFRC_SPI_COMMS_MODE) {
        
    }
    else if (dev->comms_mode == MFRC_I2C_COMMS_MODE) {

    }
    else {
        err = ESP_ERR_INVALID_ARG;
    }

    return err;
}



static void mrfc_driver_task(void *args) {
 
   while(1) {
       vTaskDelay(pdMS_TO_TICKS(10));
   }
   /** here be dragons **/
}


/****** Global Functions *************/


MFRC_DEV mfrc_init(mfrc_init_t *init) {

    MFRC_DEV handle = NULL;
    gpio_config_t pin_config = {0};
    esp_err_t err = ESP_OK;

    if(init->comms_mode => MFRC_COMMS_MODE_INVALID) {
        err = ESP_ERR_INVALID_ARG;
    }


    /** create handle **/
    if(!err) {
        handle = heap_caps_calloc(1, sizeof(MFRC522_Driver_t), MALLOC_CAP_DEFAULT);
        if(handle == NULL) {
            err = ESP_ERR_NO_MEM;
            ESP_LOGE(MFRC_TAG, "Error - unable to assign heap memory for handle [%u]", );
        }
        else {
            handle->comms_bus = init->comms_bus;
            handle->comms_mode = init->comms_mode;
            handle->irq_pin = init->irq_pin;
            handle->rst_pin = init->rst_pin;
        }
    }

    /** initialise the pins **/
    if(!err) {
    
        pin_config.intr_type = GPIO_INTR_DISABLE;
        pin_config.mode = GPIO_MODE_OUTPUT;
        pin_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
        pin_config.pull_up_en = GPIO_PULLUP_ENABLE;

        /** reset pin optional, init **/
        if(init->rst_pin > 0) {
            pin_config.pin_bit_mask = (1 << init->rst_pin);
            err = gpio_config(&pin_config);
            if(err) {
                ESP_LOGE(MFRC_TAG, "Error configuring reset pin {%u}", err);
            }
            else {
                handle->rst_en = true;
            }
        }

        if(!err && init->irq_pin) {
            pin_config.pin_bit_mask = (1 << init->irq_pin);
            pin_config.intr_type = GPIO_INTR_NEGEDGE;
            err = gpio_config(&pin_config);
            if(err) {
                ESP_LOGE(MFRC_TAG, "Error configuring irq pin {%u}", err);
            }
            else {
                err = gpio_isr_handler_add(init->irq_pin, mfrc_irq_function, handle);
                if(err) {
                    ESP_LOGE(MFRC_TAG, "Error adding isr handler {%u}", err);
                }
                else {
                    handle->irq_en = true;
                }
            }
        }
    }
    

    /** create task **/
    if(!err) {
        if(xTaskCreate(mrfc_driver_task, "mrfc_driver_task", 5012, handle, 3, &handle->task) != pdTRUE) {
            ESP_LOGE(MFRC_TAG, "Error creating driver task!");
            err = ESP_ERR_NO_MEM;
        }
    }


    if(err) {
        ESP_LOGE(MFRC_TAG, "Error initialising MFRC522 driver [%u]", err);
        if(handle) {
            heap_caps_free(handle);
        }
    } 
    else {
        ESP_LOGI(MFRC_TAG, "Succesfully started MFRC522 Driver");
    }

    return handle;
}




void mfrc_deinit(MFRC_DEV dev) {

    if(dev != NULL) {
        heap_caps_free(dev);
    }

    return;
}






