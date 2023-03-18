/***************************************
* \file     APA102_Driver.c
* \brief    Driver for a strand of APA102 leds. The led driver handle is provided 
*           to the init function unless CONFIG_DRIVERS_USE_HEAP is defined, in which
*           case driver handle memory is assigned on the heap and the init function 
*           returns the handle pointer.
*           
*           Provides getters/setters for led colour, mode, etc
*           Provides a static task to manage driver handles.
*
*           REFACTOR: Trying to use a single driver task to control multiple driver handles means
*                     I can't pass the handle as the task argument any more. That only workds for a 
*                     single device driver instance.
*
*                     Options:
*                       + Obvious - make a queue, add a command to that queue containing a device handle
*                           and a command. Replacement for the task notification system currently using
*                       + Alternatively - use the task notification value to store a pointer to the 
*                           device handle, and set a command flag in the handle. Already use the 
*                           timer id to identify the handle. This would involve less changes.
*                       + 
*
*           Uses the SPI interface to control the APA102 led strips. The driver does not 
*           provide a limit to the number of handles based on this, allowing for use of external
*           switches to control more strands. 
*
* \date
* \author
****************************************/

/********* Includes *******************/

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "APA102_Driver.h"
#include "../Utils/LedEffects.h"
#include "../Utils/Utilities.h"
#include "esp_err.h"
#include "esp_log.h"

#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "freertos/FreeRTOSConfig.h"

#define DEBUG_MODE 1

/****** Function Prototypes ***********/

static esp_err_t send_frame_dma_polling(APA_HANDLE_t strand);

static int send_frame_polling(APA_HANDLE_t strand);

static void apa102_driver_task(void *args);


/****** Global Data *******************/

static uint8_t num_instances = 0;

static TaskHandle_t driver_task_handle = NULL;

static QueueHandle_t driver_task_queue = NULL;

const char *APA_TAG = "APA102 Driver";

#ifdef CONFIG_USE_PERIPH_MANAGER

#include "CommandAPI.h"

const parameter_t apa_param_map[apa_param_len] = {
    {"NumLeds", 1, &apa_getNumleds, NULL, NULL, DATATYPE_UINT32, 0, (GET_FLAG) },
    {"Mode", 2, &apa_getMode, &apa_setMode, NULL, DATATYPE_UINT8, LEDFX_NUM_EFFECTS, (GET_FLAG | SET_FLAG) },
    {"Colour", 3, &apa_getColour, &apa_setColour, NULL, DATATYPE_UINT32, UINT32_MAX, (GET_FLAG | SET_FLAG ) },
    {"Brightness", 4, &apa_getBrightness, &apa_setBrightness, NULL, DATATYPE_UINT8, 31, (GET_FLAG | SET_FLAG )},
};

const peripheral_t apa_periph_template = {
    .handle = NULL,
    .param_len = apa_param_len,
    .params = apa_param_map,
    .peripheral_name = "APA102",
    .peripheral_id = 0,
    .periph_type = PTYPE_ADDR_LEDS,
};
#endif

#ifdef DEBUG_MODE
const uint32_t init_frame[16] = {
    0x000000ff,
    0x0000ff00,
    0x00ff0000,
    0x0000ff00,
    0x000000ff,
    0x00ff00ff,
    0x0000ffff,
    0x00ffff00,
    0x00111111,
    0x00f0f0f0,
    0x000010ff,
    0x001000ff,
    0x0000ff10,
    0x00ff1000,
    0x0010ff00,
    0x00000000,
};
#endif

/************ ISR *********************/

/**
*   fxCallbackFunction
*   
*       Callback function for the timer expiry
*       set flag for Led Control task to deal with
*       Don't call the led effects functions here
**/
void timerExpiredCallback(TimerHandle_t timer)
{
    /** unblock the task to run the next animation **/
    APA_HANDLE_t strand = (APA_HANDLE_t ) pvTimerGetTimerID(timer);
#ifdef DEBUG_MODE
    ESP_LOGI(APA_TAG, "Timer Callback");
#endif
    xTaskNotify(driver_task_handle, APA_NOTIFY_BUILD_FRAME, eSetBits);
    xTimerReset(timer, APA_SEMTAKE_TIMEOUT);
    return;
}
/****** Private Data ******************/


/****** Private Functions *************/


static esp_err_t send_frame_dma_polling(APA_HANDLE_t strand) {
    /** TODO: No point in polling DMA... Do this properly! **/
    esp_err_t txStatus = ESP_OK;

    spi_transaction_t trx = {0};
    
    trx.length = (strand->write_length * 8);
    trx.rxlength = 0;
    trx.rx_buffer = NULL;
    trx.tx_buffer = strand->data_address;

#ifdef DEBUG_MODE
    // showmem(strand->strandFrameStart, strand->strandFrameLength);
#endif

    txStatus = spi_device_polling_transmit(strand->iface_handle, &trx);
    if(txStatus != ESP_OK) {
        ESP_LOGE(APA_TAG, "Error sending spi dma {%u}", txStatus);
    }
    return txStatus;
}

/** send a polling transaction **/
static esp_err_t send_frame_polling(APA_HANDLE_t strand)
{

    esp_err_t txStatus = ESP_OK;

    spi_transaction_t tx = {0};
    tx.length = (4 * 8);
    tx.rx_buffer = NULL;
    tx.rxlength = 0;
    bool got_sem = false;

    for(uint32_t i=0; i < strand->write_length; i++) {
        tx.tx_buffer = strand->data_address + (i * 4);
        txStatus = spi_device_polling_transmit(strand->iface_handle, &tx);
    }

    if (txStatus != ESP_OK)
    {
        ESP_LOGE("SPI_TX", "Error in sending %u bytes [%u]", strand->write_length, txStatus);
    }
    
    return txStatus;
}


/** Control task **/
static void apa102_driver_task(void *args) {

    APA_HANDLE_t strand;
    apa_msg_t rx_cmd = {0};
    BaseType_t rx_success;

    while(driver_task_queue == NULL) {
        /** Do not start the task proper whilst the queue is
         *  uninitialised
         **/
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    while(1) {

//         /** Wait forever for command **/
//         rx_success = xQueueReceive(driver_task_queue, &rx_cmd, portMAX_DELAY);

//         ESP_LOGI(APA_TAG, "Command Received: %02x", rx_cmd.cmd);

//         if(rx_success == pdTRUE) {
            
//             strand = rx_cmd.strand;
//             ESP_LOGI(APA_TAG, "Command Received: %02x", rx_cmd.cmd);
//             if(rx_cmd.cmd == APA_CMD_NEW_MODE) {
// #ifdef DEBUG_MODE
//                 ESP_LOGI(APA_TAG, "Updating mode timer and animation");
// #endif
//                 strand->effects.render_new_frame = true;
//                 if(xTimerGetPeriod(strand->led_timer) != strand->effects.refresh_t) {
//                     /** stop running timer,  **/
//                     if(xTimerStop(strand->led_timer, pdMS_TO_TICKS(100)) != pdPASS) {
//                         ESP_LOGE(APA_TAG, "Error, failed to stop running timer");                        
//                     }
//                     else if(xTimerChangePeriod(strand->led_timer, strand->effects.refresh_t, pdMS_TO_TICKS(100)) != pdPASS) {
//                         ESP_LOGE(APA_TAG, "Error, failed to change timer period");
//                     }
//                     else if(xTimerReset(strand->led_timer, pdMS_TO_TICKS(100)) != pdPASS) {
//                         ESP_LOGE(APA_TAG, "Error, failed to reset timer");
//                     }
//                     else if (xTimerStart(strand->led_timer, pdMS_TO_TICKS(100)) != pdPASS){
//                         ESP_LOGE(APA_TAG, "Error, failed to reset timer");
//                     }
//                 }
//             }

//             if(rx_cmd.cmd == APA_CMD_UPDATE_FRAME || strand->effects.render_new_frame) {
// #ifdef DEBUG_MODE
//                 ESP_LOGI(APA_TAG, "Calling animation");
// #endif
//                 if(xSemaphoreTake(strand->strand_sem, pdMS_TO_TICKS(100)) != pdTRUE) {
//                     ESP_LOGE(APA_TAG, "Failed to get mem semaphore");
//                 }
//                 else
//                 {
//                     strand->effects.func((void *)strand);
//                     xSemaphoreGive(strand->strand_sem);
//                 }
//             }
//         }
//             /** Check for command or flag set from animation function called above **/
//             if(rx_cmd.cmd == APA_CMD_UPDATE_LEDS || strand->effects.write_new_frame) {
// #ifdef DEBUG_MODE
//                 ESP_LOGI(APA_TAG, "Writing frame");
// #endif
//                 if(xSemaphoreTake(strand->strand_sem, APA_SEMTAKE_TIMEOUT) != pdTRUE) {
//                     ESP_LOGE(APA_TAG, "Failed to get LED semaphore");
//                 } 
//                 else {
//                     if(strand->use_dma) {
//                         if(send_frame_dma_polling(strand) != ESP_OK) {
//                             ESP_LOGE(APA_TAG, "Failed to write to leds");                
//                         }
//                     }
//                     else {
//                         if(send_frame_polling(strand) != ESP_OK) {
//                             ESP_LOGE(APA_TAG, "Failed to write to leds");       
//                         }
//                         else {
//                             strand->effects.write_new_frame = false; 
//                         }
//                     }
//                     if( xSemaphoreGive(strand->strand_sem) != pdTRUE) {
//                         ESP_LOGE(APA_TAG, "error giving sem");
//                     }
//                 }
//             }
        // }
#ifdef DEBUG
        else {
            ESP_LOGE(APA_TASK, "Error, queue wait expired without success :(");
        }
#endif
        
        /** short delay in case of some stuck bits **/
        vTaskDelay(5);

    }
}



/****** Global Functions *************/

esp_err_t apa_getNumleds(APA_HANDLE_t strand, uint32_t *var) {
    esp_err_t status = ESP_OK;
    *var = strand->num_leds;
    return status;
}

esp_err_t apa_getMode(APA_HANDLE_t strand, uint32_t *var) {
    esp_err_t status = ESP_OK;
    *var = strand->effects.effect;
    return status;
}

esp_err_t apa_setMode(APA_HANDLE_t strand, uint8_t  *mode) {
    esp_err_t status = ESP_OK;
    uint8_t m = *mode;
    apa_msg_t cmd;

    if(m > LEDFX_NUM_EFFECTS) {
        status = ESP_ERR_INVALID_ARG;
    } else {
        ledfx_set_mode(strand, m);
        cmd.cmd = APA_CMD_NEW_MODE;
        cmd.strand = strand;
        if(xQueueSend(driver_task_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGE(APA_TAG, "Error sending command to queue (queue full)");
            status = ESP_ERR_TIMEOUT;
        }
    }
    return status;
}

esp_err_t apa_getColour(APA_HANDLE_t strand, uint32_t *var) {

    esp_err_t status = ESP_OK;
    *var = strand->effects.colour;
    return status;
}

esp_err_t apa_setColour(APA_HANDLE_t strand, uint32_t *var) {
    esp_err_t status = ESP_OK;
    uint32_t c = *var;
    strand->effects.colour = c;

    apa_msg_t msg = {
        .strand = strand,
        .cmd = APA_CMD_UPDATE_FRAME
    };
    
    if(xQueueSend(driver_task_queue, &msg, pdMS_TO_TICKS(100)) != pdPASS){
        status = ESP_ERR_NO_MEM;
    }
    return status;
}

esp_err_t apa_getBrightness(APA_HANDLE_t strand, uint8_t *var) {
    esp_err_t status = ESP_OK;
    *var = strand->effects.brightness;
    return status;
}

esp_err_t apa_setBrightness(APA_HANDLE_t strand, uint8_t *var) {
    esp_err_t status = ESP_OK;
    uint8_t v = *var;
    if(v > APA_CTRL_MAX_BR) {
        status = ESP_ERR_INVALID_ARG;
    } else {
        strand->effects.brightness = v;

        apa_msg_t msg = {
            .strand = strand,
            .cmd = APA_CMD_UPDATE_FRAME
        };
        
        if(xQueueSend(driver_task_queue, &msg, pdMS_TO_TICKS(100)) != pdPASS){
            status = ESP_ERR_NO_MEM;
        }
    }
    return status;
}


#ifdef CONFIG_DRIVERS_USE_HEAP
APA_HANDLE_t APA102_init(apa102_init_t *init)
#else
APA_HANDLE_t APA102_init(APA_HANDLE_t strand, apa102_init_t *init)
#endif
{

    esp_err_t err = ESP_OK;
    uint32_t led_mem_sz;
    uint8_t ledmem_caps;

    if(init->channel > VSPI_HOST) {
        ESP_LOGE(APA_TAG, "Error - invalid channel");
        err = ESP_ERR_INVALID_ARG;
    }

    if(num_instances >= APA_CONFIG_MAX_DEVICES) {
        ESP_LOGE(APA_TAG, "Error - too many instances running");
        err = ESP_ERR_NOT_SUPPORTED;
    }

    if(!err) {
#ifdef CONFIG_DRIVERS_USE_HEAP
        APA_HANDLE_t strand = (APA_HANDLE_t )heap_caps_calloc(1, sizeof(LedStrand_t), MALLOC_CAP_8BIT);
        if(strand == NULL) {
            ESP_LOGE(APA_TAG, "Error assigning strand memory");
            err = ESP_ERR_NO_MEM;
        }
#else
        memset(strand, 0, sizeof(LedStrand_t));
#endif
    }

    if(!err) {
        strand->led_t = LEDTYPE_APA102;
        strand->num_leds = init->numleds;
        strand->channel = init->channel;
    }


    if (err == ESP_OK)
    {
        spi_device_interface_config_t leds = {0};
        leds.mode = 1;
        leds.duty_cycle_pos = 128;
        leds.spics_io_num = -1;
        leds.queue_size = 16;
        leds.clock_speed_hz = APA_CONFIG_SPI_CLK_HZ; // APA claim to have refresh rate of 4KHz, start low.


        err = spi_bus_add_device(init->channel, &leds, &strand->iface_handle);
        if(err) {
            ESP_LOGE(APA_TAG, "Error adding SPI device! {%u}", err);
        }
    }

    /** only start the driver queue once **/
    if(driver_task_queue == NULL) {
        driver_task_queue = xQueueCreate(APA_CONFIG_CMD_QUEUE_LEN, sizeof(apa_msg_t));
        if(driver_task_queue == NULL) {
            ESP_LOGE(APA_TAG, "Error starting driver queue");
            err = ESP_ERR_NO_MEM;
        }
    }


    /** only start the driver task once **/
    if(driver_task_handle == NULL ) {
        if(xTaskCreate(apa102_driver_task, "apa102_driver_task", 5012, NULL, 3, &driver_task_handle) != pdTRUE) {
            ESP_LOGE(APA_TAG, "Error starting driver task");
            err = ESP_ERR_NO_MEM;
        }
    }


    if(!err) {
        /** memory allocation for the strand as follows: 
         *  -   4 bytes per led
         *  -   4 bytes of start bits
         *  -   4 bytes of end bits for every 2 leds
         *  **/
        led_mem_sz = (strand->num_leds * APA_BYTES_PER_PIXEL) + ((uint8_t)(strand->num_leds / 2) * APA_END_FRAME_SZ) + APA_START_FRAME_SZ;

#ifdef DEBUG_MODE
        ESP_LOGI(APA_TAG, "Assigning %u bytes for %u led strand", led_mem_sz, strand->num_leds);
#endif

        if(led_mem_sz > APA_CONFIG_MAX_LEDMEM_BYTES) {
            err = ESP_ERR_INVALID_ARG;
            ESP_LOGE(APA_TAG, "Error, too many leds! {%u}", err);

        }

        ledmem_caps = (init->use_dma == true ? (MALLOC_CAP_32BIT | MALLOC_CAP_DMA) : (MALLOC_CAP_32BIT));

        void *strand_mem = heap_caps_malloc(led_mem_sz, ledmem_caps);

        if(strand_mem == NULL) {
            ESP_LOGE(APA_TAG, "Error assigning strand heap memory");
            err = ESP_ERR_NO_MEM;
        }
        else {
            strand->data_address = strand_mem;
            strand->pixel_start = strand_mem + APA_START_FRAME_SZ;
            strand->pixel_end = strand->pixel_start + (APA_BYTES_PER_PIXEL * strand->num_leds);
            strand->write_length = led_mem_sz;
            memset(strand->data_address, 0x00, sizeof(uint8_t) * led_mem_sz);
            memset(strand->data_address, 0xFFFFFFFF, sizeof(uint32_t));
#ifdef DEBUG_MODE
        ESP_LOGI(APA_TAG, "Led strand memory at address %08x", (uint32_t )strand->data_address);
#endif
        }
    }

    if (err == ESP_OK)
    {

        strand->led_timer = xTimerCreate("led_timer", UINT16_MAX, pdTRUE, (void *)strand, timerExpiredCallback);
        strand->strand_sem = xSemaphoreCreateMutex();

        if (strand->led_timer == NULL || strand->strand_sem == NULL)
        {
            ESP_LOGE(APA_TAG, "Error in setting up timer/semaphore");
            err = ESP_ERR_NO_MEM;
        }
    }
    

#ifdef DEBUG_MODE
    if (err == ESP_OK)
    {
        //test_frame_polling(strand);
        memcpy(strand->pixel_start, init_frame, (sizeof(uint32_t) * strand->num_leds));
        showmem(strand->data_address, strand->write_length);
        xTimerStart(strand->led_timer, pdMS_TO_TICKS(5));
        send_frame_polling(strand);
    }
#endif
    
    if(err == ESP_OK) {
        ESP_LOGI(APA_TAG, "APA strand initialised");
        num_instances++;
    } else {
        ESP_LOGE(APA_TAG, "Error initialising APA strand (%u)", err);
#ifdef CONFIG_DRIVERS_USE_HEAP
        if(strand != NULL) {
            heap_caps_free(strand);
            strand = NULL;
        }
#endif
    }

    return strand;
}



