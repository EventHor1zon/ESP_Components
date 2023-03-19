/***************************************
* \file     LedStrip_Driver.c
* \brief    See header
*
* \date     March 23
* \author   RJAM
***************************************/

/********* Includes *******************/

#include "esp_err_t"

#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "freertos/FreeRTOSConfig.h"

#include "LedStrip_Driver.h"

/****** Private Data ******************/

static TaskHandle_t ledstrip_taskhandle;
static QueueHandle_t command_queue;
static bool is_initialised = false;

const static char * LS_TAG = "LedStrip Driver";


/****** Global Data *******************/

const ledtype_t ledstrip_types[2] = {
    {.name="ws2812b", .pixel_bytes=3, .pixel_index_red=1, .pixel_index_blue=2, .pixel_index_green=0, .brt_bits=0, .pixel_index_brt=0, .brt_base=0},
    {.name="apa102", .pixel_bytes=4, .pixel_index_red=3, .pixel_index_blue=1, .pixel_index_green=2, .brt_bits=5, .pixel_index_brt=0, .brt_base=0b11100000},
};

const uint8_t led_type_n = 2;

/****** Function Prototypes ***********/

/** @name ledstrip_driver_task
 *  @brief driver task awaits commands from timers or functions
 *         and executes them. Controls calling animation functions,
 *          resetting timers and calling the write function
 **/
static void ledstrip_driver_task(void *args);


/************ ISR *********************/

/******** Private Functions ***********/


/**
 * 
 * 
 **/
static void ledstrip_driver_task(void *args) {

    BaseType_t rx_success;

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
    }
}



/****** Global Functions **************/

esp_err_t ledstrip_driver_init() {

    esp_err_t err = ESP_OK;

    if(is_initialised == true) {
        ESP_LOGI(LS_TAG, "Driver is already running");
        err = ESP_ERR_INVALID_STATE;
    }

    command_queue = xQueueCreate(LEDSTRIP_CONFIG_QUEUE_LEN, sizeof(ledstrip_cmd_t)))

    if(command_queue == NULL) {
        ESP_LOGE(LS_TAG, "Unable to create command queue");
        err = ESP_ERR_NO_MEM;
    }

}