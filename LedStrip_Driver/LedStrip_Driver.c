/***************************************
* \file     LedStrip_Driver.c
* \brief    See header
*
* \date     March 23
* \author   RJAM
***************************************/

/********* Includes *******************/

#include "esp_err.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

#include "driver/rmt.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"

#include "string.h"

#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "freertos/FreeRTOSConfig.h"

#include "LedEffects.h"
#include "LedStrip_Driver.h"

/****** Private Data ******************/

static TaskHandle_t ledstrip_taskhandle;
static QueueHandle_t command_queue;
static bool is_initialised = false;
static uint8_t num_strips = 0;

const static char * LS_TAG = "LedStrip Driver";


/****** Global Data *******************/

const ledtype_t ledstrip_types[2] = {
    {.name="ws2812b", .pixel_bytes=3, .pixel_index_red=1, .pixel_index_blue=2, .pixel_index_green=0, .brt_bits=0, .pixel_index_brt=0, .brt_base=0},
    {.name="apa102", .pixel_bytes=4, .pixel_index_red=3, .pixel_index_blue=1, .pixel_index_green=2, .brt_bits=5, .pixel_index_brt=0, .brt_base=0b11100000},
};

const uint8_t led_type_n = 2;

struct ws2812b_timing_t
{
    uint32_t T0H;
    uint32_t T1H;
    uint32_t T0L;
    uint32_t T1L;
    uint32_t TRS;
};

const struct ws2812b_timing_t wsT = {
    400,
    800,
    550,
    850,
    450,
};

/****** Function Prototypes ***********/

/** @name ledstrip_driver_task
 *  @brief driver task awaits commands from timers or functions
 *         and executes them. Controls calling animation functions,
 *          resetting timers and calling the write function
 **/
static void ledstrip_driver_task(void *args);

/** @name   ledstrip_spi_write
 *  @brief  abstract spi write function
 *          for dual signal addressable leds
 */
static esp_err_t ledstrip_spi_write();

/** @name   ledstrip_rmt_write
 *  @brief  abstract remote control transceiver
 *          write function for single signal 
 *          addressable leds 
 */
static esp_err_t ledstrip_rmt_write();

/************ ISR *********************/

/**
*   fxCallbackFunction
*   
*       Callback function for the timer expiry
*       set flag for Led Control task to deal with
*       Don't call the led effects functions here
**/
void led_timer_callback(TimerHandle_t timer)
{
    /** unblock the task to run the next animation **/
    LEDSTRIP_h strip = (LEDSTRIP_h ) pvTimerGetTimerID(timer);
#ifdef DEBUG_MODE
    ESP_LOGI(LS_TAG, "Timer Callback");
#endif

}


static void IRAM_ATTR ws2812_TranslateDataToRMT(const void *src, rmt_item32_t *dest, size_t src_size,
                                                size_t wanted_num, size_t *translated_size, size_t *item_num)
{
    if (src == NULL || dest == NULL)
    {
        *translated_size = 0;
        *item_num = 0;
        return;
    }

    uint32_t duration0_0 = (wsT.T0H / LEDSTRIP_CONFIG_RMT_CLK_T_MS);
    uint32_t duration1_0 = (wsT.T0L / LEDSTRIP_CONFIG_RMT_CLK_T_MS);
    uint32_t duration0_1 = (wsT.T1H / LEDSTRIP_CONFIG_RMT_CLK_T_MS);
    uint32_t duration1_1 = (wsT.T1L / LEDSTRIP_CONFIG_RMT_CLK_T_MS);

    /* rmt_item structs - contain the logic order & timings to write 0 or 1 to ws2812 */

    const rmt_item32_t bit_0 = {
        /* logic 0 */
        .level0 = 1, /* the 1st level - for ws2812 this is high */
        .duration0 = duration0_0,
        .level1 = 0, /* 2nd level - for ws2812 it's low */
        .duration1 = duration1_0,
    };

    const rmt_item32_t bit_1 = {
        /* logic 1 */
        .level0 = 1,
        .duration0 = duration0_1,
        .level1 = 0,
        .duration1 = duration1_1,
    };

    size_t currentSize = 0;
    size_t num = 0;
    uint8_t *psrc = (uint8_t *)src;
    rmt_item32_t *pdest = dest;
    while (currentSize < src_size && num < wanted_num)
    {
        for (int i = 0; i < 8; i++)
        {
            if (*psrc & (0x1 << i))
            {
                pdest->val = bit_1.val;
            }
            else
            {
                pdest->val = bit_0.val;
            }
            num++;
            pdest++;
        }
        currentSize++;
        psrc++;
    }
    *translated_size = currentSize;
    *item_num = num;
}

/******** Private Functions ***********/


static esp_err_t ledstrip_rmt_write(LEDSTRIP_h strip)
{
    esp_err_t transmitStatus;
    transmitStatus = rmt_write_sample(strip->channel, (uint8_t *)strip->strand_mem_start, strip->write_length, false);
    return transmitStatus;
}


/** send a polling transaction **/
static esp_err_t ledstrip_spi_write(LEDSTRIP_h strip)
{

    esp_err_t txStatus = ESP_OK;

    spi_transaction_t tx = {0};
    tx.length = (32 * 8);
    tx.rx_buffer = NULL;
    tx.rxlength = 0;
    bool got_sem = false;

    for(uint32_t i=0; i < (strip->write_length % 32); i++) {
        tx.tx_buffer = strip->strand_mem_start + (i * 32);
        txStatus = spi_device_polling_transmit(strip->interface_handle, &tx);

        if (txStatus != ESP_OK)
        {
            ESP_LOGE("SPI_TX", "Error in sending %u bytes [%u]", strip->write_length, txStatus);
        }
    }

    
    return txStatus;
}


static void ledstrip_driver_task(void *args) {

    BaseType_t rx_success;
    esp_err_t err;

    LEDSTRIP_h strip;
    ls_cmd_t rx_cmd;

    while(1) {

        /** Wait forever for command **/
        rx_success = xQueueReceive(command_queue, &rx_cmd, portMAX_DELAY);

        ESP_LOGI(LS_TAG, "Command Received: %02x", rx_cmd.cmd);

        if(rx_success == pdTRUE) {
            
            strip = rx_cmd.strip;
            ESP_LOGI(LS_TAG, "Command Received: %02x", rx_cmd.cmd);
            if(rx_cmd.cmd == LS_CMD_NEW_MODE) {
#ifdef DEBUG_MODE
                ESP_LOGI(LS_TAG, "Updating mode timer and animation");
#endif
                strip->render_frame = true;
                if(xTimerGetPeriod(strip->timer) != strip->fx.frame_time) {
                    /** stop running timer,  **/
                    if(xTimerStop(strip->timer, LEDSTRIP_CONFIG_GENERIC_TIMEOUT) != pdTRUE) {
                        ESP_LOGE(LS_TAG, "Error, failed to stop running timer");                        
                    }
                    else if(xTimerChangePeriod(strip->timer, strip->fx.frame_time, LEDSTRIP_CONFIG_GENERIC_TIMEOUT) != pdTRUE) {
                        ESP_LOGE(LS_TAG, "Error, failed to change timer period");
                    }
                    else if(xTimerReset(strip->timer, LEDSTRIP_CONFIG_GENERIC_TIMEOUT) != pdTRUE) {
                        ESP_LOGE(LS_TAG, "Error, failed to reset timer");
                    }
                    else if (xTimerStart(strip->timer, LEDSTRIP_CONFIG_GENERIC_TIMEOUT) != pdTRUE){
                        ESP_LOGE(LS_TAG, "Error, failed to reset timer");
                    }
                }
            }

            /** Call the animation update function **/
            if(rx_cmd.cmd == LS_CMD_UPDATE_FRAME || strip->render_frame) {
#ifdef DEBUG_MODE
                ESP_LOGI(LS_TAG, "Calling animation function");
#endif
                if(xSemaphoreTake(strip->sem, LEDSTRIP_CONFIG_GENERIC_TIMEOUT) != pdTRUE) {
                    ESP_LOGE(LS_TAG, "Failed to get mem semaphore");
                }
                else
                {
                    strip->fx.func((void *)strip);
                    xSemaphoreGive(strip->sem);
                    strip->render_frame = false;
                }
            }

            /** Check for command or flag set from animation function called above **/
            if(rx_cmd.cmd == LS_CMD_UPDATE_LEDS || strip->write_frame) {
#ifdef DEBUG_MODE
                ESP_LOGI(LS_TAG, "Writing frame");
#endif
                if(xSemaphoreTake(strip->sem, LEDSTRIP_CONFIG_GENERIC_TIMEOUT) != pdTRUE) {
                    ESP_LOGE(LS_TAG, "Failed to get LED semaphore");
                } 
                else {
                    
                    err = strip->led_type->write(strip);

                    if(err) {
                        ESP_LOGE(LS_TAG, "Error in write function [%u]", err);
                    }

                    xSemaphoreGive(strip->sem);
                    strip->write_frame = false;
                }
            }
        }
        /** Nothing received from queue **/
        else {
            ESP_LOGI(LS_TAG, "Nothing receiving command from queue [%u]", rx_success);
        }

    }

    /** Here be dragons **/
}



/****** Global Functions **************/

esp_err_t ledstrip_driver_init() {

    esp_err_t err = ESP_OK;

    if(is_initialised == true) {
        ESP_LOGI(LS_TAG, "Driver is already running");
        err = ESP_ERR_INVALID_STATE;
    }

    if(!err) {
        command_queue = xQueueCreate(LEDSTRIP_CONFIG_QUEUE_LEN, sizeof(ls_cmd_t));

        if(command_queue == NULL) {
            ESP_LOGE(LS_TAG, "Unable to create command queue");
            err = ESP_ERR_NO_MEM;
        }
    }

    if(!err) {
        if(xTaskCreate(ledstrip_driver_task, "ledstrip_driver_task", LEDSTRIP_CONFIG_TASK_STACK, NULL, LEDSTRIP_CONFIG_TASK_PRIO, &ledstrip_taskhandle) != pdTRUE) {
            ESP_LOGE(LS_TAG, "Unable to create driver task");
            err = ESP_ERR_NO_MEM;
        }
    }

    if(!err) {
        ESP_LOGI(LS_TAG, "LedStrip driver started");
        is_initialised = true;
    }
    else {
        ESP_LOGE(LS_TAG, "Error starting LedStrip driver");
    }

    return err;
}


esp_err_t ledstrip_add_strip(LEDSTRIP_h strip, ledstrip_init_t *init) {

    esp_err_t err = ESP_OK;
    void *strip_ptr=NULL;
    uint32_t strip_mem_len=0;
    uint32_t offset;
    uint8_t mem_flags;
    ledtype_t *type=NULL;
    TimerHandle_t timer=NULL;
    SemaphoreHandle_t sem=NULL;
    char name_buffer[32];

    if(is_initialised == false) {
        ESP_LOGE(LS_TAG, "Driver has not been initialised");
        err = ESP_ERR_INVALID_STATE;
    }

    if(!err && num_strips >= LEDSTRIP_CONFIG_MAX_STRIPS) {
        ESP_LOGE(LS_TAG, "Max supported strips reached");
        err = ESP_ERR_INVALID_STATE;
    }

    if(!err && init->num_leds > LEDSTRIP_CONFIG_MAX_LEDS) {
        ESP_LOGE(LS_TAG, "Number of leds exceeds maximum");
        err = ESP_ERR_INVALID_STATE;
    }

    if(!err && init->led_type >= LED_TYPE_INVALID) {
        ESP_LOGE(LS_TAG, "Invalid led type");
        err = ESP_ERR_INVALID_ARG;
    }

    if(!err) {
        memset(name_buffer, 0, sizeof(uint8_t ) * 32);
        sprintf(name_buffer, "ls_tmr_%u", num_strips);
        timer = xTimerCreate(name_buffer, portMAX_DELAY, pdTRUE, strip, led_timer_callback);
        
        if(timer == NULL) {
            ESP_LOGE(LS_TAG, "Unable to create led timer");
            err = ESP_ERR_NO_MEM;
        }
    }

    if(!err) {
        sem = xSemaphoreCreateMutex();
        
        if(sem == NULL) {
            ESP_LOGE(LS_TAG, "Unable to create led memory semaphore");
            err = ESP_ERR_NO_MEM;
        }
    }

    if(!err) {
        /** get the led type, sum up bytes required for full frame
         *  buffer to 32-bit aligned if required
         * TODO: when is it required? 32-bit dma trx
         */
        type = &led_types[init->led_type];

        err = type->init(strip, init);
    }

    if(!err) {
        strip_mem_len = (type->pixel_bytes * init->num_leds);
        strip_mem_len += (type->start_len);
        strip_mem_len += (type->end_len);
    
#ifdef DEBUG_MODE
        ESP_LOGI("Assigning %u heap bytes for strip memory", strip_mem_len);
#endif
        mem_flags = 0;

        if(type->pixel_bytes == 4) {
            mem_flags |= (MALLOC_CAP_32BIT); 
        } 
        else {
            mem_flags |= (MALLOC_CAP_8BIT);
        }
        /** TODO: Think about mem caps **/
        strip_ptr = heap_caps_malloc((strip_mem_len + (strip_mem_len % 4)), mem_flags);
        
        if(strip_ptr == NULL) {
            ESP_LOGE(LS_TAG, "Unable to assign heap memory for led frame");
            err = ESP_ERR_NO_MEM;
        }
        else {
            memset(strip_ptr, 0, sizeof(uint8_t) * strip_mem_len);

            if(type->start_len > 0) {
                memset(strip_ptr, type->start_byte, sizeof(uint8_t)*type->start_len);
            }
        
            /** only write the end bytes if they're not zero **/
            if(type->end_len > 0 && type->end_byte != 0) {
                offset = (type->pixel_bytes * init->num_leds) + type->start_len;
                memset(strip_ptr+offset, type->end_byte, sizeof(uint8_t) * type->end_len);
            }
        }
    }

    if(!err) {
        /** clear and populate the handle **/
        memset(strip, 0, sizeof(ledstrip_t));
        strip->channel = init->channel;
        strip->num_leds = init->num_leds;
        strip->led_type = &led_types[init->led_type];
        strip->write_length = strip_mem_len;
        strip->strand_mem_start = strip_ptr;
        strip->pixel_start = strip_ptr + type->start_len;
        strip->led_type = type;
        strip->timer = timer;
        strip->sem = sem;
#ifdef DEBUG_MODE
        
#else
        lfx_set_mode(strip, LED_EFFECT_OFF);
#endif /** DEBUG_MODE **/
    }

    return err;
}


esp_err_t ledstrip_init_ws2812b(void *strip, void *init) {
    esp_err_t err = ESP_OK;
    ledstrip_init_t *init_data = init;

    rmt_config_t rmt_cfg = {
        .channel = init_data->channel,
        .rmt_mode = RMT_MODE_TX,
        .clk_div = 4,
        .mem_block_num = 1,
        .gpio_num = init_data->data_pin,
        .tx_config.loop_en = false,
        .tx_config.carrier_en = false,
        .tx_config.idle_output_en = true,
        .tx_config.idle_level = RMT_IDLE_LEVEL_LOW,
        .tx_config.carrier_level = RMT_CARRIER_LEVEL_LOW,
    };


    err = rmt_config(&rmt_cfg);

    if(err) {
        ESP_LOGE(LS_TAG, "Error configuring RMT [%u]", err);
    }
    else {
        err = rmt_driver_install((rmt_channel_t)init_data->channel, 0, 0);
    }

    if(err) {
        ESP_LOGE(LS_TAG, "Error installing RMT driver [%u]", err);
    }
    else {
        err = rmt_translator_init((rmt_channel_t)init_data->channel, ws2812_TranslateDataToRMT);
    }

    if(err) {
        ESP_LOGE(LS_TAG, "Error installing RMT translator function [%u]", err);
    }

    return err;
}


esp_err_t ledstrip_init_apa102(void *strip, void *init) {
    esp_err_t err = ESP_OK;
    ledstrip_init_t *init_data = init;
    LEDSTRIP_h str = strip;

    spi_device_interface_config_t leds = {0};
    leds.mode = 1;
    leds.duty_cycle_pos = 128;
    leds.spics_io_num = -1;
    leds.queue_size = 16;
    leds.clock_speed_hz = LEDSTRIP_CONFIG_SPI_FREQ; // APA claim to have refresh rate of 4KHz, start low.


    err = spi_bus_add_device(init_data->channel, &leds, str->interface_handle);
    if(err) {
        ESP_LOGE(LS_TAG, "Error adding SPI device! {%u}", err);
    }

    return err;
}

esp_err_t ledstrip_get_numleds(LEDSTRIP_h strip, uint8_t *num) {
    esp_err_t status = ESP_OK;
    *num = strip->num_leds;
    return status;
}

esp_err_t ledstrip_get_mode(LEDSTRIP_h strip, uint8_t *mode) {
   esp_err_t status = ESP_OK;
   *mode = strip->fx.effect;
   return status;
}

esp_err_t ledstrip_set_mode(LEDSTRIP_h strip, uint8_t *mode) {
    esp_err_t err = ESP_OK;
    uint8_t m = *mode;

    if(m >= LED_EFFECT_INVALID) {
        err = ESP_ERR_INVALID_ARG;
    }
    else {
        lfx_set_mode(strip, m);
    }
    return err;
}

esp_err_t ledstrip_get_colour(LEDSTRIP_h strip, uint32_t *colour) {
    esp_err_t err = ESP_OK;

    return err;
}

esp_err_t ledstrip_set_colour(LEDSTRIP_h strip, uint32_t *colour) {
    esp_err_t err = ESP_OK;

    return err;
}

esp_err_t ledstrip_set_brightness(LEDSTRIP_h strip, uint8_t *brightness) {
    esp_err_t err = ESP_OK;
    uint8_t b = *brightness;
    ls_cmd_t cmd;

    if(strip->led_type->brt_bits == 0) {
        err = ESP_ERR_INVALID_STATE;
        ESP_LOGE(LS_TAG, "Led type does not have configurable brightness [%u]", err);        
    }
    else if(b >= (1 << strip->led_type->brt_bits)) {
        err = ESP_ERR_INVALID_STATE;
        ESP_LOGE(LS_TAG, "Invalid led brightness (max: %u) [%u]", ((1 << strip->led_type->brt_bits)-1), err);        
    }
    else {
        strip->fx.brightness = b;
        
        cmd.cmd = LS_CMD_UPDATE_FRAME;
        cmd.strip = strip;
        if(xQueueSendToBack(command_queue, &cmd, LEDSTRIP_CONFIG_GENERIC_TIMEOUT) != pdTRUE) {
            err = ESP_ERR_TIMEOUT;
            ESP_LOGE(LS_TAG, "Error adding command to queue [%u]", err);            
        }
    }

    return err;
}

esp_err_t ledstrip_get_brightness(LEDSTRIP_h strip, uint8_t *brightness) {
    esp_err_t err = ESP_OK;
    *brightness = strip->fx.brightness;
    return err;
}



