/***************************************
* \file .c
* \brief
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
#include "LedEffects.h"
#include "Utilities.h"
#include "esp_err.h"
#include "esp_log.h"

#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "freertos/FreeRTOSConfig.h"

/****** Function Prototypes ***********/

static esp_err_t send_frame_dma_polling(StrandData_t *strand);

static int send_frame_polling(StrandData_t *strand);

static void apa102_driver_task(void *args);


/****** Global Data *******************/

const char *APA_TAG = "APA102 Driver";

#ifdef CONFIG_USE_PERIPH_MANAGER
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

#ifdef DEBUG
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

// static esp_err_t send_32bit_frame(spi_device_handle_t spi, uint32_t *data);
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
    StrandData_t *strand = (StrandData_t *) pvTimerGetTimerID(timer);
#ifdef DEBUG_MODE
    ESP_LOGI(APA_TAG, "Timer Callback");
#endif
    xTaskNotify(strand->task_handle, APA_NOTIFY_BUILD_FRAME, eSetBits);
    xTimerReset(timer, APA_SEMTAKE_TIMEOUT);
    return;
}
/****** Private Data ******************/

/****** Private Functions *************/


static esp_err_t send_frame_dma_polling(StrandData_t *strand) {

    esp_err_t txStatus = ESP_OK;

    spi_transaction_t trx = {0};
    
    trx.length = (strand->strandFrameLength * 8);
    trx.rxlength = 0;
    trx.rx_buffer = NULL;
    trx.tx_buffer = strand->strandFrameStart;

#ifdef DEBUG_MODE
    showmem(strand->strandFrameStart, strand->strandFrameLength);
#endif

    txStatus = spi_device_polling_transmit(strand->ledSPIHandle, &trx);
    if(txStatus != ESP_OK) {
        ESP_LOGE(APA_TAG, "Error sending spi dma {%u}", txStatus);
    }
    return txStatus;
}

/** send a polling transaction **/
static esp_err_t send_frame_polling(StrandData_t *strand)
{

    esp_err_t txStatus = ESP_OK;

    spi_transaction_t tx = {0};
    uint16_t length = (sizeof(uint32_t) * 8);
    bool got_sem = false;

    tx.length = 32;
    tx.flags = SPI_TRANS_USE_TXDATA;
    tx.addr = 0;
    tx.cmd = 0;
    tx.rxlength = 0;
    tx.rx_buffer = NULL;
    tx.user = NULL;


    txStatus = spi_device_polling_transmit(strand->ledSPIHandle, &tx);

    if (txStatus != ESP_OK)
    {
        ESP_LOGE("SPI_TX", "Error in sending start frame %u", txStatus);
    }

    /** sending 4 bytes (32 bits) **/
    tx.length = 32;
    tx.flags = 0;
    for (int i = 0; i < strand->strandMemLength; i+=4)
    {
        tx.tx_buffer = (void *)strand->strandMem+i;
        txStatus = spi_device_polling_transmit(strand->ledSPIHandle, &tx);
        if (txStatus != ESP_OK)
        {
            ESP_LOGE("SPI_TX", "Error in sending data frame number %d [%u]", i, txStatus);
        }
    }

    tx.length = sizeof(uint32_t) * 8;
    tx.flags = SPI_TRANS_USE_TXDATA;
    tx.tx_data[0] = 0xFF;
    tx.tx_data[1] = 0xFF;
    tx.tx_data[2] = 0xFF;
    tx.tx_data[3] = 0xFF;

    for (int i = 0; i < 3; i++)
    {
        txStatus = spi_device_polling_transmit(strand->ledSPIHandle, &tx);
    }
    if (txStatus != ESP_OK)
    {
        ESP_LOGE("SPI_TX", "Error in sending end frame %u", txStatus);
    }


    return txStatus;
}


/** Control task **/
static void apa102_driver_task(void *args) {


    StrandData_t *strand = (StrandData_t *)args;
    uint32_t action = 0;

    while(1) {


        xTaskNotifyWait(0, UINT32_MAX, &action, pdMS_TO_TICKS(10000));

        if(action & APA_NOTIFY_BUILD_FRAME) {
#ifdef DEBUG_MODE
            ESP_LOGI(APA_TAG, "Calling animation...");
#endif
            if(xSemaphoreTake(strand->memSemphr, pdMS_TO_TICKS(1000)) != pdTRUE) {
                ESP_LOGE(APA_TAG, "Failed to get mem semaphore");
            }
            else
            {
                strand->fxData->func((void *)strand);
                xSemaphoreGive(strand->memSemphr);
            }
            
            if(xTimerGetPeriod(strand->refreshTimer) != strand->fxData->refresh_t) {
                xTimerChangePeriod(strand->refreshTimer, strand->fxData->refresh_t, APA_SEMTAKE_TIMEOUT);
            }
            xTimerStart(strand->refreshTimer, portMAX_DELAY);
        }
       
        if(strand->updateLeds || (action & APA_NOTIFY_WRITE_LEDS)) {
#ifdef DEBUG_MODE
            ESP_LOGI(APA_TAG, "Writing leds...");
#endif
            if(xSemaphoreTake(strand->memSemphr, APA_SEMTAKE_TIMEOUT) != pdTRUE) {
                ESP_LOGE(APA_TAG, "Failed to get LED semaphore");
            } 
            else {
                if(strand->use_dma) {
                    if(send_frame_dma_polling(strand) != ESP_OK) {
                        ESP_LOGE(APA_TAG, "Failed to write to leds");                
                    }
                }
                else {
                    if(send_frame_polling(strand) != ESP_OK) {
                        ESP_LOGE(APA_TAG, "Failed to write to leds");       
                    }
                }
                if( xSemaphoreGive(strand->memSemphr) != pdTRUE) {
                    ESP_LOGE(APA_TAG, "error giving sem");
                }
            }
        }
        /** clear action value **/
        action = 0;
        /** short delay in case of some stuck bits **/
        vTaskDelay(5);
    }
}



/****** Global Functions *************/

esp_err_t apa_getNumleds(StrandData_t *strand, uint32_t *var) {
    esp_err_t status = ESP_OK;
    *var = strand->numLeds;
    return status;
}

esp_err_t apa_getMode(StrandData_t *strand, uint32_t *var) {
    esp_err_t status = ESP_OK;
    *var = strand->fxData->effect;
    return status;
}

esp_err_t apa_setMode(StrandData_t *strand, uint8_t  *mode) {
    esp_err_t status = ESP_OK;
    uint8_t m = *mode;
    if(m > LEDFX_NUM_EFFECTS) {
        status = ESP_ERR_INVALID_ARG;
    } else {
        strand->fxData->effect = m;
        ledFx_updateMode(strand);
        xTaskNotify(strand->task_handle, APA_NOTIFY_REFRESH_LEDS, eSetBits);
    }
    return status;
}

esp_err_t apa_getColour(StrandData_t *strand, uint32_t *var) {

    esp_err_t status = ESP_OK;
    *var = strand->fxData->colour;
    return status;
}

esp_err_t apa_setColour(StrandData_t *strand, uint32_t *var) {
    esp_err_t status = ESP_OK;
    uint32_t c = *var;
    strand->fxData->colour = c;
    strand->updateLeds = true;
    xTaskNotify(strand->task_handle, APA_NOTIFY_REFRESH_LEDS, eSetBits);
    return status;
}

esp_err_t apa_getBrightness(StrandData_t *strand, uint8_t *var) {
    esp_err_t status = ESP_OK;
    *var = strand->fxData->brightness;
    return status;
}

esp_err_t apa_setBrightness(StrandData_t *strand, uint8_t *var) {
    esp_err_t status = ESP_OK;
    uint8_t v = *var;
    if(v > APA_CTRL_MAX_BR) {
        status = ESP_ERR_INVALID_ARG;
    } else {
        strand->fxData->brightness = v;
        xTaskNotify(strand->task_handle, APA_NOTIFY_REFRESH_LEDS, eSetBits);
    }
    return status;
}


#ifdef CONFIG_DRIVERS_USE_HEAP
StrandData_t *APA102_init(apa102_init_t *init_data)
#else
StrandData_t *APA102_init(StrandData_t *handle, apa102_init_t *init_data)
#endif
{


    StrandData_t *strand = NULL;
    esp_err_t init_status = ESP_OK;
    uint32_t led_mem_sz = 0;
    spi_device_handle_t ledSPIHandle = NULL;
    TaskHandle_t thandle = NULL;
    uint8_t alignment_bytes = 0;
    uint8_t ledmem_caps = 0;
    


    if (init_data->init_spi)
    {
        if (!(init_data->spi_bus == SPI2_HOST || init_data->spi_bus == SPI3_HOST))
        {
            ESP_LOGE(APA_TAG, "Error - invalid SPI bus. Please use SPI2_HOST or SPI3_HOST");
            init_status = ESP_ERR_INVALID_ARG;
        }
        else {
            ESP_LOGI("SPI_SETUP", "[+] Setting up SPI bus");

            spi_bus_config_t buscfg = {0};
            buscfg.mosi_io_num = init_data->data_pin;
            buscfg.miso_io_num = -1;
            buscfg.sclk_io_num = init_data->clock_pin;
            buscfg.max_transfer_sz = 1024; // led data + up to 10 frames of start & end
            buscfg.quadhd_io_num = -1;
            buscfg.quadwp_io_num = -1;
            buscfg.flags = 0;
            buscfg.intr_flags = SPICOMMON_BUSFLAG_MASTER;
            uint8_t dma = init_data->use_dma ? APA_DMA_CHANNEL : 0;
            ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &buscfg, dma));
        }
    }

    if (init_status == ESP_OK)
    {

        spi_device_interface_config_t leds = {0};
        leds.command_bits = 0;
        leds.address_bits = 0;
        leds.dummy_bits = 0;
        leds.mode = 1;
        leds.duty_cycle_pos = 128;
        leds.cs_ena_posttrans = 0;
        leds.cs_ena_pretrans = 0;
        leds.spics_io_num = -1;
        leds.queue_size = 16;
        leds.clock_speed_hz = APA102_CLK_SPD; // APA claim to have refresh rate of 4KHz, start low.
        leds.input_delay_ns = 0;

        led_mem_sz = init_data->numleds * APA_BYTES_PER_PIXEL;
        /** add the zero size **/
        led_mem_sz += APA_ZERO_FRAME_SIZE_BYTES;
        /** add the 1's size - use num_leds * byte **/
        led_mem_sz += init_data->numleds;
        
        if(init_data->use_dma) {
            /** 32-bit align the memory **/
            alignment_bytes = led_mem_sz % 4;
            led_mem_sz += alignment_bytes;
        }

        ESP_LOGI(APA_TAG, "Led mem size: %u", led_mem_sz);
        /** init the spi device **/

        init_status = spi_bus_add_device(HSPI_HOST, &leds, &ledSPIHandle);
        if(init_status) {
            ESP_LOGE(APA_TAG, "Error adding SPI device! {%u}", init_status);
        }
    }

    if(init_status == ESP_OK ) {

        if(init_data->use_dma) {
            ledmem_caps = (MALLOC_CAP_DMA | MALLOC_CAP_32BIT);
        }
        else {
            ledmem_caps = (MALLOC_CAP_DEFAULT);
        }

        strand = (StrandData_t *)heap_caps_calloc(1, sizeof(StrandData_t), MALLOC_CAP_8BIT);
        uint32_t *ledMem = (uint32_t *)heap_caps_calloc(1, led_mem_sz, ledmem_caps);
        ledEffectData_t *lfx = ledEffectInit(strand);

        if (strand != NULL && lfx != NULL && ledMem != NULL)
        {
            strand->ledType = LEDTYPE_APA102;
            strand->bytes_per_pixel = APA_BYTES_PER_PIXEL;
            strand->numLeds = init_data->numleds;
            strand->spi_channel_no = init_data->spi_bus;
            strand->strandMemLength = APA_BYTES_PER_PIXEL * init_data->numleds;
            strand->strandMem = (((uint32_t)ledMem) + sizeof(uint32_t));
            strand->strandFrameStart = ledMem;
            strand->strandFrameLength = led_mem_sz;
            strand->fxData = lfx;
            strand->ledSPIHandle = ledSPIHandle;
            strand->use_dma = init_data->use_dma;

            uint32_t offset = sizeof(uint32_t) + (APA_BYTES_PER_PIXEL * strand->numLeds);

            memset((void *)(((uint32_t)strand->strandFrameStart) + offset), 0xFF, ((sizeof(uint8_t) * strand->numLeds) + alignment_bytes));
#ifdef DEBUG_MODE
            vTaskDelay(3);
            showmem(strand->strandFrameStart, strand->strandFrameLength);
            printf("\n");
            showmem((void *)(((uint32_t)strand->strandFrameStart) + offset), (strand->numLeds * 4)+alignment_bytes);
#endif
        }
        else
        {
            ESP_LOGE(APA_TAG, "Error in assigning Driver memory");
            init_status = ESP_ERR_NO_MEM;
        }
    }

    if (init_status == ESP_OK)
    {
        SemaphoreHandle_t ledSemaphore = NULL;
        TimerHandle_t refreshTimer = xTimerCreate("refreshTimer", UINT16_MAX, pdTRUE, (void *)strand, timerExpiredCallback);
        ledSemaphore = xSemaphoreCreateMutex();

        if (refreshTimer != NULL && ledSemaphore != NULL)
        {
            strand->refreshTimer = refreshTimer;
            strand->memSemphr = ledSemaphore;
        }
        else
        {
            ESP_LOGE(APA_TAG, "Error in setting up timer/semaphore");
            init_status = ESP_ERR_NOT_FOUND;
        }
    }

    if(init_status == ESP_OK) {
        ESP_LOGI(APA_TAG, "Starting the control task");
        if(xTaskCreate(apa102_driver_task, "apa102_driver_task", 2048, (void *)strand, 3, &thandle) != pdTRUE) {
            ESP_LOGE(APA_TAG, "Error creating task");
            init_status = ESP_ERR_NO_MEM;
        }
        else {
            strand->task_handle = thandle;
        }
    }
    

#ifdef DEBUG_MODE
    if (init_status == ESP_OK)
    {
        //test_frame_polling(strand);
        memcpy(strand->strandMem, init_frame, (sizeof(uint32_t) * strand->numLeds));
        xTimerStart(strand->refreshTimer, pdMS_TO_TICKS(5));
        send_frame_dma_polling(strand);
    }
#endif
    
    if(init_status == ESP_OK) {
        ESP_LOGI(APA_TAG, "APA strand initialised");
    } else {
        ESP_LOGE(APA_TAG, "Error initialising APA strand (%u)", init_status);
#ifdef CONFIG_DRIVERS_USE_HEAP
        if(strand != NULL) {
            heap_caps_free(strand);
            strand = NULL;
        }
#endif
    }

    return strand;
}



