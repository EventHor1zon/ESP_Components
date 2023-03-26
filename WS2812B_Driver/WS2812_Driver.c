
/***************************************
* \file     FREESP_WS2812.c
* \brief    A freertos flavoured ESP-IDF driver for 
*           the WS2812b Addresseble RGB LED. Supports
*           up to 8 channels of LED strands. Each 
*           with its own dedicated pixel memory.
*           Sempahore for resource access, input queue
*           for receiving ESPHome style commands.
* \date     June 2020
* \author   RJM
****************************************/

/********* Includes *******************/

/* header */

#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"

/* esp-idf */
#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_heap_caps_init.h"
#include "driver/rmt.h"
#include "driver/gpio.h"
/* FreeRTOS */
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "WS2812_Driver.h"

/****** Function Prototypes ***********/

static void WS2812_driverTask(void *args);

/****** Private Data ******************/

static uint8_t numstrands = 0;

static LedStrand_t *allStrands[WS2812_MAX_STRANDS] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};
static ws2812Ctrl_t ledControl = {0};

/****** Global Data *******************/

QueueHandle_t commandsQueue;

const size_t WS2812_BYTES_PER_PIXEL = 3;    /** <number of bytes per LED */
const char *WS2812_TAG = "[WS2812 Driver]"; /** <esp-log tag */

const ws2812b_timing_t ws2812Timings = {
    400,
    800,
    550,
    850,
    450,
};


#ifdef CONFIG_USE_PERIPH_MANAGER
const parameter_t ws2812_param_map[ws2812_param_len] = {
    {"NumLeds", 1, &ws2812_get_numleds, NULL, NULL, DATATYPE_UINT32, 0, (GET_FLAG)},
    {"Mode", 2, &ws2812_get_mode, &ws2812_set_mode, NULL, DATATYPE_UINT8, 4, (GET_FLAG | SET_FLAG)},
    {"Colour", 3, &ws2812_get_colour, &ws2812_set_colour, NULL, DATATYPE_UINT32, 0xFFFFFF, (GET_FLAG | SET_FLAG) },
    {"Brightness", 4, &ws2812_get_brightness, &ws2812_set_brightness, NULL, DATATYPE_UINT8, 5, (GET_FLAG | SET_FLAG)}
};


const peripheral_t ws_peripheral_template = {

    .handle = NULL,
    .param_len = ws2812_param_len,
    .params = ws2812_param_map,
    .peripheral_name = "WS2812B",
    .peripheral_id = 0,
    .periph_type = PTYPE_ADDR_LEDS,
};

#endif

/************ ISR *********************/

static void IRAM_ATTR ws2812_TranslateDataToRMT(const void *src, rmt_item32_t *dest, size_t src_size,
                                                size_t wanted_num, size_t *translated_size, size_t *item_num)
{
    if (src == NULL || dest == NULL)
    {
        *translated_size = 0;
        *item_num = 0;
        return;
    }

    uint32_t duration0_0 = (ws2812Timings.T0H / (WS2812_RMT_DURATION_NS * WS2812_RMT_CLK_DIVIDER));
    uint32_t duration1_0 = (ws2812Timings.T0L / (WS2812_RMT_DURATION_NS * WS2812_RMT_CLK_DIVIDER));
    uint32_t duration0_1 = (ws2812Timings.T1H / (WS2812_RMT_DURATION_NS * WS2812_RMT_CLK_DIVIDER));
    uint32_t duration1_1 = (ws2812Timings.T1L / (WS2812_RMT_DURATION_NS * WS2812_RMT_CLK_DIVIDER));

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

/**
*   fxCallbackFunction
*   
*       Callback function for the timer expiry
*       set flag for Led Control task to deal with
*       Don't call the led effects functions here
**/
void fxCallbackFunction(TimerHandle_t timer)
{
    LedStrand_t *strand = NULL;

    /** find the right strand from the timerHandle **/
    for (uint8_t i = 0; i < ledControl.numStrands; i++)
    {
        if (timer == allStrands[i]->led_timer)
        {
            strand = allStrands[i];
        }
    }

    if (strand != NULL)
    {
        strand->updateLeds = 1;
    }

    return;
}

/*** PRIVATE FUNCTIONS ***/
/**
 * Write data from led mem to the RMT data output
 **/
static esp_err_t WS2812_transmitLedData(LedStrand_t *ledStrand)
{
    esp_err_t transmitStatus;
    transmitStatus = rmt_write_sample(ledStrand->channel, (uint8_t *)ledStrand->strandMem, ledStrand->strandMemLength, true);
    return transmitStatus;
}


#ifdef DRIVER_DEVELOPMENT_MODE
/** 
 *  make a simple test frame, write b,g,r looping to the LED memory
 **/
static esp_err_t WS2812_loadTestImage(LedStrand_t *strand)
{
    uint16_t i = 0, pixelIndex = 0;
    uint8_t testPixelsLen = sizeof(testFrame) / WS2812_BYTES_PER_PIXEL;
    uint8_t *pixelAddr;
    uint8_t *pixelData;

    if (strand == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    for (i = 0; i < strand->num_leds; i++)
    {
        pixelAddr = (uint8_t *)strand->strandMem + (i * WS2812_BYTES_PER_PIXEL);
        pixelIndex = i % testPixelsLen;
        pixelData = &testFrame[pixelIndex][0];
        ESP_LOGI(WS2812_TAG, "Writing %06x to %p", *pixelData, pixelAddr);
        memcpy(pixelAddr, pixelData, WS2812_BYTES_PER_PIXEL);
    }

    esp_err_t er = WS2812_transmitLedData(strand);
    if(er != ESP_OK) {
        ESP_LOGE(WS2812_TAG, "Error in transmit %u", er);
    }
    return ESP_OK;
}

#endif

/**
 * Driver control task 
 *  Duties - Check for strand frame refresh 
 *           - Check for update requests & update as needed
 *            - check for led effects r/w requests
 **/

static void WS2812_driverTask(void *args)
{

    uint32_t colours[6] = {0x00000011, 0x00001100, 0x00110000, 0x00111100, 0x00001111, 0x00110011};
    uint8_t counter = 0;

    while (1)
    {

        for (uint8_t i = 0; i < ledControl.numStrands; i++)
        {
            if (allStrands[i] != NULL)
            {
                LedStrand_t *strand = allStrands[i];

                /*  for each strand, if there's an animation to run, take the semaphore and run it.
                *   reset the timer to the next animation refresh    
                */

                /* if the leds require updating, take the semaphore and write new data */
                if (strand->updateLeds)
                {
                    if (xSemaphoreTake(strand->strand_sem, pdMS_TO_TICKS(WS2812_SEMAPHORE_TIMEOUT)) == 0)
                    {
                        ESP_LOGE(WS2812_TAG, "Semaphore request timed out");
                    }
                    else
                    {
                        WS2812_transmitLedData(strand);
                        strand->updateLeds = 0;
                        xSemaphoreGive(strand->strand_sem);
                    }
                }
            }
        }

        vTaskDelay(1000);
    }
}

/****** Global Functions *************/


/** Driver Init function
 *  initialises driver structures/tasks from init arguments
 *  TODO: Pass Handle in return INIT!
 *  TODO: Init_data_t!
 * 
 **/
LedStrand_t *WS2812_init(ws2812_initdata_t *initdata)
{

    uint16_t totalLeds = 0;
    esp_err_t initStatus = ESP_OK;

    /* set up the rmt driver config */
    rmt_config_t rmtConfig;
    rmtConfig.channel = (rmt_channel_t)numstrands;
    rmtConfig.rmt_mode = RMT_MODE_TX;
    rmtConfig.clk_div = 4;
    rmtConfig.mem_block_num = 1;
    rmtConfig.gpio_num = initdata->dataPin; 
    rmtConfig.tx_config.loop_en = false;
    rmtConfig.tx_config.carrier_en = false;
    rmtConfig.tx_config.idle_output_en = true;
    rmtConfig.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    rmtConfig.tx_config.carrier_level = RMT_CARRIER_LEVEL_LOW;


    /* assign memory and add the pointer to allStrands */
    LedStrand_t *strand = heap_caps_calloc(1, sizeof(LedStrand_t), MALLOC_CAP_8BIT);
    if (strand != NULL)
    {
        strand->strandIndex = numstrands;
        strand->updateLeds = 0;
        allStrands[numstrands] = strand;
    }
    else
    {
        ESP_LOGE(WS2812_TAG, "Error - insufficient memory for strand data structure");
        initStatus = ESP_ERR_NO_MEM;
    }

    /* Error check */
    if(initStatus == ESP_OK) {
        if (initdata->num_leds == 0 || initdata->num_leds > WS2812_MAX_STRAND_LEDS)
        {
            ESP_LOGE(WS2812_TAG, "Error - invalid LED count (min = 1, max = %u, requested = %u)", WS2812_MAX_STRAND_LEDS, initdata->num_leds);
            initStatus = ESP_ERR_INVALID_ARG;
        }
        else
        {
            strand->num_leds = initdata->num_leds;
            totalLeds += initdata->num_leds;
        }
    }

    if(initStatus == ESP_OK) {
    /* Init Function - assign LED memory */
        uint16_t spaceRequired = (WS2812_BYTES_PER_PIXEL * initdata->num_leds);
        uint8_t *strand_mem = heap_caps_calloc(1, spaceRequired, MALLOC_CAP_8BIT);

        if (strand_mem != NULL)
        {
            strand->strandMem = strand_mem;
            strand->strandMemLength = spaceRequired;
        }
        else
        {
            ESP_LOGE(WS2812_TAG, "Error - Insufficient heap memory to assign LED pixel data ( %u needed | %u available 8-bit capable )", spaceRequired, heap_caps_get_free_size(MALLOC_CAP_8BIT));
            initStatus = ESP_ERR_NO_MEM;
        }
    }
    /* init Function - create Led Effects structure */
    if (initStatus == ESP_OK)
    {
        fxdata_t *ledFxData = ledEffectInit(strand);
        TimerHandle_t fxTimer = xTimerCreate("fxTimer", (TickType_t)UINT32_MAX, pdTRUE, NULL, fxCallbackFunction);

        if (ledFxData == NULL)
        {
            ESP_LOGE(WS2812_TAG, "Error in assigning memory for led effects");
            initStatus = ESP_ERR_NO_MEM;
        }
        else if (fxTimer == NULL)
        {
            ESP_LOGE(WS2812_TAG, "Error in creating led effect timer");
            initStatus = ESP_ERR_NO_MEM;
        }
        else
        {
            strand->led_timer = fxTimer;
            strand->effects = ledFxData;
        }
    }

    if(initStatus == ESP_OK) {
    /* Init Function - create Strand Semaphore */
        SemaphoreHandle_t strand_sem = xSemaphoreCreateMutex();
        if (strand_sem != NULL)
        {
            strand->strand_sem = strand_sem;
        }
        else
        {
            ESP_LOGE(WS2812_TAG, "Error - Unable to find space for the LED semaphore");
            initStatus = ESP_FAIL;
        }
    }

    if (initStatus == ESP_OK)
    {
        /* configure & install the RMT driver on the GPIO channel */

        ESP_ERROR_CHECK(rmt_config(&rmtConfig));
        ESP_ERROR_CHECK(rmt_driver_install((rmt_channel_t)numstrands, 0, 0));
        ESP_ERROR_CHECK(rmt_translator_init((rmt_channel_t)numstrands, ws2812_TranslateDataToRMT));

    }


    if (initStatus == ESP_OK)
    {
        /** only initialise the task on the first strand **/
        if (numstrands == 0 && xTaskCreatePinnedToCore(WS2812_driverTask, "WS2812_driverTask", 5012, NULL, 3, &ledControl.driverTaskHandle, 0) == pdTRUE)
        {
            ESP_LOGI(WS2812_TAG, "Initialised driver task!");
        }
        else
        {
            ESP_LOGE(WS2812_TAG, "Error initialising task!");
            initStatus = ESP_ERR_NO_MEM;
        }
    }

    if(initStatus == ESP_OK) {
        ESP_LOGI(WS2812_TAG, "Success! Strand %u initialised at %p ", numstrands, strand);
                /* finish setup of Driver structure */
        numstrands++;
        ledControl.numStrands = numstrands;
    } 
    else 
    {
        /** free any mem claimed **/
        if(strand != NULL) {
            if(strand->strandMem != NULL) {
                heap_caps_free(strand->strandMem);
            }
            if(strand->effects != NULL) {
                heap_caps_free(strand->effects);
            }
            heap_caps_free(strand);
        }
    }

    WS2812_loadTestImage(strand);
    return strand;
}

/** Driver deinit
*
* tear-down the driver */
esp_err_t WS2812_deinit()
{
    /* for each strand, aquire the semaphore, then free() the image memory, release semaphore & delete it     */
    /* finally, free() mem for each strand, then free mem for control structure. Also delete queue & end task */
    uint8_t currentStrand;

    for (currentStrand = 0; currentStrand < ledControl.numStrands; currentStrand++)
    {
        xSemaphoreTake(allStrands[currentStrand]->strand_sem, portMAX_DELAY);
        if (allStrands[currentStrand]->effects != NULL)
        {
            free((void *)(allStrands[currentStrand]->effects));
        }
        if (allStrands[currentStrand]->strandMem != NULL)
        {
            free((void *)(allStrands[currentStrand]->strandMem));
        }
        /* TODO: error check this */
        vSemaphoreDelete(allStrands[currentStrand]->strand_sem);
    }


    for (currentStrand = 0; currentStrand < ledControl.numStrands; currentStrand++)
    {
        free(allStrands[currentStrand]);
    }

#ifdef ESP_HOME_API_ENABLE
    /* delete queue here */
    if (commandsQueue != NULL)
    {
        vQueueDelete(commandsQueue);
    }
    /* delete the task here */
#endif

    return ESP_OK;
}


esp_err_t WS2812_ledsOff(LedStrand_t *strand)
{
    esp_err_t status = ESP_OK;

    {
        bzero(strand->strandMem, strand->strandMemLength); /* use bzero to quickly zero the pixel map */
        /** TODO: better to do all writes in ctrl task @ refresh rate? */
        status = WS2812_transmitLedData(strand); /* write the strand data */
    }

    xSemaphoreGive(strand->strand_sem);

    return status;
}


esp_err_t ws2812_get_numleds(LedStrand_t *strand, uint8_t *data) {
    esp_err_t status = ESP_OK;
    *data = strand->num_leds;
    return status;
}


esp_err_t ws2812_get_mode(LedStrand_t *strand, uint8_t *data){
    esp_err_t status = ESP_OK;
    *data = strand->effects.effect;
    return status;
}


esp_err_t ws2812_get_colour(LedStrand_t *strand, uint32_t *data){

    esp_err_t status = ESP_OK;
    *data = strand->effects.colour;
    return status;
}


esp_err_t ws2812_get_brightness(LedStrand_t *strand, uint8_t *data){
    esp_err_t status = ESP_OK;
    *data = strand->effects.brightness;
    return status;
}

esp_err_t ws2812_set_mode(LedStrand_t *strand, uint8_t *data) {
    esp_err_t status;

    if(*data > LEDFX_NUM_EFFECTS) {
        status = ESP_ERR_INVALID_ARG;
        ESP_LOGI(WS2812_TAG, "Error mode num");
    } else {
        ESP_LOGI(WS2812_TAG, "setting Mode %u", *data);

        strand->effects.effect = *data;
        lfx_set_mode(strand);
    }
    return status;
}

esp_err_t ws2812_set_colour(LedStrand_t *strand, uint32_t *data){
    esp_err_t status = ESP_OK;
    ESP_LOGI(WS2812_TAG, "Setting colour to %06x", *data);
    strand->effects.colour = *data;
    strand->updateLeds = true;
    return status;
}


esp_err_t ws2812_set_brightness(LedStrand_t *strand, uint8_t *data){
    esp_err_t status = ESP_OK;

    if(*data > WS2812_MAX_BR) {
        status = ESP_ERR_INVALID_ARG;
    } else {
        strand->effects.brightness = *data;
    }
    return status;
}


/********* UTILITY ***************/
