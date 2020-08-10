
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
#include "WS2812_Driver.h"

#include <stdio.h>
#include <string.h>
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

/****** Function Prototypes ***********/

static void showmem(uint8_t *ptr, int len);
static void WS2812_driverTask(void *args);
static uint8_t lfx_single_color_nightrider(uint8_t strandIndex, int fade_len, uint32_t colour);
static uint8_t fade_color(uint8_t color, uint8_t steps, uint8_t step_no);

/****** Private Data ******************/

static StrandData_t *allStrands[WS2812_MAX_STRANDS] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};
static ws2812Ctrl_t ledControl = {0};

/****** Global Data *******************/

QueueHandle_t commandsQueue;

const size_t WS2812_BYTES_PER_PIXEL = 3;    /** <number of bytes per LED */
const char *WS2812_TAG = "[WS2812 Driver]"; /** <esp-log tag */

const size_t pixelSize = 3;

const uint8_t ws2812_black_pixel[3] = {0x00, 0x00, 0x00}; /** < black (off) pixel */
const uint8_t ws2812_white_pixel[3] = {0xff, 0xff, 0xff}; /** < white pixel */
const uint8_t ws2812_blue_pixel[3] = {0xff, 0x00, 0x00};  /** < blue pixel */
const uint8_t ws2812_red_pixel[3] = {0x00, 0x00, 0xff};   /** < red pixel */
const uint8_t ws2812_green_pixel[3] = {0x00, 0xff, 0x00}; /** < green pixel */

const ws2812b_timing_t ws2812Timings = {
    400,
    800,
    550,
    850,
    450,
};

#ifdef ESP_HOME_API_ENABLE

const peripheralInstruction_t availableCommands[] = {
    /** TODO: replace magic numbers */
    {WS2812_CMD_INDEX_ENABLE, CMD_TYPE_BOOLEAN, 1, 0},       /** < boolean ON/OFF command */
    {WS2812_CMD_INDEX_COLOUR, CMD_TYPE_U32INT, 0xFFFFFF, 0}, /** < set Colour */
    {WS2812_CMD_INDEX_EFFECT, CMD_TYPE_U8INT, 3, 0},         /** < set effect */
    {WS2812_CMD_INDEX_BRIGHT, CMD_TYPE_U8INT, 10, 0},        /** < set brightness */
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

/****** Private Functions *************/

static void showmem(uint8_t *memptr, int len)
{
    for (int i = 0; i < len; i++)
    {
        if (i % 8 == 0)
        {
            printf("[%p] %02x", memptr, *memptr);
        }
        else if (i % 8 == 7)
        {
            printf(" %02x\n", *memptr);
        }
        else
        {
            printf(" %02x", *memptr);
        }
        memptr++;
    }
}
/**
 * Write data from led mem to the RMT data output
 **/
static esp_err_t WS2812_transmitLedData(StrandData_t *ledStrand)
{
    esp_err_t transmitStatus;
    transmitStatus = rmt_write_sample(ledStrand->dataChannel, ledStrand->strandMem, ledStrand->strandMemLength, true);
    return transmitStatus;
}

#ifdef DRIVER_DEVELOPMENT_MODE
/** 
 *  make a simple test frame, write b,g,r looping to the LED memory
 **/
static esp_err_t WS2812_loadTestImage(StrandData_t *strand)
{
    uint16_t i = 0, pixelIndex = 0;
    uint8_t testPixelsLen = sizeof(testFrame) / WS2812_BYTES_PER_PIXEL;
    uint8_t *pixelAddr;
    uint8_t *pixelData;

    if (strand == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    for (i = 0; i < strand->numLeds; i++)
    {
        pixelAddr = strand->strandMem + (i * WS2812_BYTES_PER_PIXEL);
        pixelIndex = i % testPixelsLen;
        pixelData = &testFrame[pixelIndex][0];
        ESP_LOGI(WS2812_TAG, "Writing %u to %p", *pixelData, pixelAddr);
        memcpy(pixelAddr, pixelData, WS2812_BYTES_PER_PIXEL);
    }

    return ESP_OK;
}

#endif
/****** Global Functions *************/

/** Driver Init function
 *  initialises driver structures/tasks from init arguments
 * 
 **/
esp_err_t WS2812_init(uint8_t numStrands, uint16_t *numLeds, gpio_num_t *dataPin)
{

    uint16_t counter = 0, totalLeds = 0;
    esp_err_t initStatus = ESP_OK;

    /* check number of strands */
    if (numStrands > WS2812_MAX_STRANDS)
    {
        initStatus = ESP_ERR_INVALID_ARG;
    }

    /* initialise the structures for each strand - place on heap for convenience */
    if (initStatus == ESP_OK)
    {

        /* set up the rmt driver config */
        rmt_config_t rmtConfig;
        rmtConfig.channel = 0;
        rmtConfig.rmt_mode = RMT_MODE_TX;
        rmtConfig.clk_div = 4;
        rmtConfig.mem_block_num = 1;
        rmtConfig.gpio_num = 0; /* change this parameter for each led channel */
        rmtConfig.tx_config.loop_en = false;
        rmtConfig.tx_config.carrier_en = false;
        rmtConfig.tx_config.idle_output_en = true;
        rmtConfig.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
        rmtConfig.tx_config.carrier_level = RMT_CARRIER_LEVEL_LOW;

        /* set up the LED strands */
        for (counter = 0; counter < numStrands; counter++)
        {
            /* assign memory and add the pointer to allStrands */
            StrandData_t *strand = heap_caps_calloc(1, sizeof(StrandData_t), MALLOC_CAP_8BIT);
            if (strand != NULL)
            {
                strand->strandIndex = counter;
                strand->updateLeds = 0;
                allStrands[counter] = strand;
            }
            else
            {
                ESP_LOGE(WS2812_TAG, "Error - insufficient memory for strand data structure");
                initStatus = ESP_ERR_NO_MEM;
                break;
            }

            if (numLeds[counter] == 0 || numLeds[counter] > WS2812_MAX_STRAND_LEDS)
            {
                ESP_LOGE(WS2812_TAG, "Error - invalid LED count (min = 1, max = %u, requested = %u)", WS2812_MAX_STRAND_LEDS, numLeds[counter]);
                initStatus = ESP_ERR_INVALID_ARG;
                break;
            }
            else
            {
                strand->numLeds = numLeds[counter];
                totalLeds += numLeds[counter];
            }

            /* assign LED memory */
            uint16_t spaceRequired = (WS2812_BYTES_PER_PIXEL * numLeds[counter]);
            uint8_t *ledMem = heap_caps_calloc(1, spaceRequired, MALLOC_CAP_8BIT);

            if (ledMem != NULL)
            {
                strand->strandMem = ledMem;
                strand->strandMemLength = spaceRequired;
            }
            else
            {
                ESP_LOGE(WS2812_TAG, "Error - Insufficient heap memory to assign LED pixel data ( %u needed | %u available 8-bit capable )", spaceRequired, heap_caps_get_free_size(MALLOC_CAP_8BIT));
                initStatus = ESP_ERR_NO_MEM;
                break;
            }

            spaceRequired = sizeof(ledEffectData_t);
            ledEffectData_t *effectData = heap_caps_calloc(1, spaceRequired, MALLOC_CAP_8BIT);
            if (effectData != NULL)
            {
                strand->fxData = effectData;
            }
            else
            {
                ESP_LOGE(WS2812_TAG, "Error! Insufficient heap memory to assign LED effects data memory ( %u needed | %u available 8-bit capable )", spaceRequired, heap_caps_get_free_size(MALLOC_CAP_8BIT));
                initStatus = ESP_ERR_NO_MEM;
                break;
            }

            /* create Semaphore */
            SemaphoreHandle_t memSemphr = xSemaphoreCreateMutex();
            if (memSemphr != NULL)
            {
                strand->memSemphr = memSemphr;
            }
            else
            {
                ESP_LOGE(WS2812_TAG, "Error - Unable to find space for the LED semaphore");
                initStatus = ESP_FAIL;
                break;
            }

            if (initStatus == ESP_OK)
            {
                /* configure & install the RMT driver on the GPIO channel */
                rmtConfig.gpio_num = dataPin[counter];
                rmtConfig.channel = (rmt_channel_t)counter;

                ESP_ERROR_CHECK(rmt_config(&rmtConfig));
                ESP_ERROR_CHECK(rmt_driver_install((rmt_channel_t)counter, 0, 0));
                ESP_ERROR_CHECK(rmt_translator_init((rmt_channel_t)counter, ws2812_TranslateDataToRMT));

                strand->dataChannel = (rmt_channel_t)counter;
            }

            ESP_LOGI(WS2812_TAG, "Success! Strand %u initialised at %p ", counter, strand);

            // showmem((uint8_t *)strand, sizeof(StrandData_t));
        }

        /* finish setup of Driver structure */

        ledControl.numStrands = numStrands;

        if (initStatus == ESP_OK)
        {
            ESP_LOGI(WS2812_TAG, "Done! %u LED strands added", numStrands);
        }
    }

    if (initStatus == ESP_OK)
    {
        if (xTaskCreatePinnedToCore(WS2812_driverTask, "WS2812_driverTask", 5012, NULL, 3, &ledControl.driverTaskHandle, 0) == pdTRUE)
        {
            ESP_LOGI(WS2812_TAG, "Initialised driver task!");
        }
        else
        {
            ESP_LOGE(WS2812_TAG, "Error initialising task!");
        }
    }

    return initStatus;
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
        xSemaphoreTake(allStrands[currentStrand]->memSemphr, portMAX_DELAY);
        if (allStrands[currentStrand]->fxData != NULL)
        {
            free((void *)(allStrands[currentStrand]->fxData));
        }
        if (allStrands[currentStrand]->strandMem != NULL)
        {
            free((void *)(allStrands[currentStrand]->strandMem));
        }
    }
    /* TODO: error check this */
    vSemaphoreDelete(allStrands[currentStrand]->memSemphr);

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
                StrandData_t *strand = allStrands[i];

                if (xSemaphoreTake(strand->memSemphr, pdMS_TO_TICKS(WS2812_SEMAPHORE_TIMEOUT)) == pdFALSE)
                {
                    ESP_LOGE(WS2812_TAG, "Semaphore request timed out");
                }
                else
                {
                    WS2812_setAllLedColour(strand, colours[counter]);
                    counter++;
                    if (counter > 5)
                    {
                        counter = 0;
                    }
                    //lfx_single_color_nightrider(0, 0, 0x00119911);
                    if (xSemaphoreGive(strand->memSemphr) == pdFALSE)
                    {
                        ESP_LOGE(WS2812_TAG, "Error in releasing semaphore");
                    }
                }

                /* if the leds require updating, take the semaphore and write new data */
                if (strand->updateLeds)
                {
                    if (xSemaphoreTake(strand->memSemphr, pdMS_TO_TICKS(WS2812_SEMAPHORE_TIMEOUT)) == pdFALSE)
                    {
                        ESP_LOGE(WS2812_TAG, "Semaphore request timed out");
                    }
                    else
                    {
                        WS2812_transmitLedData(strand);
                        strand->updateLeds = 0;
                        xSemaphoreGive(strand->memSemphr);
                    }
                }
            }
        }

        vTaskDelay(1000);
    }
}

/**
 *  ws2812_ledsOff
 * ** TODO: think of a better name 
*/

esp_err_t WS2812_ledsOff(StrandData_t *strand)
{
    esp_err_t status = ESP_OK;

    {
        bzero(strand->strandMem, strand->strandMemLength); /* use bzero to quickly zero the pixel map */
        /** TODO: better to do all writes in ctrl task @ refresh rate? */
        status = WS2812_transmitLedData(strand); /* write the strand data */
    }

    if (xSemaphoreGive(strand->memSemphr) == pdFAIL)
    {
        status = ESP_FAIL;
    }

    return status;
}

/**
 *  WS2812_ledSetColour - set all leds to colour
 * 
**/

esp_err_t WS2812_setAllLedColour(StrandData_t *strand, uint32_t colour)
{

    esp_err_t status = ESP_OK;
    uint8_t r, g, b;
    uint8_t *ptr = strand->strandMem;

    if (strand == NULL)
    {
        status = ESP_ERR_INVALID_ARG;
        ESP_LOGE(WS2812_TAG, "Error: Invalid strand");
    }
    else
    {
        r = fade_color((uint8_t)colour, 2, 1);
        g = fade_color((uint8_t)(colour >> 8), 2, 1);
        b = fade_color((uint8_t)(colour >> 16), 2, 1);

        for (uint8_t offset = 0; offset < strand->strandMemLength; offset++)
        {
            if (offset % 3 == 0)
            {
                *ptr = g;
            }
            else if (offset % 3 == 1)
            {
                *ptr = r;
            }
            else if (offset % 3 == 2)
            {
                *ptr = b;
            }
            ptr++;
        }
    }

    strand->updateLeds = 1;

    return status;
}

static uint8_t lfx_single_color_nightrider(uint8_t strandIndex, int fade_len, uint32_t colour)
{

    static bool direction = 1;
    static int16_t led_pos = 0;
    StrandData_t *strand = allStrands[strandIndex];
    uint16_t numLeds = strand->numLeds;

    if (fade_len > strand->numLeds)
    {
        fade_len = 0;
    }

    // TODO: replace these with MACROS
    uint16_t data_length = strand->strandMemLength;
    uint8_t r = colour;
    uint8_t g = ((colour) >> 8);
    uint8_t b = ((colour) >> 16);

    memset((void *)strand->strandMem, 0, data_length);
    if (direction)
    {
        int pixel_offset = led_pos * WS2812_BYTES_PER_PIXEL;

        /* write color data to led_pos */
        uint8_t *addr = (uint8_t *)strand->strandMem + pixel_offset;
        uint8_t *anchor = addr;

        *addr = r;
        addr++;
        *addr = g;
        addr++;
        *addr = b;

        /* write fading data */
        if (fade_len && led_pos > 0)
        {
            /* as long as not mapping past number of posible leds & not longer than fade len */
            for (int i = 0; (i < fade_len) || (i < led_pos); i++)
            {
                uint8_t *addr = anchor + pixel_offset - ((i + 1) * 3);
                *addr = fade_color(g, fade_len, i);
                addr++;
                *addr = fade_color(r, fade_len, i);
                addr++;
                *addr = fade_color(b, fade_len, i);
            }
        }
        /* Increment the led position */
        led_pos++;
        if (led_pos > strand->numLeds - 1)
        {
            led_pos = strand->numLeds - 1;
            /*if we've reached the end, change dir */
            direction = !direction;
        }
    }
    else
    {
        /* TODO: This, better. Why did I duplicate a whole bunch of code? */

        int pixel_offset = led_pos * WS2812_BYTES_PER_PIXEL;
        /* write color data to led_pos */
        uint8_t *addr = (uint8_t *)strand->strandMem + pixel_offset;
        *addr = g;
        addr++;
        *addr = r;
        addr++;
        *addr = b;

        /* write fading data */
        if (fade_len && led_pos < strand->numLeds)
        {
            /* as long as not mapping past number of posible leds & not longer than fade len */
            for (int i = 0; i < fade_len || i < ((strand->numLeds - 1) - led_pos); i++)
            {
                uint8_t *addr = (uint8_t *)strand->strandMem + pixel_offset + ((i + 1) * 3);
                *addr = fade_color(g, fade_len, i);
                addr++;
                *addr = fade_color(r, fade_len, i);
                addr++;
                *addr = fade_color(b, fade_len, i);
            }
        }
        /* Decrement the led_position */
        led_pos--;
        if (led_pos < 0)
        {
            led_pos = 0;
            /*if we've reached the end, change dir */
            direction = !direction;
        }
    }

    strand->updateLeds = 1;

    return 0;
}

/********* UTILITY ***************/

static uint8_t fade_color(uint8_t color, uint8_t steps, uint8_t step_no)
{

    if (step_no >= steps)
    {
        return 0;
    }
    int steps_remaining = steps - step_no;
    int difference = color / (2 * steps_remaining);
    uint8_t faded = color - difference;
    return faded;
}
