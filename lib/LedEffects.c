/***************************************
* \file     LedEffects.c
* \brief    Initialises the led effect structure and 
*           houses various LED effects. As many as I can 
*           be arsed writing/stealing
*
* \date     August 2020
* \author   RJAM
****************************************/

/********* Includes *******************/
#include <stdint.h>
#include <string.h>
#include "WS2812_Driver.h"
#include "LedEffects.h"

#include "esp_log.h"
/****** Function Prototypes ***********/

/****** Global Data *******************/

uint8_t numTimers = 0;

/************ ISR *********************/

/****** Private Data ******************/

/****** Private Functions *************/

/****** Global Functions *************/

/** \brief  LedEffectsInit();
 * 
 *          initialises and returns a pointer to the led effects structure
 *  \return a pointer to a ledEffects structure
*/

void *ledEffectInit(void *arg)
{
    StrandData_t *strand = (StrandData_t *)arg;
    esp_err_t initStatus = ESP_OK;
    uint16_t spaceRequired = sizeof(ledEffectData_t);
    ledEffectData_t *effectData = NULL;

    if (numTimers > LEDFX_MAX_TIMERS)
    {
        initStatus = ESP_ERR_NOT_FOUND;
    }

    effectData = heap_caps_calloc(1, spaceRequired, MALLOC_CAP_8BIT);
    if (effectData != NULL)
    {
        effectData->LedEffectData_t = 0;
        effectData->refresh_t = 1000;
        effectData->effect = LED_EFFECT_SINGLE_COLOUR;
        effectData->colour = 0;
        effectData->func = NULL;
    }
    else
    {
        ESP_LOGE(WS2812_TAG, "Error! Insufficient heap memory to assign LED effects data memory ( %u needed | %u available 8-bit capable )", spaceRequired, heap_caps_get_free_size(MALLOC_CAP_8BIT));
        initStatus = ESP_ERR_NO_MEM;
    }

    /** Keep a record of timers/Strands so we can use a single callback function for all
     *  Just need a pointer to strand and the timer handle
     *  TODO: Think of a better way to do this
     **/

    numTimers++;
    return (void *)effectData;
}

/**  \brief     A basic night-rider style effect with optional fade 
 *      
 *  \param      strand - a pointer to a led control strand
 *  \param      fade_len - length of fade trail
 *  \param      colour - 32 bit colour XBGR format 
 * 
*/
void ledEffects_nightrider(void *arg, int fade_len, uint32_t colour)
{

    static bool direction = 1;
    static int16_t led_pos = 0;
    StrandData_t *strand = (StrandData_t *)arg;
    uint16_t numLeds = strand->numLeds;

    if (fade_len > numLeds)
    {
        fade_len = 0;
    }

    // TODO: replace these with MACROS
    uint16_t data_length = strand->strandMemLength;
    uint8_t r = colour;
    uint8_t g = ((colour) >> 8);
    uint8_t b = ((colour) >> 16);

    memset((void *)strand->strandMem, 0, data_length);

    int pixel_offset = led_pos * WS2812_BYTES_PER_PIXEL;

    /* write colour data to led_pos */
    uint8_t *addr = (uint8_t *)strand->strandMem + pixel_offset;

    *addr = r;
    addr++;
    *addr = g;
    addr++;
    *addr = b;

    if (direction)
    {
        /* write fading data */
        if (fade_len && led_pos > 0)
        {
            /* as long as not mapping past number of posible leds & not longer than fade len */
            for (int i = 0; (i < fade_len) || (i < led_pos); i++)
            {
                uint8_t *addr = (uint8_t *)strand->strandMem + pixel_offset - ((i + 1) * 3);
                *addr = fade_color(g, fade_len, i);
                addr++;
                *addr = fade_color(r, fade_len, i);
                addr++;
                *addr = fade_color(b, fade_len, i);
            }
        }
        /* Increment the led position */
        led_pos++;
        if (led_pos > numLeds - 1)
        {
            led_pos = numLeds - 1;
            /*if we've reached the end, change dir */
            direction = !direction;
        }
    }
    else
    {
        /* write fading data */
        if (fade_len && led_pos < numLeds)
        {
            /* as long as not mapping past number of posible leds & not longer than fade len */
            for (int i = 0; (i < fade_len) || (i < ((numLeds - 1) - led_pos)); i++)
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
}

/** 
 *  \brief fade_color - reduces the value of an 8-bit colour 
 * 
 *  \param colour - 8-bit colour value
 *  \param steps  - number of steps to dim
 *  \param step_no - the current step niumber
 * 
 *  \return  the adjusted colour value
*/
uint8_t fade_color(uint8_t colour, uint8_t steps, uint8_t step_no)
{

    if (step_no >= steps)
    {
        return 0;
    }
    int steps_remaining = steps - step_no;
    int difference = colour / (2 * steps_remaining);
    uint8_t faded = colour - difference;
    return faded;
}
