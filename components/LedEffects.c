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
#include "APA102_Driver.h"
#include "LedEffects.h"
#include "Utilities.h"

#include "esp_log.h"
#include "esp_err.h"

/****** Function Prototypes ***********/

static uint8_t fade_color(uint8_t colour, uint8_t steps, uint8_t step_no);

/****** Global Data *******************/

uint8_t numTimers = 0;



/************ ISR *********************/

/****** Private Data ******************/

/****** Private Functions *************/

/** single colour **/


 /** 
 *  \brief fade_color - reduces the value of an 8-bit colour 
 * 
 *  \param colour - 8-bit colour value
 *  \param steps  - number of steps to dim
 *  \param step_no - the current step niumber
 * 
 *  \return  the adjusted colour value
*/
static uint8_t fade_color(uint8_t colour, uint8_t steps, uint8_t step_no)
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

/****** Global Functions *************/

/** \brief  LedEffectsInit();
 * 
 *          initialises and returns a pointer to the led effects structure
 *  \return a pointer to a ledEffects structure
*/

ledEffectData_t *ledEffectInit(StrandData_t *strand)
{
    esp_err_t initStatus = ESP_OK;
    uint16_t spaceRequired = sizeof(ledEffectData_t);
    ledEffectData_t *effectData = NULL;

    effectData = heap_caps_calloc(1, spaceRequired, MALLOC_CAP_8BIT);
    if (effectData != NULL)
    {
        effectData->brightness = 1;
        effectData->LedEffectData_t = 0;
        effectData->refresh_t = 1000;
        effectData->effect = LED_EFFECT_SINGLE_COLOUR;
        effectData->colour = 0x00001100;
        effectData->func = &all_single_colour;
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
    ESP_LOGI("LEDFX", "LEDFX struct initialised!");
    return effectData;
}


/**
 *  update led funciton pointer
 */
void ledFx_updateMode(StrandData_t *strand) {
    if(strand->fxData->effect == LED_EFFECT_OFF) {
        strand->fxData->colour = 0x00000000;
        strand->fxData->func = &all_single_colour;
        strand->updateLeds = true;    
    }
    else if (strand->fxData->effect == LED_EFFECT_SINGLE_COLOUR) {
        strand->fxData->func = &all_single_colour; 
        strand->updateLeds = true;
    }
    else if (strand->fxData->effect == LED_EFFECT_NIGHTRIDER) {
        strand->fxData->func = &ledEffects_nightrider;
        strand->updateLeds = true;

    }
    else {
        ESP_LOGE("LFX", "Unknown mode");
    }
    return;
}

/**  \brief     A basic night-rider style effect with optional fade    
 *   \param      strand - a pointer to a led control strand
*/
void ledEffects_nightrider(StrandData_t *strand)
{

    ledEffectData_t *fx = strand->fxData;
    if(fx == NULL) {
        return;
    }
    int fade_len = fx->var1;
    bool direction = fx->var5;
    uint32_t colour = fx->colour;
    int16_t led_pos = fx->var2;
    uint16_t numLeds = strand->numLeds;

    if (fade_len > numLeds)
    {
        fade_len = 0;
    }

    /** TODO: replace these with MACROS **/
    uint16_t data_length = strand->strandMemLength;
    /** TODO: RGB ORDER! **/
    uint8_t r = colour;
    uint8_t g = ((colour) >> 8);
    uint8_t b = ((colour) >> 16);

    memset((void *)strand->strandMem, 0, data_length);

    int pixel_offset = led_pos * strand->bytes_per_pixel; 

    /* write colour data to led_pos */
    uint8_t *addr = (uint8_t *)strand->strandMem + pixel_offset;

    if(strand->ledType == LEDTYPE_APA102) {
        *addr = (APA_CTRL_BRT_MASK | fx->brightness);
        addr++;
    }
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
                uint8_t *addr = (uint8_t *)strand->strandMem + pixel_offset - ((i + 1) * strand->bytes_per_pixel);
                if(strand->ledType == LEDTYPE_APA102) {
                    *addr = (APA_CTRL_BRT_MASK | fx->brightness);
                    addr++;
                }
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
                uint8_t *addr = (uint8_t *)strand->strandMem + pixel_offset + ((i + 1) * strand->bytes_per_pixel);
                if(strand->ledType == LEDTYPE_APA102) {
                    *addr = (APA_CTRL_BRT_MASK | fx->brightness);
                    addr++;
                }
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

    /** save variables for next run */
    fx->var5 = direction;
    fx->var2 = led_pos;

    strand->updateLeds = 1;
}


void all_single_colour(StrandData_t *strand) {

    ESP_LOGI(APA_TAG, "In animation");
    uint8_t r, g, b;
    uint8_t *ptr = (uint8_t *)strand->strandMem;

    r = (uint8_t)strand->fxData->colour;
    g = (uint8_t)(strand->fxData->colour >> 8);
    b = (uint8_t)(strand->fxData->colour >> 16);

    for (uint8_t offset = 0; offset < (strand->strandMemLength / strand->bytes_per_pixel); offset++)
    {
        if(strand->ledType == LEDTYPE_APA102) {
            *ptr = (APA_CTRL_BRT_MASK | strand->fxData->brightness);
            ptr++;
        }
        *ptr = r;
        ptr++;
        *ptr = g;
        ptr++;
        *ptr = b;
        ptr++;
    }


    strand->updateLeds = 1;
}