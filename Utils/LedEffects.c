/***************************************
* \file     LedEffects.c
* \date     August 2020
* \author   RJAM
* \brief    Initialises the led effect structure and 
*           houses various LED effects. As many as I can 
*           be arsed writing/stealing
*           TODO: Refactor
*               -These utility functions should ONLY 
*                   - modify led memory
*                   - alert tasks that updates are complete
*               - Use a led_settings_t to determine how to handle
*                   different led types
*               - accept pointer to strand
*               - use semaphores on led mem as it's shared
*               - fresh vs stale data - flag?
*               - remove led-fx init, we don't need a dedicated driver
*                 for this. Use the led strand drivers, these are translation 
*                 functions
*
****************************************/

/********* Includes *******************/
#include <string.h>
#include "LedEffects.h"
#include "Utilities.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_types.h"

#include "CommandAPI.h"

#define RED_FROM_COLOUR32(colour) ((uint8_t)((colour >> 16) & 0xFF))
#define GREEN_FROM_COLOUR32(colour) ((uint8_t)((colour >> 8) & 0xFF))
#define BLUE_FROM_COLOUR32(colour) ((uint8_t)(colour & 0xFF))

#define PIXEL_RED_INDEX(pixel_type) (pixel_type->pixel_index_red)
#define PIXEL_GREEN_INDEX(pixel_type) (pixel_type->pixel_index_green)
#define PIXEL_BLUE_INDEX(pixel_type) (pixel_type->pixel_index_blue)
#define PIXEL_BRT_INDEX(pixel_type) (pixel_type->pixel_index_brt)

#define PIXEL_BYTES_FROM_TYPE(led_type) (led_type->pixel_bytes)

/****** Function Prototypes ***********/

static uint8_t fade_color(uint8_t colour, uint8_t steps, uint8_t step_no);

/****** Global Data *******************/

/************ ISR *********************/

/****** Private Data ******************/

/****** Private Functions *************/

static esp_err_t set_pixel_brightness(pixel_t *p_type, void *pixel, uint8_t brightness) {
    uint8_t *p_arr = pixel;
    
    if(p_type->brt_bits > 0 && brightness <= MAX_VAL_FROM_BITS(p_type->brt_bits)) {
        p_arr[PIXEL_BRT_INDEX(p_type)] = (p_type->brt_base | brightness);
    }
    else {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

static void set_pixel_colour32(pixel_t *p_type, void *pixel, uint32_t colour) {
    uint8_t *p_arr = pixel;
    p_arr[PIXEL_RED_INDEX(p_type)] = RED_FROM_COLOUR32(colour);
    p_arr[PIXEL_GREEN_INDEX(p_type)] = GREEN_FROM_COLOUR32(colour);
    p_arr[PIXEL_BLUE_INDEX(p_type)] = BLUE_FROM_COLOUR32(colour);
}


/** @brief set_pixel_brt_colour32 - set a pixel's colour and brightness 
 *         Setting the colour from a 32-bit integer is easy enough...
 *          For brightness more complicated. Can fade the led colour if no dedicated
 *          brightness control available?
 *          Do brightness by percentage or raw value?
 * 
 */
static void set_pixel_brt_colour32(pixel_t *p_type, void *pixel, uint32_t colour, uint8_t brightness) {

    set_pixel_colour32(p_type, pixel, colour);
    /** TODO: Pixel brightness **/
}


/** single colour **/


static void clear_mem(LedStrand_t *strand) {
    /** obvs be careful with this **/
    for(uint16_t i=0; i<strand->num_leds; i++) {
        set_pixel_colour32(strand->led_t, (strand->pixel_start + (i * PIXEL_BYTES_FROM_TYPE(strand->led_type)), LEDFX_RBG_COL_BLACK);
    }
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



/**
 *  update led funciton pointer
 *  TODO: make esp_err_t type
 */
void led_set_effect(LedStrand_t *strand, ledEffect_t effect) {

    ESP_LOGI("FX", "Updating function %u", effect);

    if(effect == LED_EFFECT_OFF) {
        strand->effects.colour = 0x00000000;
        strand->effects.func = &all_single_colour;
    }
    else if (effect == LED_EFFECT_SINGLE_COLOUR) {
        ESP_LOGI("FX", "Got sc req");
        strand->effects.func = &all_single_colour;
        strand->effects.refresh_t = 1000;
    }
    else if (effect == LED_EFFECT_NIGHTRIDER) {
        ESP_LOGI("FX", "Got nr req");
        strand->effects.func = &ledEffects_nightrider;
        strand->effects.refresh_t = 500;

    } else if (effect == LED_EFFECT_SLOW_FADE) {
        ESP_LOGI("FX", "Got sf req");
        strand->effects.func = &soft_glow;
        strand->effects.refresh_t = 50;
    }
    else {
        ESP_LOGE("LFX", "Unknown mode");
    }

    strand->effects.render_new_frame = true;
    strand->effects.effect = effect;
    return;
}

/**  \brief     A basic night-rider style effect with optional fade    
 *   \param      strand - a pointer to a led control strand
*/
void ledEffects_nightrider(LedStrand_t *strand)
{

    ledEffectData_t *fx = &strand->effects;
    if(fx == NULL) {
        return;
    }
    int fade_len = 2; //fx->var1;
    bool direction = fx->var5;
    uint32_t colour = fx->colour;
    int16_t led_pos = fx->var2;
    uint16_t num_leds = strand->num_leds;

    if (fade_len > num_leds)
    {
        fade_len = 0;
    }

    /** TODO: replace these with MACROS **/
    uint16_t data_length = strand->write_length;
    /** TODO: RGB ORDER! **/
    uint8_t r = colour;
    uint8_t g = ((colour) >> 8);
    uint8_t b = ((colour) >> 16);
    int pixel_offset = 0;

    clear_mem(strand);

    pixel_offset = led_pos * strand->bytes_per_pixel; 

    /* write colour data to led_pos */
    uint8_t *addr = (uint8_t *)strand->strandMem + pixel_offset;

    if(strand->led_t == LEDTYPE_APA102) {
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
                if(strand->led_t == LEDTYPE_APA102) {
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
    }
    else
    {
        /* write fading data */
        if (fade_len && led_pos < num_leds)
        {
            /* as long as not mapping past number of posible leds & not longer than fade len */
            for (int i = 0; (i < fade_len) || (i < ((num_leds - 1) - led_pos)); i++)
            {
                uint8_t *addr = (uint8_t *)strand->strandMem + pixel_offset + ((i + 1) * strand->bytes_per_pixel);
                if(strand->led_t == LEDTYPE_APA102) {
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
    }
        /* Decrement the led_position */
    led_pos = direction ? led_pos+1 : led_pos-1;
    if (led_pos == 0 || led_pos > num_leds-1) 
    {
        led_pos = direction ? num_leds : 0;
        /*if we've reached the end, change dir */
        direction = !direction;
    }

    /** save variables for next run */
    fx->var5 = direction;
    fx->var2 = led_pos;
    strand->updateLeds = 1;
}


void all_single_colour(LedStrand_t *strand) {

    if(strand == NULL || strand->strandMem == NULL || strand->effects == NULL) {
        return;
    }

    uint8_t r, g, b;
    uint8_t *ptr = (uint8_t *)strand->strandMem;

    r = (uint8_t)strand->effects.colour;
    g = (uint8_t)(strand->effects.colour >> 8);
    b = (uint8_t)(strand->effects.colour >> 16);

    for (uint8_t offset = 0; offset < (strand->strandMemLength / strand->bytes_per_pixel); offset++)
    {
        if(strand->led_t == LEDTYPE_APA102) {
            *ptr = (APA_CTRL_BRT_MASK | strand->effects.brightness);
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


void rainbow(LedStrand_t *strand) {

    ledEffectData_t *data = strand->effects;

    return;
}


void soft_glow(LedStrand_t *strand) {

    uint16_t numleds = strand->num_leds;


    if(strand == NULL || strand->strandMem == NULL || strand->effects) {
        return;
    }

    uint8_t r, g, b;
    uint8_t *ptr = (uint8_t *)strand->strandMem;
    uint16_t br_mod = strand->effects.var1;
    bool direction = strand->effects.var5;
    if(br_mod == 0) {
        br_mod = 5;
    };

    r = (uint8_t)strand->effects.colour;
    g = (uint8_t)(strand->effects.colour >> 8);
    b = (uint8_t)(strand->effects.colour >> 16);

    for (uint8_t offset = 0; offset < (strand->strandMemLength / strand->bytes_per_pixel); offset++)
    {
        if(strand->led_t == LEDTYPE_APA102) {
            *ptr = (APA_CTRL_BRT_MASK | strand->effects.brightness);
            ptr++;
        }
        *ptr = fade_color(r, br_mod, (offset % br_mod));
        ptr++;
        *ptr = fade_color(g, br_mod, (offset % br_mod));
        ptr++;
        *ptr = fade_color(b, br_mod, (offset % br_mod));
        ptr++;
    }

    strand->effects.var5 = direction;

    if(br_mod == 1 || br_mod > 7) {
        direction = !direction;
        strand->effects.var5 = direction;    
    }
    strand->effects.var1 = direction ? (br_mod - 1) : (br_mod + 1); 
    strand->updateLeds = 1;
    return;
}