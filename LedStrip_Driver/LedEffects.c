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
*               - accept pointer to strip
*               - use semaphores on led mem as it's shared
*               - fresh vs stale data - flag?
*               - remove led-fx init, we don't need a dedicated driver
*                 for this. Use the led strip drivers, these are translation 
*                 functions
*
****************************************/

/********* Includes *******************/
#include <string.h>
#include "LedStrip_Driver.h"
#include "LedEffects.h"
#include "Utilities.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_types.h"

#include "CommandAPI.h"

#define RED_FROM_COLOUR32(colour) ((uint8_t)((colour >> 16) & 0xFF))
#define GREEN_FROM_COLOUR32(colour) ((uint8_t)((colour >> 8) & 0xFF))
#define BLUE_FROM_COLOUR32(colour) ((uint8_t)(colour & 0xFF))

#define MAX_VAL_FROM_BITS(bits) ((1 << bits)-1)

#define PIXEL_RED_INDEX(pixel_type) (pixel_type->pixel_index_red)
#define PIXEL_GREEN_INDEX(pixel_type) (pixel_type->pixel_index_green)
#define PIXEL_BLUE_INDEX(pixel_type) (pixel_type->pixel_index_blue)
#define PIXEL_BRT_INDEX(pixel_type) (pixel_type->pixel_index_brt)

#define PIXEL_BYTES_FROM_TYPE(led_type) (led_type->pixel_bytes)
#define PIXEL_FROM_INDEX(strip, index) (strip->pixel_start + (strip->led_type->pixel_bytes * index))

/****** Function Prototypes ***********/

/****** Global Data *******************/

/************ ISR *********************/

/****** Private Data ******************/

/****** Private Functions *************/

static esp_err_t set_pixel_brightness(ledtype_t *p_type, void *pixel, uint8_t brightness) {
    uint8_t *p_arr = pixel;
    
    if(p_type->brt_bits > 0 && brightness <= MAX_VAL_FROM_BITS(p_type->brt_bits)) {
        p_arr[PIXEL_BRT_INDEX(p_type)] = (p_type->brt_base | brightness);
    }
    else {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

static void set_pixel_colour32(ledtype_t *p_type, uint8_t *pixel, uint32_t colour) {
    pixel[p_type->pixel_index_red] = RED_FROM_COLOUR32(colour);
    pixel[p_type->pixel_index_green] = GREEN_FROM_COLOUR32(colour);
    pixel[p_type->pixel_index_blue] = BLUE_FROM_COLOUR32(colour);
}


/** @brief set_pixel_brt_colour32 - set a pixel's colour and brightness 
 *         Setting the colour from a 32-bit integer is easy enough...
 *          For brightness more complicated. Can fade the led colour if no dedicated
 *          brightness control available?
 *          Do brightness by percentage or raw value?
 * 
 */
static void set_pixel_brt_colour32(ledtype_t *p_type, void *pixel, uint32_t colour, uint8_t brightness) {

    set_pixel_colour32(p_type, pixel, colour);
    /** TODO: Pixel brightness **/
    set_pixel_brightness(p_type, pixel, brightness);
}


/** single colour **/


static void clear_mem(LEDSTRIP_h strip) {
    /** obvs be careful with this **/
    for(uint16_t i=0; i<strip->num_leds; i++) {
        set_pixel_colour32(strip->led_type, PIXEL_FROM_INDEX(strip, i), LEDFX_RBG_COL_BLACK);
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


/**  \brief     A basic night-rider style effect with optional fade    
 *   \param      strip - a pointer to a led control strip
*/
void lfx_nightrider(LEDSTRIP_h strip)
{

    fxdata_t *fx = &strip->fx;
    int fade_len = 2; /** TODO: variablise **/
    bool direction = fx->u8bitfield;
    int16_t led_pos = fx->u16storage_a;

    if (fade_len >= strip->num_leds)
    {
        fade_len = 0;
    }

    /** TODONE: So much easier with macros! **/
    for(int i=0; i < strip->num_leds; i++) {
        set_pixel_colour32(strip->led_type, strip->pixel_start, LEDFX_RBG_COL_BLACK);
    }

    set_pixel_brt_colour32(strip->led_type, PIXEL_FROM_INDEX(strip, led_pos), strip->fx.colour, LEDFX_BRIGHTNESS_MAX);

    if (direction)
    {
        /* write fading data */
        if (fade_len && led_pos > 0)
        {
            /* as long as not mapping past number of posible leds & not longer than fade len */
            for (int i = 0; (i < fade_len) || (i < led_pos); i++)
            {
                set_pixel_brt_colour32(strip->led_type, PIXEL_FROM_INDEX(strip, led_pos-i), strip->fx.colour, (int)(LEDFX_BRIGHTNESS_MAX / i+2));
            }
        }
    }
    else
    {
        /* write fading data */
        if (fade_len && led_pos < strip->num_leds)
        {
            /* as long as not mapping past number of posible leds & not longer than fade len */
            for (int i = 0; (i < fade_len) || (i < ((strip->num_leds - 1) - led_pos)); i++)
            {
                set_pixel_brt_colour32(strip->led_type, PIXEL_FROM_INDEX(strip, led_pos+i), strip->fx.colour, (int)(LEDFX_BRIGHTNESS_MAX / i+2));
            }
        }
    }
        /* Decrement the led_position */
    led_pos = (direction ? led_pos+1 : led_pos-1);
    if ((led_pos == 0 && !direction) || (led_pos == strip->num_leds-1 && direction)) 
    {
        /*if we've reached the end, change dir */
        direction = !direction;
    }

    /** save variables for next run */
    fx->u8bitfield = direction;
    fx->u16storage_a = led_pos;
    strip->write_frame = 1;
}


void lfx_single_colour(LEDSTRIP_h strip) {

    uint32_t offset = 0;
    for(int i=0; i < strip->num_leds; i++) {
        offset = (strip->led_type->pixel_bytes * i);
        set_pixel_brt_colour32(strip->led_type, strip->pixel_start+offset, strip->fx.colour, strip->fx.brightness);
    }

    strip->write_frame = 1;
}


void rainbow(LEDSTRIP_h strip) {

    return;
}


void lfx_soft_glow(LEDSTRIP_h strip) {

    uint8_t r, g, b;
    uint16_t br_mod = strip->fx.u16storage_a;
    bool direction = strip->fx.u8bitfield;
    if(br_mod == 0 || br_mod > 10) {
        br_mod = 5;
    };

    for (uint8_t offset = 0; offset < strip->num_leds; offset++)
    {
        set_pixel_brt_colour32(strip->led_type, PIXEL_FROM_INDEX(strip, offset), strip->fx.colour, br_mod*10);
    }

    strip->fx.u8bitfield = direction;

    if(br_mod == 1 || br_mod > 9) {
        direction = !direction;
        strip->fx.u8bitfield = direction;    
    }
    strip->fx.u16storage_a = direction ? (br_mod - 1) : (br_mod + 1); 
    strip->write_frame = true;
    return;
}


/**
 *  update led funciton pointer
 *  TODO: make esp_err_t type
 */
esp_err_t lfx_set_mode(LEDSTRIP_h strip, ledEffect_t effect) {
    
    esp_err_t err = ESP_OK;

#ifdef DEBUG_MODE
    ESP_LOGI("FX", "Updating function %u", effect);
#endif

    if(effect == LED_EFFECT_OFF) {
        strip->fx.colour = 0x00000000;
        strip->fx.func = &lfx_single_colour;
    }
    else if (effect == LED_EFFECT_SINGLE_COLOUR) {
        ESP_LOGI("FX", "Got sc req");
        strip->fx.func = &lfx_single_colour;
        strip->fx.frame_time = 1000;
    }
    else if (effect == LED_EFFECT_NIGHTRIDER) {
        ESP_LOGI("FX", "Got nr req");
        strip->fx.func = &lfx_nightrider;
        strip->fx.frame_time = 500;

    } else if (effect == LED_EFFECT_SLOW_FADE) {
        ESP_LOGI("FX", "Got sf req");
        strip->fx.func = &lfx_soft_glow;
        strip->fx.frame_time = 50;
    }
    else {
        err = ESP_ERR_INVALID_ARG;
        ESP_LOGE("LFX", "Unknown mode");
    }

    if(!err) {
        strip->render_frame = true;
        strip->fx.effect = effect;
    }

    return err;
}

