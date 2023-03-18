/****************************************
* \file     LedEffects.h
* \brief    Header File for the ledEffects lib
* \date     August 2020
* \author   RJAM
*
*
*
*
*
*
*
*
****************************************/

#ifndef LEDEFFECTS_H
#define LEDEFFECTS_H

/********* Includes ********************/
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "sdkconfig.h"

#include "driver/rmt.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "CommandAPI.h"
/********* Definitions *****************/

#define LEDFX_MAX_TIMERS 12 
#define LEDFX_NUM_EFFECTS 3

#define LEDFX_RBG_COL_BLACK 0x00000000
#define LEDFX_RGB_COL_RED   0x00FF0000
#define LEDFX_RGB_COL_GREEN 0x0000FF00
#define LEDFX_RGB_COL_BLUE  0x000000FF

#define LEDFX_BRIGHTNESS_MAX    100
#define LEDFX_BRIGHTNESS_MIN    1
#define LEDFX_BRIGHTNESS_OFF    0


typedef void (*EffectFunction)(void *);

typedef struct 
{
    /* data */  
    char *name;                 /** Name of led type - usused (remove maybe) **/
    uint8_t pixel_bytes;        /** number of bytes per pixel **/
    uint8_t pixel_index_red;    /** pixel red index **/
    uint8_t pixel_index_green;  /** pixel green index **/
    uint8_t pixel_index_blue;   /** pixel blue index **/
    uint8_t pixel_index_brt;    /** pixel brightness index **/
    uint8_t brt_bits;           /** number of bits used to define brightness
                                    0 indicates no brightness byte
                                     **/
    uint8_t brt_base; 
} pixel_t;


const pixel_t pixel_types[2];


const uint8_t led_type_n;


/********** Types **********************/

typedef enum ledEffect
{
    LED_EFFECT_OFF,
    LED_EFFECT_SINGLE_COLOUR,
    LED_EFFECT_NIGHTRIDER,
    LED_EFFECT_SLOW_FADE,
    // LED_EFFECT_RAINBOW,
    // LED_EFFECT_COLOUR_FADE,

    LED_EFFECT_NO_EFFECT = 0xFF
} ledEffect_t;

/** LedType_t 
 * 
 *  enum of different types of RGB addressable LED
 * 
*/

typedef enum led_t
{
    LEDTYPE_WS2812B,
    LEDTYPE_APA102,
    LEDTYPE_INVALID
} ledType_t;

/**
 *  Led effect struct
 *  Used for tracking persistent LED effect variables?
 *  Try to build all effects to use the same type of data struct 
 *  TODO: better variable names, use union or bitfield?
*/
typedef struct ledEffectData
{
    ledEffect_t effect;       /** < the current effect in enumeration */
    EffectFunction func;      /** < a pointer to the LedEffects function */
    uint32_t colour;          /** < colour - colour */
    uint8_t brightness;       /** < led brigthness  */
    uint16_t refresh_t;       /** < refresh time in ms - 0 for no refresh */
    uint16_t var1;            /**< some variables for function use **/
    uint16_t var2;
    uint32_t var3;
    bool var5;
    bool write_new_frame;
    bool render_new_frame;

} ledEffectData_t;

typedef struct StrandData
{
    ledType_t led_t;                /** < type of LED **/
    uint16_t num_leds;                 /** < num leds in strand           */
    SemaphoreHandle_t strand_sem;      /** < sempahore for led data access*/
    uint32_t write_length;
    ledEffectData_t effects;
    void *data_address;
    void *pixel_start;
    void *pixel_end;
    pixel_t *led_type;
    uint8_t channel;                  /** < the channel of the SPI peripheral used **/
    bool use_dma;
    TimerHandle_t led_timer;       /** < handle for the effect refresh timer **/
    spi_device_handle_t iface_handle; /** < spi device handle **/
} LedStrand_t;

/******** Function Definitions *********/

ledEffectData_t *ledEffectInit(LedStrand_t *strand);

/**  \brief     A basic night-rider style effect with optional fade 
 *      
 *  \param      strand - a pointer to a led control strand
*/
void ledEffects_nightrider(LedStrand_t *strand);

/** \brief: set all leds to colour
 *  \param: strand pointer
 **/
void all_single_colour(LedStrand_t *strand);


void ledfx_set_mode(LedStrand_t *strand, ledEffect_t effect);

void soft_glow(LedStrand_t *strand);

#endif /* LEDEFFECTS_H */
