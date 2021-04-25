/****************************************
* \file     LedEffects.h
* \brief    Header File for the ledEffects lib
* \date     August 2020
* \author   RJAM
****************************************/

#ifndef LEDEFFECTS_H
#define LEDEFFECTS_H

/********* Includes ********************/
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "driver/rmt.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "CommandAPI.h"
/********* Definitions *****************/

#define LEDFX_MAX_TIMERS 12 
#define LEDFX_NUM_EFFECTS 3

typedef void (*EffectFunction)(void *);

// typedef struct ledFx
// {
//     StrandData_t *strand;
//     TimerHandle_t timerHandle;
// } ledFx_t;

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

typedef enum ledType
{
    LEDTYPE_WS2812B,
    LEDTYPE_APA102,
    LEDTYPE_NONE = 0xFF
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
    uint16_t refresh_t;       /** < refresh time in ms - 0 for no refresh */
    bool updateEffect;        /** < led effect has been changed - adjust parameters */
    uint8_t brightness;       /** < led brigthness  */
    uint32_t LedEffectData_t; /** < struct for holding led effect variables if needed */
    uint16_t var1;            /**< some variables for function use **/
    uint16_t var2;
    uint32_t var3;
    uint32_t var4;
    bool var5;

} ledEffectData_t;

typedef struct StrandData
{
    ledType_t ledType;                /** < type of LED **/
    uint8_t strandIndex;              /** < index of the current strand  */
    uint8_t updateLeds;               /** < update flag - led mem has changed */
    uint8_t bytes_per_pixel;          /** < bytes per pixel */  
    uint16_t numLeds;                 /** < num leds in strand           */
    uint16_t strandMemLength;         /** < length of memory    */
    uint32_t *strandMem;              /** < pointer to LED data          */
    ledEffectData_t *fxData;          /** < pointer to led effect data */
    SemaphoreHandle_t memSemphr;      /** < sempahore for led data access*/
    uint8_t spi_channel_no;           /** < the channel of the SPI peripheral used **/
    TimerHandle_t refreshTimer;       /** < handle for the effect refresh timer **/
    rmt_channel_t dataChannel;        /** < TODO: replace this & next with union/bitfield **/
    spi_device_handle_t ledSPIHandle; /** < spi device handle **/
} StrandData_t;

/******** Function Definitions *********/

ledEffectData_t *ledEffectInit(StrandData_t *strand);

/**  \brief     A basic night-rider style effect with optional fade 
 *      
 *  \param      strand - a pointer to a led control strand
*/
void ledEffects_nightrider(StrandData_t *strand);

/** \brief: set all leds to colour
 *  \param: strand pointer
 **/
void all_single_colour(StrandData_t *strand);


void ledFx_updateMode(StrandData_t *strand);

void soft_glow(StrandData_t *strand);

#endif /* LEDEFFECTS_H */
