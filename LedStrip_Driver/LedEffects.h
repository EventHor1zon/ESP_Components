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
#include "LedStrip_Driver.h"

/********* Definitions *****************/


#define LEDFX_CONFIG_MIN_FRAME_T 50     /** minimum frame refresh time in ms **/

#define LEDFX_RBG_COL_BLACK 0x00000000
#define LEDFX_RGB_COL_RED   0x00FF0000
#define LEDFX_RGB_COL_GREEN 0x0000FF00
#define LEDFX_RGB_COL_BLUE  0x000000FF

#define LEDFX_BRIGHTNESS_MAX    100
#define LEDFX_BRIGHTNESS_MIN    1
#define LEDFX_BRIGHTNESS_OFF    0



typedef void (*EffectFunction)(void *);


/********** Types **********************/

typedef enum ledEffect
{
    LED_EFFECT_OFF,
    LED_EFFECT_SINGLE_COLOUR,
    LED_EFFECT_NIGHTRIDER,
    LED_EFFECT_SLOW_FADE,
    LED_EFFECT_RAINBOW,
    LED_EFFECT_COLOUR_FADE,
    
    LED_EFFECT_INVALID,
} ledEffect_t;


/** @struct fxdata_t
 *  @brief  Led effect struct
 *          Used for tracking persistent LED effect variables?
 *          Try to build all effects to use the same type of data struct 
 *          TODO: better variable names, use unions or bitfields?
*/
typedef struct 
{
    ledEffect_t effect;         /** the current effect in enumeration */
    effect_fn func;             /** a pointer to the LedEffects function */
    uint32_t rgb_col;           /** colour - colour */
    uint8_t brightness;         /** led brigthness  */
    uint16_t frame_time;        /** refresh time in ms - 0 for no refresh   **/
    uint8_t u8bitfield;         /** Bitfield storage for effect functions   **/
    uint16_t u16storage_a;      /** Value storage for effect functions      **/
    uint16_t u16storage_b;      /** Value storage for effect functions      **/
    uint32_t u32storage_a;      /** Value storage for effect functions      **/
    uint32_t u32storage_b;      /** Value storage for effect functions      **/
} fxdata_t;


/**  \brief     A basic night-rider style effect with optional fade 
 *      
 *  \param      strand - a pointer to a led control strand
*/
void lfx_nightrider(void *strand);

void lfx_single_colour(void *strand);

void lfx_soft_glow(void *strand);

void lfx_test_mode(void *strand);

void lfx_rainbow_scroll(void *strand);

void lfx_rainbow_strip(void *strand);

void ledfx_set_mode(void *strand, ledEffect_t effect);

#endif /* LEDEFFECTS_H */
