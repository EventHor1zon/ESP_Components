/****************************************
* \file     LedEffects.h
* \brief    Header File for the ledEffects lib
* \date     August 2020
* \author   RJAM
****************************************/

#ifndef LEDEFFECTS_H
#define LEDEFFECTS_H

/********* Includes ********************/

#include "./WS2812_Driver.h"

/********* Definitions *****************/

#define LEDFX_MAX_TIMERS 12

typedef void (*EffectFunction)(void);

// typedef struct ledFx
// {
//     StrandData_t *strand;
//     TimerHandle_t timerHandle;
// } ledFx_t;

/********** Types **********************/

/******** Function Definitions *********/

/**  \brief     A basic night-rider style effect with optional fade 
 *      
 *  \param      strand - a pointer to a led control strand
 *  \param      fade_len - length of fade trail
 *  \param      colour - 32 bit colour XBGR format 
 * 
*/
void ledEffects_nightrider(StrandData_t *strand, int fade_len, uint32_t colour);

/** 
 *  \brief fade_color - reduces the value of an 8-bit colour 
 * 
 *  \param colour - 8-bit colour value
 *  \param steps  - number of steps to dim
 *  \param step_no - the current step niumber
 * 
 *  \return  the adjusted colour value
*/
uint8_t fade_color(uint8_t colour, uint8_t steps, uint8_t step_no);

#endif /* LEDEFFECTS_H */
