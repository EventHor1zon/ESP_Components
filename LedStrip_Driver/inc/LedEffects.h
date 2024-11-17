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



/********** Types **********************/

/**  \brief     A basic night-rider style effect with optional fade 
 *      
 *  \param      strand - a pointer to a led control strand
*/
void lfx_nightrider(LEDSTRIP_h strip);

void lfx_single_colour(LEDSTRIP_h strip);

void lfx_soft_glow(LEDSTRIP_h strip);

// void lfx_test_mode(LEDSTRIP_h strip);

// void lfx_rainbow_scroll(LEDSTRIP_h strip);

// void lfx_rainbow_strip(LEDSTRIP_h strip);

esp_err_t lfx_set_mode(LEDSTRIP_h strip, ledEffect_t effect);

#endif /* LEDEFFECTS_H */
