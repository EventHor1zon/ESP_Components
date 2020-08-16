/****************************************
* \file     APA102_Driver,h
* \brief    Header file for the APA102_Driver
* \date     August 2020 
* \author   RJAM
****************************************/

#ifndef APA102_DRIVER_H
#define APA102_DRIVER_H

/********* Includes ********************/

#include "LedEffects.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

/********* Definitions *****************/

#define APA_BYTES_PER_PIXEL 4

/********** Types **********************/

typedef struct strandData
{
    uint8_t strandIndex;         /** <index of the current strand  */
    uint8_t updateLeds;          /** <update flag - led mem has changed */
    uint16_t numLeds;            /** <num leds in strand           */
    uint16_t strandMemLength;    /** <length of memory    */
    uint8_t *strandMem;          /** <pointer to LED data          */
    ledEffectData_t *fxData;     /** < pointer to led effect data */
    SemaphoreHandle_t memSemphr; /** <sempahore for led data access*/
    //rmt_channel_t dataChannel;   /** <rmt channel driving leds (uint8_t)    */
    uint8_t spi_channel_no;     /** < the channel of the SPI peripheral used **/
    TimerHandle_t refreshTimer; /** < handle for the effect refresh timer **/
} StrandData_t;

/******** Function Definitions *********/

#endif /* APA102_DRIVER_H */
