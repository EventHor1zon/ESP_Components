/****************************************
* \file     APA102_Driver,h
* \brief    Header file for the APA102_Driver
* \date     August 2020 
* \author   RJAM
****************************************/

#ifndef APA102_DRIVER_H
#define APA102_DRIVER_H

/********* Includes ********************/

#include "./LedEffects.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

/********* Definitions *****************/

#define DEBUG_MODE 1

#define APA_BYTES_PER_PIXEL 4
#define APA_TIMER_ID 0
#define APA_CTRL_MAX_BR 31
#define APA_CTRL_BRT_MASK 0b11100000

#define APA_SEMTAKE_TIMEOUT 500

/********** Types **********************/
const char *APA_TAG;

// typedef struct apa102StrandData
// {
//     uint8_t strandIndex;              /** < index of the current strand  */
//     uint8_t updateLeds;               /** < update flag - led mem has changed */
//     uint16_t numLeds;                 /** < num leds in strand           */
//     uint16_t strandMemLength;         /** < length of memory    */
//     uint32_t *strandMem;              /** < pointer to LED data          */
//     ledEffectData_t *fxData;          /** < pointer to led effect data */
//     SemaphoreHandle_t memSemphr;      /** < sempahore for led data access*/
//     uint8_t spi_channel_no;           /** < the channel of the SPI peripheral used **/
//     TimerHandle_t refreshTimer;       /** < handle for the effect refresh timer **/
//     spi_device_handle_t ledSPIHandle; /** < spi device handle **/
// } apaStrandData_t;

typedef struct apa102_init 
{
    uint8_t numleds;
    uint16_t clock_pin;
    uint16_t data_pin;
    uint8_t spi_bus;
    bool init_spi;
} apa102_init_t;


/******** Function Definitions *********/

StrandData_t *APA102_init(apa102_init_t *init_data);

#endif /* APA102_DRIVER_H */
