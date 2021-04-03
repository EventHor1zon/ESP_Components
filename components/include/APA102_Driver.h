/****************************************
* \file     APA102_Driver,h
* \brief    Header file for the APA102_Driver
* \date     August 2020 
* \author   RJAM
****************************************/

#ifndef APA102_DRIVER_H
#define APA102_DRIVER_H

#ifdef CONFIG_USE_PERIPH_MANAGER
#include "CommandAPI.h"
#endif

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

//#define CONFIG_USE_PERIPH_MANAGER 0

/********** Types **********************/
const char *APA_TAG;

#ifdef CONFIG_USE_PERIPH_MANAGER
#define apa_param_len 5
const parameter_t apa_param_mappings[apa_param_len];
#endif 

typedef struct apa102_init 
{
    uint8_t numleds;
    uint16_t clock_pin;
    uint16_t data_pin;
    uint8_t spi_bus;
    bool init_spi;
} apa102_init_t;


/******** Function Definitions *********/

esp_err_t apa_getNumleds(StrandData_t *strand, uint32_t *var);

esp_err_t apa_getMode(StrandData_t *strand, uint32_t *var);

esp_err_t apa_setMode(StrandData_t *strand, uint8_t *var);

esp_err_t apa_getColour(StrandData_t *strand, uint32_t *var);

esp_err_t apa_setColour(StrandData_t *strand, uint32_t *var);

esp_err_t apa_getBrightness(StrandData_t *strand, uint8_t *var);

esp_err_t apa_setBrightness(StrandData_t *strand, uint8_t *var);

StrandData_t *APA102_init(apa102_init_t *init_data);

#endif /* APA102_DRIVER_H */
