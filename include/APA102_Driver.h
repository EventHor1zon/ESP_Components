/****************************************
* \file     APA102_Driver,h
* \brief    Header file for the APA102_Driver
* \date     August 2020 
* \author   RJAM
****************************************/

#ifndef APA102_DRIVER_H
#define APA102_DRIVER_H

#include "sdkconfig.h" 

#ifdef CONFIG_USE_PERIPH_MANAGER
#include "CommandAPI.h"
#endif

/********* Includes ********************/

#include "./LedEffects.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "sdkconfig.h"


/********* Definitions *****************/

#define APA102_CLK_SPD 200000
#define APA_BYTES_PER_PIXEL 4
#define APA_TIMER_ID 0
#define APA_CTRL_MAX_BR 31
#define APA_CTRL_BRT_MASK 0b11100000
#define APA_ZERO_FRAME_SIZE_BYTES 4
#define APA_MAX_NUM_LEDS 200
#define APA_DMA_CHANNEL 1

#define APA_ZERO_FRAME_SIZE 4 
#define APA_MAX_TRANSFER_SIZE (APA_MAX_NUM_LEDS * APA_BYTES_PER_PIXEL) + APA_ZERO_FRAME_SIZE + APA_MAX_NUM_LEDS + 32 

#define APA_SEMTAKE_TIMEOUT 500

#define APA_NOTIFY_BUILD_FRAME (1 << 2)
#define APA_NOTIFY_WRITE_LEDS (1 << 3)
#define APA_NOTIFY_REFRESH_LEDS (APA_NOTIFY_BUILD_FRAME | APA_NOTIFY_WRITE_LEDS)

/********** Types **********************/
const char *APA_TAG;

#ifdef CONFIG_USE_PERIPH_MANAGER
#include "CommandAPI.h"
#define apa_param_len 5
const parameter_t apa_param_map[apa_param_len];
const peripheral_t apa_periph_template;

#endif 

typedef struct apa102_init 
{
    uint8_t numleds;
    uint16_t clock_pin;
    uint16_t data_pin;
    uint8_t spi_bus;
    bool init_spi;
    bool use_dma;
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
