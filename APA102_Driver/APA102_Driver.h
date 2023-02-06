/****************************************
* \file     APA102_Driver,h
* \brief    Header file for the APA102_Driver
* \date     August 2020 
* \author   RJAM
****************************************/

#ifndef APA102_DRIVER_H
#define APA102_DRIVER_H

#include "sdkconfig.h" 


/********* Includes ********************/

#include "./LedEffects.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "sdkconfig.h"


/********* Definitions
 *  @defgroup APA102_Definitions
 *  @brief  APA102 definitions - definitions for the device driver
 *  @{
 **/ 

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

/** @struct apa102_init_t
 *  @brief Contains the information used to intialise the driver
 */
typedef struct apa102_init 
{
    uint8_t numleds;                /*!< number of leds in strand **/
    uint16_t clock_pin;             /*!< SPI clock pin **/
    uint16_t data_pin;              /*!< SPI data pin **/
    uint8_t spi_bus;                /*!< SPI bus number **/
    bool init_spi;                  /*!< driver should initialise the spi bus **/
    bool use_dma;                   /*!< SPI should use DMA interface **/
} apa102_init_t;

/** @} APA102_definitions */


/** @defgroup APA102_functions
 *  @brief The APA102 Driver functions 
 *  @{
 */

/**
 * @brief Initialises the APA102 Driver and returns the driver handle
 *
 * @param   init    Pointer to a populated APA102_init struct
 * @return  Device handle or NULL on error
 */
StrandData_t *APA102_init(apa102_init_t *init_data);

/**
 * @brief Get the number of leds in the strand
 *
 * @param   strand - the APA102 driver device handle
 * @param   var - pointer to value storage
 * @return  ESP_OK or Error
 */
esp_err_t apa_getNumleds(StrandData_t *strand, uint32_t *var);

/**
 * @brief Get the current LED animation mode
 *
 * @param   dev - the A4988 driver device handle
 * @param   steps - pointer to value storage
 * @return  ESP_OK or Error
 */
esp_err_t apa_getMode(StrandData_t *strand, uint32_t *var);

/**
 * @brief Set the current LED animation mode - see
 *          LedEffects driver for valid values
 *
 * @param   dev - the APA102 driver device handle
 * @param   var - pointer to value 
 * @return  ESP_OK or Error
 */
esp_err_t apa_setMode(StrandData_t *strand, uint8_t *var);

/**
 * @brief Get the current LED colour - a 32-bit value 
 *          in order of XRGB
 *          
 * @param   dev - the APA102 driver device handle
 * @param   var - pointer to value storage
 * @return  ESP_OK or Error
 */
esp_err_t apa_getColour(StrandData_t *strand, uint32_t *var);

/**
 * @brief Set the current LED colour - a 32-bit value 
 *          in order of XRGB
 *          
 * @param   dev - the APA102 driver device handle
 * @param   var - pointer to value 
 * @return  ESP_OK or Error
 */
esp_err_t apa_setColour(StrandData_t *strand, uint32_t *var);

/**
 * @brief Get the current LED brightness
 *          
 * @param   dev - the APA102 driver device handle
 * @param   var - pointer to value storage
 * @return  ESP_OK or Error
 */
esp_err_t apa_getBrightness(StrandData_t *strand, uint8_t *var);

/**
 * @brief Set the current LED brightness
 *          
 * @param   dev - the APA102 driver device handle
 * @param   var - pointer to value storage
 * @return  ESP_OK or Error
 */
esp_err_t apa_setBrightness(StrandData_t *strand, uint8_t *var);

/** @} APA102_functions **/

#endif /* APA102_DRIVER_H */
