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

#include "../Utils/LedEffects.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "sdkconfig.h"


/********* Definitions
 *  @defgroup APA102_Definitions
 *  @brief  APA102 definitions - definitions for the device driver
 *  @{
 **/ 


#define APA_CONFIG_MAX_DEVICES          8
#define APA_CONFIG_CMD_QUEUE_LEN        4
#define APA_CONFIG_SPI_CLK_HZ           200000
#define APA_CONFIG_MAX_LEDMEM_BYTES     2048

#define APA_BYTES_PER_PIXEL             4

#define APA_CTRL_MAX_BR                 31
#define APA_CTRL_BRT_MASK               0b11100000
#define APA_DMA_CHANNEL 1
#define APA_FRAME_LEN 4

#define APA_START_FRAME_SZ  APA_FRAME_LEN
#define APA_END_FRAME_SZ    APA_FRAME_LEN

#define APA_SEMTAKE_TIMEOUT 500

#define APA_NOTIFY_BUILD_FRAME (1 << 2)
#define APA_NOTIFY_WRITE_LEDS (1 << 3)
#define APA_NOTIFY_REFRESH_LEDS (APA_NOTIFY_BUILD_FRAME | APA_NOTIFY_WRITE_LEDS)

/********** Types **********************/
const char *APA_TAG;

#ifdef CONFIG_USE_PERIPH_MANAGER
/** Config use periph manager includes/defines **/
#include "CommandAPI.h"
#define apa_param_len 5
const parameter_t apa_param_map[apa_param_len];
const peripheral_t apa_periph_template;

#endif 


typedef LedStrand_t * APA_HANDLE_t;

typedef enum {
    APA_CMD_OFF,
    APA_CMD_UPDATE_FRAME,
    APA_CMD_UPDATE_LEDS,
    APA_CMD_NEW_MODE,
    APA_CMD_MAX,
} APA_CMD_t;

/** @struct apa102_init_t
 *  @brief Contains the information used to intialise the driver
 */
typedef struct apa102_init 
{
    uint8_t numleds;                /*!< number of leds in strand **/
    uint16_t clock_pin;             /*!< SPI clock pin **/
    uint16_t data_pin;              /*!< SPI data pin **/
    uint8_t channel;                /*!< SPI bus number **/
    bool init_spi;                  /*!< driver should initialise the spi bus **/
    bool use_dma;                   /*!< SPI should use DMA interface **/
} apa102_init_t;


/** @struct apa102_cmd_t
 *  @brief structure of a command to send to the 
 *          driver task
 */

typedef struct apa102_cmd_t
{
    /* data */
    APA_HANDLE_t strand;
    APA_CMD_t cmd;
} apa_msg_t;


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
#ifdef CONFIG_DRIVERS_USE_HEAP
APA_HANDLE_t APA102_init(apa102_init_t *init_data);
#else
APA_HANDLE_t APA102_init(APA_HANDLE_t strand, apa102_init_t *init_data);
#endif
/**
 * @brief Get the number of leds in the strand
 *
 * @param   strand - the APA102 driver device handle
 * @param   var - pointer to value storage
 * @return  ESP_OK or Error
 */
esp_err_t apa_getNumleds(APA_HANDLE_t strand, uint32_t *var);

/**
 * @brief Get the current LED animation mode
 *
 * @param   dev - the A4988 driver device handle
 * @param   steps - pointer to value storage
 * @return  ESP_OK or Error
 */
esp_err_t apa_getMode(APA_HANDLE_t strand, uint32_t *var);

/**
 * @brief Set the current LED animation mode - see
 *          LedEffects driver for valid values
 *
 * @param   dev - the APA102 driver device handle
 * @param   var - pointer to value 
 * @return  ESP_OK or Error
 */
esp_err_t apa_setMode(APA_HANDLE_t strand, uint8_t *var);

/**
 * @brief Get the current LED colour - a 32-bit value 
 *          in order of XRGB
 *          
 * @param   dev - the APA102 driver device handle
 * @param   var - pointer to value storage
 * @return  ESP_OK or Error
 */
esp_err_t apa_getColour(APA_HANDLE_t strand, uint32_t *var);

/**
 * @brief Set the current LED colour - a 32-bit value 
 *          in order of XRGB
 *          
 * @param   dev - the APA102 driver device handle
 * @param   var - pointer to value 
 * @return  ESP_OK or Error
 */
esp_err_t apa_setColour(APA_HANDLE_t strand, uint32_t *var);

/**
 * @brief Get the current LED brightness
 *          
 * @param   dev - the APA102 driver device handle
 * @param   var - pointer to value storage
 * @return  ESP_OK or Error
 */
esp_err_t apa_getBrightness(APA_HANDLE_t strand, uint8_t *var);

/**
 * @brief Set the current LED brightness
 *          
 * @param   dev - the APA102 driver device handle
 * @param   var - pointer to value storage
 * @return  ESP_OK or Error
 */
esp_err_t apa_setBrightness(APA_HANDLE_t strand, uint8_t *var);

/** @} APA102_functions **/

#endif /* APA102_DRIVER_H */
