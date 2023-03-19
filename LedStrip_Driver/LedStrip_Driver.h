/****************************************
* \file     LedStrip_Driver.h
* \brief    This driver is used to control an assortment of addressable LEDs
*           Driver started because the refactor of individual led strip drivers
*           was becoming more & more abstract & the led effects function never 
*           really fit in properly.
*           Idea is to use abstract translation functions to merge the existing
*           led strip drivers & utilise the led effects functions more efficiently
*           Try to apply some smart object-based thinking to this driver.
*
* \date     March 23
* \author   RJAM
****************************************/

#ifndef LEDSTRIP_DRIVER_H
#define LEDSTRIP_DRIVER_H

#include "esp_err.h"

/********* Includes ********************/

/********* Definitions *****************/


/** Driver settings **/
#define LEDSTRIP_CONFIG_QUEUE_LEN   4
#define LEDSTRIP_CONFIG_TASK_PRIO   5
#define LEDSTRIP_CONFIG_TASK_STACK  5012

typedef esp_err_t (*write_fn)(void *, void *);

/********** Types **********************/

/** @defgroup   LedStrip_Structures 
 *  @brief      Core structures for initialising and using the 
 *              LedStrip driver
 *  @{ 
 */

/** @struct ledtype_t 
 *  @brief  description of the led 
 *          type - used internally
 *          To add a new led, hopefully just add a new definition 
 *          to the table. Want to do translation functions here 
 *          but difficult with the RMT callbacks etc.  
 **/
typedef struct {
    /* data */  
    char *name;                 /** name of led strip, for ease of use **/
    uint8_t pixel_bytes;        /** number of bytes per pixel **/
    uint8_t pixel_index_red;    /** pixel red index **/
    uint8_t pixel_index_green;  /** pixel green index **/
    uint8_t pixel_index_blue;   /** pixel blue index **/
    uint8_t pixel_index_brt;    /** pixel brightness index **/
    uint8_t brt_bits;           /** number of bits used to define brightness
                                    0 indicates no brightness byte.
                                **/
    uint8_t brt_base;           /** base value of the brightness byte **/
    write_fn write;             /** function to write frame data **/
} ledtype_t;


/** @struct ledstrip_init_t 
 *  @brief  the neccessary info to initialise the led
 *          strip
 **/
typedef struct {
    uint8_t channel;            /** the RMT or SPI channel (depending on led type)  **/
    uint8_t num_leds;           /** number of leds in the strip                     **/
    ledtype_t led_type;         /** the type of leds in the strip **/
} ledstrip_init_t;


/** @struct ledstrip_t 
 *  @brief  This is the handle for an individual strand of LEDs 
 *          Contains all the info a healthy growing led strand needs 
 *          All functions require a pointer to the handle structure
 **/
typedef struct {

    uint8_t channel;            /** the RMT or SPI channel (depending on led type)  **/
    uint8_t num_leds;           /** number of leds in the strip                     **/

    ledtype_t led_type;         /** the type of leds in the strip **/
    void *strand_mem_start;     /** start of the strand memory  - currently just heap mem **/
    void *pixel_start;          /** start of the led pixel memory **/
    uint32_t write_length;      /** length of the full strand memory **/

    SemaphoreHandle_t sem;      /** Semaphore for strand memory **/
    TimerHandle_t timer;        /** timer used for led effect refresh **/

} ledstrip_t;


typedef ledstrip_t * LEDSTRIP_h;    /** ledstrip handle pointer **/

/** @} LedStrip_Structures */

/** @defgroup Driver_Structures
 *  @brief    Structures used internally 
 *              by the driver
 * @{
 */

/** @enum led_cmd_t 
 *  @brief specifies the command for 
 *          the driver task
 */
typedef enum {
    LS_CMD_OFF,
    LS_CMD_UPDATE_FRAME,
    LS_CMD_UPDATE_LEDS,
    LS_CMD_NEW_MODE,
    LS_CMD_MAX,
} led_cmd_t;


/** @struct ledstrip_cmd_t 
 *  @brief  used internally to send commands 
 *          to the driver task
 */
typedef struct apa102_cmd_t
{
    LEDSTRIP_h strip;
    led_cmd_t cmd;
} ledstrip_cmd_t;

/** @} Driver_Structures */


/** @ **/
const ledtype_t pixel_types[2];
const uint8_t led_type_n;



/******** Function Definitions *********/


/** \brief  ledstrip_driver_init()
 *          Starts the ledstrip driver task, queues, etc.
 *          Must be called before adding led strips
 *  \return ESP_OK or error
 **/
esp_err_t ledstrip_driver_init();


esp_err_t ledstrip_add(ledstrip_init_t *init);



#endif /* LEDSTRIP_DRIVER_H */
