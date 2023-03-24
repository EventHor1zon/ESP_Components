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
#include "./LedEffects.h"

/********* Includes ********************/

/********* Definitions *****************/


/** Driver settings **/
#define LEDSTRIP_CONFIG_QUEUE_LEN   4       /** length of the pending command queue **/
#define LEDSTRIP_CONFIG_TASK_PRIO   5       /** task priority        **/
#define LEDSTRIP_CONFIG_TASK_STACK  5012    /** task stack size      **/
#define LEDSTRIP_CONFIG_MAX_STRIPS  8       /** max supported strips **/
#define LEDSTRIP_CONFIG_MAX_LEDS    120     /** max leds per strip   **/

typedef esp_err_t (*write_fn)(void *, void *);
typedef esp_err_t (*init_fn)(void *);
typedef void (*effect_fn)(void *);
/********** Types **********************/

/** @defgroup   LedStrip_Structures 
 *  @brief      Core structures for initialising and using the 
 *              LedStrip driver
 *  @{ 
 */

/** @enum   led_type_e
 *  @brief  Selector for initialisation 
 *          Should correspond to the index of the led type in
 *          the ledtype_t array.
 */
typedef enum {
    LED_TYPE_WS2812,
    LED_TYPE_APA102,
    /** @todo Add more led types here */
    LED_TYPE_INVALID,
} led_type_e;

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
    uint8_t start_byte;         /** value of start frame **/
    uint8_t start_len;          /** number of start bytes to write **/
    uint8_t end_byte;           /** value of the end frame **/
    uint8_t end_len;            /** number of end bytes to write **/
    write_fn write;             /** function to write frame data **/
    init_fn init;               /** function to initialise the peripheral **/
} ledtype_t;


/** @struct ledstrip_init_t 
 *  @brief  the neccessary info to initialise the led
 *          strip
 **/
typedef struct {
    uint8_t channel;            /** the RMT or SPI channel (depending on led type)  **/
    uint8_t num_leds;           /** number of leds in the strip                     **/
    led_type_e led_type;        /** the type of leds in the strip **/
} ledstrip_init_t;


/** @struct ledstrip_t 
 *  @brief  This is the handle for an individual strand of LEDs 
 *          Contains all the info a healthy growing led strand needs 
 *          All functions require a pointer to the handle structure
 **/
typedef struct {

    uint8_t channel;            /** the RMT or SPI channel (depending on led type)  **/
    uint8_t num_leds;           /** number of leds in the strip                     **/

    ledtype_t *led_type;         /** pointer to the ledtype_t descriptor **/
    void *strand_mem_start;     /** start of the strand memory  - currently just heap mem **/
    void *pixel_start;          /** start of the led pixel memory **/
    uint32_t write_length;      /** length of the full strand memory **/

    fxdata_t fx;                /** led effect data struct **/

    void *interface_handle;     /** handle for SPI/RMT/other interface **/

    SemaphoreHandle_t sem;      /** Semaphore for strand memory **/
    TimerHandle_t timer;        /** timer used for led effect refresh **/

} ledstrip_t;

/** @name LEDSTRIP_h
 *  @brief type name for pointer to handle struct,
 *          expected as argument for all object functions
 * **/
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


/** @name   led_types
 *  @brief  array holding the configuration info for 
 *          supported led types
 */
const ledtype_t led_types[2];
const uint8_t led_type_n;



/******** Function Definitions *********/


/** \brief  ledstrip_driver_init()
 *          Starts the ledstrip driver task, queues, etc.
 *          Must be called before adding led strips
 *  \return ESP_OK or error
 **/
esp_err_t ledstrip_driver_init();


esp_err_t ledstrip_add_strip(LEDSTRIP_h strip, ledstrip_init_t *init);




#endif /* LEDSTRIP_DRIVER_H */
