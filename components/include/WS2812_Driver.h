

/****************************************
* \file     FREESP_WS2812.h
* \brief    Header file for the FREESP_WS2812 component
* \date     28 June 2020 
* \author   ABDS
****************************************/

#ifndef FREESP_WS2812_H
#define FREESP_WS2812_H

/********* Includes ********************/

#include "driver/rmt.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define WS2812_DRIVER_USE_MANAGER

#ifdef ESP_HOME_API_ENABLE
#include "FREESP_PeripheralManager.h"
#endif

/********* Definitions *****************/
#define WS2812_DRIVER_USE_MANAGER /** < Define for using the manager task */
//#define ESP_HOME_API_ENABLE 1     /** < enable interaction with ESPhome API */

#define WS2812_RMT_CLK_DIVIDER 4    /** < divisor for the APB clock */
#define WS2812_RMT_DURATION_NS 12.5 /** < clock duration of rmt clk */
#define WS2812_CFG_REFRESH_RATE 100 /** < refresh rate (delay for task) */
#define WS2812_MAX_STRANDS 8        /** < Max number of strands (max rmt channels)*/
#define WS2812_MAX_STRAND_LEDS 200  /** < max number of leds per strand - arbitrary for now. time later */
#define WS2812_MAX_TOTAL_LEDS 1000  /** < max number of leds total - arbitrary */

#define WS2812_SEMAPHORE_TIMEOUT 100 /** <semaphore wait timeout in ms*/

#define DRIVER_DEVELOPMENT_MODE

#ifdef ESP_HOME_API_ENABLE
#define WS2812_CMD_INDEX_ENABLE 0
#define WS2812_CMD_INDEX_COLOUR 1
#define WS2812_CMD_INDEX_EFFECT 2
#define WS2812_CMD_INDEX_BRIGHT 3
#endif

/********** Types **********************/

typedef enum ledEffect
{
    LED_EFFECT_OFF,
    LED_EFFECT_SINGLE_COLOUR,
    LED_EFFECT_SLOW_FADE,
    LED_EFFECT_RAINBOW,
    LED_EFFECT_NIGHTRIDER,
    LED_EFFECT_COLOUR_FADE,

    LED_EFFECT_NO_EFFECT = 0xFF
} ledEffect_t;

/** WS2812b bit timings. 
 *  Hardware specific, the initalisation of values below. 
 **/
typedef struct
{
    uint32_t T0H;
    uint32_t T1H;
    uint32_t T0L;
    uint32_t T1L;
    uint32_t TRS;
} ws2812b_timing_t;

/**
 *  Peripheral init data struct
*/
typedef struct ws2812_initData
{
    uint8_t numStrands;  /** < number of strands to be init */
    uint16_t *numLeds;   /** < pointer to an array of number of leds on each strand */
    gpio_num_t *dataPin; /** < pointer to an array of gpio data pin for each strand */
} ws2812_initData_t;

/**
 *  Led effect struct
 *  Used for tracking persistent LED effect variables 
 *  Try to build all effects to use the same type of data struct 
 *  TODO: better variable names, use union or bitfield?
*/

typedef struct ws2812_ledEffectData
{
    ledEffect_t effect; /** < the current effect */
    uint8_t variableA;  /** < variableA - step counter/led Index */
    uint8_t variableB;  /** < variableB - speed, fade, totalSteps, etc */
    uint8_t variableC;  /** < variableC - delta, booleans, etc */
    uint32_t colour;    /** < colour - colour */
} ledEffectData_t;

/** Strand Data. 
 *  Data for each individual strand of leds 
 **/
typedef struct strandData
{
    uint8_t strandIndex;          /** <index of the current strand  */
    uint8_t updateLeds;           /** <update flag - led mem has changed */
    uint16_t numLeds;             /** <num leds in strand           */
    uint16_t strandMemLength;     /** <length of memory    */
    uint8_t *strandMem;           /** <pointer to LED data          */
    ledEffectData_t *fxData;      /** < pointer to led effect data */
    SemaphoreHandle_t *memSemphr; /** <sempahore for led data access*/
    rmt_channel_t dataChannel;    /** <rmt channel driving leds     */
} StrandData_t;

/** ws2812 Driver Control structure 
 *  Data storage for the driver. 
 **/
typedef struct ws2812Control
{
    uint8_t numStrands;            /** <number of led strands (max 8) */
    uint16_t frameRate;            /** <refresh_rate of LED strands - common to all strands as managed by a single task */
    StrandData_t *allStrands;      /** <an array containing all connected strands */
    TaskHandle_t driverTaskHandle; /** <handle for the driver main task */
} ws2812Ctrl_t;

/********** Constants ******************/

const size_t WS2812_BYTES_PER_PIXEL;  /** <number of bytes per LED */
const char *WS2812_TAG;               /** <esp-log tag */
const ws2812b_timing_t ws2812Timings; /** <ws2812 bit timings  */
const size_t pixelSize;

const uint8_t ws2812_black_pixel[3]; /** < black (off) pixel */
const uint8_t ws2812_white_pixel[3]; /** < white pixel */
const uint8_t ws2812_blue_pixel[3];  /** < blue pixel */
const uint8_t ws2812_red_pixel[3];   /** < red pixel */
const uint8_t ws2812_green_pixel[3]; /** < green pixel */

static uint8_t testFrame[3][3] = {
    /** < some standard frames */
    {0xff, 0x00, 0x00}, /** <blue  */
    {0x00, 0xff, 0x00}, /** <green */
    {0x00, 0x00, 0xff}  /** <red   */
};

/******** Function Definitions *********/

/**
 * \brief ws2812_init() - initialise the ws2812 driver
 * 
 * description:  first checks the parameters for errors, then initialises the strand data for each
 *               LED strand. Then creates a semaphore for each led strand. Then sets up heap memory for 
 *               LED strand data. 
 * 
 * \param numStrands - number of strands being initialised  
 * \param numLeds    - pointer to array/variable of uint16_t number(s) of leds per strip
 * \param dataPins   - pointer to array/variable of GPIO(s) data pin(s)
 *
 * \warning            The numLeds and dataPins must point to an array of at least
 *                      numStrands in length. If numStrands = 1, a pointer to a single variable
 *                     works. Although checks are made, reading from unintended memory is never 
 *                      desireable.
 * \return ESP_OK or Error Code
 **/
esp_err_t WS2812_init(uint8_t numStrands, uint16_t *numLeds, gpio_num_t *dataPin);

/**
 * \brief ws2812_deinit() - deinint the driver
 *  
 *  description - goes through the existing strands, aquires their semaphores and frees image memory
 *                then frees semaphore and other data structures. Finally, removes the control task
 *  \return  ESP_OK or error code
 **/
esp_err_t WS2812_deinit(void);

/**
 * \brief - control task for the driver 
*/

void showmem(uint8_t *memptr, int len);
#endif /* FREESP_WS2812_H */