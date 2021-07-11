/****************************************
* \file     A4988_Driver.h
* \brief    Header file
* \date     May 22
* \author   RJAM
****************************************/

#ifndef A4988_DRIVER_H
#define A4988_DRIVER_H

/********* Includes ********************/


#include "esp_err.h"
#include "esp_types.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "sdkconfig.h"

/********* Definitions *****************/

#ifdef CONFIG_USE_PERIPH_MANAGER

#include "CommandAPI.h"
#define a4988_param_len 9

const parameter_t a4988_param_map[a4988_param_len];
const peripheral_t a4988_periph_template;

#endif


#define A4988_DEFAULT_STEP_PULSE_LEN 1
#define A4988_DEFAULT_STEP_DELAY 100
/********** Types **********************/

typedef enum {
    FULL_STEP_T,
    HALF_STEP_T,
    QURT_STEP_T,
    EGTH_STEP_T,
    SXTN_STEP_T = 7,

} step_size_t;


typedef struct A4988_Init
{
    /* data */
    
    gpio_num_t sleep;       /**< sleep pin - optional **/
    gpio_num_t enable;      /**< enable pin - optional **/
    gpio_num_t step;        /**< step pin - must be set **/
    gpio_num_t rst;         /**< rst pin  - must be set **/
    gpio_num_t dir;         /**< dir pin    must be set **/
    gpio_num_t ms1;         /**< ms1 pin  - optional **/
    gpio_num_t ms2;         /**< ms2 pin  - optional **/
    gpio_num_t ms3;         /**< ms3 pin  - optional **/
    step_size_t step_size;  /**< step size - if driver controls 
                                ms pins, leave at zero, else describes 
                                the ms pin states
                            **/
} a4988_init_t;

typedef struct A4988_Driver
{
    /* data */
    gpio_num_t sleep;       /**< sleep pin - optional **/
    gpio_num_t enable;      /**< enable pin - optional **/
    gpio_num_t step;        /**< step pin - must be set **/
    gpio_num_t rst;         /**< rst pin  - must be set **/
    gpio_num_t dir;         /**< dir pin    must be set **/
    gpio_num_t ms1;         /**< ms1 pin  - optional **/
    gpio_num_t ms2;         /**< ms2 pin  - optional **/
    gpio_num_t ms3;         /**< ms3 pin  - optional **/

    uint32_t step_pulse_len;    /**< step pulse hold time **/
    uint32_t steps_queued;      /**< number of steps queued  **/
    uint16_t step_wait;         /**< delay between executing queued steps **/


    bool is_sleeping;       /**< is device sleeping **/
    bool is_enabled;        /**< is device enabled **/
    bool direction;         /**< current direction 
                                (depends on wiring, dir is "true/false" for
                                software purposes ) 
                            **/
    bool _en;               /**< driver manages enabled pin **/
    bool _sleep;            /**< driver manages sleep pin **/
    bool _ms;               /**< driver manages 3 microstep select pins **/

    step_size_t step_size;  /**< step size setting **/
    TimerHandle_t timer;    /**< timer handle **/
    TaskHandle_t t_handle;  /**< driver task handle **/

} a4988_handle_t;


typedef a4988_handle_t * A4988_DEV;


/******** Function Definitions *********/

A4988_DEV a4988_init(a4988_init_t *init);



esp_err_t a4988_step(A4988_DEV dev);

esp_err_t a4988_reset(A4988_DEV dev);

esp_err_t a4988_clear_step_queue(A4988_DEV dev);

esp_err_t a4988_get_queued_steps(A4988_DEV dev, uint16_t *steps);

esp_err_t a4988_set_queued_steps(A4988_DEV dev, uint16_t *steps);

esp_err_t a4988_get_stepsize(A4988_DEV dev, uint8_t *sz);

esp_err_t a4988_set_stepsize(A4988_DEV dev, uint8_t *sz);

esp_err_t a4988_get_sleepstate(A4988_DEV dev, bool *slp);

esp_err_t a4988_set_sleepstate(A4988_DEV dev, bool *slp);

esp_err_t a4988_get_enable(A4988_DEV dev, bool *en);

esp_err_t a4988_set_enable(A4988_DEV dev, bool *en);

esp_err_t a4988_get_step_delay(A4988_DEV dev, uint16_t *sz);

esp_err_t a4988_set_step_delay(A4988_DEV dev, uint16_t *sz);

esp_err_t a4988_get_direction(A4988_DEV dev, bool *dir);

esp_err_t a4988_set_direction(A4988_DEV dev, bool *dir);

#endif /* A4988_DRIVER_H */
