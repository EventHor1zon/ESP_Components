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

/********* Definitions *****************/


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
    step_size_t step_size;

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

    uint32_t step_pulse_len;
    uint32_t steps_queued;
    uint16_t step_wait;


    bool is_sleeping;
    bool is_enabled;

    bool _en;
    bool _sleep;
    bool _ms;

    step_size_t step_size;
    TimerHandle_t timer;
    TaskHandle_t t_handle;

} a4988_handle_t;


typedef a4988_handle_t * A4988_DEV;


/******** Function Definitions *********/

A4988_DEV a4988_init(a4988_init_t *init);



esp_err_t a4988_step(A4988_DEV dev);

esp_err_t a5988_reset(A4988_DEV dev);

esp_err_t a4988_clear_step_queue(A4988_DEV dev);

esp_err_t a4988_queue_steps(A4988_DEV dev, uint16_t *steps);

esp_err_t a4988_get_stepsize(A4988_DEV dev, uint8_t *sz);

esp_err_t a4988_set_stepsize(A4988_DEV dev, uint8_t *sz);

esp_err_t a4988_get_sleepstate(A4988_DEV dev, bool *slp);

esp_err_t a4988_set_sleepstate(A4988_DEV dev, bool *slp);

esp_err_t a4988_get_enable(A4988_DEV dev, bool *en);

esp_err_t a4988_set_enable(A4988_DEV dev, bool *en);

esp_err_t a4988_get_step_delay(A4988_DEV dev, uint16_t *sz);

esp_err_t a4988_set_step_delay(A4988_DEV dev, uint16_t *sz);



#endif /* A4988_DRIVER_H */
