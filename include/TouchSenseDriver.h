/****************************************
* \file
* \brief
* \date
* \author
****************************************/

#ifndef TOUCHSENSE_DRIVER_H
#define TOUCHSENSE_DRIVER_H

#define TOUCHSENSE_MAX_PINS 6

/********* Includes ********************/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"

/********* Definitions *****************/

/********** Types **********************/   



typedef struct TouchSenseInit
{
    /* data */
    gpio_num_t touch_pins[TOUCHSENSE_MAX_PINS];
    uint8_t num_pins;
    void *event_loop;

} touchsense_init_t;


typedef struct TouchSenseDriver
{
    /* data */

    gpio_num_t touch_pins[TOUCHSENSE_MAX_PINS];
    uint8_t num_pins;
    TaskHandle_t t_handle;
    TimerHandle_t timer;
} touchsense_data_t;


typedef touchsense_data_t * TSENSE_HANDLE;


/******** Function Definitions *********/


TSENSE_HANDLE touchsense_init(touchsense_init_t *ts_init);




#endif /* TOUCHSENSE_DRIVER_H */
