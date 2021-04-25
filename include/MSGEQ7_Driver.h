/****************************************
* \file
* \brief
* \date
* \author
****************************************/

#ifndef MSGEQ7_DRIVER_H
#define MSGEQ7_DRIVER_H

/********* Includes ********************/

#include "esp_types.h"
#include "driver/adc_common.h"
#include "driver/gpio.h"

/********* Definitions *****************/

/********** Types **********************/

typedef struct msg_init 
{
    /* data */
    gpio_num_t data;
    gpio_num_t rst;
    gpio_num_t strobe;
} msg_init_t;


typedef struct msg_handle
{
    /* data */
    gpio_num_t data_pin;
    gpio_num_t rst_pin;
    gpio_num_t strobe_pin;
    adc_channel_t adc_channel;
    adc_bits_width_t adc_bits;


} msg_handle_t;


/******** Function Definitions *********/


msg_handle_t *msg_init(msg_init_t *init); 

#endif /* MSGEQ7_DRIVER_H */
