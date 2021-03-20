/****************************************
* \file
* \brief
* \date
* \author
****************************************/

#ifndef APDS9960_DRIVER_H
#define APDS9960_DRIVER_H

/********* Includes ********************/

#include "esp_types.h"
#include "esp_err.h"
#include "driver/gpio.h"

/********* Definitions *****************/



/********** Types **********************/


typedef struct APDS9960_Init
{
    /* data */
    uint8_t i2c_bus;
    gpio_num_t isr_pin;
} apds_init_t;


typedef struct APDS9960_Driver
{
    /* data */
} apds_driver_t;



typedef apds_driver_t * APDS_DEV; 

/******** Function Definitions *********/


APDS_DEV apds_init(apds_init_t *ini);

#endif /* APDS9960_DRIVER_H */
