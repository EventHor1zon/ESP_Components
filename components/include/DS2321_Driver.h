/****************************************
* \file     DS2321_Driver.h
* \brief    Header file
* \date     March 2021
* \author   RJAM
****************************************/

#ifndef DS2321_DRIVER_H
#define DS2321_DRIVER_H

/********* Includes ********************/

#include "esp_err.h"
#include "esp_log.h"
#include "esp_types.h"

/********* Definitions *****************/

/********** Types **********************/


typedef struct DS2321_Init
{
    /* data */
    uint8_t  i2c_bus;

} ds2321_init_t;



typedef struct DS2321_Driver
{
    /* data */
    uint8_t i2c_bus;

} ds2321_handle_t;


typedef ds2321_handle_t * DS2321_DEV;


/******** Function Definitions *********/

DS2321_DEV ds2321_init(ds2321_init_t *ini);


#endif /* DS2321_DRIVER_H */
