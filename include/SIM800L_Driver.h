/****************************************
* \file     Sim800L_Driver.h
* \brief    header file for Sim800L Driver
* \date     Aug 2021
* \author   RJAM
****************************************/

#ifndef SIM800L_DRIVER_H
#define SIM800L_DRIVER_H

/********* Includes ********************/

#include "driver/gpio.h"
#include "driver/uart.h"


/********* Definitions *****************/

/********** Types **********************/

typedef struct sim800_init
{
    /* data */
    uart_port_t port;
    gpio_num_t tx;
    gpio_num_t rx;  
} sim800_init_t;


typedef struct SIM800L_Driver
{
    /* data */
    gpio_num_t tx;
    gpio_num_t rx;
    uart_port_t port;

} simdriver_t ;


typedef simdriver_t * SIM800L_t;

/******** Function Definitions *********/

#endif /* SIM800L_DRIVER_H */
