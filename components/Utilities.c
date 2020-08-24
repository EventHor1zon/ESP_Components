/***************************************
* \file .c
* \brief
*
* \date
* \author
****************************************/

/********* Includes *******************/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "esp_err.h"
#include "esp_log.h"

/****** Function Prototypes ***********/

/************ ISR *********************/

/****** Private Data ******************/

/****** Private Functions *************/

/****** Global Data *******************/

/****** Global Functions *************/
void showmem(uint8_t *memptr, int len)
{
    for (int i = 0; i < len; i++)
    {
        if (i % 8 == 0)
        {
            printf("[%p] %02x", memptr, *memptr);
        }
        else if (i % 8 == 7)
        {
            printf(" %02x\n", *memptr);
        }
        else
        {
            printf(" %02x", *memptr);
        }
        memptr++;
    }
}