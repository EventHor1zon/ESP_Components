/***************************************
* \file     Utilities.c
* \brief    some useful functions
*
* \date     Aug 2020
* \author   RJAM
****************************************/

/********* Includes *******************/
#include <stdint.h>

#include "esp_log.h"
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

void printByteBits(uint8_t num)
{
    printf("[");
    for (int bit = 0; bit < 8; bit++)
    {
        printf("%d", num & 0x01);
        num = num >> 1;
    }
    printf("]");
}
}
