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
#include <string.h>
#include "esp_types.h"
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

void printBytesOrderExplicit(uint8_t val) {
    uint8_t set = 0;
    printf("MSB ------> LSB\n");
    printf("7 6 5 4 3 2 1 0\n");
    for(int bit = 0; bit < 8; bit++) {
        set = (val & (1 << (7 - bit))) > 0 ? 1 : 0;
        printf("%u ", set);
    }
    printf("\n");
}
