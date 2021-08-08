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
#include "driver/adc.h"
#include "driver/adc_common.h"
#include "soc/adc_channel.h"

/****** Function Prototypes ***********/

/************ ISR *********************/

/****** Private Data ******************/

/****** Private Functions *************/

/****** Global Data *******************/

/****** Global Functions *************/


/** Prints some memory - many useful uses! **/
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


/** Prints a byte's individual bits, good for
 * reading register contents & stuff 
 **/
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


int8_t adc_channel_from_gpio(uint32_t gpio_n) {
        
        int8_t channel = -1;
        switch(gpio_n) {
        case 32:
                channel = ADC1_GPIO32_CHANNEL;
                break;
        case 33:
                channel = ADC1_GPIO33_CHANNEL;
                break;
        case 34:
                channel = ADC1_GPIO34_CHANNEL;
                break;
        case 35:
                channel = ADC1_GPIO35_CHANNEL;
                break;
        case 36:
                channel = ADC1_GPIO36_CHANNEL;
                break;
        case 37:
                channel = ADC1_GPIO37_CHANNEL;
                break;
        case 38:
                channel = ADC1_GPIO38_CHANNEL;
                break;
        case 39:
                channel = ADC1_GPIO39_CHANNEL;
                break;
        default:
                break;
        }

    return channel;
}