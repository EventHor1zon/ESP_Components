/****************************************
* \file     utilities.h
* \brief    useful macros & definitions
* \date     July 2020
* \author   RJAM
****************************************/

#ifndef UTILITIES_H
#define UTILITIES_H

/********* Includes ********************/
#include "esp_types.h"
/********* Definitions *****************/

#define INCREMENT_TO_MAX(i, max) ((i == max) ? (max) : (i + 1))
#define DECREMENT_TO_MIN(i, min) ((i == min) ? (min) : (i - 1))

/********** Types **********************/

/******** Function Definitions *********/

void showmem(uint8_t *ptr, uint16_t len);
void printByteBits(uint8_t data);
void printBytesOrderExplicit(uint8_t val);
uint8_t unset_bits(uint8_t byte, uint8_t unset);
uint8_t unset_bits(uint8_t byte, uint8_t unset);

#endif /* UTILITIES_H */
