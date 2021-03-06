/****************************************
* \file     utilities.h
* \brief    useful macros & definitions
* \date     July 2020
* \author   RJAM
****************************************/

#ifndef UTILITIES_H
#define UTILITIES_H

/********* Includes ********************/
#include <stdint.h>
/********* Definitions *****************/

#define INCREMENT_TO_MAX(i, max) ((i == max) ? (max) : (i + 1))
#define DECREMENT_TO_MIN(i, min) ((i == min) ? (min) : (i - 1))

/********** Types **********************/

/******** Function Definitions *********/

void showmem(uint8_t *ptr, uint16_t len);
void printByteBits(uint8_t data);
#endif /* UTILITIES_H */
