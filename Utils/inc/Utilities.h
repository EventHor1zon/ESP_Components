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
#include "math.h"
/********* Definitions *****************/

#define BITS_PER_BYTE (8u)

#define INCREMENT_TO_MAX(i, max) ((i == max) ? (max) : (i + 1))
#define DECREMENT_TO_MIN(i, min) ((i == min) ? (min) : (i - 1))
#define BYTE_SET_BITS(i, set) (i |= set)
#define BYTE_UNSET_BITS(i, unset) (i &= ~(unset))
#define MAX_VAL_FROM_BITS(bits) ((1 << (bits+1))-1)

/********** Types **********************/


/******** Function Definitions *********/

void showmem(uint8_t *ptr, uint16_t len);

void printByteBits(uint8_t data);

void printBytesOrderExplicit(uint8_t val);

uint8_t unset_bits(uint8_t byte, uint8_t unset);

uint8_t set_bits(uint8_t byte, uint8_t set);

/**
 * @brief replaceBits   - clears then sets the bits specified
 *                        correct shift is required in clear and
 *                        set mask
 * @param data - ptr to data byte to modify
 * @param clear_mask - clear these bits
 * @param set_mask - set these bits.
 */
void replaceBits(uint8_t *data, uint8_t clear_mask, uint8_t set_mask);


uint8_t largest_from_array(uint8_t *array, uint8_t len);

uint8_t index_of_largest(uint8_t *array, uint8_t len);


#endif /* UTILITIES_H */
