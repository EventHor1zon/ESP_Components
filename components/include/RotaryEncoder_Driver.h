/****************************************
* \file     RotaryEncoder_Driver.h
* \brief    The header file for the rotary encoder driver
* \date     July2020
* \author   RJAM
****************************************/

#ifndef ROTARYENCODER_DRIVER_H
#define ROTARYENCODER_DRIVER_H

/********* Includes ********************/

#include "driver/gpio.h"

/********* Definitions *****************/

typedef union {
    uint16_t uValue; /** < rotary encoder value (unsigned version) **/
    int16_t Value;   /** < rotary encoder value (signed version ) **/
} rotaryCount;

typedef struct rotaryEncoder
{

    rotaryCount count;

    bool signedCounter;    /** < using signed or unsigned value **/
    bool buttonEnabled;    /** < Enable the button input       **/
    bool btnDebounceState; /** < button debounce state **/
    bool dirLast;          /** < last turn direction 0 - c-clkwise, 1 - clkwise **/
    bool btnState;         /** < current state of button **/

    gpio_num_t dataPinNum;  /** < data pin gpio  **/
    gpio_num_t clockPinNum; /** < clock pin gpio **/

    uint16_t stepSize;  /** < increment/decrement size **/
    uint32_t tLastStep; /** < time since the last rotary step **/
    uint32_t tLastBtn;  /** < time since last button push **/
} rotaryEncoder_t;

/********** Types **********************/

/******** Function Definitions *********/

#endif /* ROTARYENCODER_DRIVER_H */
