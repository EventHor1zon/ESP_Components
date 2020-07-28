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

#define RE_NOTIFY_CW_STEP (1)
#define RE_NOTIFY_CC_STEP (1 << 1)
#define RE_NOTIFY_BTN_DWN (1 << 2)
#define RE_NOTIFY_BTN_UP (1 << 3)

#define INCREMENT_TO_MAX(i, max) ((i == max) ? (max) : (i + 1))
#define DECREMENT_TO_MIN(i, min) ((i == min) ? (min) : (i - 1))

typedef union {
    uint16_t uValue; /** < rotary encoder value (unsigned version) **/
    int16_t Value;   /** < rotary encoder value (signed version ) **/
} rotaryCount;

typedef struct buttonDebouce
{
    uint32_t tDebounce;
    uint32_t tLastPress;
    bool debounceActive;
} buttonDebouce_t;

typedef struct rotaryEncoder
{

    rotaryCount count;

    bool signedCounter;     /** < using signed or unsigned value **/
    bool buttonEnabled;     /** < Enable the button input       **/
    bool btnDebounceEnable; /** < enable to button debounce **/
    bool btnDebounceState;  /** < button debounce state **/
    bool dirLast;           /** < last turn direction 0 - c-clkwise, 1 - clkwise **/
    bool btnState;          /** < current state of button **/
    bool alertBtn;          /** < send task notification if button pressed **/
    bool alertStep;         /** < send task notification if step changes **/
    bool halfBtnInterrupt;  /** < send the notify on btn press, rather than release */

    gpio_num_t dataPinNum;  /** < data pin gpio  **/
    gpio_num_t clockPinNum; /** < clock pin gpio **/
    gpio_num_t buttonPin;   /** < button pin gpio **/

    uint16_t counterMax; /** < max counter value **/
    uint16_t counterMin; /** < min counter value **/
    uint16_t stepSize;   /** < increment/decrement size **/
    uint16_t btnCount;   /** < number of button pushes **/

    uint32_t tLastStep; /** < time since the last rotary step **/
    uint32_t tLastBtn;  /** < time since last button push **/
    uint32_t tBtnPress; /** < time the button is held down for **/

    TimerHandle_t debounceTimer; /** < Timer for debounce **/
    TaskHandle_t parentTask;     /** < Task to notify **/

} rotaryEncoder_t;

/********** Types **********************/

/******** Function Definitions *********/

#endif /* ROTARYENCODER_DRIVER_H */
