/****************************************
* \file     RotaryEncoder_Driver.h
* \brief    The header file for the rotary encoder driver
* \date     July2020
* \author   RJAM
****************************************/

#ifndef ROTARYENCODER_DRIVER_H
#define ROTARYENCODER_DRIVER_H

/********* Includes ********************/

#include "PushButton_Driver.h"

#include "driver/gpio.h"
#include "freertos/timers.h"
#include "freertos/task.h"

/********* Definitions *****************/

#define RE_NOTIFY_CW_STEP (1)
#define RE_NOTIFY_CC_STEP (1 << 1)
#define RE_NOTIFY_BTN_DWN (1 << 2)
#define RE_NOTIFY_BTN_UP (1 << 3)

typedef union {
    uint16_t uValue; /** < rotary encoder value (unsigned version) **/
    int16_t Value;   /** < rotary encoder value (signed version ) **/
} rotaryCount;

typedef struct rotaryEncoder
{

    rotaryCount count;

    bool signedCounter;  /** < using signed or unsigned value **/
    bool debounceEnable; /** < enable to button debounce **/
    bool debounceState;  /** < button debounce state **/
    bool dirLast;        /** < last turn direction 0 - c-clkwise, 1 - clkwise **/
    bool alertStep;      /** < send task notification if step changes **/

    gpio_num_t dataPinNum;  /** < data pin gpio  **/
    gpio_num_t clockPinNum; /** < clock pin gpio **/

    uint16_t counterMax; /** < max counter value **/
    uint16_t counterMin; /** < min counter value **/
    uint16_t stepSize;   /** < increment/decrement size **/

    uint32_t tLastStep; /** < time since the last rotary step **/

    TimerHandle_t debounceTimer; /** < Timer for debounce **/
    TaskHandle_t parentTask;     /** < Task to notify **/

} rotaryEncoder_t;

/********** Types **********************/

/******** Function Definitions *********/

esp_err_t rotaryEncoderInit(gpio_num_t dataPin, gpio_num_t clockPin, bool installISR, TaskHandle_t parentTask);

#endif /* ROTARYENCODER_DRIVER_H */
