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

#include "esp_event_base.h"

/********* Definitions *****************/

#define RE_NOTIFY_CW_STEP (1)
#define RE_NOTIFY_CC_STEP (1 << 1)
#define RE_NOTIFY_BTN_DWN (1 << 2)
#define RE_NOTIFY_BTN_UP (1 << 3)
#define RE_NOTIFY_DEBOUNCE_EXPIRED (1 << 4)
#define RE_DEBOUNCE_TIME_MS 20

#ifdef CONFIG_USE_EVENTS

#define RE_EVENT_ID_BASE    0x46
#define RE_EVENT_CW_STEP    (RE_EVENT_ID_BASE << 16 | RE_NOTIFY_CW_STEP)
#define RE_EVENT_CC_STEP    (RE_EVENT_ID_BASE << 16 | RE_NOTIFY_CC_STEP)
#define RE_EVENT_BTN_DWN    (RE_EVENT_ID_BASE << 16 | RE_NOTIFY_BTN_DWN)
#define RE_EVENT_BTN_UP    (RE_EVENT_ID_BASE << 16 | RE_NOTIFY_BTN_UP)

#endif

typedef union {
    uint16_t uValue; /** < rotary encoder value (unsigned version) **/
    int16_t Value;   /** < rotary encoder value (signed version ) **/
} rotaryCount;

typedef struct rotaryEncoder
{

    rotaryCount count;              /** < rotary encoder counter **/
 
    bool signedCounter;             /** < using signed or unsigned value **/
    bool debounceEnable;            /** < enable to button debounce **/
    bool debounceState;             /** < button debounce state **/
    bool dirLast;                   /** < last turn direction 0 - c-clkwise, 1 - clkwise **/
    bool alertStep;                 /** < send task notification if step changes **/

    bool use_events;                /** < Send events on Button & RE events **/
    esp_event_loop_handle_t loop;   /** < event loop to send events to **/

    gpio_num_t dataPinNum;          /** < data pin gpio  **/
    gpio_num_t clockPinNum;         /** < clock pin gpio **/

    uint16_t counterMax;            /** < max counter value **/
    uint16_t counterMin;            /** < min counter value **/
    uint16_t stepSize;              /** < increment/decrement size **/

    uint32_t tLastStep;             /** < time since the last rotary step **/

    TimerHandle_t debounceTimer;    /** < Timer for debounce **/
    TaskHandle_t parentTask;        /** < Task to notify **/

} rotaryEncoder_t;

/********** Types **********************/

/******** Function Definitions *********/

esp_err_t rotaryEncoderInit(gpio_num_t dataPin, gpio_num_t clockPin, bool installISR);

#endif /* ROTARYENCODER_DRIVER_H */
