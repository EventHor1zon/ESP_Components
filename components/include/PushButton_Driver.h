/****************************************
* \file
* \brief
* \date
* \author
****************************************/

#ifndef HEADER_H
#define HEADER_H

/********* Includes ********************/

#include <stdlib.h>

#include "driver/gpio.h"

#include "freertos/task.h"
#include "freertos/timers.h"

/********* Definitions *****************/

#define NOTIFY_BTN_UP (1 << 2)
#define NOTIFY_BTN_DWN (1 << 3)

#define BTN_DEFAULT_DEBOUNCE_T (50)

/********** Types **********************/

typedef enum
{
    BTN_CONFIG_ACTIVELOW,
    BTN_CONFIG_ACTIVELOW_PULLUP,
    BTN_CONFIG_ACTIVEHIGH,
    BTN_CONFIG_ACTIVEHIGH_PULLDOWN,
} btn_config_t;

/*****************************************/ /*
 *  Struct buttonData_t 
 *  \brief Used to control the button driver component
 * 
 *******************************************/
typedef struct buttonData
{
    bool btnDebounceEnable; /** < bool btnDebounceEnable: enable to button debounce **/
    bool btnDebounceState;  /** < bool btnDebounceState: button debounce state **/
    bool btnState;          /** < bool btnState: current state of button (ie pin state) **/
    bool alertBtn;          /** < bool alertBtn: send task notification if button pressed - default on with parentTask **/
    bool halfBtnInterrupt;  /** < send the notify on each edge change, rather than whole button cycle */

    uint16_t btnCount;  /** < number of button pushes **/
    uint32_t tLastBtn;  /** < time since last button push **/
    uint32_t tBtnPress; /** < time the button is held down for **/
    uint16_t tDebounce; /** < length of DB timer cooldown in ms **/

    TimerHandle_t debounceTimer; /** < Timer for debounce **/
    TaskHandle_t parentTask;     /** < Task to notify **/
    gpio_num_t btnPin;           /** < gpio pin **/

} buttonData_t;

/******** Function Definitions *********/

/** \brief pushBtn_Init - initialises a pushbutton component driver
 * 
 *  \param btnPin   - The gpio number of the button connection
 *  
 *  \param btnConfig - one of btn_config_t type button setups
 * 
 *  \param parentTask - the task handle for a parent task to be notified
 * 
 *  \return ESP_OK or Error
 **/
esp_err_t pushBtn_Init(gpio_num_t btnPin, btn_config_t btnConfig, TaskHandle_t parentTask);

#endif /* HEADER_H */
