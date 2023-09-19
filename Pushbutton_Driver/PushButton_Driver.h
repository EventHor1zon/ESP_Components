/****************************************
* \file
* \brief
* \date
* \author
****************************************/

#ifndef PUSHBUTTON_DRIVER_H
#define PUSHBUTTON_DRIVER_H

/********* Includes ********************/

#include <stdlib.h>
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "esp_event_base.h"

#include "freertos/task.h"
#include "freertos/timers.h"

/********* Definitions *****************/

#define NOTIFY_BTN_UP (1 << 2)
#define NOTIFY_BTN_DWN (1 << 3)

#define BTN_DEFAULT_DEBOUNCE_T 150

#ifdef CONFIG_USE_PERIPH_MANAGER
#include "CommandAPI.h"
#endif  /** CONFIG_USE_PERIPH_MANAGER **/


#ifdef CONFIG_USE_EVENTS
#define BTN_ID_BASE         0x45

#define BTN_EVENT_BTNDOWN   (BTN_ID_BASE << 16 | NOTIFY_BTN_DWN)
#define BTN_EVENT_BTNUP     (BTN_ID_BASE << 16 | NOTIFY_BTN_UP)

#endif  /** CONFIG_USE_EVENTS **/

/********** Types **********************/

typedef enum
{
    BTN_CONFIG_ACTIVELOW,
    BTN_CONFIG_ACTIVELOW_PULLUP,
    BTN_CONFIG_ACTIVEHIGH,
    BTN_CONFIG_ACTIVEHIGH_PULLDOWN,
} btn_config_t;

typedef struct 
{
    gpio_num_t btn_pin;
    btn_config_t btn_config;
#ifdef CONFIG_USE_EVENTS
    esp_event_loop_handle_t event_loop;
#endif
} pushbtn_init_t;


/*****************************************/ /*
 *  Struct buttonData_t 
 *  \brief Used to control the button driver component
 * 
 *******************************************/
typedef struct buttonData
{
    bool btn_debounce_en; /** < bool btn_debounce_en: enable to button debounce **/
    bool btnDebounceState;  /** < bool btnDebounceState: button debounce state **/
    bool btn_state;          /** < bool btn_state: current state of button (ie pin state) **/
    bool alertBtn;          /** < bool alertBtn: send task notification if button pressed - default on with parentTask **/
    bool halfBtnInterrupt;  /** < send the notify on each edge change, rather than whole button cycle */

    btn_config_t btn_setting;
    uint16_t btnCount;  /** < number of button pushes **/
    uint32_t tLastBtn;  /** < time since last button push **/
    uint32_t tBtnPress; /** < time the button is held down for **/
    uint16_t debounce_time; /** < length of DB timer cooldown in ms **/

    esp_event_loop_handle_t loop;
    TimerHandle_t debounceTimer; /** < Timer for debounce **/
    TaskHandle_t parentTask;     /** < Task to notify **/
    gpio_num_t btn_pin;           /** < gpio pin **/

} buttonData_t;


typedef buttonData_t * BTN_DEV; 

/******** Function Definitions *********/

#ifdef CONFIG_DRIVERS_USE_HEAP
BTN_DEV pushbutton_init(pushbtn_init_t *init);
#else
/**
 * @brief  initialise the pushbutton driver
 * 
 * @param btn 
 * @param init 
 * @return ** BTN_DEV 
 */
BTN_DEV pushbutton_init(BTN_DEV btn, pushbtn_init_t *init);
#endif /** CONFIG_DRIVERS_USE_HEAP **/


#endif /* PUSHBUTTON_DRIVER_H */
