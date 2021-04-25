/****************************************
* \file     SystemComponent.h
* \brief    Header file for the System Component
* \date     Dec 2020
* \author   RJAM
****************************************/

#ifndef SYSTEM_COMPONENT_H
#define SYSTEM_COMPONENT_H

/********* Includes ********************/

#include "esp_types.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#include "driver/gpio.h"

/********* Definitions *****************/


#define SYS_CONFIG_WDTT_TIMEOUT_S 5

typedef enum {
    SYS_NO_SLEEP,
    SYS_LIGHT_SLEEP,
    SYS_AUTOLIGHT_WOW,
    SYS_DEEP_SLEEP_WOI,
    SYS_DEEP_SLEEP_WOT,
} sleepstate_t;


typedef enum {
    WAKETYPE_NOT_SET,
    WAKETYPE_EXT0,
    WAKETYPE_EXT1,
    WAKETYPE_GPIO
} waketype_t;

typedef struct WakePin
{
    gpio_num_t pin;
    uint8_t level;
    waketype_t waketype; /** 0 - unregistered, 1, - ext0, 2 - ext1 **/
    gpio_pull_mode_t pull;
} wake_pin_t;



typedef struct SystemHandle
{
    sleepstate_t sleep_state;
    uint32_t uptime_counter;
    TimerHandle_t uptimer;
    wake_pin_t wakepins[10];
    uint8_t no_wakepins;
    char devicename[32];
    uint32_t device_type;

} system_handle_t;


/********** Types **********************/

/******** Function Definitions *********/

system_handle_t *system_init();

//*********** ISR **********************//

void uptime_timer_callback(void *timer);


//*********** Gets ********************//


/** TODO: Sort these **/
#ifdef CONFIG_USE_PERIPH_MANAGER
#include "CommandAPI.h"
#define sys_cmd_len 1
#define sys_act_len 1
peripheral_t sys_peripheral_template;
#endif

/********** Types **********************/

/******** Function Definitions *********/

void dump_system_info(void);


/** \brief sys_get_memory_total - get the total memory available
 *  \param val - pointer to a variable store
 *  \return ESP_OK or error
 **/
esp_err_t sys_get_total_memory(system_handle_t *handle, uint32_t *val);

esp_err_t sys_sleep_until_woken(system_handle_t *handle);


esp_err_t system_get_ip_addr(system_handle_t *handle, char *buffer);

esp_err_t system_get_mac_addr(system_handle_t *handle, char *buffer);

esp_err_t system_get_bt_mac_addr(system_handle_t *handle, char *buffer);

esp_err_t system_get_uptime(system_handle_t *handle, uint32_t *uptime);

esp_err_t system_get_datetime_string(system_handle_t *handle, char *datetime);

esp_err_t system_get_autosleep_wow(system_handle_t *handle, uint8_t *val);


//********** Sets ******************//

esp_err_t system_set_time_hour(system_handle_t *handle, uint8_t *hour);

esp_err_t system_set_time_minute(system_handle_t *handle, uint8_t *hour);

esp_err_t system_sleep_x_mins(system_handle_t *handle, uint16_t *mins);

esp_err_t system_set_autosleep_wow(system_handle_t *handle, uint8_t *val);


//*********** Actions *************//
esp_err_t system_enable_ext0_io(system_handle_t *handle);

esp_err_t system_enable_ext1_io(system_handle_t *handle);

void system_deep_sleep(system_handle_t *handle);



esp_err_t system_register_wakeup_pin(system_handle_t *handle, wake_pin_t *pin_conf);

#endif /* SYSTEM_COMPONENT_H */
