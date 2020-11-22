/****************************************
* \file     SystemInterface.h
* \brief    Header file for system interface component  
* \date     Oct 2020
* \author   RJAM
****************************************/

#ifndef SYSTEM_INTERFACE_H
#define SYSTEM_INTERFACE_H

/********* Includes ********************/

#include "esp_err.h"

/********* Definitions *****************/

/********** Types **********************/

/******** Function Definitions *********/

void dump_system_info(void);

esp_err_t sys_get_total_memory(uint16_t *val);

/** \brief sys_get_wifi_mac - get the mac address
 *  \param val - pointer to an 8-bit int buffer, of LEN 6!!
 *  \return ESP_OK or error
 **/
esp_err_t sys_get_wifi_mac(uint8_t *val);

/** \brief sys_get_bt_mac - get the mac address of the bluetooth interface
 *  \param val - pointer to an 8-bit int buffer, of LEN 6!!
 *  \return ESP_OK or error
 **/
esp_err_t sys_get_bt_mac(uint8_t *val);

/** \brief sys_get_memory_total - get the total memory available
 *  \param val - pointer to a variable store
 *  \return ESP_OK or error
 **/
esp_err_t sys_get_mem_total(uint16_t *val);

/** \brief sys_sleep_for_time - sleep for given seconds
 *  \param val - pointer to a variable store
 *  \return ESP_OK or error
 **/
esp_err_t sys_sleep_for_time(uint32_t *val);

/** \brief sys_sleep_until_woken(void) - sleep until wake signal sent
 *  \return None
 **/
esp_err_t sys_sleep_until_woken(void);

#endif /* SYSTEM_INTERFACE_H */
