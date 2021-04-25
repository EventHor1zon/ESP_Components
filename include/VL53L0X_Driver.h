/****************************************
* \file
* \brief
* \date
* \author
****************************************/

#ifndef VL53L0X_DRIVER_H
#define VL53L0X_DRIVER_H

#include "esp_err.h"

#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"


/********* Includes ********************/

/********* Definitions *****************/

#ifdef CONFIG_USE_PERIPH_MANAGER

#include "CommandAPI.h"
#define vl53_param_leng 5
const parameter_t vl53_param_map[vl53_param_len];
#endif

/********** Types **********************/

/******** Function Definitions *********/


/** 
 * \brief : Initialise the VL53 Driver
 * \return Device handle or null
 **/
VL53L0X_DEV vl53_init(void);


/** 
 * \brief Get the device mode
 * \param Dev - device handle
 * \param val - pointer to value storage
 * \return esp_ok or error
 **/
esp_err_t vl53_getDeviceMode(VL53L0X_DEV Dev, uint8_t *val);

/** 
 * \brief Tell device to update
 * \param Dev - device handle
 * \return esp_ok or error
 **/
esp_err_t vl53_UpdateMeasurement(VL53L0X_DEV Dev);


/** 
 * \brief Get the device mode
 * \param Dev - device handle
 * \param val - pointer to value storage
 * \return esp_ok or error
 **/
esp_err_t vl53_setDevicePwr(VL53L0X_DEV Dev, uint8_t *val);


/** 
 * \brief Get the device mode
 * \param Dev - device handle
 * \param val - pointer to value storage
 * \return esp_ok or error
 **/
esp_err_t vl53_getDevicePwr(VL53L0X_DEV Dev, uint8_t *val);


/** 
 * \brief Get the device mode
 * \param Dev - device handle
 * \param val - pointer to value storage
 * \return esp_ok or error
 **/
esp_err_t vl53_getDeviceSampleConfig(VL53L0X_DEV Dev, uint8_t *val);


/** 
 * \brief Get the device mode
 * \param Dev - device handle
 * \param val - pointer to value storage
 * \return esp_ok or error
 **/
esp_err_t vl53_setDeviceSampleConfig(VL53L0X_DEV Dev, uint8_t *val);

#endif /* VL53L0X_DRIVER_H */
