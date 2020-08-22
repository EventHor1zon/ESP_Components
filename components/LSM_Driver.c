/***************************************
* \file     LSM_Driver.c
* \brief    Driver for the LSM6DS3 Gyro and Accelerometer
*           Can be configured to use i2c or spi
*           Basic functionality for now, more complex later
* \date     Aug 2020
* \author   RJAM
****************************************/

/********* Includes *******************/
#include <stdint.h>
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/****** Function Prototypes ***********/

/************ ISR *********************/

/****** Private Data ******************/

/****** Private Functions *************/

/****** Global Data *******************/

/****** Global Functions *************/

/** \brief LSM_init()
 *      
 *          initilise the LSM driver. Assumes a single device. 
 *          takes a pointer to an LSM init struct
 * 
 *  \param LSM_initData_t initData 
 *  \return ESP_OK or error
 */
esp_err_t
LSM_init(LSM_initData_t initData);

/** 
 *  LSM_deinit() 
 *      tear down the LSM driver 
 *  
 *  \return ESP_OK or error
*/

esp_err_t LSM_deInit();

esp_err_t LSM_setFIFOmode(LSM_FIFOMode_t mode);
esp_err_t LSM_setFIFOwatermark(uint16_t watermark);
esp_err_t LSM_getFIFOCount(uint16_t *count);
esp_err_t LSM_setFIFOpackets(LSM_FifoPktCfg_t config, uint8_t fifoPacket);
esp_err_t LSM_configInt(LSM_DeviceSettings_t *device, uint8_t intNum);

esp_err_t LSM_readFifoBlock(LSM_DeviceSettings_t *device, uint16_t length);
esp_err_t LSM_readWhoAmI(LSM_DeviceSettings_t *device);
