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

/********** Types **********************/

/******** Function Definitions *********/

VL53L0X_DEV vl53_init(void);

esp_err_t vl53_getDeviceMode(VL53L0X_DEV Dev, uint8_t *val);

esp_err_t vl53_UpdateMeasurement(VL53L0X_DEV Dev);

esp_err_t vl53_setDevicePwr(VL53L0X_DEV Dev, uint8_t *val);

esp_err_t vl53_getDevicePwr(VL53L0X_DEV Dev, uint8_t *val);
#endif /* VL53L0X_DRIVER_H */
