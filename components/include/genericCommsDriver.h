/****************************************
* \file     genericCommsDriver.h
* \brief
* \date
* \author
****************************************/

#ifndef GENERIC_COMMS_DRIVER_H
#define GENERIC_COMMS_DRIVER_H

/********* Includes ********************/

#include <stdint.h>
#include "esp_err.h"

/********* Definitions *****************/

#define GENERIC_I2C_COMMS_TIMEOUT_MS 100
/********** Types **********************/

/******** Function Definitions *********/

/** \brief  genericI2CReadFromAddress
 *          Perform a read from an address on an i2c channel
 *  \param  i2cChannel - a valid i2cChannel 
 *  \param  deviceAddr - the i2c address (unshifted) of the device
 *  \param  regAddr    - the register address
 *  \param  readLen    - length to read
 *  \param  rxBuffer   - a pointer to the rx buffer 
 * 
 *  \return ESP_OK or error
 * **/
esp_err_t genericI2CReadFromAddress(uint8_t i2cChannel, uint8_t deviceAddr, uint8_t regAddr, uint16_t readLen, uint8_t *rxBuffer);

/** \brief  genericI2CWriteToAddress
 *          Perform a read from an address on an i2c channel
 *  \param  i2cChannel - a valid i2cChannel 
 *  \param  deviceAddr - the i2c address (unshifted) of the device
 *  \param  regAddr    - the register address on the device
 *  \param  writeLen   - length of write
 *  \param  txBuffer   - a pointer to the tx buffer 
 * 
 *  \return ESP_OK or error
 * **/
esp_err_t genericI2CwriteToAddress(uint8_t i2cChannel, uint8_t deviceAddr, uint8_t regAddr, uint16_t writeLen, uint8_t *txBuffer);

#endif /* GENERIC_COMMS_DRIVER_H */
