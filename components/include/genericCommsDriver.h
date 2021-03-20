/****************************************
* \file     genericCommsDriver.h
* \brief    some generic comms functions for 
*           general use
* \date     Aug 2020
* \author   RJAM
****************************************/

#ifndef GENERIC_COMMS_DRIVER_H
#define GENERIC_COMMS_DRIVER_H

/********* Includes ********************/

#include "esp_err.h"

/********* Definitions *****************/

#define GENERIC_I2C_COMMS_SHORTWAIT_MS 10
#define GENERIC_I2C_COMMS_TIMEOUT_MS 100

/********** Types **********************/

/******** Function Definitions *********/

/** \brief  gcd_i2c_read_address
 *          Perform a read from an address on an i2c channel
 *  \param  i2cChannel - a valid i2cChannel 
 *  \param  deviceAddr - the i2c address (unshifted) of the device
 *  \param  regAddr    - the register address
 *  \param  readLen    - length to read
 *  \param  rxBuffer   - a pointer to the rx buffer 
 * 
 *  \return ESP_OK or error
 * **/
esp_err_t gcd_i2c_read_address(uint8_t i2cChannel, uint8_t deviceAddr, uint8_t regAddr, uint16_t readLen, uint8_t *rxBuffer);

/** \brief  gcd_i2c_write_address
 *          Perform a read from an address on an i2c channel
 *  \param  i2cChannel - a valid i2cChannel 
 *  \param  deviceAddr - the i2c address (unshifted) of the device
 *  \param  regAddr    - the register address on the device
 *  \param  writeLen   - length of write
 *  \param  txBuffer   - a pointer to the tx buffer 
 * 
 *  \return ESP_OK or error
 * **/
esp_err_t gcd_i2c_write_address(uint8_t i2cChannel, uint8_t deviceAddr, uint8_t regAddr, uint16_t writeLen, uint8_t *txBuffer);

/** \brief  gcd_i2c_init()
 *          initialise an i2c bus
 *  \param  dataPin     -    duh
 *  \param  clockPin    -   also duh
 *  \param  clockSpeed  -   bus speed
 *  \param  busNum      -   i2c bus number (0 or 1)
 *  \return ESP_OK or error
 * **/
esp_err_t gcd_i2c_init(int16_t dataPin, int16_t clockPin, uint32_t clockSpeed, uint8_t busNum);


/** \brief: gcd_spi_init 
 *          initialise an spi bus
 *  \param clk_pin
 *  \param mosi_pin
 *  \param miso_pin
 *  \param spi_bus - 1 or 2
 *  \return ESP_OK or error
 **/
esp_err_t gcd_spi_init(int16_t clk_pin, int16_t mosi_pin, int16_t miso_pin, uint8_t spi_bus);



#endif /* GENERIC_COMMS_DRIVER_H */
