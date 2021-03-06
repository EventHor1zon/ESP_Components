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

#include <stdint.h>
#include "esp_err.h"

#include "driver/i2c.h"


/********* Definitions *****************/

#define GCD_SEMAPHORE_TIMEOUT 100
#define GENERIC_I2C_COMMS_SHORTWAIT_MS 10
#define GENERIC_I2C_COMMS_TIMEOUT_MS 100
/********** Types **********************/

typedef struct genericCommsDriver
{
    /* data */
    bool i2c0_is_init;
    bool i2c1_is_init;
    bool hspi_is_init;
    bool vspi_is_init;
    SemaphoreHandle_t i2c0_sem;
    SemaphoreHandle_t i2c1_sem;
    SemaphoreHandle_t hspi_sem;
    SemaphoreHandle_t vspi_sem;

} gcd_status_t;



/******** Function Definitions *********/

/** \brief is_bus_init 
 *         checks if selected bus has been init or inot.
 *  \param bus - the i2c bus to check
 *  \return boolean value of i2c bus init state
 **/ 
bool genericI2C_is_bus_init(uint8_t bus);


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

/** \brief  genericI2Cinit()
 *          initialise an i2c bus
 *  \param  dataPin     -    duh
 *  \param  clockPin    -   also duh
 *  \param  clockSpeed  -   bus speed
 *  \param  busNum      -   i2c bus number (0 or 1)
 *  \param  use_smphr   -   create a semaphore for access ctrl (sem will be used if not null)
 *  \return ESP_OK or error
 * **/
esp_err_t genericI2Cinit(int16_t dataPin, int16_t clockPin, uint32_t clockSpeed, uint8_t busNum, bool create_smphr);


/** \brief: generic_spi_init 
 *          initialise an spi bus
 *  \param clk_pin
 *  \param mosi_pin
 *  \param miso_pin
 *  \param spi_bus - 1 or 2
 *  \return ESP_OK or error
 **/
esp_err_t generic_spi_init(int16_t clk_pin, int16_t mosi_pin, int16_t miso_pin, uint8_t spi_bus);
#endif /* GENERIC_COMMS_DRIVER_H */
