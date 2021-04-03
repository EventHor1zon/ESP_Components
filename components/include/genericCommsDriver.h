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

#include "esp_types.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

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

<<<<<<< HEAD
bool gdc_i2c_check_bus(uint8_t bus);
=======

bool gcd_check_i2c_bus(uint8_t bus);
>>>>>>> HMC5883

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
 *  \param  use_smphr   -   create a semaphore for access ctrl (sem will be used if not null)
 *  \return ESP_OK or error
 * **/
esp_err_t gcd_i2c_init(int16_t dataPin, int16_t clockPin, uint32_t clockSpeed, uint8_t busNum, bool use_smphr);
<<<<<<< HEAD


/** \brief: gcd_spi_init 
 *          initialise an spi bus
 *  \param clk_pin
 *  \param mosi_pin
 *  \param miso_pin
 *  \param spi_bus - 1 or 2
 *  \return ESP_OK or error
 **/
esp_err_t gcd_spi_init(int16_t clk_pin, int16_t mosi_pin, int16_t miso_pin, uint8_t spi_bus,  bool use_smphr);


=======


/** \brief: generic_spi_init 
 *          initialise an spi bus
 *  \param clk_pin
 *  \param mosi_pin
 *  \param miso_pin
 *  \param spi_bus - 1 or 2
 *  \return ESP_OK or error
 **/
esp_err_t gcd_spi_init(int16_t clk_pin, int16_t mosi_pin, int16_t miso_pin, uint8_t spi_bus, bool use_semphr);
>>>>>>> HMC5883
#endif /* GENERIC_COMMS_DRIVER_H */
