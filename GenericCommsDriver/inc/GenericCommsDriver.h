/****************************************
* \file     GenericCommsDriver.h
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

#define GCD_CONFIG_UART_TX_BUFF_SIZE 2048
#define GCD_CONFIG_UART_RX_BUFF_SIZE 2048

/********** Types **********************/

typedef struct GenericCommsDriver
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
    QueueHandle_t uart_queue;

} gcd_status_t;


typedef struct i2c_transaction {
    uint8_t bus;    /** i2c bus **/
    uint8_t dev;    /** device address **/
    uint8_t reg;    /**< register address **/
    uint32_t len;   /**< length of read **/
} gcd_transaction_t;


/******** Function Definitions *********/


/** \brief - Checks if valid i2c bus
 *  \param bus - the bus number
 *  \return true for good bus, false for bad bus
 **/
bool gcd_i2c_check_bus(uint8_t bus);

/** \brief - Checks if valid spi bus
 *  \param spi_bus bus num
 *  \return true or false
 **/
bool gcd_spi_check_bus(uint8_t spi_bus);

/** \brief if there's a semaphore
 *          for channel, claim it
 * \param channel i2c bus number
 * \return esp_ok or error
 **/ 
esp_err_t gcd_i2c_bus_claim(uint8_t i2c_channel);

/** \brief unclaim the semaphore
 *          for channel
 * \param channel i2c bus number
 * \return esp_ok or error
 **/ 
esp_err_t gcd_i2c_bus_unclaim(uint8_t i2c_channel);

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


/** \brief gcd_i2c_write_block
 *          Writes a block of data. yup.
 *   
 * 
 * 
 **/
esp_err_t gcd_i2c_write_block(uint8_t i2cChannel, uint8_t deviceAddr, uint16_t writeLen, uint8_t *txBuffer);


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


/** \brief: gcd_spi_init 
 *          initialise an spi bus
 *  \param clk_pin
 *  \param mosi_pin
 *  \param miso_pin
 *  \param spi_bus - 1 or 2
 *  \return ESP_OK or error
 **/
esp_err_t gcd_spi_init(int16_t clk_pin, int16_t mosi_pin, int16_t miso_pin, uint8_t spi_bus,  bool use_smphr);


/** \brief: Initialise a uart bus with some default values
 *  \param bus - bus number to init - 0,1,2
 *  \param tx_pin - gpio pin to use as Tx
 *  \param rx_pin - gpio pin to use as Rx
 *  \param cts_pin - optional cts pin
 *  \param rts_pin - optional rts_pin
 *  \param stop_bits - number of stop bits 
 *  \param parity - parity setting
 *  \return ESP_OK or Error
 * 
 */
esp_err_t gcd_uart_init(uint8_t bus, int16_t tx_pin, int16_t rx_pin, int16_t cts_pin, int16_t rts_pin, uint32_t baud, uint8_t stop_bits, uint8_t parity);


esp_err_t gcd_uart_transmit_blocking(uint8_t bus, uint8_t *data, uint16_t len);


esp_err_t gcd_uart_transmit_non_blocking(uint8_t bus, uint8_t *data, uint32_t len);


esp_err_t gcd_i2c_read_mod_write(uint8_t i2cChannel, uint8_t deviceAddr, uint8_t regAddr, uint8_t clear_bits, uint8_t set_bits);

#endif /* GENERIC_COMMS_DRIVER_H */
