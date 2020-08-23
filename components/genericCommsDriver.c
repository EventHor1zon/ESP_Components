/***************************************
* \file     genericCommsDriver.c
* \brief    Some generic i2c/spi/other comms setup and 
*           read/write functions
*
* \date     Aug 2020
* \author   RJAM
****************************************/

/********* Includes *******************/
#include <stdint.h>

#include "genericCommsDriver.h"

#include "esp_err.h"
#include "esp_log.h"

#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"

/****** Function Prototypes ***********/

/************ ISR *********************/

/****** Private Data ******************/

/****** Private Functions *************/

/****** Global Data *******************/

/****** Global Functions *************/

esp_err_t genericI2CReadFromAddress(uint8_t i2cChannel, uint8_t deviceAddr, uint8_t regAddr, uint16_t readLen, uint8_t *rxBuffer)
{

    esp_err_t txStatus;

    if ((i2cChannel == I2C_NUM_0) || (i2cChannel == I2C_NUM_1))
    {

        i2c_cmd_handle_t txHandle = i2c_cmd_link_create();
        /** write register address **/
        i2c_master_start(txHandle);
        i2c_master_write_byte(txHandle, (deviceAddr << 1 | I2C_MASTER_WRITE), 1);
        i2c_master_write_byte(txHandle, regAddr, 1);
        /** read from register address **/
        i2c_master_start(txHandle);
        i2c_master_write_byte(txHandle, (deviceAddr << 1 | I2C_MASTER_READ));
        if (readLen > 1)
        {
            i2c_master_read(txHandle, rxBuffer, (readLen - 1), I2C_MASTER_ACK);
        }
        i2c_master_read(txHandle, rxBuffer + (readLen - 1), I2C_MASTER_NACK);
        i2c_master_stop(txHandle);

        txStatus = i2c_master_cmd_begin(i2cChannel, txHandle, pdMS_TO_TICKS(GENERIC_I2C_COMMS_TIMEOUT_MS));
        if (txStatus != ESP_OK)
        {
            ESP_LOGE("GenericI2CreadFromAddress", "Error during transmission [%u]", txStatus);
        }

        i2c_cmd_link_delete(txStatus);
    }
    else
    {
        ESP_LOGE("GenericI2CreadFromAddress", "Error - invalid i2c channel");
        txStatus = ESP_ERR_INVALID_ARG;
    }

    return txStatus;
}

esp_err_t genericI2CwriteToAddress(uint8_t i2cChannel, uint8_t deviceAddr, uint8_t regAddr, uint16_t writeLen, uint8_t *txBuffer)
{
    if ((i2cChannel == I2C_NUM_0) || (i2cChannel == I2C_NUM_1))
    {
        esp_err_t trxStatus;
        i2c_cmd_handle_t rxHandle = i2c_cmd_link_create();
        i2c_master_start(rxHandle);
        i2c_master_write_byte(rxHandle, (deviceAddr << 1 | I2C_MASTER_WRITE), 1);
        i2c_master_write(rxHandle, txBuffer, writeLen, 1);
        i2c_master_stop(rxHandle);

        trxStatus = i2c_master_cmd_begin(bmCtrl->i2cChannel, rxHandle, pdMS_TO_TICKS(BM_DRIVER_I2C_TRX_TIMEOUT));
        if (txStatus != ESP_OK)
        {
            ESP_LOGE("GenericI2CwriteToAddress", "Error during transmission [%u]", txStatus);
        }
        i2c_cmd_link_delete(rxHandle);
    }
    else
    {
        ESP_LOGE("GenericI2CreadFromAddress", "Error - invalid i2c channel");
        txStatus = ESP_ERR_INVALID_ARG;
    }

    return txStatus;
}