/***************************************
* \file     genericCommsDriver.c
* \brief    Some generic i2c/spi/other comms setup and 
*           read/write functions
*
* \date     Aug 2020
* \author   RJAM
****************************************/

/********* Includes *******************/

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

esp_err_t gcd_i2c_read_address(uint8_t i2cChannel, uint8_t deviceAddr, uint8_t regAddr, uint16_t readLen, uint8_t *rxBuffer)
{
    esp_err_t ret = ESP_OK;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    if (regAddr != -1)
    {
        i2c_master_write_byte(cmd, deviceAddr << 1 | 0, 1);
        i2c_master_write_byte(cmd, regAddr, 1);
        i2c_master_start(cmd);
    }
    i2c_master_write_byte(cmd, deviceAddr << 1 | 1, 1);
    if (readLen > 1)
    {
        i2c_master_read(cmd, rxBuffer, readLen - 1, 0);
    }
    i2c_master_read_byte(cmd, rxBuffer + readLen - 1, 1);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2cChannel, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_ERR_TIMEOUT)
    {
        ESP_LOGW("GenericI2C Read", "I2C Timeout error");
    } 
    if (ret != ESP_OK)
    {
        ESP_LOGW("GenericI2C Read", "I2C error");
    }

    return ret;
}

esp_err_t gcd_i2c_write_address(uint8_t i2cChannel, uint8_t deviceAddr, uint8_t regAddr, uint16_t writeLen, uint8_t *txBuffer)
{

    esp_err_t txStatus;

    if ((i2cChannel == I2C_NUM_0) || (i2cChannel == I2C_NUM_1))
    {
        i2c_cmd_handle_t rxHandle = i2c_cmd_link_create();
        i2c_master_start(rxHandle);
        i2c_master_write_byte(rxHandle, (deviceAddr << 1 | I2C_MASTER_WRITE), 1);
        i2c_master_write_byte(rxHandle, regAddr, 1);
        i2c_master_write(rxHandle, txBuffer, writeLen, 1);
        i2c_master_stop(rxHandle);

        txStatus = i2c_master_cmd_begin(i2cChannel, rxHandle, pdMS_TO_TICKS(GENERIC_I2C_COMMS_TIMEOUT_MS));
        if (txStatus != ESP_OK)
        {
            ESP_LOGE("gcd_i2c_write_address", "Error during transmission [%u]", txStatus);
        }
        i2c_cmd_link_delete(rxHandle);
    }
    else
    {
        ESP_LOGE("gcd_i2c_read_address", "Error - invalid i2c channel");
        txStatus = ESP_ERR_INVALID_ARG;
    }

    return txStatus;
}

esp_err_t gcd_i2c_init(int16_t dataPin, int16_t clockPin, uint32_t clockSpeed, uint8_t busNum)
{
    ESP_LOGI("GenericI2C Init", "Initialsing i2c bus");
    esp_err_t status = ESP_OK;

    if (clockSpeed > 1000000)
    {
        ESP_LOGE("I2C Init", "Error, the clock speed too damn high!");
        status = ESP_ERR_INVALID_ARG;
    }
    else if (busNum != I2C_NUM_0 && busNum != I2C_NUM_1)
    {
        ESP_LOGE("I2C Init", "Error, invalid i2c Bus number");
        status = ESP_ERR_INVALID_ARG;
    }
    else
    {
        ESP_LOGI("GenericI2C Init", "starting driver %u %u %u %u", dataPin, clockPin, clockSpeed, busNum);
        status = i2c_driver_install(busNum, I2C_MODE_MASTER, 0, 0, 0);
    }

    if (status == ESP_OK)
    {
        i2c_config_t i2cConf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = dataPin,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_io_num = clockPin,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = clockSpeed
            };
        status = i2c_param_config(busNum, &i2cConf);
    }
    if (status == ESP_OK)
    {
        ESP_LOGI("GenericI2C Init", "I2C driver started on bus %d", busNum);
    }
    return status;
}


esp_err_t gcd_spi_check_bus(uint8_t spi_bus) {
    esp_err_t status = ESP_OK;
    if (!(spi_bus == SPI2_HOST || spi_bus == SPI3_HOST))
    {
        ESP_LOGE("SPI_SETUP", "Error - invalid SPI bus. Please use SPI2_HOST or SPI3_HOST");
        status = ESP_ERR_INVALID_ARG;
    }

    return status;
}


esp_err_t gcd_spi_init(int16_t clk_pin, int16_t mosi_pin, int16_t miso_pin, uint8_t spi_bus) {

    esp_err_t status = ESP_OK;
    ESP_LOGI("SPI_SETUP", "[+] Setting up SPI bus");

    status = gcd_spi_check_bus(spi_bus);
    if(status == ESP_OK) {
        spi_bus_config_t buscfg = {0};
        buscfg.mosi_io_num = mosi_pin;
        buscfg.miso_io_num = miso_pin;
        buscfg.sclk_io_num = clk_pin;
        buscfg.max_transfer_sz = 0; // led data + up to 10 frames of start & end
        buscfg.quadhd_io_num = -1;
        buscfg.quadwp_io_num = -1;
        buscfg.flags = SPICOMMON_BUSFLAG_MASTER;
        buscfg.intr_flags = 0;
        status = spi_bus_initialize(spi_bus, &buscfg, 1);
    }

    return status;
}


