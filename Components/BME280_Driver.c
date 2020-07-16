/***************************************
* \file     BME280_Driver.c
* \brief    A FreeRTOS/ESP-IDF driver for the BME280 bosch humidity/temp/pressure 
*           sensor. Can be called in periodic or on-demand sensing, and in 
*           a number of different sampling modes
*
* \date     July 2020
* \author   RJAM
****************************************/

/********* Includes *******************/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_err.h"

#include "BME280_Driver.h"

/****** Function Prototypes ***********/
static int32_t BME280_compensate_T_int32(int32_t adc_T);
static uint32_t BME280_compensate_P_int64(int32_t adc_P);
static uint32_t bme280_compensate_H_int32(int32_t adc_H);
/************ ISR *********************/

/****** Global Data *******************/

const char *BME_DRIVER_TAG = "BME280 DRIVER::";

BME280_controlData_t bme280Control = {0};

/****** Private Data ******************/

/****** Private Functions *************/

/* Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC. */
/* t_fine carries fine temperature as global value int32_t t_fine; */
static int32_t BME280_compensate_T_int32(int32_t adc_T)
{
    int32_t var1, var2, T;

    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

/* Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).*/
/* Output value of “24674867” represents 24674867 / 256 = 96386.2 Pa = 963.862 hPa */
static uint32_t BME280_compensate_P_int64(int32_t adc_P)
{
    int64_t a;
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
    if (var1 == 0)
    {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
    return (uint32_t)p;
}

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of “47445” represents 47445/1024 = 46.333 %RH

static uint32_t bme280_compensate_H_int32(int32_t adc_H)
{
    int32_t v_x1_u32r;
    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (uint32_t)(v_x1_u32r >> 12);
}

static esp_err_t bme280_i2cWriteToAddress(BME280_controlData_t *bme280Control, uint8_t regAddress, uint16_t writeLength, uint8_t *txBuffer)
{

    esp_err_t trxStatus;
    uint8_t commands[2];

    /** TODO: Find out if write bit needed or automatic **/
    commands[0] = bme280Control->deviceAddress << 1 | I2C_MASTER_WRITE;
    commands[1] = regAddress;

    i2c_cmd_handle_t cmdHandle = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmdHandle));
    ESP_ERROR_CHECK(i2c_master_write(cmdHandle, commands, 2, TRUE));
    ESP_ERROR_CHECK(i2c_master_read(cmdHandle, rxBuffer, (readLength - 1), TRUE));
    ESP_ERROR_CHECK(i2c_master_read(cmdHandle, rxBuffer, 1, FALSE));
    ESP_ERROR_CHECK(i2c_master_stop(cmdHandle));
    trxStatus = i2c_master_cmd_begin(bme280Control->i2cChannel, cmdHandle, pdMS_TO_TICKS(BME_DRIVER_I2C_TRX_TIMEOUT));
    i2c_cmd_link_delete(cmdHandle);

    return trxStatus;
}

static esp_err_t bme280_i2cReadFromAddress(BME280_controlData_t *bme280Control, uint8_t regAddress, uint16_t readLength, uint8_t *rxBuffer)
{
    /** TODO: replace this with a generic i2c readfromregister function **/
    esp_err_t trxStatus;
    uint8_t commands[2];
    commands[0] = bme280Control->deviceAddress << 1 | I2C_MASTER_WRITE;
    commands[1] = regAddress;

    i2c_cmd_handle_t cmdHandle = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmdHandle));
    ESP_ERROR_CHECK(i2c_master_write(cmdHandle, commands, 2, TRUE));
    ESP_ERROR_CHECK(i2c_master_read(cmdHandle, rxBuffer, (readLength - 1), TRUE));
    ESP_ERROR_CHECK(i2c_master_read(cmdHandle, rxBuffer, 1, FALSE));
    ESP_ERROR_CHECK(i2c_master_stop(cmdHandle));
    trxStatus = i2c_master_cmd_begin(bme280Control->i2cChannel, cmdHandle, pdMS_TO_TICKS(BME_DRIVER_I2C_TRX_TIMEOUT));
    i2c_cmd_link_delete(cmdHandle);

    return trxStatus;
}

static esp_err_t bme280_getDeviceStatus(BME280_controlData_t *bme280Control)
{
    esp_err_t trxStatus = ESP_OK;

    uint8_t statusBuffer = 0;
    trxStatus = bme280_i2cReadFromAddress(bme280control, BMP_REG_ADDR_DEV_STATUS, 1, &statusBuffer);
    if (trxStatus == ESP_OK)
    {
        bme280Control->sensorData.statusMeasure = (statusBuffer & BMP_STATUS_MEASURE_MASK) ? 1 : 0;
        bme280Control->sensorData.statusUpdate = (statusBuffer & BMP_STATUS_UPDATE_MASK) ? 1 : 0;
    }

    return trxStatus;
}

static esp_err_t bme280_getCalibrationData(BME280_controlData_t *bme280Control)
{
    esp_err_t trxStatus = ESP_OK;

    uint8_t buffer[BMP_CALIBR_DATA_LEN + 1] = {0}; /** buffer len 33 **/

    /** retrieve the calibration data - add +1 to length of bank1 as there's an unused byte (A0, index[24]) in there **/
    trxStatus = bme280_i2cReadFromAddress(bme280Control, (uint8_t)BMP_REG_ADDR_DIGT1_LSB, BMP_CALIBR_DATA_BANK1_LEN + 1, buffer);
    trxStatus = bme280_i2cReadFromAddress(bme280Control, (uint8_t)BMP_REG_ADDR_DIGH2_LSB, BMP_CALIBR_DATA_BANK2_LEN, &buffer[BMP_CALIBR_DATA_BANK1_LEN + 1]);

    /** now for some rejigging... **/

    if (trxStatus == ESP_OK)
    {
        bme280Control->calibrationData.dig_T1 = (uint16_t)buffer[0] << 8 | (uint16_t)buffer[1];
        bme280Control->calibrationData.dig_T2 = (int16_t)buffer[2] << 8 | (int16_t)buffer[3];
        bme280Control->calibrationData.dig_T3 = (int16_t)buffer[4] << 8 | (int16_t)buffer[5];
        bme280Control->calibrationData.dig_P1 = (uint16_t)buffer[6] << 8 | (uint16_t)buffer[7];
        bme280Control->calibrationData.dig_P2 = (int16_t)buffer[8] << 8 | (int16_t)buffer[9];
        bme280Control->calibrationData.dig_P3 = (int16_t)buffer[10] << 8 | (int16_t)buffer[11];
        bme280Control->calibrationData.dig_P4 = (int16_t)buffer[12] << 8 | (int16_t)buffer[13];
        bme280Control->calibrationData.dig_P5 = (int16_t)buffer[14] << 8 | (int16_t)buffer[15];
        bme280Control->calibrationData.dig_P6 = (int16_t)buffer[16] << 8 | (int16_t)buffer[17];
        bme280Control->calibrationData.dig_P7 = (int16_t)buffer[18] << 8 | (int16_t)buffer[19];
        bme280Control->calibrationData.dig_P8 = (int16_t)buffer[20] << 8 | (int16_t)buffer[21];
        bme280Control->calibrationData.dig_P9 = (int16_t)buffer[22] << 8 | (int16_t)buffer[23];
        bme280Control->calibrationData.dig_H1 = buffer[25];
        bme280Control->calibrationData.dig_H2 = (int16_t)buffer[26] << 8 | (int16_t)buffer[27];
        bme280Control->calibrationData.dig_H3 = buffer[28];
        bme280Control->calibrationData.dig_H4 = (int16_t)buffer[28] << 8 | (int16_t)buffer[29];
        bme280Control->calibrationData.dig_H5 = (int16_t)buffer[30] << 8 | (int16_t)buffer[31];
        bme280Control->calibrationData.dig_H6 = (int8_t)buffer[32];
    }

    return trxStatus;
}

static esp_err_t bme280_getDeviceID(BME280_controlData_t *bme280control, uint8_t deviceID)
{

    esp_err_t trxStatus = ESP_OK;

    uint8_t devID = 0;

    trxStatus = bme280_i2cReadFromAddress(bme280control, (uint8_t)BMP_REG_ADDR_DEVICEID, 1, &devID);

    if (trxStatus == ESP_OK)
    {
        deviceID = devID;
    }

    return trxStatus;
}

static bme280_InitDeviceSettings(BME280_controlData_t bme280control)
{
    uint8_t rwAddress;
}

/****** Global Functions *************/

esp_err_t bme280_init(bme_initData_t *initData)
{
    esp_err_t initStatus = ESP_OK;

    if (initData->i2cChannel == 0)
    {
        i2c_config_t i2cConf;
        /* init the i2c driver if not provided with an i2c driver handle */

        i2cConf.scl_io_num = DEBUG_I2C_CLOCK_PIN;
        i2cConf.sda_io_num = DEBUG_I2C_DATA_PIN;
        i2cConf.mode = I2C_MODE_MASTER;
        i2cConf.master.clk_speed = 200000;
        i2cConf.scl_pullup_en = TRUE;
        i2cConf.sda_pullup_en = TRUE;

        initStatus = i2c_param_config(0, &i2cConf);
        if (initStatus != ESP_OK)
        {
            ESP_LOGE(BME_DRIVER_TAG, "Error in configuring the I2C driver - %u", initStatus);
        }

        if (initStatus == ESP_OK)
        {
            initStatus = i2c_driver_install(0, I2C_MODE_MASTER, 0, 0, ESP_INTR_FLAG_LEVEL4);
            if (initStatus != ESP_OK)
            {
                ESP_LOGE(BME_DRIVER_TAG, "Error in installing the driver");
            }
        }
    }
    else
    {
        bme280Control.i2cChannel = initData->i2cChannel;
        bme280Control.mode = initData->mode;
        if (initData->addressPinState)
        {
            bme280Control.deviceAddress = BMP_I2C_ADDRESS_SDHIGH;
        }
        else
        {
            bme280Control.deviceAddress = BMP_I2C_ADDRESS_SDLOW;
        }
    }

    ESP_LOGI(BME_DRIVER_TAG, "Contacting device at address %02x", bme280Control.deviceAddress);

    if (initStatus == ESP_OK)
    {
        uint8_t deviceID = 0;
        initStatus = bme280_getDeviceID(&bme280Control, deviceID);
        if (deviceID == (uint8_t)BMP_DEVICE_ID)
        {
            ESP_LOGI(BME_DRIVER_TAG, "Device ID checks out!");
        }
    }

    if (initStatus == ESP_OK)
    {
        initStatus = bme280_getCalibrationData(&bme280control);
        if (initStatus == ESP_OK)
        {
            ESP_LOGI(BME_DRIVER_TAG, "Succesfully got the calibration data!");
        }
    }

    if (initStatus == ESP_OK)
    {
        initStatus = bme280_initDeviceSettings(&bme280Control);
    }
}