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
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "genericCommsDriver.h"
#include "esp_err.h"
#include "esp_log.h"
#include "Utilities.h"

#include "BME280_Driver.h"

/****** Function Prototypes ***********/
static int32_t bm280_compensate_T_int32(bm_controlData_t *bmCtrl);
static uint32_t bm280_compensate_P_int64(bm_controlData_t *bmCtrl);
#ifdef BME_280
static uint32_t bm280_compensate_H_int32(bm_controlData_t *bmCtrl);
#endif
static esp_err_t bm280_getDeviceStatus(bm_controlData_t *bmCtrl);
/************ ISR *********************/

/****** Global Data *******************/

// #define DEBUG 1

const char *BM_DRIVER_TAG = "[BM280 DRIVER]";

const int wait_sample_fin = 10;
const int wait_new_sample = 50;
const int wait_idle = 1000;
/****** Private Functions *************/

/** Calibration Functions :  The following calibration fucntions are adapted from the Bosch BME280 Data sheet **/

/* Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC. */
/* bmCtrl->calibrationData.t_fine carries fine temperature as global value int32_t bmCtrl->calibrationData.t_fine; */
static int32_t bm280_compensate_T_int32(bm_controlData_t *bmCtrl)
{
    if (bmCtrl->calibrationAquired)
    {
        int32_t adc_T = bmCtrl->sensorData.rawTemperature;
        int32_t var1, var2, T;

        var1 = ((((adc_T >> 3) - ((int32_t)bmCtrl->calibrationData.dig_T1 << 1))) * ((int32_t)bmCtrl->calibrationData.dig_T2)) >> 11;
        var2 = (((((adc_T >> 4) - ((int32_t)bmCtrl->calibrationData.dig_T1)) * ((adc_T >> 4) - ((int32_t)bmCtrl->calibrationData.dig_T1))) >> 12) * ((int32_t)bmCtrl->calibrationData.dig_T3)) >> 14;
        bmCtrl->calibrationData.t_fine = var1 + var2;
        T = (bmCtrl->calibrationData.t_fine * 5 + 128) >> 8;
        return T;
    }
    else
    {
        ESP_LOGE(BM_DRIVER_TAG, "Error Calibration data not aquired yet!");
        return 0;
    }
}

/* Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).*/
/* Output value of “24674867” represents 24674867 / 256 = 96386.2 Pa = 963.862 hPa */
static uint32_t bm280_compensate_P_int64(bm_controlData_t *bmCtrl)
{
    if (bmCtrl->calibrationAquired)
    {
        int32_t adc_P = bmCtrl->sensorData.rawPressure;
        int64_t var1, var2, p;
        var1 = ((int64_t)bmCtrl->calibrationData.t_fine) - 128000;
        var2 = var1 * var1 * (int64_t)bmCtrl->calibrationData.dig_P6;
        var2 = var2 + ((var1 * (int64_t)bmCtrl->calibrationData.dig_P5) << 17);
        var2 = var2 + (((int64_t)bmCtrl->calibrationData.dig_P4) << 35);
        var1 = ((var1 * var1 * (int64_t)bmCtrl->calibrationData.dig_P3) >> 8) + ((var1 * (int64_t)bmCtrl->calibrationData.dig_P2) << 12);
        var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmCtrl->calibrationData.dig_P1) >> 33;
        if (var1 == 0)
        {
            return 0; // avoid exception caused by division by zero
        }
        p = 1048576 - adc_P;
        p = (((p << 31) - var2) * 3125) / var1;
        var1 = (((int64_t)bmCtrl->calibrationData.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
        var2 = (((int64_t)bmCtrl->calibrationData.dig_P8) * p) >> 19;
        p = ((p + var1 + var2) >> 8) + (((int64_t)bmCtrl->calibrationData.dig_P7) << 4);
        return (uint32_t)p;
    }
    else
    {
        ESP_LOGE(BM_DRIVER_TAG, "Error Calibration data not aquired yet!");
        return 0;
    }
}

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of “47445” represents 47445/1024 = 46.333 %RH
#ifdef BME_280
static uint32_t bm280_compensate_H_int32(bm_controlData_t *bmCtrl)
{
    if (bmCtrl->calibrationAquired)
    {
        int32_t adc_H = bmCtrl->sensorData.rawHumidity;
        int32_t v_x1_u32r;
        v_x1_u32r = (bmCtrl->calibrationData.t_fine - ((int32_t)76800));
        v_x1_u32r = (((((adc_H << 14) - (((int32_t)bmCtrl->calibrationData.dig_H4) << 20) - (((int32_t)bmCtrl->calibrationData.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)bmCtrl->calibrationData.dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)bmCtrl->calibrationData.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)bmCtrl->calibrationData.dig_H2) + 8192) >> 14));
        v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)bmCtrl->calibrationData.dig_H1)) >> 4));
        v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
        v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
        return (uint32_t)(v_x1_u32r >> 12);
    }
    else
    {
        ESP_LOGE(BM_DRIVER_TAG, "Error Calibration data not aquired yet!");
        return 0;
    }
}
#endif

static uint16_t bm280_sleepTime(bm_controlData_t *bmCtrl)
{
    uint8_t timeSetting = bmCtrl->devSettings.sampleInterval;
    uint16_t time;
    switch (timeSetting)
    {
    case BM_T_STDBY_0_5MS:
        time = 1; /** this one is trick? divide by 2, I guess **/
        break;
    case BM_T_STDBY_62_5MS:
        time = 63;
        break;
    case BM_T_STDBY_125MS:
        time = 125;
        break;
    case BM_T_STDBY_250MS:
        time = 250;
        break;
    case BM_T_STDBY_500MS:
        time = 500;
        break;
    case BM_T_STDBY_1000MS:
        time = 1000;
        break;
#ifdef BME_280
    case BM_T_STDBY_10MS:
        time = 10;
        break;
    case BM_T_STDBY_20MS:
        time = 20;
        break;
#else
    case BM_T_STDBY_2000MS:
        time = 2000;
        break;
    case BM_T_STDBY_4000MS:
        time = 4000;
        break;
#endif
    default:
        time = 1000;
        break;
    }

    return time;
}

static esp_err_t bm280_getDeviceStatus(bm_controlData_t *bmCtrl)
{
    esp_err_t trxStatus = ESP_OK;

    uint8_t statusBuffer = 0;
    trxStatus = genericI2CReadFromAddress(bmCtrl->i2cChannel, bmCtrl->deviceAddress, BM_REG_ADDR_DEV_STATUS, 1, &statusBuffer);
    if (trxStatus == ESP_OK)
    {
        bmCtrl->sensorData.statusMeasure = (statusBuffer & BM_STATUS_MEASURE_MASK) ? 1 : 0;
        bmCtrl->sensorData.statusUpdate = (statusBuffer & BM_STATUS_UPDATE_MASK) ? 1 : 0;
    }

    return trxStatus;
}

static esp_err_t bm280_getCalibrationData(bm_controlData_t *bmCtrl)
{
    esp_err_t trxStatus = ESP_OK;

    uint8_t buffer[BM_CALIBR_DATA_BANK1_LEN] = {0};

    /* check the status register for calibration load complete */
    uint8_t statusReg = 0;
    genericI2CReadFromAddress(bmCtrl->i2cChannel, bmCtrl->deviceAddress, (uint8_t)BM_REG_ADDR_DEV_STATUS, 1, &statusReg);

    while (statusReg & BM_STATUS_UPDATE_MASK)
    {
        vTaskDelay(10);
        genericI2CReadFromAddress(bmCtrl->i2cChannel, bmCtrl->deviceAddress, (uint8_t)BM_REG_ADDR_DEV_STATUS, 1, &statusReg);
    }

#ifdef BME_280
    /** retrieve the calibration data - add +1 to length of bank1 as there's an unused byte (A0, index[24]) in there **/
    uint8_t bufferB[BM_CALIBR_DATA_BANK2_LEN] = {0};
    trxStatus = genericI2CReadFromAddress(bmCtrl->i2cChannel, bmCtrl->deviceAddress, (uint8_t)BM_REG_ADDR_DIGT1_LSB, BM_CALIBR_DATA_BANK1_LEN, buffer);
    trxStatus = genericI2CReadFromAddress(bmCtrl->i2cChannel, bmCtrl->deviceAddress, (uint8_t)BM_REG_ADDR_DIGH2_LSB, BM_CALIBR_DATA_BANK2_LEN, bufferB);
#else
    trxStatus = genericI2CReadFromAddress(bmCtrl, (uint8_t)BM_REG_ADDR_DIGT1_LSB, BM_CALIBR_DATA_BANK1_LEN, buffer);
#endif

    if (trxStatus == ESP_OK)
    {
        bmCtrl->calibrationData.dig_T1 = (uint16_t)buffer[1] << 8 | (uint16_t)buffer[0];
        bmCtrl->calibrationData.dig_T2 = (int16_t)buffer[3] << 8 | (int16_t)buffer[2];
        bmCtrl->calibrationData.dig_T3 = (int16_t)buffer[5] << 8 | (int16_t)buffer[4];
        bmCtrl->calibrationData.dig_P1 = (uint16_t)buffer[7] << 8 | (uint16_t)buffer[6];
        bmCtrl->calibrationData.dig_P2 = (int16_t)buffer[9] << 8 | (int16_t)buffer[8];
        bmCtrl->calibrationData.dig_P3 = (int16_t)buffer[11] << 8 | (int16_t)buffer[10];
        bmCtrl->calibrationData.dig_P4 = (int16_t)buffer[13] << 8 | (int16_t)buffer[12];
        bmCtrl->calibrationData.dig_P5 = (int16_t)buffer[15] << 8 | (int16_t)buffer[14];
        bmCtrl->calibrationData.dig_P6 = (int16_t)buffer[17] << 8 | (int16_t)buffer[16];
        bmCtrl->calibrationData.dig_P7 = (int16_t)buffer[19] << 8 | (int16_t)buffer[18];
        bmCtrl->calibrationData.dig_P8 = (int16_t)buffer[21] << 8 | (int16_t)buffer[20];
        bmCtrl->calibrationData.dig_P9 = (int16_t)buffer[23] << 8 | (int16_t)buffer[22];

#ifdef BME_280
        int16_t dig_H4_lsb = 0, dig_H4_msb = 0, dig_H5_lsb = 0, dig_H5_msb = 0;
        bmCtrl->calibrationData.dig_H1 = buffer[25];
        bmCtrl->calibrationData.dig_H2 = (int16_t)bufferB[1] << 8 | (int16_t)bufferB[0];
        bmCtrl->calibrationData.dig_H3 = bufferB[2];
        dig_H4_msb = (int16_t)(int8_t)bufferB[3] * 16;
        dig_H4_lsb = (int16_t)(bufferB[4] & 0x0F);
        bmCtrl->calibrationData.dig_H4 = dig_H4_msb | dig_H4_lsb;
        dig_H5_msb = (int16_t)(int8_t)bufferB[5] * 16;
        dig_H5_lsb = (int16_t)(bufferB[4] >> 4);
        bmCtrl->calibrationData.dig_H5 = dig_H5_msb | dig_H5_lsb;
        bmCtrl->calibrationData.dig_H6 = (int8_t)bufferB[6];
#endif
        bmCtrl->calibrationAquired = true;
    }
    else
    {
        ESP_LOGI(BM_DRIVER_TAG, "Error reading calibration data");
    }

    return trxStatus;
}

static esp_err_t bm280_InitDeviceSettings(bm_controlData_t *bmCtrl)
{

    esp_err_t trxStatus = ESP_OK;
    uint8_t sampleMode = bmCtrl->initData->sampleMode;
    uint8_t sampleType = bmCtrl->initData->sampleType;

    uint8_t commands[BM_CONFIG_WRITE_LEN] = {0};

    /* put the device in sleep mode */
    genericI2CwriteToAddress(bmCtrl->i2cChannel, bmCtrl->deviceAddress, BM_REG_ADDR_CTRL_MEASURE, 1, commands);

    switch (sampleMode)
    {
    case BM_SAMPLE_OFF:
        break;
    case BM_FORCE_MODE:
        commands[0] = (uint8_t)BM_FORCE_MODE;
        break;
    case BM_NORMAL_MODE:
        commands[0] = (uint8_t)BM_NORMAL_MODE;
        commands[1] |= BM_DEFAULT_T_STDBY; /** t_standby = 0.5ms **/
        break;
    default:
        ESP_LOGE(BM_DRIVER_TAG, "Error - incorrect sample mode type selected");
        break;
    }

    switch (sampleType)
    {
    case BM_MODE_TEMP:
        commands[0] |= BM_CTRL_TEMP_BIT;
        break;
    case BM_MODE_TEMP_PRESSURE:
        commands[0] |= (BM_CTRL_TEMP_BIT | BM_CTRL_PRESSURE_BIT);
        break;
#ifdef BME_280
    case BM_MODE_TEMP_HUMIDITY:
        commands[2] = 1;
        commands[0] |= BM_CTRL_TEMP_BIT;
        break;
    case BM_MODE_TEMP_PRESSURE_HUMIDITY:
        commands[2] = 1;
        commands[0] |= (BM_CTRL_TEMP_BIT | BM_CTRL_PRESSURE_BIT);
        break;
#endif
    default:
        break;
    }

#ifdef BME_280
    /** have to do 2 writes to 0xF2 and 0xF4-5, because 0xF3 is read only :/ **/
    trxStatus = genericI2CwriteToAddress(bmCtrl->i2cChannel, bmCtrl->deviceAddress, BM_REG_ADDR_CTRL_HUMID, 1, &commands[2]);
#endif
    trxStatus = genericI2CwriteToAddress(bmCtrl->i2cChannel, bmCtrl->deviceAddress, BM_REG_ADDR_CONFIG, 1, &commands[1]);
    trxStatus = genericI2CwriteToAddress(bmCtrl->i2cChannel, bmCtrl->deviceAddress, BM_REG_ADDR_CTRL_MEASURE, 1, commands);
    if (trxStatus != ESP_OK)
    {
        ESP_LOGE(BM_DRIVER_TAG, "Error in setting control registers");
    }
    else
    {
        bmCtrl->sampleMask = commands[0];
        bmCtrl->configMask = commands[1];
        bmCtrl->devSettings.sampleMode = sampleMode;
        bmCtrl->devSettings.sampleType = sampleType;
    }

    return trxStatus;
}

/****** Global Functions *************/

esp_err_t bm280_getDeviceID(bm_controlData_t *bmCtrl, uint8_t *deviceID)
{

    esp_err_t trxStatus = ESP_OK;

    uint8_t devID = 0;
    trxStatus = genericI2CReadFromAddress(bmCtrl->i2cChannel, bmCtrl->deviceAddress, (uint8_t)BM_REG_ADDR_DEVICEID, 1, &devID);

    if (trxStatus == ESP_OK)
    {
        *deviceID = devID;
    }
    else
    {
        printf("txStatus = %d", trxStatus);
    }
    return trxStatus;
}

esp_err_t bm280_updateMeasurements(bm_controlData_t *bmCtrl)
{

    esp_err_t trxStatus = ESP_OK;
    uint8_t forcedMeasure = bmCtrl->sampleMask | BM_CTRL_MODE_FORCED;
    uint8_t rxBuffer[BM_MEASURE_READ_LEN] = {0};

    if (bmCtrl->devSettings.sampleMode == BM_FORCE_MODE)
    {
#ifdef DEBUG
        ESP_LOGI(BM_DRIVER_TAG, "Telling device to sample...");
#endif
        trxStatus = genericI2CwriteToAddress(bmCtrl->i2cChannel, bmCtrl->deviceAddress, BM_REG_ADDR_CTRL_MEASURE, 1, &forcedMeasure);
        if (trxStatus != ESP_OK)
        {
            ESP_LOGE(BM_DRIVER_TAG, "Error in writing to mode");
        }
        vTaskDelay(pdMS_TO_TICKS(wait_new_sample));

        /** check sample finished **/
        uint8_t reg = 0;
        if (genericI2CReadFromAddress(bmCtrl->i2cChannel, bmCtrl->deviceAddress, BM_REG_ADDR_DEV_STATUS, 1, &reg) & BM_STATUS_MEASURE_MASK)
        {
            while (genericI2CReadFromAddress(bmCtrl->i2cChannel, bmCtrl->deviceAddress, BM_REG_ADDR_DEV_STATUS, 1, &reg) & BM_STATUS_MEASURE_MASK)
            {
                vTaskDelay(pdMS_TO_TICKS(wait_sample_fin));
            }
        }
    }

    trxStatus = genericI2CReadFromAddress(bmCtrl->i2cChannel, bmCtrl->deviceAddress, BM_REG_ADDR_PRESSURE_MSB, (uint16_t)BM_MEASURE_READ_LEN, rxBuffer);

    if (trxStatus == ESP_OK)
    {
        int32_t regP = (uint32_t)rxBuffer[0] << 16 | (uint32_t)rxBuffer[1] << 8 | (uint32_t)rxBuffer[2];
        int32_t regT = (uint32_t)rxBuffer[3] << 16 | (uint32_t)rxBuffer[4] << 8 | (uint32_t)rxBuffer[5];
        bmCtrl->sensorData.rawPressure = regP >> 4;
        bmCtrl->sensorData.rawTemperature = regT >> 4;
#ifdef BME_280
        bmCtrl->sensorData.rawHumidity = (uint32_t)rxBuffer[6] << 8 | (uint32_t)rxBuffer[7];
#endif

#ifdef DEBUG
        if (regT == 0x80000)
        {
            ESP_LOGI(BM_DRIVER_TAG, "Info: Temp is disabled");
        }
        if (regP == 0x80000)
        {
            ESP_LOGI(BM_DRIVER_TAG, "Info: Pressure is disabled");
        }
        if (bmCtrl->sensorData.rawHumidity == 0x8000)
        {
            ESP_LOGI(BM_DRIVER_TAG, "Info: Humid is disabled");
        }
#endif
        switch (bmCtrl->devSettings.sampleType)
        {
        case BM_MODE_TEMP:
            bmCtrl->sensorData.calibratedTemperature = bm280_compensate_T_int32(bmCtrl);
            bmCtrl->sensorData.realTemperature = (float)bmCtrl->sensorData.calibratedTemperature / 100.0;
            bmCtrl->devSettings.tempOS = 1;
            break;
        case BM_MODE_TEMP_PRESSURE:
            bmCtrl->sensorData.calibratedTemperature = bm280_compensate_T_int32(bmCtrl);
            bmCtrl->sensorData.calibratedPressure = bm280_compensate_P_int64(bmCtrl);
            bmCtrl->sensorData.realTemperature = (float)bmCtrl->sensorData.calibratedTemperature / 100.0;
            bmCtrl->sensorData.realPressure = (float)bmCtrl->sensorData.calibratedPressure / 256;
            bmCtrl->devSettings.tempOS = 1;
            bmCtrl->devSettings.pressOS = 1;
            break;
#ifdef BME_280
        case BM_MODE_TEMP_HUMIDITY:
            bmCtrl->sensorData.calibratedTemperature = bm280_compensate_T_int32(bmCtrl);
            bmCtrl->sensorData.realTemperature = (float)bmCtrl->sensorData.calibratedTemperature / 100.0;
            bmCtrl->sensorData.calibratedHumidity = bm280_compensate_H_int32(bmCtrl);
            bmCtrl->sensorData.realHumidity = (float)bmCtrl->sensorData.calibratedHumidity / 1024;
            bmCtrl->devSettings.tempOS = 1;
            bmCtrl->devSettings.humidOS = 1;
            break;
        case BM_MODE_TEMP_PRESSURE_HUMIDITY:
            bmCtrl->sensorData.calibratedTemperature = bm280_compensate_T_int32(bmCtrl);
            bmCtrl->sensorData.realTemperature = (float)bmCtrl->sensorData.calibratedTemperature / 100.0;
            bmCtrl->sensorData.calibratedPressure = bm280_compensate_P_int64(bmCtrl);
            bmCtrl->sensorData.realPressure = (float)bmCtrl->sensorData.calibratedPressure / 256;
            bmCtrl->sensorData.calibratedHumidity = bm280_compensate_H_int32(bmCtrl);
            bmCtrl->sensorData.realHumidity = (float)bmCtrl->sensorData.calibratedHumidity / 1024;
            bmCtrl->devSettings.tempOS = 1;
            bmCtrl->devSettings.humidOS = 1;
            break;
#endif
        default:
            break;
        }
    }
    else
    {
        ESP_LOGE(BM_DRIVER_TAG, "Error in reading data [%u]", trxStatus);
    }

    return trxStatus;
}

esp_err_t bm280_getTemperature(bm_controlData_t *bmCtrl, float *realTemp)
{
    esp_err_t status = ESP_OK;

    *realTemp = bmCtrl->sensorData.realTemperature;

    return status;
}

esp_err_t bm280_getPressure(bm_controlData_t *bmCtrl, float *realPressure)
{
    esp_err_t status = ESP_OK;

    *realPressure = bmCtrl->sensorData.realPressure;

    return status;
}

esp_err_t bm280_getHumidity(bm_controlData_t *bmCtrl, float *realHumidity)
{
    esp_err_t status = ESP_OK;

    *realHumidity = bmCtrl->sensorData.realHumidity;

    return status;
}

esp_err_t bm280_getHumidityOS(bm_controlData_t *bmCtrl, uint8_t *humidOS)
{
    esp_err_t status = ESP_OK;
    *humidOS = bmCtrl->devSettings.humidOS;
    return status;
}

esp_err_t bm280_getTemperatureOS(bm_controlData_t *bmCtrl, uint8_t *tempOS)
{
    esp_err_t status = ESP_OK;
    *tempOS = bmCtrl->devSettings.humidOS;
    return status;
}

esp_err_t bm280_getPressureOS(bm_controlData_t *bmCtrl, uint8_t *presOS)
{
    esp_err_t status = ESP_OK;
    *presOS = bmCtrl->devSettings.humidOS;
    return status;
}

esp_err_t bm280_setHumidityOS(bm_controlData_t *bmCtrl, BM_overSample_t *os)
{
    esp_err_t status = ESP_OK;
    uint8_t OSlevel = *os;
    status = genericI2CwriteToAddress(bmCtrl->i2cChannel, bmCtrl->deviceAddress, BM_REG_ADDR_CTRL_HUMID, 1, &OSlevel);
    bmCtrl->devSettings.humidOS = *os;
    if (OSlevel > 0)
    {
        bmCtrl->devSettings.sampleType |= BM_MODE_HUMIDITY;
    }
    else
    {
        bmCtrl->devSettings.sampleType &= ~(BM_MODE_HUMIDITY);
    }
    return status;
}

esp_err_t bm280_setTemperatureOS(bm_controlData_t *bmCtrl, BM_overSample_t *os)
{
    esp_err_t status = ESP_OK;

    uint8_t reg = 0;
    status = genericI2CReadFromAddress(bmCtrl->i2cChannel, bmCtrl->deviceAddress, BM_REG_ADDR_CTRL_MEASURE, 1, &reg);
    if (status == ESP_OK)
    {
        reg &= 0b00011111;        /* clear the top 3 bits */
        uint8_t level = *os << 5; /** shift level over 5 bits */
        reg &= level;             /* set the new value in reg */
        status = genericI2CwriteToAddress(bmCtrl->i2cChannel, bmCtrl->deviceAddress, BM_REG_ADDR_CTRL_MEASURE, 1, &reg);
    }
    bmCtrl->devSettings.tempOS = *os;
    if (*os > 0)
    {
        bmCtrl->devSettings.sampleType |= BM_MODE_TEMP;
    }
    else
    {
        bmCtrl->devSettings.sampleType &= ~(BM_MODE_TEMP);
    }
    return status;
}

esp_err_t bm280_setPressureOS(bm_controlData_t *bmCtrl, BM_overSample_t *os)
{
    esp_err_t status = ESP_OK;

    uint8_t reg = 0;
    status = genericI2CReadFromAddress(bmCtrl->i2cChannel, bmCtrl->deviceAddress, BM_REG_ADDR_CTRL_MEASURE, 1, &reg);
    if (status == ESP_OK)
    {
        reg &= 0b11100011;        /* clear the mid 3 bits */
        uint8_t level = *os << 2; /** shift level over 2 bits */
        reg &= level;             /* set the new value in reg */
        status = genericI2CwriteToAddress(bmCtrl->i2cChannel, bmCtrl->deviceAddress, BM_REG_ADDR_CTRL_MEASURE, 1, &reg);
    }
    bmCtrl->devSettings.pressOS = *os;
    if (*os > 0)
    {
        bmCtrl->devSettings.sampleType |= BM_MODE_PRESSURE;
    }
    else
    {
        bmCtrl->devSettings.sampleType &= ~(BM_MODE_PRESSURE);
    }
    return status;
}

esp_err_t bm280_getSampleMode(bm_controlData_t *bmCtrl, uint8_t *sampleMode)
{
    esp_err_t status = ESP_OK;

    *sampleMode = bmCtrl->devSettings.sampleMode;

    return status;
}

esp_err_t bm280_setSampleMode(bm_controlData_t *bmCtrl, BM_sampleMode_t *sampleMode)
{
    esp_err_t status = ESP_OK;

    uint8_t reg = 0;
    if (genericI2CReadFromAddress(bmCtrl->i2cChannel, bmCtrl->deviceAddress, BM_REG_ADDR_CTRL_MEASURE, 1, &reg) == ESP_OK)
    {
        reg &= 0b11111100;
        reg |= *sampleMode;
        status = genericI2CwriteToAddress(bmCtrl->i2cChannel, bmCtrl->deviceAddress, BM_REG_ADDR_CTRL_MEASURE, 1, &reg);
    }
    bmCtrl->devSettings.sampleMode = *sampleMode;
    return status;
}

esp_err_t bm280_getSampleType(bm_controlData_t *bmCtrl, uint8_t *sampleType)
{
    esp_err_t status = ESP_OK;

    *sampleType = bmCtrl->devSettings.sampleType;

    return status;
}

esp_err_t bm280_getFilterSetting(bm_controlData_t *bmCtrl, uint8_t *filter)
{
    esp_err_t status = ESP_OK;
    *filter = bmCtrl->devSettings.filterCoefficient;
    return status;
}

esp_err_t bm280_setFilterSetting(bm_controlData_t *bmCtrl, bm_filter_t *filter)
{
    esp_err_t status = ESP_OK;
    uint8_t reg = 0;
    if (genericI2CReadFromAddress(bmCtrl->i2cChannel, bmCtrl->deviceAddress, BM_REG_ADDR_CONFIG, 1, &reg) == ESP_OK)
    {
        reg &= 0b11100011;
        reg |= (*filter << 2);
        status = genericI2CwriteToAddress(bmCtrl->i2cChannel, bmCtrl->deviceAddress, BM_REG_ADDR_CONFIG, 1, &reg);
        bmCtrl->devSettings.filterCoefficient = *filter;
    }
    return status;
}

esp_err_t bm280_getSampleInterval(bm_controlData_t *bmCtrl, uint8_t *dT)
{
    esp_err_t status = ESP_OK;
    *dT = bmCtrl->devSettings.sampleInterval;
    return status;
}

esp_err_t bm280_setSampleInterval(bm_controlData_t *bmCtrl, BM_standbyT_t *dT)
{
    esp_err_t status = ESP_OK;
    uint8_t reg = 0;
    if (genericI2CReadFromAddress(bmCtrl->i2cChannel, bmCtrl->deviceAddress, BM_REG_ADDR_CONFIG, 1, &reg) == ESP_OK)
    {
        reg &= 0b00011111;
        reg |= (*dT << 5);
        status = genericI2CwriteToAddress(bmCtrl->i2cChannel, bmCtrl->deviceAddress, BM_REG_ADDR_CONFIG, 1, &reg);
        bmCtrl->devSettings.sampleInterval = *dT;
    }
    return status;
}

void bmCtrlTask(void *args)
{
    /**  DONE: Message pump - dont really need - only actuve command is sample?
     *        task delay based on timing? 
     *        auto sample mode for latest?
     * **/
    bm_controlData_t *bmCtrl = (bm_controlData_t *)args;

    uint16_t sleep_time = 0;

    while (1)
    {

        if (bmCtrl->devSettings.sampleMode == BM_FORCE_MODE)
        {
            /** wait for task notify  to sample**/
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            ESP_LOGI(BM_DRIVER_TAG, "Sampling device");

            bm280_updateMeasurements(bmCtrl);
        }
        else if (bmCtrl->devSettings.sampleMode == BM_NORMAL_MODE)
        {
            uint8_t reg = 0;

            /** check the device isn't copying sample data... **/
            if (genericI2CReadFromAddress(bmCtrl->i2cChannel, bmCtrl->deviceAddress, BM_REG_ADDR_DEV_STATUS, 1, &reg) & BM_STATUS_MEASURE_MASK)
            {
                while (genericI2CReadFromAddress(bmCtrl->i2cChannel, bmCtrl->deviceAddress, BM_REG_ADDR_DEV_STATUS, 1, &reg) & BM_STATUS_MEASURE_MASK)
                {
                    vTaskDelay(pdMS_TO_TICKS(wait_sample_fin));
                }
            }
            bm280_updateMeasurements(bmCtrl);
            sleep_time = bm280_sleepTime(bmCtrl);

            /** sleep until next measurement - this can get busy at high frequencies **/
            /** TODO: Test at high frequency sampling **/
            vTaskDelay(pdMS_TO_TICKS(sleep_time));
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(wait_idle));
        }
    }
}

bm_controlData_t *bm280_init(bm_initData_t *initData)
{
    esp_err_t initStatus = ESP_OK;

    /** assign some heap memory for the control structure **/

    bm_controlData_t *bmCtrl = (bm_controlData_t *)calloc(1, sizeof(bm_controlData_t));
    if (bmCtrl == NULL)
    {
        ESP_LOGE(BM_DRIVER_TAG, "Error in assigning control structure memory!");
        initStatus = ESP_ERR_NO_MEM;
    }

    if (initStatus == ESP_OK)
    {
        bmCtrl->devSettings.sampleType = initData->sampleType;
        bmCtrl->devSettings.sampleMode = initData->sampleMode;
        bmCtrl->devSettings.humidOS = 0;
        bmCtrl->devSettings.tempOS = 0;
        bmCtrl->devSettings.pressOS = 0;

        bmCtrl->i2cChannel = initData->i2cChannel;

        if (initData->addressPinState)
        {
            bmCtrl->deviceAddress = (uint8_t)BM_I2C_ADDRESS_SDHIGH;
        }
        else
        {
            bmCtrl->deviceAddress = (uint8_t)BM_I2C_ADDRESS_SDLOW;
        }

        ESP_LOGI(BM_DRIVER_TAG, "Device address - %u", bmCtrl->deviceAddress);
    }

    //bm280_debugPrintRegs(bmCtrl);

    if (initStatus == ESP_OK)
    {
        initStatus = bm280_getCalibrationData(bmCtrl);
        if (initStatus == ESP_OK)
        {
            ESP_LOGI(BM_DRIVER_TAG, "Succesfully got the calibration data!");
        }
        else
        {
            ESP_LOGI(BM_DRIVER_TAG, "Error getting Calibration data!");
        }
    }

    if (initStatus == ESP_OK)
    {
        initStatus = bm280_InitDeviceSettings(bmCtrl);
        if (initStatus == ESP_OK)
        {
            ESP_LOGI(BM_DRIVER_TAG, "Succesfully wrote device settings data!");
        }
        else
        {
            ESP_LOGI(BM_DRIVER_TAG, "Error writing device settings!");
        }
    }

    bm280_getDeviceStatus(bmCtrl);
    xTaskCreate(bmCtrlTask, "bmCtrlTask", 5012, (void *)bmCtrl, 3, NULL);

    return bmCtrl;
}