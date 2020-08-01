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
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"

#include "BME280_Driver.h"

/****** Function Prototypes ***********/
static int32_t bm280_compensate_T_int32(bm_controlData_t *bmCtrl);
static uint32_t bm280_compensate_P_int64(bm_controlData_t *bmCtrl);
#ifdef BME_280
static uint32_t bm280_compensate_H_int32(bm_controlData_t *bmCtrl);
#endif
static esp_err_t bm280_debugPrintRegs(bm_controlData_t *bmCtrl);

/************ ISR *********************/

/****** Global Data *******************/

#define DEBUG

const char *BM_DRIVER_TAG = "BME280 DRIVER::";
/****** Private Functions *************/

/** Calibration Functions :  The following calibration fucntions are adapted from the Bosch BME280 Data sheet **/

/* Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC. */
/* bmCtrl->sensorData.t_fine carries fine temperature as global value int32_t bmCtrl->sensorData.t_fine; */
static int32_t bm280_compensate_T_int32(bm_controlData_t *bmCtrl)
{
    if (bmCtrl->calibrationAquired)
    {
        int32_t adc_T = bmCtrl->sensorData.rawTemperature;
        int32_t var1, var2, T;

        var1 = ((((adc_T >> 3) - ((int32_t)bmCtrl->calibrationData.dig_T1 << 1))) * ((int32_t)bmCtrl->calibrationData.dig_T2)) >> 11;
        var2 = (((((adc_T >> 4) - ((int32_t)bmCtrl->calibrationData.dig_T1)) * ((adc_T >> 4) - ((int32_t)bmCtrl->calibrationData.dig_T1))) >> 12) * ((int32_t)bmCtrl->calibrationData.dig_T3)) >> 14;
        bmCtrl->sensorData.t_fine = var1 + var2;
        T = (bmCtrl->sensorData.t_fine * 5 + 128) >> 8;
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
        var1 = ((int64_t)bmCtrl->sensorData.t_fine) - 128000;
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

/** bm280_compensate_H_int32: Compensate Humidity Data **/
#ifdef BME_280
static uint32_t bm280_compensate_H_int32(bm_controlData_t *bmCtrl)
{
    if (bmCtrl->calibrationAquired)
    {
        int32_t adc_H = bmCtrl->sensorData.rawHumidity;
        int32_t v_x1_u32r;
        v_x1_u32r = (bmCtrl->sensorData.t_fine - ((int32_t)76800));
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

/** Generic I2C write to address function **/
static esp_err_t bm280_i2cWriteToAddress(bm_controlData_t *bmCtrl, uint8_t regAddress, uint16_t writeLength, uint8_t *txBuffer)
{

    esp_err_t trxStatus;
    uint8_t commands[2];

    /** TODO: Find out if write bit needed or automatic **/
    commands[0] = bmCtrl->deviceAddress << 1 | I2C_MASTER_WRITE;
    commands[1] = regAddress;

#ifdef DEBUG
    ESP_LOGI(BM_DRIVER_TAG, "in i2c write to address: device Address is: 0x%02x regAddr is: 0x%02x", bmCtrl->deviceAddress, regAddress);
    printf("Writing %d byes: ", writeLength);
    for (uint8_t i = 0; i < writeLength; i++)
    {
        printf("%u ", txBuffer[i]);
    }
    printf("\n");
#endif

    i2c_cmd_handle_t cmdHandle = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmdHandle));
    ESP_ERROR_CHECK(i2c_master_write(cmdHandle, commands, 2, true));
    ESP_ERROR_CHECK(i2c_master_write(cmdHandle, txBuffer, writeLength, true));
    ESP_ERROR_CHECK(i2c_master_stop(cmdHandle));
    trxStatus = i2c_master_cmd_begin(bmCtrl->i2cChannel, cmdHandle, pdMS_TO_TICKS(BM_DRIVER_I2C_TRX_TIMEOUT));
    i2c_cmd_link_delete(cmdHandle);

    return trxStatus;
}

static esp_err_t bm280_i2cReadFromAddress(bm_controlData_t *bmCtrl, uint8_t regAddress, uint16_t readLength, uint8_t *rxBuffer)
{
    /** TODO: replace this with a generic i2c readfromregister function **/
    esp_err_t trxStatus;
    uint8_t chip_addr = bmCtrl->deviceAddress;

#ifdef DEBUG
    ESP_LOGI(BM_DRIVER_TAG, "in readFromAddress device Address is: 0x%02x regAddr is: 0x%02x", chip_addr, regAddress);
#endif
    /** write address **/
    i2c_cmd_handle_t cmdHandle = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmdHandle));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmdHandle, chip_addr << 1 | I2C_MASTER_WRITE, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmdHandle, regAddress, 1));
    ESP_ERROR_CHECK(i2c_master_start(cmdHandle));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmdHandle, (chip_addr << 1 | I2C_MASTER_READ), 1));
    if (readLength > 1)
    {
        ESP_ERROR_CHECK(i2c_master_read(cmdHandle, rxBuffer, (readLength - 1), 0));
    }
    ESP_ERROR_CHECK(i2c_master_read_byte(cmdHandle, rxBuffer + (readLength - 1), 1));
    ESP_ERROR_CHECK(i2c_master_stop(cmdHandle));
    trxStatus = i2c_master_cmd_begin(bmCtrl->i2cChannel, cmdHandle, 50 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmdHandle);

    if (trxStatus != ESP_OK)
    {
        ESP_LOGE(BM_DRIVER_TAG, "Error transmitting: %u", trxStatus);
    }

    return trxStatus;
}

static esp_err_t bm280_debugPrintRegs(bm_controlData_t *bmCtrl)
{

    uint8_t id = 0, ctrl_meas = 0, config = 0;
    uint8_t measure[6] = {0};

    bm280_i2cReadFromAddress(bmCtrl, BM_REG_ADDR_DEVICEID, 1, &id);
    bm280_i2cReadFromAddress(bmCtrl, BM_REG_ADDR_CTRL_MEASURE, 1, &ctrl_meas);
    bm280_i2cReadFromAddress(bmCtrl, BM_REG_ADDR_CONFIG, 1, &config);
    bm280_i2cReadFromAddress(bmCtrl, BM_REG_ADDR_PRESSURE_MSB, 6, measure);

    ESP_LOGI(BM_DRIVER_TAG, "DEVICE-ID: Reg: 0x%02x\t -  0x%02x - %u\n", BM_REG_ADDR_DEVICEID, id, id);
    ESP_LOGI(BM_DRIVER_TAG, "CTRLMESRE: Reg: 0x%02x\t - 0x%02x - %u\n", BM_REG_ADDR_CTRL_MEASURE, ctrl_meas, ctrl_meas);
    ESP_LOGI(BM_DRIVER_TAG, "CONFIG   :Reg: 0x%02x\t - 0x%02x - %u\n", BM_REG_ADDR_CONFIG, config, config);
    for (uint8_t i = 0; i < 6; i++)
    {
        ESP_LOGI(BM_DRIVER_TAG, "MEASURE: Reg 0x%02x\t - 0x%02x - %u\n", BM_REG_ADDR_PRESSURE_MSB + i, measure[i], measure[i]);
    }
    return ESP_OK;
}
static esp_err_t bm280_getDeviceStatus(bm_controlData_t *bmCtrl)
{
    esp_err_t trxStatus = ESP_OK;

    uint8_t statusBuffer = 0;
    trxStatus = bm280_i2cReadFromAddress(bmCtrl, BM_REG_ADDR_DEV_STATUS, 1, &statusBuffer);
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

    uint8_t buffer[BM_CALIBR_DATA_LEN + 1] = {0};

#ifdef BME_280
    /** retrieve the calibration data - add +1 to length of bank1 as there's an unused byte (A0, index[24]) in there **/
    trxStatus = bm280_i2cReadFromAddress(bmCtrl, (uint8_t)BM_REG_ADDR_DIGT1_LSB, BM_CALIBR_DATA_BANK1_LEN + 1, buffer);
    trxStatus = bm280_i2cReadFromAddress(bmCtrl, (uint8_t)BM_REG_ADDR_DIGH2_LSB, BM_CALIBR_DATA_BANK2_LEN, &buffer[BM_CALIBR_DATA_BANK1_LEN + 1]);
#else
    trxStatus = bm280_i2cReadFromAddress(bmCtrl, (uint8_t)BM_REG_ADDR_DIGT1_LSB, BM_CALIBR_DATA_LEN, buffer);
#endif
    /** now for some rejigging... **/

    if (trxStatus == ESP_OK)
    {
        bmCtrl->calibrationData.dig_T1 = (uint16_t)buffer[0] << 8 | (uint16_t)buffer[1];
        bmCtrl->calibrationData.dig_T2 = (int16_t)buffer[2] << 8 | (int16_t)buffer[3];
        bmCtrl->calibrationData.dig_T3 = (int16_t)buffer[4] << 8 | (int16_t)buffer[5];
        bmCtrl->calibrationData.dig_P1 = (uint16_t)buffer[6] << 8 | (uint16_t)buffer[7];
        bmCtrl->calibrationData.dig_P2 = (int16_t)buffer[8] << 8 | (int16_t)buffer[9];
        bmCtrl->calibrationData.dig_P3 = (int16_t)buffer[10] << 8 | (int16_t)buffer[11];
        bmCtrl->calibrationData.dig_P4 = (int16_t)buffer[12] << 8 | (int16_t)buffer[13];
        bmCtrl->calibrationData.dig_P5 = (int16_t)buffer[14] << 8 | (int16_t)buffer[15];
        bmCtrl->calibrationData.dig_P6 = (int16_t)buffer[16] << 8 | (int16_t)buffer[17];
        bmCtrl->calibrationData.dig_P7 = (int16_t)buffer[18] << 8 | (int16_t)buffer[19];
        bmCtrl->calibrationData.dig_P8 = (int16_t)buffer[20] << 8 | (int16_t)buffer[21];
        bmCtrl->calibrationData.dig_P9 = (int16_t)buffer[22] << 8 | (int16_t)buffer[23];
#ifdef BME_280
        bmCtrl->calibrationData.dig_H1 = buffer[25];
        bmCtrl->calibrationData.dig_H2 = (int16_t)buffer[26] << 8 | (int16_t)buffer[27];
        bmCtrl->calibrationData.dig_H3 = buffer[28];
        bmCtrl->calibrationData.dig_H4 = (int16_t)buffer[28] << 8 | (int16_t)buffer[29];
        bmCtrl->calibrationData.dig_H5 = (int16_t)buffer[30] << 8 | (int16_t)buffer[31];
        bmCtrl->calibrationData.dig_H6 = (int8_t)buffer[32];
#endif
        bmCtrl->calibrationAquired = true;
    }
    else
    {
        ESP_LOGI(BM_DRIVER_TAG, "Error reading calibration data");
    }

    return trxStatus;
}

static esp_err_t bm280_getDeviceID(bm_controlData_t *bmCtrl, uint8_t *deviceID)
{

    esp_err_t trxStatus = ESP_OK;

    uint8_t devID = 0;

    trxStatus = bm280_i2cReadFromAddress(bmCtrl, (uint8_t)BM_REG_ADDR_DEVICEID, 1, &devID);

    ESP_LOGI(BM_DRIVER_TAG, "Got device ID: %02x", devID);
    if (trxStatus == ESP_OK)
    {
        *deviceID = devID;
    }

    return trxStatus;
}

static esp_err_t bm280_InitDeviceSettings(bm_controlData_t *bmCtrl, bm_initData_t *initData)
{

    esp_err_t trxStatus = ESP_OK;
    uint8_t sampleMode = initData->sampleMode;
    uint8_t sampleType = initData->sampleType;

    uint8_t commands[BM_CONFIG_WRITE_LEN] = {0};

    switch (sampleMode)
    {
    case BM_SAMPLE_OFF:
        break;
    case BM_FORCE_MODE:
        commands[0] |= (uint8_t)BM_FORCE_MODE;
        break;
    case BM_NORMAL_MODE:
        commands[0] |= (uint8_t)BM_NORMAL_MODE;
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
    case BM_MODE_PRESSURE:
        commands[0] |= BM_CTRL_PRESSURE_BIT;
        break;
#ifdef BME_280
    case BM_MODE_HUMIDITY:
        commands[2] = 1;
        break;
    case BM_MODE_HUMIDITY_PRESSURE:
        commands[2] = 1;
        commands[0] |= BM_CTRL_PRESSURE_BIT;
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
    trxStatus = bm280_i2cWriteToAddress(bmCtrl, BM_REG_ADDR_CTRL_HUMID, 1, &commands[2]);
#endif
    trxStatus = bm280_i2cWriteToAddress(bmCtrl, BM_REG_ADDR_CTRL_MEASURE, 2, &commands[0]);
    if (trxStatus != ESP_OK)
    {
        ESP_LOGE(BM_DRIVER_TAG, "Error in setting control registers");
    }
    else
    {
        bmCtrl->sampleMask = commands[0];
        bmCtrl->configMask = commands[1];
    }

    //bm280_debugPrintRegs(bmCtrl);

    return trxStatus;
}

/****** Global Functions *************/

esp_err_t bm280_setOverSampling(bm_controlData_t *bmCtrl)
{

    uint8_t data = 0xFF;
    esp_err_t status = bm280_i2cWriteToAddress(bmCtrl, BM_REG_ADDR_CTRL_MEASURE, 1, &data);
    return status;
}

esp_err_t bm280_updateMeasurements(bm_controlData_t *bmCtrl)
{

    esp_err_t trxStatus = ESP_OK;
    uint8_t forcedMeasure = bmCtrl->sampleMask | BM_CTRL_MODE_FORCED;
    uint8_t rxBuffer[BM_MEASURE_READ_LEN] = {0};

    if (bmCtrl->sampleMode == BM_FORCE_MODE)
    {
#ifdef DEBUG
        ESP_LOGI(BM_DRIVER_TAG, "Telling device to sample...");
#endif
        trxStatus = bm280_i2cWriteToAddress(bmCtrl, BM_REG_ADDR_CTRL_MEASURE, 1, &forcedMeasure);
        if (trxStatus != ESP_OK)
        {
            ESP_LOGE(BM_DRIVER_TAG, "Error in writing to mode");
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
#ifdef DEBUG
    //bm280_debugPrintRegs(bmCtrl);
    ESP_LOGI(BM_DRIVER_TAG, "Writing data %u", forcedMeasure);
#endif
    trxStatus = bm280_i2cReadFromAddress(bmCtrl, BM_REG_ADDR_PRESSURE_MSB, (uint16_t)BM_MEASURE_READ_LEN, rxBuffer);
#ifdef DEBUG
    //bm280_debugPrintRegs(bmCtrl);
    ESP_LOGI(BM_DRIVER_TAG, "Reading new data...");
#endif
    if (trxStatus == ESP_OK)
    {
        bmCtrl->sensorData.rawPressure = (uint32_t)rxBuffer[0] << 12 | (uint32_t)rxBuffer[1] << 4 | (uint32_t)rxBuffer[2] >> 4;
        bmCtrl->sensorData.rawTemperature = (uint32_t)rxBuffer[3] << 12 | (uint32_t)rxBuffer[4] << 4 | (uint32_t)rxBuffer[5] >> 4;
        bmCtrl->sensorData.calibratedPressure = bm280_compensate_P_int64(bmCtrl);

#ifdef BME_280
        bmCtrl->sensorData.rawHumidity = (uint32_t)rxBuffer[6] << 8 | (uint32_t)rxBuffer[7];
#endif
        switch (bmCtrl->sampleType)
        {
        case BM_MODE_TEMP:
            bmCtrl->sensorData.calibratedTemperature = bm280_compensate_T_int32(bmCtrl);
            bmCtrl->sensorData.realTemperature = (float)bmCtrl->sensorData.calibratedTemperature / 100.0;
            break;
        case BM_MODE_PRESSURE:
            bmCtrl->sensorData.calibratedPressure = bm280_compensate_P_int64(bmCtrl);
            bmCtrl->sensorData.realPressure = (float)bmCtrl->sensorData.calibratedPressure / 256;
            break;
        case BM_MODE_TEMP_PRESSURE:
            bmCtrl->sensorData.calibratedTemperature = bm280_compensate_T_int32(bmCtrl);
            bmCtrl->sensorData.calibratedPressure = bm280_compensate_P_int64(bmCtrl);
            bmCtrl->sensorData.realTemperature = (float)bmCtrl->sensorData.calibratedTemperature / 100.0;
            bmCtrl->sensorData.realPressure = (float)bmCtrl->sensorData.calibratedPressure / 256;
            break;
#ifdef BME_280
        case BM_MODE_TEMP_HUMIDITY:
            bmCtrl->sensorData.calibratedTemperature = bm280_compensate_T_int32(bmCtrl);
            bmCtrl->sensorData.realTemperature = (float)bmCtrl->sensorData.calibratedTemperature / 100.0;
            bmCtrl->sensorData.calibratedHumidity = bm280_compensate_H_int32(bmCtrl);
            bmCtrl->sensorData.realHumidity = (float)bmCtrl->sensorData.calibratedHumidity / 1024;
            break;
        case BM_MODE_HUMIDITY_PRESSURE:
            bmCtrl->sensorData.calibratedPressure = bm280_compensate_P_int64(bmCtrl);
            bmCtrl->sensorData.realPressure = (float)bmCtrl->sensorData.calibratedPressure / 256;
            bmCtrl->sensorData.calibratedHumidity = bm280_compensate_H_int32(bmCtrl);
            bmCtrl->sensorData.realHumidity = (float)bmCtrl->sensorData.calibratedHumidity / 1024;
            break;
        case BM_MODE_HUMIDITY:
            bmCtrl->sensorData.calibratedHumidity = bm280_compensate_H_int32(bmCtrl);
            bmCtrl->sensorData.realHumidity = (float)bmCtrl->sensorData.calibratedHumidity / 1024;
            break;
        case BM_MODE_TEMP_PRESSURE_HUMIDITY:
            bmCtrl->sensorData.calibratedTemperature = bm280_compensate_T_int32(bmCtrl);
            bmCtrl->sensorData.realTemperature = (float)bmCtrl->sensorData.calibratedTemperature / 100.0;
            bmCtrl->sensorData.calibratedPressure = bm280_compensate_P_int64(bmCtrl);
            bmCtrl->sensorData.realPressure = (float)bmCtrl->sensorData.calibratedPressure / 256;
            bmCtrl->sensorData.calibratedHumidity = bm280_compensate_H_int32(bmCtrl);
            bmCtrl->sensorData.realHumidity = (float)bmCtrl->sensorData.calibratedHumidity / 1024;
            break;
#endif
        default:
            break;
        }
#ifdef DEBUG
        ESP_LOGI(BM_DRIVER_TAG, "Deets: %ul %ul", bmCtrl->sensorData.rawTemperature, bmCtrl->sensorData.calibratedTemperature);
#endif
    }
    else
    {
        ESP_LOGE(BM_DRIVER_TAG, "Error in reading data %u", trxStatus);
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
        if (initData->i2cChannel == 0)
        {
            i2c_config_t i2cConf;
            /* init the i2c driver if not provided with an i2c driver handle */

            i2cConf.scl_io_num = DEBUG_I2C_CLOCK_PIN;
            i2cConf.sda_io_num = DEBUG_I2C_DATA_PIN;
            i2cConf.mode = I2C_MODE_MASTER;
            i2cConf.master.clk_speed = 100000;
            i2cConf.scl_pullup_en = GPIO_PULLUP_ENABLE;
            i2cConf.sda_pullup_en = GPIO_PULLUP_ENABLE;

            initStatus = i2c_param_config(DEBUG_I2C_CHANNEL, &i2cConf);
            if (initStatus != ESP_OK)
            {
                ESP_LOGE(BM_DRIVER_TAG, "Error in configuring the I2C driver - %u", initStatus);
            }

            if (initStatus == ESP_OK)
            {
                initStatus = i2c_driver_install(DEBUG_I2C_CHANNEL, I2C_MODE_MASTER, 0, 0, 0);
                if (initStatus != ESP_OK)
                {
                    ESP_LOGE(BM_DRIVER_TAG, "Error in installing the driver");
                }
            }
        }
        else
        {
            bmCtrl->i2cChannel = DEBUG_I2C_CHANNEL;
        }

        bmCtrl->sampleMode = initData->sampleMode;
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
    ESP_LOGI(BM_DRIVER_TAG, "Contacting device at address %02x", bmCtrl->deviceAddress);

    if (initStatus == ESP_OK)
    {
        uint8_t deviceID = 0;
        initStatus = bm280_getDeviceID(bmCtrl, &deviceID);
        if (deviceID == (uint8_t)DEVICE_ID)
        {
            ESP_LOGI(BM_DRIVER_TAG, "Device ID checks out!: 0x%02x", deviceID);
        }
        else
        {
            ESP_LOGI(BM_DRIVER_TAG, "Weird - device ID is different: 0x%02x", deviceID);
        }
    }

    bm280_debugPrintRegs(bmCtrl);

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
        initStatus = bm280_InitDeviceSettings(bmCtrl, initData);
        if (initStatus == ESP_OK)
        {
            ESP_LOGI(BM_DRIVER_TAG, "Succesfully wrote device settings data!");
        }
        else
        {
            ESP_LOGI(BM_DRIVER_TAG, "Error writing device settings!");
        }
    }

    return bmCtrl;
}