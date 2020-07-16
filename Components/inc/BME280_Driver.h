/****************************************
* \file     BME280_Driver.h
* \brief    Header file for the BME280 bosch temperature sensor
*           Driver.
* \date     July 2020 
* \author   RJAM
****************************************/

#ifndef BME280_DRIVER_H
#define BME280_DRIVER_H

/********* Includes ********************/

#include <stdint.h>

/********* Definitions *****************/

#define BMP_I2C_ADDRESS_SDLOW 0x76
#define BMP_I2C_ADDRESS_SDHIGH 0x77

#define BMP_TRANSACTION_READ_BIT 0x01

#define BMP_REG_ADDR_DIGT1_LSB 0x88
#define BMP_REG_ADDR_DIGT1_MSB 0x89
#define BMP_REG_ADDR_DIGT2_LSB 0x8A
#define BMP_REG_ADDR_DIGT2_MSB 0x8B
#define BMP_REG_ADDR_DIGT3_LSB 0x8C
#define BMP_REG_ADDR_DIGT3_MSB 0x8D
#define BMP_REG_ADDR_DIGP1_LSB 0x8E
#define BMP_REG_ADDR_DIGP1_MSB 0x8F
#define BMP_REG_ADDR_DIGP2_LSB 0x90
#define BMP_REG_ADDR_DIGP2_MSB 0x91
#define BMP_REG_ADDR_DIGP3_LSB 0x92
#define BMP_REG_ADDR_DIGP3_MSB 0x93
#define BMP_REG_ADDR_DIGP4_LSB 0x94
#define BMP_REG_ADDR_DIGP4_MSB 0x95
#define BMP_REG_ADDR_DIGP5_LSB 0x96
#define BMP_REG_ADDR_DIGP5_MSB 0x97
#define BMP_REG_ADDR_DIGP6_LSB 0x98
#define BMP_REG_ADDR_DIGP6_MSB 0x99
#define BMP_REG_ADDR_DIGP7_LSB 0x9A
#define BMP_REG_ADDR_DIGP7_MSB 0x9B
#define BMP_REG_ADDR_DIGP8_LSB 0x9C
#define BMP_REG_ADDR_DIGP8_MSB 0x9D
#define BMP_REG_ADDR_DIGP9_LSB 0x9E
#define BMP_REG_ADDR_DIGP9_MSB 0x9F
#define BMP_REG_ADDR_DIGH1 0xA1

#define BMP_REG_ADDR_DEVICEID 0xD0
#define BMP_REG_ADDR_SOFTRESET 0xE0

#define BMP_REG_ADDR_DIGH2_LSB 0xE1
#define BMP_REG_ADDR_DIGH2_MSB 0xE2
#define BMP_REG_ADDR_DIGH3 0xE3
#define BMP_REG_ADDR_DIGH4_LSB 0xE4
#define BMP_REG_ADDR_DIGH4_MSB 0xE5
#define BMP_REG_ADDR_DIGH5_LSB 0xE5
#define BMP_REG_ADDR_DIGH5_MSB 0xE6
#define BMP_REG_ADDR_DIGH6 0xE7

#define BMP_REG_ADDR_CTRL_HUMID 0xF2
#define BMP_REG_ADDR_DEV_STATUS 0xF3
#define BMP_REG_ADDR_CTRL_MEASURE 0xF4
#define BMP_REG_ADDR_CONFIG 0xF5
#define BMP_REG_ADDR_PRESSURE_MSB 0xF7
#define BMP_REG_ADDR_PRESSURE_LSB 0xF8
#define BMP_REG_ADDR_PRESSURE_XLSB 0xF9
#define BMP_REG_ADDR_TEMP_MSB 0xFA
#define BMP_REG_ADDR_TEMP_LSB 0xFB
#define BMP_REG_ADDR_TEMP_XLSB 0xFC
#define BMP_REG_ADDR_HUMIDITY_MSB 0xFD
#define BMP_REG_ADDR_HUMIDITY_LSB 0xFE

#define BMP_CALIBR_DATA_BANK1_LEN 26
#define BMP_CALIBR_DATA_BANK2_LEN 8
#define BMP_CALIBR_DATA_LEN 32

#define BMP_DEVICE_ID 0x60
#define BMP_STATUS_IM_UPDATE_BIT 0
#define BMP_STATUS_MEASURE_BIT (4)
#define BMP_STATUS_UPDATE_MASK 0x01
#define BMP_STATUS_MEASURE_MASK 0b00010000

/**
 *  E4:   -  digH4[11:4]
 *  E5:   -  digH4[3:0]
 **/

/********** Types **********************/

typedef enum bme_oversampling
{
    BME_OS_0 = 0,
    BME_OS_1 = 0x01,
    BME_OS_2 = 0x02,
    BME_OS_4 = 0x03,
    BME_OS_8 = 0x04,
    BME_OS_16 = 0x05
} BME_overSample_t;

typedef enum bme_sampleMode
{
    BME_SLEEP_MODE = 0x00,
    BME_FORCE_MODE = 0x01,
    BME_NORMAL_MODE = 0x03
} BME_sampleMode_t;

typedef enum bme_standbyT
{
    BME_T_STDBY_0_5MS = 0x00,
    BME_T_STDBY_62_5MS = 0x01,
    BME_T_STDBY_125MS = 0x02,
    BME_T_STDBY_250MS = 0x03,
    BME_T_STDBY_500MS = 0x04,
    BME_T_STDBY_1000MS = 0x05,
    BME_T_STDBY_10MS = 0x06,
    BME_T_STDBY_20MS = 0x07
} BME_standbyT_t;

typedef enum bme_modes
{
    BME_MODE_TEMP,
    BME_MODE_TEMP_HUMIDITY,
    BME_MODE_PRESSURE,
    BME_MODE_ALL
} BME_sampleTypes_t;

typedef struct BME_CalibrationData
{
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    int8_t dig_H6;
} BME_calibrationData_t;

typedef struct BME_sensorData
{

    uint32_t calibratedTemperature; /** < output of the temp calibration */
    uint32_t calibratedPressure;    /** < output of the pressure calibration */
    uint32_t calibratedHumidity;    /** < "    "  "    " humidity calibration */

    float realTemperature; /** < real temperature in degrees C */
    float realPressure;    /** < real pressure in hPa */
    float realHumidity;    /** < real humidity in %RH */

    uint32_t t_fine; /** < fine temperature used in calibration */

    uint8_t statusMeasure;
    uint8_t statusUpdate;

} BME_sensorData_t;

typedef struct BME280_controlData
{
    BME_calibrationData_t calibrationData;
    BME_sensorData_t sensorData;
    BME_sampleTypes_t sampleType;
    BME_sampleMode_t sampleMode;

    uint8_t peripheralID;
    uint8_t deviceAddress;
    uint8_t i2cChannel;

} BME280_controlData_t;

typedef struct bme_initData
{
    BME_sampleTypes_t sampleType;
    bool addressPinState;
    uint8_t i2cChannel; /** < 0 - no i2c initialised, driver will init. 1 | 2, valid i2c channels */

} bme_initData_t;

/******** Function Definitions *********/

#endif /* BME280_DRIVER_H */
