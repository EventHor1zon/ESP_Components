/****************************************
* \file     BM280_Driver.h
* \brief    Header file for the BMP280/BME280 bosch temperature sensors
*           Driver.
* \date     July 2020 
* \author   RJAM
****************************************/

#ifndef BM280_DRIVER_H
#define BM280_DRIVER_H

#include "sdkconfig.h"

#define BME_280

#ifdef CONFIG_USE_PERIPH_MANAGER
/** PM structures **/
#include "CommandAPI.h"
#define bm_param_len 12
const parameter_t bm_param_map[bm_param_len];
const peripheral_t bme_peripheral_template;

#endif

/********* Includes ********************/

#include "esp_types.h"



/********* Definitions *****************/


#define BM_I2C_ADDRESS_SDLOW 0x76
#define BM_I2C_ADDRESS_SDHIGH 0x77

#define BM_TRANSACTION_READ_BIT 0x01

#define BM_REG_ADDR_DIGT1_LSB 0x88
#define BM_REG_ADDR_DIGT1_MSB 0x89
#define BM_REG_ADDR_DIGT2_LSB 0x8A
#define BM_REG_ADDR_DIGT2_MSB 0x8B
#define BM_REG_ADDR_DIGT3_LSB 0x8C
#define BM_REG_ADDR_DIGT3_MSB 0x8D
#define BM_REG_ADDR_DIGP1_LSB 0x8E
#define BM_REG_ADDR_DIGP1_MSB 0x8F
#define BM_REG_ADDR_DIGP2_LSB 0x90
#define BM_REG_ADDR_DIGP2_MSB 0x91
#define BM_REG_ADDR_DIGP3_LSB 0x92
#define BM_REG_ADDR_DIGP3_MSB 0x93
#define BM_REG_ADDR_DIGP4_LSB 0x94
#define BM_REG_ADDR_DIGP4_MSB 0x95
#define BM_REG_ADDR_DIGP5_LSB 0x96
#define BM_REG_ADDR_DIGP5_MSB 0x97
#define BM_REG_ADDR_DIGP6_LSB 0x98
#define BM_REG_ADDR_DIGP6_MSB 0x99
#define BM_REG_ADDR_DIGP7_LSB 0x9A
#define BM_REG_ADDR_DIGP7_MSB 0x9B
#define BM_REG_ADDR_DIGP8_LSB 0x9C
#define BM_REG_ADDR_DIGP8_MSB 0x9D
#define BM_REG_ADDR_DIGP9_LSB 0x9E
#define BM_REG_ADDR_DIGP9_MSB 0x9F
#define BM_REG_ADDR_DIGH1 0xA1

#define BM_REG_ADDR_DEVICEID 0xD0
#define BM_REG_ADDR_SOFTRESET 0xE0

#define BM_REG_ADDR_DIGH2_LSB 0xE1
#define BM_REG_ADDR_DIGH2_MSB 0xE2
#define BM_REG_ADDR_DIGH3 0xE3
#define BM_REG_ADDR_DIGH4_LSB 0xE4
#define BM_REG_ADDR_DIGH4_MSB 0xE5
#define BM_REG_ADDR_DIGH5_LSB 0xE5
#define BM_REG_ADDR_DIGH5_MSB 0xE6
#define BM_REG_ADDR_DIGH6 0xE7

#define BM_REG_ADDR_CTRL_HUMID 0xF2
#define BM_REG_ADDR_DEV_STATUS 0xF3
#define BM_REG_ADDR_CTRL_MEASURE 0xF4
#define BM_REG_ADDR_CONFIG 0xF5
#define BM_REG_ADDR_PRESSURE_MSB 0xF7
#define BM_REG_ADDR_PRESSURE_LSB 0xF8
#define BM_REG_ADDR_PRESSURE_XLSB 0xF9
#define BM_REG_ADDR_TEMP_MSB 0xFA
#define BM_REG_ADDR_TEMP_LSB 0xFB
#define BM_REG_ADDR_TEMP_XLSB 0xFC
#define BM_REG_ADDR_HUMIDITY_MSB 0xFD
#define BM_REG_ADDR_HUMIDITY_LSB 0xFE

#ifdef BME_280
#define BM_CALIBR_DATA_BANK1_LEN 26
#define BM_CALIBR_DATA_BANK2_LEN 7
#define BM_CALIBR_DATA_LEN 32
#define BM_CONFIG_WRITE_LEN 3
#define BM_MEASURE_READ_LEN 8
#else
#define BM_CALIBR_DATA_LEN 24
#define BM_CONFIG_WRITE_LEN 2
#define BM_MEASURE_READ_LEN 6
#endif

#ifdef BME_280
#define DEVICE_ID 0x60
#else
#define DEVICE_ID 0x58
#endif

#define BMP_DEVICE_ID 0x58
#define BME_DEVICE_ID 0x60

#define BM_STATUS_IM_UPDATE_BIT 0
#define BM_STATUS_MEASURE_BIT (4)
#define BM_STATUS_MEASURE_MASK (1 << 3)
#define BM_CTRL_TEMP_BIT (1 << 5)
#define BM_CTRL_PRESSURE_BIT (1 << 2)
#define BM_CTRL_MODE_FORCED 1

#define BM_STATUS_UPDATE_MASK 0x01

#define BM_DRIVER_I2C_TRX_TIMEOUT 100


#ifdef DEBUG_MODE
#define DEBUG_I2C_CLOCK_PIN 25
#define DEBUG_I2C_DATA_PIN 26
#define DEBUG_I2C_CHANNEL 0
#endif

/** defaults **/
#define BM_DEFAULT_HUM_CTRL 0x01
#define BM_DEFAULT_TEMP_CTRL 0x01
#define BM_DEFAULT_T_STDBY (1 << 5)

/********** Types **********************/




typedef enum bm_devTypes
{
    BMP_280_DEVICE,
    BME_280_DEVICE

} BM_deviceType_t;

typedef enum bm_oversampling
{
    BM_OS_0 = 0x00,
    BM_OS_1 = 0x01,
    BM_OS_2 = 0x02,
    BM_OS_4 = 0x03,
    BM_OS_8 = 0x04,
    BM_OS_16 = 0x05
} BM_overSample_t;

typedef enum bme_sampleMode
{
    BM_SAMPLE_OFF = 0x00, /**< sampling off **/
    BM_FORCE_MODE = 0x01, /**< sample on demand **/
    BM_NORMAL_MODE = 0x03 /**< sample at interval **/
} BM_sampleMode_t;

typedef enum bme_standbyT
{
    BM_T_STDBY_0_5MS = 0x00,
    BM_T_STDBY_62_5MS = 0x01,
    BM_T_STDBY_125MS = 0x02,
    BM_T_STDBY_250MS = 0x03,
    BM_T_STDBY_500MS = 0x04,
    BM_T_STDBY_1000MS = 0x05,
#ifdef BME_280
    BM_T_STDBY_10MS = 0x06,
    BM_T_STDBY_20MS = 0x07,
#else
    BM_T_STDBY_2000MS = 0x06,
    BM_T_STDBY_4000MS = 0x07,
#endif
    BM_T_STDBY_END
} BM_standbyT_t;

/** bm_filter_t
 *  -   controls filter setting 
 **/
typedef enum bm_Filter
{
    BM_FILTER_OFF = 0,
    BM_FILTER_2 = 1,
    BM_FILTER_4 = 2,
    BM_FILTER_8 = 3,
    BM_FILTER_16 = 4,
    BM_FILTER_END
} bm_filter_t;

/** bm_sampleType_t
 *  -   sampling done by the device 
 *  -   iff sampling hum/pres must sample temp
 **/
typedef enum bm_sampleTypes
{
    BM_MODE_OFF = 0x00,
    BM_MODE_TEMP = 0x01,
    BM_MODE_PRESSURE = 0x02,
    BM_MODE_TEMP_PRESSURE = 0x03,
#ifdef BME_280
    BM_MODE_HUMIDITY = 0x04,
    BM_MODE_TEMP_HUMIDITY = 0x05,
    BM_MODE_TEMP_PRESSURE_HUMIDITY = 0x07
#endif
} bm_sampleType_t;

typedef struct BM_CalibrationData
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
#ifdef BME_280
    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    int8_t dig_H6;
#endif
    uint32_t t_fine; /** < fine temperature used in calibration */

} bm_calibrationData_t;

typedef struct BM_sensorData
{

    float realTemperature;          /** < real temperature in degrees C */
    uint32_t calibratedTemperature; /** < output of the temp calibration */
    uint32_t rawTemperature;        /** < output of the device register */

    float realPressure;          /** < real pressure in hPa */
    uint32_t calibratedPressure; /** < output of the pressure calibration */
    uint32_t rawPressure;        /** < output of the device register */

#ifdef BME_280
    float realHumidity;          /** < real humidity in %RH */
    uint32_t calibratedHumidity; /** < "    "  "    " humidity calibration */
    uint32_t rawHumidity;        /** < output of the device register */
#endif

    uint8_t statusMeasure;
    uint8_t statusUpdate;

} bm_sensorData_t;

typedef struct bm_initData
{
    BM_deviceType_t devType;
    bm_sampleType_t sampleType;
    BM_sampleMode_t sampleMode;

    bool addressPinState;
    uint8_t i2cChannel; /** < 0 - no i2c initialised, driver will init. 1 | 2, valid i2c channels */

} bm_initData_t;

typedef struct bm_deviceSettings
{
    bm_sampleType_t sampleType;
    BM_sampleMode_t sampleMode;
    BM_overSample_t tempOS;
    BM_overSample_t pressOS;
    BM_overSample_t humidOS;
    BM_standbyT_t sampleInterval;
    bm_filter_t filterCoefficient;
} bm_deviceSettings_t;

typedef struct bm_controlData
{
    bm_calibrationData_t calibrationData;   /**< the device calibrationdata **/
    bm_sensorData_t sensorData;             /**< the device sensor data **/
    bm_deviceSettings_t devSettings;        /**< the device settings **/
    bm_initData_t *initData;    /**< pointer to the init data **/

    uint8_t peripheralID;   /**< the periph_id **/
    uint8_t deviceAddress; /**< the device's I2C address **/
    uint8_t i2cChannel; /**< the i2c channel used **/

    bool calibrationAquired;    /**< calibration data aquired  **/
    uint8_t sampleMask; /**< sample mask **/
    uint8_t configMask; /**< config mask **/
#ifdef CONFIG_USE_PERIPH_MANAGER
    peripheral_t *peripheral_template;
#endif


} bm_controlData_t;

/******** Function Definitions *********/

/** \brief bm280_init
 *         initialises the device driver & returns a device handle
 *  \param initData - of type bm_initData_t, initial device settings
 *  \return pointer to device handle, or NULL
*/
bm_controlData_t *bm280_init(bm_initData_t *initData);

/** \brief getSampleInterval - returns the sample interval. Matters in 
 *          auto mode only. See enum bm_standbyT_t for int > time
 *  \param  bmCtrl - device handle
 *  \param  dT  - pointer to integer space
 *  \return ESP_OK or FAIL
 **/
esp_err_t bm280_getSampleInterval(bm_controlData_t *bmCtrl, uint8_t *dT);

/** \brief setSampleInterval - sets the sample interval time. Only for auto mode
 *  \param  bmCtrl - device handle
 *  \param  dT - the time interval - see enum bm_standbyT_ts
 * \return  ESP_OK or FAIL
 **/
esp_err_t bm280_setSampleInterval(bm_controlData_t *bmCtrl, BM_standbyT_t *dT);

/** \brief getFilterSetting - gets the current filter level
 *  \param  bmCtrl - device handle
 *  \param  filter - storage for return value
 * \return  ESP_OK or FAIL
 **/
esp_err_t bm280_getFilterSetting(bm_controlData_t *bmCtrl, uint8_t *filter);

/** \brief setFilterSetting - set the filter level
 *  \param  bmCtrl - device handle
 *  \param  filter - pointer to filter value
 * \return  ESP_OK or FAIL
 **/
esp_err_t bm280_setFilterSetting(bm_controlData_t *bmCtrl, bm_filter_t *filter);

/** \brief  updateMeasurements - read measurements from device
 *  \param  bmCtrl - device handle
 * \return  ESP_OK or FAIL
 **/
esp_err_t bm280_updateMeasurements(bm_controlData_t *bmCtrl);

/** \brief  getDeviceID - get the device ID - should be 0x60
 *  \param  bmCtrl - device handle
 *  \param  deviceId - storage for return value
 * \return  ESP_OK or FAIL
 **/
esp_err_t bm280_getDeviceID(bm_controlData_t *bmCtrl, uint8_t *deviceID);

/** \brief  getTemperature - return the latest temperature measurement retrieved - in degreesC
 *  \param  bmCtrl - device handle
 *  \param  realTemp - storage for return value
 * \return  ESP_OK or FAIL
 **/
esp_err_t bm280_getTemperature(bm_controlData_t *bmCtrl, float *realTemp);

/** \brief  getPressure = returns latest pressure measurement retrieved - in hPa
 *  \param  bmCtrl - device handle
 *  \param  realPressure - storage for return value
 * \return  ESP_OK or FAIL
 **/
esp_err_t bm280_getPressure(bm_controlData_t *bmCtrl, float *realPressure);


#ifdef BME_280
/** \brief  getHumidity - gets the humidity measuremennt retrieved: in %
 *  \param  bmCtrl - device handle
 *  \param   realHumidity- storage for return value
 * \return  ESP_OK or FAIL
 **/
esp_err_t bm280_getHumidity(bm_controlData_t *bmCtrl, float *realHumidity);

/** \brief  getHumidityOS - get the sample rate of humidity
 *  \param  bmCtrl - device handle
 *  \param  humidOS - storage for return value
 * \return  ESP_OK or FAIL
 **/
esp_err_t bm280_getHumidityOS(bm_controlData_t *bmCtrl, uint8_t *humidOS);

/** \brief setHumidityOS - set the sample rate of humidity
 *  \param bmCtrl - handle
 *  \param os       value to set
 *  \return ESP_OK or FAIL
 **/
esp_err_t bm280_setHumidityOS(bm_controlData_t *bmCtrl, BM_overSample_t *os);
#endif

esp_err_t bm280_getTemperatureOS(bm_controlData_t *bmCtrl, uint8_t *tempOS);
esp_err_t bm280_getPressureOS(bm_controlData_t *bmCtrl, uint8_t *presOS);

esp_err_t bm280_setTemperatureOS(bm_controlData_t *bmCtrl, BM_overSample_t *os);
esp_err_t bm280_setPressureOS(bm_controlData_t *bmCtrl, BM_overSample_t *os);

esp_err_t bm280_getSampleMode(bm_controlData_t *bmCtrl, uint8_t *sampleMode);
esp_err_t bm280_setSampleMode(bm_controlData_t *bmCtrl, BM_sampleMode_t *sampleMode);

esp_err_t bm280_getSampleType(bm_controlData_t *bmCtrl, uint8_t *sampleType);

esp_err_t bm280_getFilterSetting(bm_controlData_t *bmCtrl, uint8_t *filter);
esp_err_t bm280_setFilterSetting(bm_controlData_t *bmCtrl, bm_filter_t *filter);


#endif /* BM280_DRIVER_H */
