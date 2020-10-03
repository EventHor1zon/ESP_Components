/****************************************
* \file     LSM_Driver.h
* \brief    Header file for the LSM Driver
* \date     Aug 2020
* \author   RJAM
****************************************/
/*   the dev plan - do the i2c mode first 
*                 - cover the major actions
*                 - expose basic functionality
*                 - timer based read, interrupt based 
*                 - basic fifo modes 
*/

#ifndef LSM_DRIVER_H
#define LSM_DRIVER_H

/********* Includes ********************/

#include <stdint.h>
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/********* Definitions *****************/

/** i2c stuff **/
#define LSM_I2C_ADDR 0b1101010
#define LSM_I2C_SA_HIGH (1)
#define LSM_I2C_SA_LOW (0)
#define LSM_WHOAMI 0x69

#define LSM_FUNC_CFG_REG 0x01
#define LSM_SNES_SYNC_REG 0x04
#define LSM_FIFO_CTRL1_REG 0x06
#define LSM_FIFO_CTRL2_REG 0x07
#define LSM_FIFO_CTRL3_REG 0x08
#define LSM_FIFO_CTRL4_REG 0x09
#define LSM_FIFO_CTRL5_REG 0x0A

#define LSM_ORIENT_CFG_REG 0x0B
#define LSM_INT1_CTRL_REG 0x0D
#define LSM_INT2_CTRL_REG 0x0E
#define LSM_WHOAMI_REG 0x0F
#define LSM_CTRL1_XL_REG 0x10
#define LSM_CTRL2_G_REG 0x11
#define LSM_CTRL3_C_REG 0x12
#define LSM_CTRL4_C_REG 0x13
#define LSM_CTRL5_C_REG 0x14
#define LSM_CTRL6_C_REG 0x15
#define LSM_CTRL7_G_REG 0x16
#define LSM_CTRL8_XL_REG 0x17
#define LSM_CTRL9_XL_REG 0x18
#define LSM_CTRL10_C_REG 0x19
#define LSM_MASTER_CFG_REG 0x1A
#define LSM_WAKEUP_SRC_REG 0x1B
#define LSM_TAP_SRC_REG 0x1C
#define LSM_D6D_SRC_REG 0x1D
#define LSM_STATUS_REG 0x1E
#define LSM_TEMP_LSB_REG 0x20
#define LSM_TEMP_MSB_REG 0x21
#define LSM_GYROX_LSB_REG 0x22
#define LSM_GYROX_MSB_REG 0x23
#define LSM_GYROY_LSB_REG 0x24
#define LSM_GYROY_MSB_REG 0x25
#define LSM_GYROZ_LSB_REG 0x26
#define LSM_GYROZ_MSB_REG 0x27
#define LSM_ACCELX_LSB_REG 0x28
#define LSM_ACCELX_MSB_REG 0x29
#define LSM_ACCELY_LSB_REG 0x2A
#define LSM_ACCELY_MSB_REG 0x2B
#define LSM_ACCELZ_LSB_REG 0x2C
#define LSM_ACCELZ_MSB_REG 0x2D
/** SOME SENSORHUB REGISTERS HERE */
#define LSM_FIFO_STATUS1_REG 0x3A
#define LSM_FIFO_STATUS2_REG 0x3B
#define LSM_FIFO_STATUS3_REG 0x3C
#define LSM_FIFO_STATUS4_REG 0x3D
#define LSM_FIFO_DATA_LSB_REG 0x3E
#define LSM_FIFO_DATA_MSB_REG 0x3F
#define LSM_TIMESTAMP_0_REG 0x40
#define LSM_TIMESTAMP_1_REG 0x41
#define LSM_TIMESTAMP_2_REG 0x43
#define LSM_STEPTIME_LSB_REG 0x49
#define LSM_STEPTIME_MSB_REG 0x4A
#define LSM_STEPCOUNT_LSB_REG 0x4B
#define LSM_STEPCOUNT_MSB_REG 0x4C
/** more sensorhub registers here */
#define LSM_FUNC_SRC_REG 0x53
#define LSM_TAP_CFG_REG 0x58
#define LSM_TAP_THS_6D_REG 0x59
#define LSM_INT_DUR2_REG 0x5A
#define LSM_WKEUP_THS_REG 0x5B
#define LSM_WAKEUP_DUR_REG 0x5C
#define LSM_FREEFALL_REG 0x5D
#define LSM_MD1_CFG_REG 0x5E
#define LSM_MD2_CFG_REG 0x5F

/** int1 ctrl reg **/

#define LSM_INT1CTRL_STEP_ISR_BIT (1 << 7)
#define LSM_INT1CTRL_SGFT_MTN_ISR_BIT (1 << 6)
#define LSM_INT1CTRL_FIFOFULL_ISR_BIT (1 << 5)
#define LSM_INT1CTRL_FIFOOVRN_ISR_BIT (1 << 4)
#define LSM_INT1CTRL_FIFOTHRSHLD_ISR_BIT (1 << 3)
#define LSM_INT1CTRL_BOOTSTATUS_ISR_BIT (1 << 2)
#define LSM_INT1CTRL_GYRO_DTRDY_ISR_BIT (1 << 1)
#define LSM_INT1CTRL_ACCEL_DTRDY_ISR_BIT (1)

/** intcrtl 2 reg **/

#define LSM_INT2CTRL_STEP_ISR_BIT (1 << 7)
#define LSM_INT2CTRL_SGFT_MTN_ISR_BIT (1 << 6)
#define LSM_INT2CTRL_FIFOFULL_ISR_BIT (1 << 5)
#define LSM_INT2CTRL_FIFOOVRN_ISR_BIT (1 << 4)
#define LSM_INT2CTRL_FIFOTHRSHLD_ISR_BIT (1 << 3)
#define LSM_INT2CTRL_BOOTSTATUS_ISR_BIT (1 << 2)
#define LSM_INT2CTRL_GYRO_DTRDY_ISR_BIT (1 << 1)
#define LSM_INT2CTRL_ACCEL_DTRDY_ISR_BIT (1)

/** ctrl3 register bits */
#define LSM_CTRL3_BOOT_BIT (1 << 7)
#define LSM_CTRL3_BLOCKDATA_UPDATE_BIT (1 << 6)
#define LSM_CTRL3_ISR_ACTIVE_LVL_BIT (1 << 5)
#define LSM_CTRL3_ISR_PUSHPULL_OD_BIT (1 << 4)
#define LSM_CTRL3_SPI_MODE_BIT (1 << 3)
#define LSM_CTRL3_AUTOINC_ADDR_BIT (1 << 2) /** < use this bit to read fifo ? */
#define LSM_CTRL3_ENDIAN_BIT (1 << 1)
#define LSM_CTRL3_SW_RESET_BIT (1)

/** CTRL4 REGISTER BITS **/
#define LSM_CTRL4_ACCEL_BW_SEL_BIT (1 << 7)
#define LSM_CTRL4_GYRO_SLEEP_BIT (1 << 6)
#define LSM_CTRL4_ALL_ISR_PAD1_BIT (1 << 5)
#define LSM_CTRL4_FIFO_TEMP_EN_BIT (1 << 4)
#define LSM_CTRL4_DRDY_MASK_BIT (1 << 3)
#define LSM_CTRL4_I2C_DISABLE_BIT (1 << 2)
#define LSM_CTRL4_FIFO_THRLD_EN_BIT (1)

/** ctrl 6 register bits **/

#define LSM_CTRL6_GYRO_TRIG_EN_BIT (1 << 7)
#define LSM_CTRL6_GYRO_LVL_EN_BIT (1 << 6)
#define LSM_CTRL6_GYRO_LATCH_EN_BIT (1 << 5)
#define LSM_CTRL6_ACCEL_HPMODE_DISABLE_BIT (1 << 4)

/** CTRL 7 REGISTER BITS **/
#define LSM_CTRL7_GYRO_HPMODE_DISABLE_BIT (1 << 7)
#define LSM_CTRL7_GYRO_HIPASS_EN_BIT (1 << 6)
#define LSM_CTRL7_GYRO_HIGHPASS_RST_BIT (1 << 5)
#define LSM_CTRL7_ROUNDING_EN_BIT (1 << 4)

/** CTRL 8 REGISTER BITS **/

#define LSM_CTRL8_ACCEL_LPF2_EN_BIT (1 << 7)
#define LSM_CTRL8_ACCEL_HPSLOPE_EN_BIT (1 << 2)
#define LSM_CTRL8_ACCEL_6D_LOWPASS_EN_BIT (1)

/** CTRL 9 REGISTER BITS */
#define LSM_CTRL9_ACCEL_Z_EN_BIT (1 << 5)
#define LSM_CTRL9_ACCEL_Y_EN_BIT (1 << 4)
#define LSM_CTRL9_ACCEL_X_EN_BIT (1 << 3)
#define LSM_CTRL9_SOFTIRON_MGT_EN_BIT (1 << 2)

/** CTRL 10 REGISTER BITS */
#define LSM_CTRL10_GYRO_Z_EN_BIT (1 << 5)
#define LSM_CTRL10_GYRO_Y_EN_BIT (1 << 4)
#define LSM_CTRL10_GYRO_X_EN_BIT (1 << 3)
#define LSM_CTRL10_EMBEDDED_FUNC_EN_BIT (1 << 2)
#define LSM_CTRL10_PEDO_RST_STEP_BIT (1 << 1)
#define LSM_CTRL10_SGNF_MOTN_EN_BIT (1)

/** MASTER CONFIG BITS */
#define LSM_MSTRCONF_DRDY_INT1_EN_BIT (1 << 7)
#define LSM_MSTRCONF_FIFO_DVALID_BIT (1 << 6)
#define LSM_MSTRCONF_STARTCFG_BIT (1 << 5)
#define LSM_MSTRCONF_I2C_PULLUP_EN_BIT (1 << 4)
#define LSM_MSTRCONF_I2C_PASSTHRU_EN_BIT (1 << 3)
#define LSM_MSTRCONF_HARDIRON_MGT_EN_BIT (1 << 2)
#define LSM_MSTRCONF_SENSEHUB_I2C_MASTER_EN_BIT (1)

/** WAKEUP SOURCE  REG **/
#define LSM_WKUP_FREEFALL_DETECT_EN_BIT (1 << 5)
#define LSM_WKUP_SLEEP_EVT_STATUS_BIT (1 << 4)
#define LSM_WKUP_EVT_STATUS_BIT (1 << 3)
#define LSM_WKUP_EVTX_BIT (1 << 2)
#define LSM_WKUP_EVTY_BIT (1 << 1)
#define LSM_WKUP_EVTZ_BIT (1)

/** TODO: TAP SOURCE REG **/
/** TODO: D6D SOURCE REG **/

/** STATUS REG **/

#define LSM_STATUS_TEMP_AVAIL_BIT (1 << 2)
#define LSM_STATUS_GYRO_AVAIL_BIT (1 << 1)
#define LSM_STATUS_ACCEL_AVAIL_BIT (1)

/** TODO: SENSORHUB STUFF? SEEMS LIKE A LOT OF WORK THO... */

/** FIFO STATUS2 REG **/
#define LSM_FIFO2_WATERMARK_BIT (1 << 7)
#define LSM_FIFO2_OVRRUN_BIT (1 << 6)
#define LSM_FIFO2_FULL_BIT (1 << 5)
#define LSM_FIFO2_EMPTY_BIT (1 << 4)

#define LSM_FIFOCTRL2_PEDO_STEPT_EN_BIT (1 << 7)
#define LSM_FIFOCTRL2_PEDO_WRITESTEP_EN_BIT (1 << 6)

#define LSM_FIFOCTRL4_MSBDATA_ONLY_BIT (1 << 6)

/** FUNC SOURCE REG **/

#define LSM_FUNCSRC_PEDO_DELTA_T_BIT (1 << 7)
#define LSM_FUNCSRC_SGNFT_MTN_DETECT_BIT (1 << 6)
#define LSM_FUNCSRC_TILT_EVT_DETECT_BIT (1 << 5)
#define LSM_FUNCSRC_STEP_EVT_DETECT_BIT (1 << 4)
#define LSM_FUNCSRC_STEP_OVERFLOW_BIT (1 << 3)
#define LSM_FUNCSRC_IRONCALC_STATUS_BIT (1 << 1)
#define LSM_FUNCSRC_SENSHUB_COMM_STATUS_BIT (1)

#define LSM_FIFO_BUFFER_MEM_LEN 4096 * 2
#define LSM_DRIVER_SAMPLE_WAIT_READTRIES 200

/** TODO: MORE TAP STUFF... **/
/** TODO: WAKEUP EVENTS **/

/********** Types **********************/

/** Device operating modes **/
typedef enum LSM_OperatingMode
{
    LSM_OPMODE_ACCEL_ONLY,
    LSM_OPMODE_GYRO_ONLY,
    LSM_OPMODE_GYRO_ACCEL
} LSM_OperatingMode_t;

typedef enum LSM_GyroPowerMode
{
    LSM_GYROPWR_OFF,
    LSM_GYROPWR_LOWPWR,
    LSM_GYROPWR_NORMAL,
    LSM_GYROPWR_HPMODE
} LSM_GyroPwrMode_t;

typedef enum LSM_AccelPowerMode
{
    LSM_ACCELPWR_OFF,
    LSM_ACCELPWR_LOWPWR,
    LSM_ACCELPWR_NORMAL,
    LSM_ACCELPWR_HPMODE
} LSM_AccelPwrMode_t;

typedef enum LSM_FIFOMode
{
    LSM_FIFO_MODE_BYPASS = 0,         /** < Fifo is not used */
    LSM_FIFO_MODE_FIFO = 1,           /** < Fifo stores until full */
    LSM_FIFO_MODE_CONT_TO_FIFO = 3,   /** < both fifo & cont, changes depending on event trigger */
    LSM_FIFO_MODE_BYPASS_TO_FIFO = 4, /** < same as above but with bypass */
    LSM_FIFO_MODE_CONTINUOUS = 6,     /** < continuously updates fifo dumping older data */
    LSM_FIFO_MODE_END = 7
} LSM_FIFOMode_t;

typedef enum LSM_FIFOodr
{
    LSM_FIFO_ODR_DISABLED,
    LSM_FIFO_ODR_12_5HZ,
    LSM_FIFO_ODR_26_HZ,
    LSM_FIFO_ODR_52_HZ,
    LSM_FIFO_ODR_104_HZ,
    LSM_FIFO_ODR_208_HZ,
    LSM_FIFO_ODR_416_HZ,
    LSM_FIFO_ODR_833_HZ,
    LSM_FIFO_ODR_1_66KHZ,
    LSM_FIFO_ODR_3_33KHZ,
    LSM_FIFO_ODR_6_66KHZ,
} LSM_FIFOodr_t;

typedef enum LSM_FifoPktCfg
{
    LSM_FIFO_UNUSED,
    LSM_FIFO_0_DECM,
    LSM_FIFO_2_DECM,
    LSM_FIFO_3_DECM,
    LSM_FIFO_4_DECM,
    LSM_FIFO_8_DECM,
    LSM_FIFO_16_DECM,
    LSM_FIFO_32_DECM,
} LSM_FifoPktCfg_t;

/** make pkttypes binary so user can OR them **/
typedef enum LSM_PktType
{
    LSM_PKT1_GYRO = 1,
    LSM_PKT2_ACCL = 2,
    LSM_PKT3_SENSHUB = 4,
    LSM_PKT4_STEP_OR_TEMP = 8
} LSM_PktType_t;

/**  ISR control 1 **/

typedef enum LSM_interrupt
{
    LSM_INT_TYPE_CLEAR = 0,
    LSM_INT_TYPE_ACC_RDY = 1,
    LSM_INT_TYPE_GYR_RDY = 2,
    LSM_INT_TYPE_TMP_RDY = 3,
    LSM_INT_TYPE_BOOTSTAT = 3,
    LSM_INT_TYPE_FIFO_THR = 4,
    LSM_INT_TYPE_FIFO_OVR = 5,
    LSM_INT_TYPE_FIFOFULL = 6,
    LSM_INT_TYPE_STEPOVR = 7,
    LSM_INT_TYPE_SIGMOTION = 7,
    LSM_INT_TYPE_STEPDELTA = 8,
    LSM_INT_TYPE_STEPBASIC = 8,
    LSM_INT_TYPE_END = 9
} LSM_interrupt_t;

/** ctrl1 accel */
typedef enum LSM_AccelODR
{
    LSM_ACCODR_PWR_OFF,
    LSM_ACCODR_12_5HZ,
    LSM_ACCODR_26_HZ,
    LSM_ACCODR_52_HZ,
    LSM_ACCODR_104_HZ,
    LSM_ACCODR_208_HZ,
    LSM_ACCODR_416_HZ,
    LSM_ACCODR_833_HZ,
    LSM_ACCODR_1_66KHZ,
    LSM_ACCODR_3_33KHZ,
    LSM_ACCODR_6_66KHZ,
} LSM_AccelODR_t;

typedef enum LSM_AccelScale
{
    LSM_ACCSCALE_2G,
    LSM_ACCSCALE_16G,
    LSM_ACCSCALE_4G,
    LSM_ACCSCALE_8G
} LSM_AccelScale_t;

typedef enum LSM_AccelAntiAliasBW
{
    LSM_ACC_AA_BW_400HZ,
    LSM_ACC_AA_BW_200HZ,
    LSM_ACC_AA_BW_100HZ,
    LSM_ACC_AA_BW_50HZ
} LSM_AccelAntiAliasBW_t;

typedef enum LSM_HighpassSlopeSettings
{
    LSM_HPLP_ODR_4_LP_50,
    LSM_HPLP_ODR_100,
    LSM_HPLP_ODR_9,
    LSM_HPLP_ODR_400

} LSM_HighpassSlopeSettings_t;

/** ctrl2 Gyro */

typedef enum LSM_GyroScale
{
    LSM_GYRO_SCALE_250DPS,
    LSM_GYRO_SCALE_500DPS,
    LSM_GYRO_SCALE_1000DPS,
    LSM_GYRO_SCALE_2000DPS
} LSM_GyroScale_t;

typedef enum LSM_GyroODR
{
    LSM_GYRO_ODR_PWR_OFF,
    LSM_GYRO_ODR_12_5HZ,
    LSM_GYRO_ODR_26_HZ,
    LSM_GYRO_ODR_52_HZ,
    LSM_GYRO_ODR_104_HZ,
    LSM_GYRO_ODR_208_HZ,
    LSM_GYRO_ODR_416_HZ,
    LSM_GYRO_ODR_833_HZ,
    LSM_GYRO_ODR_1_66KHZ,
} LSM_GyroODR_t;

typedef enum LSM_GyroHighpassCutoff
{
    LSM_GYRO_HPCUTOFF_8mHZ,
    LSM_GYRO_HPCUTOFF_324mHZ,
    LSM_GYRO_HPCUTOFF_2HZ,
    LSM_GYRO_HPCUTOFF_16_3HZ
} LSM_GyroHighpassCutoff_t;

/** CTRL5 REGISTER ST MODES **/

typedef enum LSM_AccelSelfTest
{
    LSM_ACCEL_SELFTEST_NORMAL = 0,
    LSM_ACCEL_SELFTEST_POSITIVE = 1,
    LSM_ACCEL_SELFTEST_NEGATIVE = 3
} LSM_AccelSelfTest_t;

typedef enum LSM_GyroSelfTest
{
    LSM_GYRO_SELFTEST_NORMAL = 0,
    LSM_GYRO_SELFTEST_POSITIVE = 1,
    LSM_GYRO_SELFTEST_NEGATIVE = 2
} LSM_GyroSelfTest_t;

typedef enum LSM_DeviceCommMode
{
    LSM_DEVICE_COMM_MODE_I2C,
    LSM_DEVICE_COMM_MODE_SPI,
    LSM_DEVICE_COMM_MODE_SPI_3
} LSM_DeviceCommMode_t;

/** initData struct 
 *  - keep it simple, let user configure with extra functions
 * 
 */
typedef struct LSM_initData
{
    gpio_num_t int1Pin;   /** < gpio pin for int 1 - 0 if unused */
    gpio_num_t int2Pin;   /** < gpio pin for int 2 - 0 if unused */
    uint8_t commsChannel; /** < comms channel for i2c or spi */
    uint8_t addrPinState;
    LSM_DeviceCommMode_t commMode;
    LSM_OperatingMode_t opMode;
    LSM_AccelODR_t accelRate;
    LSM_GyroODR_t gyroRate;
    bool assignFifoBuffer; /** < assign an 8Kb dma-cap buffer for burst fifo reads **/
} LSM_initData_t;

typedef struct LSM_deviceSettings
{
    LSM_OperatingMode_t opMode;
    LSM_HighpassSlopeSettings_t highPass;

    LSM_AccelScale_t accelScale;
    LSM_AccelODR_t accelRate;
    LSM_AccelPwrMode_t accelPwr; /** obsolete? **/
    LSM_AccelAntiAliasBW_t accelAA;
    LSM_FifoPktCfg_t accelDec;

    LSM_GyroScale_t gyroScale;
    LSM_GyroODR_t gyroRate;
    LSM_GyroPwrMode_t gyroPwr; /** obsolete ? **/
    LSM_GyroHighpassCutoff_t GHPcutoff;
    LSM_FifoPktCfg_t gyroDec;

    LSM_interrupt_t int1;
    LSM_interrupt_t int2;

    LSM_FIFOMode_t fifoMode;
    LSM_FIFOodr_t fifoODR;

    uint8_t fifoPktLen;

} LSM_DeviceSettings_t;

typedef struct LSM_DeviceMeasures
{
    float calibGyroX; /** < in mDegrees/s **/
    float calibGyroY;
    float calibGyroZ;

    float calibAccelX; /** < in mG's **/
    float calibAccelY; /** < in mG's **/
    float calibAccelZ; /** < in mG's **/

    uint8_t rawGyro[6];
    uint8_t rawAccel[6];

} LSM_DeviceMeasures_t;

/**
 *  LSM_DriverSettings_t - a settings struct for the device
 *  keep track of various things..
 * 
 * 
*/
typedef struct LSM_DriverSettings
{
    uint16_t i1Pin;                    /** < gpio interrupt pin 1 **/
    uint16_t i2Pin;                    /** < gpio interrupt pin 2 **/
    bool int1En;                       /** < enabled interrupt 1 **/
    bool int2En;                       /** < enabled interrupt 2 **/
    uint8_t int1Function;              /** < interrupt function **/
    uint8_t int2Function;              /** < interrupt function **/
    LSM_DeviceCommMode_t commMode;     /** < comms mode **/
    LSM_DeviceSettings_t settings;     /** < a holder for device settings **/
    LSM_DeviceMeasures_t measurements; /** < the device's latest measurtements **/
    uint8_t commsChannel;              /** < the i2c or spi channel being used */
    uint8_t devAddr;                   /** < the device i2c address **/
    void *commsHandle;                 /** < can be used to hold a device handle */
    void *fifoBuffer;                  /** < ptr to fifo buffer memory **/
    TaskHandle_t taskHandle;           /** < handle to the task **/
} LSM_DriverSettings_t;

/******** Function Definitions *********/

/** \brief LSM_init()
 *      
 *          initilise the LSM driver. Assumes a single device. 
 *          takes a pointer to an LSM init struct
 * 
 *  \param LSM_initData_t initData 
 *  \return ESP_OK or error
 */
LSM_DriverSettings_t *LSM_init(LSM_initData_t *initData);

/** 
 *  LSM_deinit() 
 *      tear down the LSM driver 
 *  
 *  \return ESP_OK or error
*/
esp_err_t LSM_deInit();

/** \brief: samples latest measurements. Waits status 
 * 
 * **/
esp_err_t LSM_sampleLatest(LSM_DriverSettings_t *dev);

/** \brief  getGyroX - returns most recent gyro X axis value
 *  \param  dev - pointer to device struct 
 *  \param  x   - pointer to value place
 *  \return ESP_OK or error
 **/
esp_err_t LSM_getGyroX(LSM_DriverSettings_t *dev, float *x);

/** \brief  getGyroY - returns most recent gyro Y axis value
 *  \param  dev - pointer to device struct 
 *  \param  y   - pointer to value place
 *  \return ESP_OK or error
 **/
esp_err_t LSM_getGyroY(LSM_DriverSettings_t *dev, float *y);

/** \brief  getGyroZ - returns most recent gyro z axis value
 *  \param  dev - pointer to device struct 
 *  \param  z   - pointer to value place
 *  \return ESP_OK or error
 **/
esp_err_t LSM_getGyroZ(LSM_DriverSettings_t *dev, float *z);

/** \brief  getAccelX - returns most recent accel X axis value
 *  \param  dev - pointer to device struct 
 *  \param  x   - pointer to value place
 *  \return ESP_OK or error
 **/
esp_err_t LSM_getAccelX(LSM_DriverSettings_t *dev, float *x);

/** \brief  getAccelY - returns most recent accel Y axis value
 *  \param  dev - pointer to device struct 
 *  \param  y   - pointer to value place
 *  \return ESP_OK or error
 **/
esp_err_t LSM_getAccelY(LSM_DriverSettings_t *dev, float *y);

/** \brief  getAccelZ - returns most recent accel Z axis value
 *  \param  dev - pointer to device struct 
 *  \param  x   - pointer to value place
 *  \return ESP_OK or error
 **/
esp_err_t LSM_getAccelZ(LSM_DriverSettings_t *dev, float *z);

/** \brief  setOpMode - set the operating mode
 *  \param  dev - pointer to device struct 
 *  \param  mode  - pointer to mode value
 *  \return ESP_OK or error
 **/
esp_err_t LSM_setOpMode(LSM_DriverSettings_t *dev, LSM_OperatingMode_t *mode);

/** \brief  setAccelODRMode - set the accel data rate
 *  \param  dev - pointer to device struct 
 *  \param  mode - pointer to value 
 *  \return ESP_OK or error
 **/
esp_err_t LSM_setAccelODRMode(LSM_DriverSettings_t *dev, LSM_AccelODR_t mode);

/** \brief  setGyroODRMode - set the gyro output data rate
 *  \param  dev - pointer to device struct 
 *  \param  mode   - pointer to value 
 *  \return ESP_OK or error
 **/
esp_err_t LSM_setGyroODRMode(LSM_DriverSettings_t *dev, LSM_GyroODR_t mode);

/** \brief  setFifoMode - set the fifo mode 
 *                      - only bypass/standard currently supported
 *  \param  dev - pointer to device struct 
 *  \param  mode   - pointer to value 
 *  \return ESP_OK or error
 **/
esp_err_t LSM_setFIFOmode(LSM_DriverSettings_t *dev, LSM_FIFOMode_t mode);

/** \brief  setFIFOwatermark - set the fifo watermark
 *  \param  dev - pointer to device struct 
 *  \param  x   - pointer to value 
 *  \return ESP_OK or error
 **/
esp_err_t LSM_setFIFOwatermark(LSM_DriverSettings_t *dev, uint16_t watermark);

/** \brief  setFIFOpackets - set the fifo packet type
 *  \param  dev - pointer to device struct 
 *  \param  pktType - pointer to packet type. Should be an OR'd value of enum LSM_PktType_t 
 *  \return ESP_OK or error
 **/
esp_err_t LSM_setFIFOpackets(LSM_DriverSettings_t *dev, LSM_PktType_t *pktType);

/** \brief  configInt - set the fifo packet type
 *  \param  dev - pointer to device struct 
 *  \param intNum - interrupt num (1 or 2)
 *  \param  intr - interrupt type (one of LSM_interrupt_t)
 *  \return ESP_OK or error
 **/
esp_err_t LSM_configInt(LSM_DriverSettings_t *device, uint8_t intNum, LSM_interrupt_t intr);

/** \brief  readFifoBlock - read fifo packet data into buffer
 *  \param  dev - pointer to device struct 
 *  \param  length - length of data to read
 *  \return ESP_OK or error
 **/
esp_err_t LSM_readFifoBlock(LSM_DriverSettings_t *device, uint16_t length);

#endif /* LSM_DRIVER_H */
