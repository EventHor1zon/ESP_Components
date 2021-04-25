/***************************************
* \file     LSM_Driver.c
* \brief    Driver for the LSM6DS3 Gyro and Accelerometer
*           Can be configured to use i2c or spi
*           Basic functionality for now, more complex later
*           This driver expects an initialised comms bus (spi/i2c)
*           but it will intiialise the gpio interrupt pins itself
*
* \date     Aug 2020
* \author   RJAM
****************************************/

/********* Includes *******************/
#include "esp_types.h"

#include "LSM_Driver.h"

#include "genericCommsDriver.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/****** Function Prototypes ***********/


#ifdef CONFIG_USE_PERIPH_MANAGER

#include "CommandAPI.h"

// const parameter_t lsm_parameter_mappings[lsm_param_mappings_len] = {
//     {"Gyro X", 1, &LSM_getGyroX, NULL, PARAMTYPE_INT16, 0, (GET_FLAG) },
//     {"Gyro Y", 2, &LSM_getGyroY, NULL, PARAMTYPE_INT16, 0, (GET_FLAG) },
//     {"Gyro Z", 3, &LSM_getGyroZ, NULL, PARAMTYPE_INT16, 0, (GET_FLAG) },
//     {"Accel X", 4, &LSM_getAccelX, NULL, PARAMTYPE_INT16, 0, (GET_FLAG) },
//     {"Accel Y", 5, &LSM_getAccelY, NULL, PARAMTYPE_INT16, 0, (GET_FLAG) },
//     {"Accel Z", 6, &LSM_getAccelZ, NULL, PARAMTYPE_INT16, 0, (GET_FLAG) },
//     {"Op Mode", 7, &LSM_getOpMode, &LSM_setOpMode, PARAMTYPE_UIN8, 2, (GET_FLAG | SET_FLAG)},
// };


const peripheral_t lsm_periph_template;


#endif


static void ISR_int1(void *args);
static void ISR_int2(void *args);

static void LSM_processAccel(LSM_DriverHandle_t *dev);
static void LSM_processGyro(LSM_DriverHandle_t *dev);
static uint16_t LSM_fifoPattern(LSM_DriverHandle_t *dev);
static esp_err_t LSM_getFIFOpktCount(LSM_DriverHandle_t *dev, uint16_t *count);
static esp_err_t LSM_waitSampleReady(LSM_DriverHandle_t *dev, uint8_t mask);
static esp_err_t LSM_getWhoAmI(LSM_DriverHandle_t *device, uint8_t *whoami);

void LSMDriverTask(void *args);



/************ ISR *********************/

static void ISR_int1(void *args)
{
}

static void ISR_int2(void *args)
{
}
/****** Private Data ******************/

/****** Private Functions *************/

static void LSM_processAccel(LSM_DriverHandle_t *dev)
{
    float factor = 0;

    uint16_t s_ax = ((int16_t)(dev->measurements.rawAccel[1] << 8) | (int16_t)(dev->measurements.rawAccel[0]));
    uint16_t s_ay = ((int16_t)(dev->measurements.rawAccel[3] << 8) | (int16_t)(dev->measurements.rawAccel[2]));
    uint16_t s_az = ((int16_t)(dev->measurements.rawAccel[5] << 8) | (int16_t)(dev->measurements.rawAccel[4]));

    switch (dev->settings.accelScale)
    {
    case LSM_ACCSCALE_2G:
        factor = 61.0;
        dev->measurements.calibAccelX = (float)s_ax * factor / 1000.0f;
        dev->measurements.calibAccelY = (float)s_ay * factor / 1000.0f;
        dev->measurements.calibAccelZ = (float)s_az * factor / 1000.0f;
        break;
    case LSM_ACCSCALE_4G:
        factor = 122.0;
        dev->measurements.calibAccelX = (float)s_ax * factor / 1000.0f;
        dev->measurements.calibAccelY = (float)s_ay * factor / 1000.0f;
        dev->measurements.calibAccelZ = (float)s_az * factor / 1000.0f;
        break;
    case LSM_ACCSCALE_8G:
        factor = 244.0;
        dev->measurements.calibAccelX = (float)s_ax * factor / 1000.0f;
        dev->measurements.calibAccelY = (float)s_ay * factor / 1000.0f;
        dev->measurements.calibAccelZ = (float)s_az * factor / 1000.0f;
        break;
    case LSM_ACCSCALE_16G:
        factor = 488.0;
        dev->measurements.calibAccelX = (float)s_ax * factor / 1000.0f;
        dev->measurements.calibAccelY = (float)s_ay * factor / 1000.0f;
        dev->measurements.calibAccelZ = (float)s_az * factor / 1000.0f;
        break;
    default:
        factor = 488.0;
        dev->measurements.calibAccelX = 0;
        dev->measurements.calibAccelY = 0;
        dev->measurements.calibAccelZ = 0;
        break;
    }
}

static void LSM_processGyro(LSM_DriverHandle_t *dev)
{
    float factor = 0;

    switch (dev->settings.gyroScale)
    {
    case LSM_GYRO_SCALE_250DPS:
        factor = 8750 / 1000.0f;
        break;
    case LSM_GYRO_SCALE_500DPS:
        factor = 1750 / 100.0f;
        break;
    case LSM_GYRO_SCALE_1000DPS:
        factor = 35.0;
        break;
    case LSM_GYRO_SCALE_2000DPS:
        factor = 70.0;
        break;
    default:
        factor = 70;
        break;
    }

    dev->measurements.calibGyroX = ((float)((int16_t)(dev->measurements.rawGyro[1] << 8) | (int16_t)(dev->measurements.rawGyro[0])) * factor);
    dev->measurements.calibGyroY = ((float)((int16_t)(dev->measurements.rawGyro[3] << 8) | (int16_t)(dev->measurements.rawGyro[2])) * factor);
    dev->measurements.calibGyroZ = ((float)((int16_t)(dev->measurements.rawGyro[5] << 8) | (int16_t)(dev->measurements.rawGyro[4])) * factor);
}

static uint16_t LSM_fifoPattern(LSM_DriverHandle_t *dev)
{

    uint8_t regVals[2] = {0, 0};
    ESP_ERROR_CHECK(gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_FIFO_STATUS3_REG, 2, regVals));

    uint16_t pattern = (((uint16_t)regVals[1] << 8) | regVals[0]);
    return pattern;
}

static esp_err_t LSM_getFIFOpktCount(LSM_DriverHandle_t *dev, uint16_t *count)
{
    esp_err_t status = ESP_OK;
    uint8_t rxBuffer[2] = {0};
    uint8_t msbMask = 0b1111;
    uint16_t fifoCount = 0;

    status = gcd_i2c_read_address(dev->commsChannel, (uint8_t)LSM_I2C_ADDR, LSM_FIFO_STATUS1_REG, 2, rxBuffer);

    if (status == ESP_OK)
    {
        msbMask &= rxBuffer[1];
        fifoCount = ((uint16_t)msbMask << 8 | (uint16_t)rxBuffer[0]);
        *count = fifoCount;
    }

    return status;
}

static esp_err_t LSM_waitSampleReady(LSM_DriverHandle_t *dev, uint8_t mask)
{
    esp_err_t status = ESP_OK;
    uint8_t regVal = 0, tries = 0;
    while (!(regVal & mask))
    {
        status = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_STATUS_REG, 1, &regVal);
        tries++;
        if (tries > LSM_DRIVER_SAMPLE_WAIT_READTRIES)
        {
            status = ESP_ERR_INVALID_RESPONSE;
            break;
        }
    }

    return status;
}

static esp_err_t LSM_getWhoAmI(LSM_DriverHandle_t *device, uint8_t *whoami)
{
    esp_err_t status = ESP_OK;

    status = gcd_i2c_read_address(device->commsChannel, device->devAddr, LSM_WHOAMI_REG, 1, whoami);

    return status;
}






/**
 *  LSM_DriverTask - if interrupts configured, wait for unblock to proceed 
 *                         - if interrupt mode == FIFO_FULL/THRESHOLD/OVR
 *                           - zero fifo buffer
 *                           - read new samples
 *                         - elif mode == accel/data rdy, 
 *                           - read new samples (no sample rdy check)
 *                         - else
 *                              custom functionality?
 *                  - else:
 *                          -if fifo used:
 *                              - check fifo for ovr/threashold/full
 *                              - zero fifo buffer 
 *                              - read new samples
 *                          - else 
 *                              - Check data ready
 *                              - Read new samples
 **/
void LSMDriverTask(void *args)
{

    LSM_DriverHandle_t *dev = (LSM_DriverHandle_t *)args;

    while (1)
    {
        LSM_sampleLatest(dev);
        printf("m: %f\t|%f\t|%f\t| %f\t|%f\t|%f\n", dev->measurements.calibGyroX, dev->measurements.calibGyroY, dev->measurements.calibGyroZ, dev->measurements.calibAccelX, dev->measurements.calibAccelY, dev->measurements.calibAccelZ);
        vTaskDelay(100);

        if (dev->int1En || dev->int2En)
        {

            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }
    }
}

/****** Global Data *******************/

LSM_DriverHandle_t *device = NULL;

#ifdef CONFIG_USE_PERIPH_MANAGER


const parameter_t lsm_parameter_mappings[lsm_param_mappings_len] = {
    {"Operating Mode", 1, NULL, &LSM_setOpMode, PARAMTYPE_UINT8, 2, (SET_FLAG) },
    {"Accelerometer ODR", 2, NULL, &LSM_setAccelODRMode, PARAMTYPE_UINT8, 10, (SET_FLAG) },
    {"Gyroscope ODR", 3, NULL, &LSM_setGyroODRMode, PARAMTYPE_UINT8, 8, (SET_FLAG) },
    {"Gyro X", 4, NULL, &LSM_getGyroX, PARAMTYPE_FLOAT, 0, (GET_FLAG) },
    {"Gyro Y", 5, NULL, &LSM_getGyroY, PARAMTYPE_FLOAT, 0, (GET_FLAG) },
    {"Gyro Z", 6, NULL, &LSM_getGyroZ, PARAMTYPE_FLOAT, 0, (GET_FLAG) },
    {"Accel X", 7, NULL, &LSM_getAccelX, PARAMTYPE_FLOAT, 0, (GET_FLAG) },
    {"Accel Y", 8, NULL, &LSM_getAccelY, PARAMTYPE_FLOAT, 0, (GET_FLAG) },
    {"Accel Z", 9, NULL, &LSM_getAccelZ, PARAMTYPE_FLOAT, 0, (GET_FLAG) },
    {"FIFO mode", 10, &LSM_getFIFOmode, &LSM_setFIFOmode, PARAMTYPE_INT8, 7, (GET_FLAG | SET_FLAG) },

};

#endif


/****** Global Functions *************/

LSM_DriverHandle_t *LSM_init(LSM_initData_t *initData)
{

    esp_err_t initStatus = ESP_OK;

    if (initData == NULL)
    {
        ESP_LOGE("LSM Driver", "Error - NULL ptr to init data");
        initStatus = ESP_ERR_INVALID_ARG;
    }
    else if ((initData->commMode == LSM_DEVICE_COMM_MODE_SPI && initData->commsChannel == 0) || initData->commsChannel > 2)
    {
        ESP_LOGE("LSM Driver", "Error - invalid comms channel");
        initStatus = ESP_ERR_INVALID_ARG;
    }
    else
    {
        /** allocate memory on the heap for the control structure */
        device = (LSM_DriverHandle_t *)heap_caps_calloc(1, sizeof(LSM_DriverHandle_t), MALLOC_CAP_8BIT);

        if (device == NULL)
        {
            ESP_LOGE("LSM Driver", "Error assigning mem for the device structure");
            initStatus = ESP_ERR_NO_MEM;
        }
        else
        {

            device->commsHandle = 0;

            if ((initData->commMode == LSM_DEVICE_COMM_MODE_SPI || initData->commMode == LSM_DEVICE_COMM_MODE_SPI_3))
            {
                /** TODO: setup spi  **/
                return NULL;
            }
            else if (initData->commMode == LSM_DEVICE_COMM_MODE_I2C)
            {
                device->commMode = LSM_DEVICE_COMM_MODE_I2C;
                device->commsChannel = initData->commsChannel;

                if (initData->addrPinState)
                {
                    device->devAddr = LSM_I2C_ADDR + 1;
                }
                else
                {
                    device->devAddr = LSM_I2C_ADDR;
                }
            }
            else
            {
                initStatus = ESP_ERR_INVALID_ARG;
            }
        }

        /** set up interrupts **/
        if (initStatus == ESP_OK && (initData->int1Pin || initData->int2Pin))
        {
            uint32_t gpioMask = 0;
            if (initData->int1Pin)
            {
                gpioMask |= (1ULL << initData->int1Pin);
            } /** this should work? **/
            if (initData->int2Pin)
            {
                gpioMask |= (1ULL << initData->int2Pin);
            }

            gpio_config_t gpioInit = {0};
            gpioInit.intr_type = GPIO_INTR_NEGEDGE;
            gpioInit.mode = GPIO_MODE_INPUT;
            gpioInit.pin_bit_mask = gpioMask;
            gpioInit.pull_up_en = GPIO_PULLUP_ENABLE;
            gpioInit.pull_down_en = GPIO_PULLDOWN_DISABLE;

            ESP_ERROR_CHECK(gpio_config(&gpioInit));

            if (initStatus == ESP_OK)
            {
                ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1));
                if (initStatus == ESP_ERR_INVALID_STATE)
                {
                    /** driver already initialised, don't freak out */
                    ESP_LOGI("LSM_Driver", "Cannot install gpio isr service");
                    initStatus = ESP_OK;
                }

                if (initStatus == ESP_OK)
                {
                    if (initData->int1Pin)
                    {
                        ESP_ERROR_CHECK(gpio_isr_handler_add(initData->int1Pin, ISR_int1, device));
                        device->i1Pin = initData->int1Pin;
                        device->int1En = 1;
                    }
                    if (initData->int2Pin)
                    {
                        ESP_ERROR_CHECK(gpio_isr_handler_add(initData->int2Pin, ISR_int2, device));
                        device->i2Pin = initData->int2Pin;
                        device->int2En = 1;
                    }
                }
                else
                {
                    ESP_LOGE("LSM Driver", "Error installing ISR service [%u]", initStatus);
                }
            }
            else
            {
                ESP_LOGE("LSM Driver", "Error configuring GPIO pins! [%u]", initStatus);
            }
        }

        if (initStatus == ESP_OK && initData->assignFifoBuffer)
        {
            size_t dmaMem = heap_caps_get_free_size(MALLOC_CAP_DMA);
            if (dmaMem < LSM_FIFO_BUFFER_MEM_LEN)
            {
                ESP_LOGE("LSM Driver", "Error: insufficient DMA cap mem. Only %d bytes available", dmaMem);
                initStatus = ESP_ERR_NO_MEM;
            }
            else
            {
                void *fifoMem = heap_caps_malloc(LSM_FIFO_BUFFER_MEM_LEN, MALLOC_CAP_DMA);
                if (fifoMem != NULL)
                {
                    device->fifoBuffer = fifoMem;
                    ESP_LOGI("LSM_Driver", "FIFO Buffer initialised at %p", fifoMem);
                }
                else
                {
                    initStatus = ESP_ERR_NO_MEM;
                }
            }
        }

        /** device settings - start these all at default values (unless specified in initData) **/
        device->settings.accelAA = 0;
        device->settings.accelRate = initData->accelRate;
        device->settings.accelScale = LSM_ACCSCALE_2G;
        device->settings.accelDec = 1;

        device->settings.gyroDec = 1;
        device->settings.gyroPwr = LSM_GYROPWR_NORMAL;
        device->settings.gyroRate = initData->gyroRate;
        device->settings.gyroScale = LSM_GYRO_SCALE_250DPS;
        device->settings.highPass = 0;

        device->settings.opMode = initData->opMode;
        device->settings.int1 = LSM_INT_TYPE_CLEAR;
        device->settings.int2 = LSM_INT_TYPE_CLEAR;
        device->settings.fifoMode = LSM_FIFO_MODE_BYPASS;
        device->settings.fifoODR = LSM_FIFO_ODR_DISABLED;
        device->settings.fifoPktLen = 0;

        if (initStatus == ESP_OK)
        {
            /** initialise device settings **/
            ESP_LOGI("LSM_Driver", "Setting Data rates");
            LSM_setOpMode(device, &initData->opMode);
            LSM_setAccelODRMode(device, &initData->accelRate);
            LSM_setGyroODRMode(device, &initData->gyroRate);
        }

        if (initStatus == ESP_OK)
        {
            TaskHandle_t taskHandle;
            if (xTaskCreate(LSMDriverTask, "LSMDriverTask", 5012, (void *)device, 3, &taskHandle) == pdFALSE)
            {
                ESP_LOGE("LSM_Driver", "Error creating control task!");
                initStatus = ESP_ERR_INVALID_RESPONSE;
            }

            device->taskHandle = taskHandle;
        }
    }

    if (initStatus == ESP_OK)
    {
        ESP_LOGI("LSM_Driver", "Succesfully started driver!");
    }
    return device;
}


esp_err_t LSM_deInit(LSM_DriverHandle_t *dev)
{
    if (dev->fifoBuffer != NULL)
    {
        heap_caps_free(dev->fifoBuffer);
    }

    free(dev);

    return ESP_OK;
}

/******* SAMPLE SETTINGS (BASIC) ********/

esp_err_t LSM_setOpMode(LSM_DriverHandle_t *dev, LSM_OperatingMode_t *mode)
{
    esp_err_t status = ESP_OK;
    uint8_t regVals[2] = {0};
    uint8_t writeVals[2] = {0};
    status = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_CTRL9_XL_REG, 2, regVals);

    /** enable all axis for eac dev **/
    if (*mode == LSM_OPMODE_ACCEL_ONLY)
    {
        writeVals[0] = ((LSM_CTRL9_ACCEL_Z_EN_BIT) | (LSM_CTRL9_ACCEL_Y_EN_BIT) | (LSM_CTRL9_ACCEL_X_EN_BIT) | (regVals[0] & 0b111));
    }
    else if (*mode == LSM_OPMODE_GYRO_ONLY)
    {
        writeVals[1] = ((LSM_CTRL10_GYRO_Z_EN_BIT) | (LSM_CTRL10_GYRO_Y_EN_BIT) | (LSM_CTRL10_GYRO_X_EN_BIT) | (regVals[1] & 0b111));
    }
    else if (*mode == LSM_OPMODE_GYRO_ACCEL)
    {
        writeVals[0] = ((LSM_CTRL9_ACCEL_Z_EN_BIT) | (LSM_CTRL9_ACCEL_Y_EN_BIT) | (LSM_CTRL9_ACCEL_X_EN_BIT) | (regVals[0] & 0b111));
        writeVals[1] = ((LSM_CTRL10_GYRO_Z_EN_BIT) | (LSM_CTRL10_GYRO_Y_EN_BIT) | (LSM_CTRL10_GYRO_X_EN_BIT) | (regVals[1] & 0b111));
    }
    else
    {
        status = ESP_ERR_INVALID_ARG;
    }

    if (status == ESP_OK)
    {
        status = gcd_i2c_write_address(dev->commsChannel, dev->devAddr, LSM_CTRL9_XL_REG, 2, writeVals);
    }

    ESP_LOGI("LSM_Driver", "Set opmode to %02x", *mode);

    return status;
}



esp_err_t LSM_setAccelODRMode(LSM_DriverHandle_t *dev, LSM_AccelODR_t *m)
{
    esp_err_t status = ESP_OK;
    uint8_t accelEn = 0, regVal = 0;
    uint8_t mode = *m;

    if (mode > LSM_ACCODR_6_66KHZ)
    {
        status = ESP_ERR_INVALID_ARG;
    }
    else
    {
        /** get axis enabled status */
        status = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_CTRL9_XL_REG, 1, &accelEn);
        /** if no axis are active and mode > LSM_ACCELPWR_OFF turn on axis **/

        if ((accelEn == 0) && mode > LSM_ACCODR_PWR_OFF)
        {
            accelEn |= ((LSM_CTRL9_ACCEL_Z_EN_BIT) | (LSM_CTRL9_ACCEL_Y_EN_BIT) | (LSM_CTRL9_ACCEL_X_EN_BIT));
            ESP_LOGI("LSM_Driver", "Writing %02x to Accel ctrl reg", accelEn);
            status = gcd_i2c_write_address(1, LSM_I2C_ADDR, LSM_CTRL9_XL_REG, 1, &accelEn);
        }
        /** get register value, clear mode & set new **/
        status = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_CTRL1_XL_REG, 1, &regVal);
        regVal &= 0b1111;

        regVal |= (mode << 4);
        status = gcd_i2c_write_address(dev->commsChannel, dev->devAddr, LSM_CTRL1_XL_REG, 1, &regVal);
        ESP_LOGI("LSM_Driver", "Set ACCEL CTRL1 to %02x", regVal);
    }
    return status;
}

esp_err_t LSM_setGyroODRMode(LSM_DriverHandle_t *dev, LSM_GyroODR_t *m)
{
    esp_err_t status = ESP_OK;
    uint8_t accelEn = 0, regVal = 0;
    uint8_t mode = *m;

    if (mode > LSM_GYRO_ODR_1_66KHZ)
    {
        status = ESP_ERR_INVALID_ARG;
    }
    else
    {
        /** get axis enabled status */
        status = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_CTRL10_C_REG, 1, &accelEn);
        /** if no axis are active and mode > LSM_ACCELPWR_OFF turn on axis **/
        if (!(accelEn) && mode > LSM_GYRO_ODR_PWR_OFF)
        {
            accelEn |= ((LSM_CTRL10_GYRO_Z_EN_BIT) | (LSM_CTRL10_GYRO_Y_EN_BIT) | (LSM_CTRL10_GYRO_X_EN_BIT));
            status = gcd_i2c_write_address(dev->commsChannel, dev->devAddr, LSM_CTRL10_C_REG, 1, &accelEn);
        }
        /** get register value, clear mode & set new **/
        status = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_CTRL2_G_REG, 1, &regVal);
        regVal &= 0b1111;
        regVal |= (mode << 4);
        status = gcd_i2c_write_address(dev->commsChannel, dev->devAddr, LSM_CTRL2_G_REG, 1, &regVal);
        ESP_LOGI("LSM_Driver", "Set Gyro CTRL2 to %02x", regVal);
    }
    return status;
}

/** CORE FUNCTIONALITY **/

esp_err_t LSM_sampleLatest(LSM_DriverHandle_t *dev)
{

    esp_err_t status = ESP_OK;
    uint8_t regVal = 0, wait = 0;

    /** check the status bit */

    status = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_STATUS_REG, 1, &regVal);
    switch (dev->settings.opMode)
    {
    case LSM_OPMODE_ACCEL_ONLY:

        status = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_ACCELX_LSB_REG, 6, dev->measurements.rawAccel);
        LSM_processAccel(dev);
        break;

    case LSM_OPMODE_GYRO_ONLY:

        status = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_GYROX_LSB_REG, 6, dev->measurements.rawGyro);
        LSM_processGyro(dev);
        break;

    case LSM_OPMODE_GYRO_ACCEL:

        status = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_GYROX_LSB_REG, 12, dev->measurements.rawGyro);
        LSM_processAccel(dev);
        LSM_processGyro(dev);

    default:
        break;
    }

    return status;
}

esp_err_t LSM_getGyroX(LSM_DriverHandle_t *dev, float *x)
{
    esp_err_t status = ESP_OK;
    *x = dev->measurements.calibGyroX;
    return status;
}

esp_err_t LSM_getGyroY(LSM_DriverHandle_t *dev, float *y)
{
    esp_err_t status = ESP_OK;
    *y = dev->measurements.calibGyroY;
    return status;
}

esp_err_t LSM_getGyroZ(LSM_DriverHandle_t *dev, float *z)
{
    esp_err_t status = ESP_OK;
    *z = dev->measurements.calibGyroZ;
    return status;
}

esp_err_t LSM_getAccelX(LSM_DriverHandle_t *dev, float *x)
{
    esp_err_t status = ESP_OK;
    *x = dev->measurements.calibAccelX;
    return status;
}

esp_err_t LSM_getAccelY(LSM_DriverHandle_t *dev, float *y)
{
    esp_err_t status = ESP_OK;
    *y = dev->measurements.calibAccelY;
    return status;
}

esp_err_t LSM_getAccelZ(LSM_DriverHandle_t *dev, float *z)
{
    esp_err_t status = ESP_OK;
    *z = dev->measurements.calibAccelZ;
    return status;
}

/** FIFO SETTINGS **/

esp_err_t LSM_setFIFOmode(LSM_DriverHandle_t *dev, LSM_FIFOMode_t *m)
{
    uint8_t regvalue = 0, writevalue = 0;
    uint8_t mode = *m;
    esp_err_t status = ESP_OK;

    if (mode == LSM_FIFO_MODE_BYPASS || /** because several reserved values, have to do this the long way... **/
        mode == LSM_FIFO_MODE_FIFO ||
        mode == LSM_FIFO_MODE_CONT_TO_FIFO ||
        mode == LSM_FIFO_MODE_BYPASS_TO_FIFO ||
        mode == LSM_FIFO_MODE_CONTINUOUS)
    {
        writevalue = mode;
    }
    else
    {
        status = ESP_ERR_INVALID_ARG;
    }

    if (status == ESP_OK)
    {
        status = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_FIFO_CTRL5_REG, 1, &regvalue);
        regvalue &= (0b11111000); /** clear low 3 bits **/
        writevalue |= regvalue;   /** set mode **/
        status = gcd_i2c_write_address(dev->commsChannel, dev->devAddr, LSM_FIFO_CTRL5_REG, 1, &writevalue);
    }

    return status;
}

esp_err_t LSM_getFIFOmode(LSM_DriverHandle_t *dev, uint8_t *mode)
{
    esp_err_t status = ESP_OK;

    return status;
}

esp_err_t LSM_setFIFOwatermark(LSM_DriverHandle_t *dev, uint16_t *watermark)
{
    esp_err_t status = ESP_OK;
    uint16_t val = *watermark;

    if(val > LSM_WATERMARK_MAX) {
        status = ESP_ERR_INVALID_ARG;
    } else {
        uint8_t regval = 0;
        uint8_t writeval[2] = {0,0};
        writeval[0] = (uint8_t) val;
        writeval[1] = (uint8_t)(val >> 8);

        esp_err_t status = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_FIFO_CTRL2_REG, 1, &regval);
        writeval[1] |= regval; 
        status = gcd_i2c_write_address(dev->commsHandle, dev->devAddr, LSM_FIFO_CTRL1_REG, 2, writeval);
    }
    return status;
}

esp_err_t LSM_getFIFOwatermark(LSM_DriverHandle_t *dev, uint16_t *watermark) {
    
    esp_err_t status = ESP_OK;
    *watermark = dev->settings.watermark;
    return status;
}

esp_err_t LSM_setFIFOpackets(LSM_DriverHandle_t *device, LSM_PktType_t *pT)
{
    esp_err_t status = ESP_OK;
    uint8_t pktType = *pT;
    uint8_t writeA = 0, writeB = 0, regVal = 0, blank = 0, pktSize = 0;
    uint16_t writeVal = 0;

    if (pktType & LSM_PKT1_GYRO)
    {
        writeA |= (1 << 3);
        pktSize += 2;
    }

    if (pktType & LSM_PKT2_ACCL)
    {
        writeA |= 1;
        pktSize += 2;
    }

    if (pktType & LSM_PKT3_SENSHUB)
    {
        writeB |= 1;
        pktSize += 2;
    }

    if (pktType & LSM_PKT4_STEP_OR_TEMP)
    {
        writeB |= (1 << 3);
        pktSize += 2;
    }

    writeVal = (((uint16_t)writeA << 8) | writeB);

    status = gcd_i2c_write_address(device->commsChannel, device->devAddr, LSM_FIFO_CTRL3_REG, 2, (uint8_t *)&writeVal);

    /** restart the fifo (this clears old packets) **/
    status = gcd_i2c_read_address(device->commsChannel, device->devAddr, LSM_FIFO_CTRL5_REG, 1, &regVal);
    /** clear fifo mode to zero, wait short time and restore **/
    status = gcd_i2c_write_address(device->commsChannel, device->devAddr, LSM_FIFO_CTRL5_REG, 1, &blank);
    vTaskDelay(pdMS_TO_TICKS(GENERIC_I2C_COMMS_SHORTWAIT_MS));
    status = gcd_i2c_write_address(device->commsChannel, device->devAddr, LSM_FIFO_CTRL5_REG, 1, &regVal);

    return status;
}

esp_err_t LSM_readFifoBlock(LSM_DriverHandle_t *device, uint16_t *length)
{

    esp_err_t status = ESP_OK;

    return status;
}

/**** INTERUPT SETTINGS  ****/

esp_err_t LSM_configInt(LSM_DriverHandle_t *device, uint8_t intNum, LSM_interrupt_t intr)
{
    esp_err_t status = ESP_OK;
    uint8_t writeval = 0, regval = 0;

    if (intr >= LSM_INT_TYPE_END)
    {
        status = ESP_ERR_INVALID_ARG;
    }
    else if (intNum == 1 && device->i1Pin > 0)
    {
        writeval = (intr == LSM_INT_TYPE_CLEAR) ? 0 : (1 << (intr - 1));
        status = gcd_i2c_read_address(device->commsChannel, device->devAddr, LSM_INT1_CTRL_REG, 1, &regval);
        writeval = (writeval == 0) ? 0 : writeval | regval;
        status = gcd_i2c_write_address(device->commsChannel, device->devAddr, LSM_INT1_CTRL_REG, 1, &writeval);
    }
    else if (intNum == 2 && device->i2Pin > 0)
    {
        writeval = (intr == LSM_INT_TYPE_CLEAR) ? 0 : (1 << (intr - 1));
        status = gcd_i2c_read_address(device->commsChannel, device->devAddr, LSM_INT2_CTRL_REG, 1, &regval);
        writeval = (writeval == 0) ? 0 : writeval | regval;
        status = gcd_i2c_write_address(device->commsChannel, device->devAddr, LSM_INT2_CTRL_REG, 1, &writeval);
    }
    else
    {
        ESP_LOGE("LSM_DRIVER", "Error: Pin not configured!");
        status = ESP_ERR_NOT_FOUND;
    }

    return status;
}
