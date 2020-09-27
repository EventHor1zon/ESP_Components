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
#include <stdint.h>

#include "LSM_Driver.h"

#include "genericCommsDriver.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/****** Function Prototypes ***********/

/************ ISR *********************/

static void ISR_int1(void *args)
{
}

static void ISR_int2(void *args)
{
}
/****** Private Data ******************/

LSM_DriverSettings_t *device = NULL;
/****** Private Functions *************/

static void LSM_processAccel(LSM_DriverSettings_t *dev)
{
    float factor = 0;

    switch (dev->settings.accelScale)
    {
    case LSM_ACCSCALE_2G:
        factor = 0.61;
        break;
    case LSM_ACCSCALE_4G:
        factor = 0.122;
        break;
    case LSM_ACCSCALE_8G:
        factor = 0.244;
        break;
    case LSM_ACCSCALE_16G:
        factor = 0.488;
        break;
    default:
        factor = 0.488;
        break;
    }

    dev->measurements.calibAccelX = ((uint16_t)dev->measurements.rawAccel[1] << 8) | ((uint16_t)dev->measurements.rawAccel[0]) * factor;
    dev->measurements.calibAccelY = ((uint16_t)dev->measurements.rawAccel[3] << 8) | ((uint16_t)dev->measurements.rawAccel[2]) * factor;
    dev->measurements.calibAccelZ = ((uint16_t)dev->measurements.rawAccel[5] << 8) | ((uint16_t)dev->measurements.rawAccel[4]) * factor;
}

static void LSM_processGyro(LSM_DriverSettings_t *dev)
{
    float factor = 0;

    switch (dev->settings.gyroScale)
    {
    case LSM_GYRO_SCALE_250DPS:
        factor = 8.75;
        break;
    case LSM_GYRO_SCALE_500DPS:
        factor = 17.5;
        break;
    case LSM_GYRO_SCALE_1000DPS:
        factor = 35;
        break;
    case LSM_GYRO_SCALE_2000DPS:
        factor = 70;
        break;
    default:
        factor = 70;
        break;
    }

    dev->measurements.calibGyroX = ((uint16_t)(dev->measurements.rawGyro[1] << 8) | (uint16_t)(dev->measurements.rawGyro[0])) * factor;
    dev->measurements.calibGyroY = ((uint16_t)(dev->measurements.rawGyro[3] << 8) | (uint16_t)(dev->measurements.rawGyro[2])) * factor;
    dev->measurements.calibGyroZ = ((uint16_t)(dev->measurements.rawGyro[5] << 8) | (uint16_t)(dev->measurements.rawGyro[4])) * factor;
}

static esp_err_t LSM_getFIFOpktCount(LSM_DriverSettings_t *dev, uint16_t *count)
{
    esp_err_t status = ESP_OK;
    uint8_t rxBuffer[2] = {0};
    uint8_t msbMask = 0b1111;
    uint16_t fifoCount = 0;

    status = genericI2CReadFromAddress(dev->commsChannel, (uint8_t)LSM_I2C_ADDR, LSM_FIFO_STATUS1_REG, 2, rxBuffer);

    if (status == ESP_OK)
    {
        msbMask &= rxBuffer[1];
        fifoCount = ((uint16_t)msbMask << 8 | (uint16_t)rxBuffer[0]);
        *count = fifoCount;
    }

    return status;
}

/****** Global Data *******************/

/****** Global Functions *************/

esp_err_t LSM_init(LSM_initData_t *initData)
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
        device = (LSM_DriverSettings_t *)heap_caps_calloc(1, sizeof(LSM_DriverSettings_t), MALLOC_CAP_8BIT);

        if (device == NULL)
        {
            ESP_LOGE("LSM Driver", "Error assigning mem for the device structure");
            initStatus = ESP_ERR_NO_MEM;
        }
        else
        {
            /** copy over the comms info */
            device->clkPin = initData->clockPin;
            device->dtPin = initData->dataPin;
            device->commsHandle = NULL;

            if ((initData->commMode == LSM_DEVICE_COMM_MODE_SPI || initData->commMode == LSM_DEVICE_COMM_MODE_SPI_3))
            {
                /** TODO: setup spi  **/
                return ESP_ERR_NOT_SUPPORTED;
            }
            else
            {
                device->commMode = LSM_DEVICE_COMM_MODE_I2C;
                if (initData->addrPinState)
                {
                    device->devAddr = LSM_I2C_ADDR;
                }
                else
                {
                    device->devAddr = LSM_I2C_ADDR + 1;
                }
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
            gpioInit.intr_type = GPIO_INTR_LOW_LEVEL;
            gpioInit.mode = GPIO_MODE_INPUT;
            gpioInit.pin_bit_mask = gpioMask;
            gpioInit.pull_up_en = GPIO_PULLUP_ENABLE;
            gpioInit.pull_down_en = GPIO_PULLDOWN_DISABLE;

            initStatus = gpio_config(&gpioInit);

            if (initStatus == ESP_OK)
            {
                initStatus = gpio_install_isr_service(ESP_INTR_FLAG_LOWMED);
                if (initStatus == ESP_ERR_INVALID_STATE)
                {
                    /** driver already initialised, don't freak out */
                    initStatus = ESP_OK;
                }

                if (initStatus == ESP_OK)
                {
                    if (initData->int1Pin)
                    {
                        gpio_isr_handler_add(initData->int1Pin, ISR_int1, NULL);
                    }
                    if (initData->int2Pin)
                    {
                        gpio_isr_handler_add(initData->int2Pin, ISR_int2, NULL);
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
                }
                else
                {
                    initStatus = ESP_ERR_NO_MEM;
                }
            }
        }

        if (initStatus == ESP_OK)
        {
            /** initialise device settings **/
            LSM_setAccelODRMode(device, initData->accelRate);
            LSM_setGyroODRMode(device, initData->gyroRate);
        }
    }

    return initStatus;
}

esp_err_t LSM_deInit(LSM_DriverSettings_t *dev)
{
    if (dev->fifoBuffer != NULL)
    {
        heap_caps_free(dev->fifoBuffer);
    }

    free(dev);

    return ESP_OK;
}

/******* SAMPLE SETTINGS (BASIC) ********/

esp_err_t LSM_setOpMode(LSM_DriverSettings_t *dev, LSM_OperatingMode_t *mode)
{
    esp_err_t status = ESP_OK;
    uint8_t regVals[2] = {0};
    uint8_t writeVals[2] = {0};
    status = genericI2CReadFromAddress(dev->commsChannel, dev->devAddr, LSM_CTRL9_XL_REG, 2, regVals);

    /** enable all axis for eac dev **/
    if (*mode == LSM_OPMODE_ACCEL_ONLY)
    {
        writeVals[0] = ((((LSM_CTRL9_ACCEL_Z_EN_BIT) | (LSM_CTRL9_ACCEL_Y_EN_BIT) | (LSM_CTRL9_ACCEL_X_EN_BIT)) << 3) | (regVals[0] & 0b111)));
    }
    else if (*mode == LSM_OPMODE_GYRO_ACCEL)
    {
        writeVals[1] = ((((LSM_CTRL10_GYRO_Z_EN_BIT) | (LSM_CTRL10_GYRO_Y_EN_BIT) | (LSM_CTRL10_GYRO_X_EN_BIT)) << 3) | (regVals[1] & 0b111));
    }
    else if (*mode == LSM_OPMODE_GYRO_ACCEL)
    {
        writeVals[0] = ((((LSM_CTRL9_ACCEL_Z_EN_BIT) | (LSM_CTRL9_ACCEL_Y_EN_BIT) | (LSM_CTRL9_ACCEL_X_EN_BIT)) << 3) | (regVals[0] & 0b111)));
        writeVals[1] = ((((LSM_CTRL10_GYRO_Z_EN_BIT) | (LSM_CTRL10_GYRO_Y_EN_BIT) | (LSM_CTRL10_GYRO_X_EN_BIT)) << 3) | (regVals[1] & 0b111));
    }
    else
    {
        status = ESP_ERR_INVALID_ARG;
    }

    if (status == ESP_OK)
    {
        status = genericI2CwriteToAddress(dev->commsChannel, dev->devAddr, LSM_CTRL9_XL_REG, 2, writeVals);
    }

    return status;
}

esp_err_t LSM_setAccelODRMode(LSM_DriverSettings_t *dev, LSM_AccelODR_t mode)
{
    esp_err_t status = ESP_OK;
    uint8_t accelEn = 0, regVal = 0;

    if (mode > LSM_ACCODR_6_66KHZ)
    {
        status = ESP_ERR_INVALID_ARG;
    }
    else
    {
        /** get axis enabled status */
        status = genericI2CReadFromAddress(dev->commsChannel, dev->devAddr, LSM_CTRL9_XL_REG, 1, &accelEn);
        /** if no axis are active and mode > LSM_ACCELPWR_OFF turn on axis **/
        if (!(accelEn) && mode > LSM_ACCODR_PWR_OFF)
        {
            accelEn |= ((LSM_CTRL9_ACCEL_Z_EN_BIT) | (LSM_CTRL9_ACCEL_Y_EN_BIT) | (LSM_CTRL9_ACCEL_X_EN_BIT));
            status = genericI2CwriteToAddress(dev->commsChannel, dev->devAddr, LSM_CTRL9_XL_REG, 1, &accelEn);
        }
        /** get register value, clear mode & set new **/
        status = genericI2CReadFromAddress(dev->commsChannel, dev->devAddr, LSM_CTRL1_XL_REG, 1, &regVal);
        regVal &= 0b1111;
        regVal |= (mode << 4);
        status = genericI2CwriteToAddress(dev->commsChannel, dev->devAddr, LSM_CTRL1_XL_REG, 1, &regVal);
    }
    return status;
}

esp_err_t LSM_setGyroODRMode(LSM_DriverSettings_t *dev, LSM_GyroODR_t mode)
{
    esp_err_t status = ESP_OK;
    uint8_t accelEn = 0, regVal = 0;

    if (mode > LSM_GYRO_ODR_1_66KHZ)
    {
        status = ESP_ERR_INVALID_ARG;
    }
    else
    {
        /** get axis enabled status */
        status = genericI2CReadFromAddress(dev->commsChannel, dev->devAddr, LSM_CTRL10_C_REG, 1, &accelEn);
        /** if no axis are active and mode > LSM_ACCELPWR_OFF turn on axis **/
        if (!(accelEn) && mode > LSM_GYRO_ODR_PWR_OFF)
        {
            accelEn |= ((LSM_CTRL10_GYRO_Z_EN_BIT) | (LSM_CTRL10_GYRO_Y_EN_BIT) | (LSM_CTRL10_GYRO_X_EN_BIT));
            status = genericI2CwriteToAddress(dev->commsChannel, dev->devAddr, LSM_CTRL10_C_REG, 1, &accelEn);
        }
        /** get register value, clear mode & set new **/
        status = genericI2CReadFromAddress(dev->commsChannel, dev->devAddr, LSM_CTRL2_G_REG, 1, &regVal);
        regVal &= 0b1111;
        regVal |= (mode << 4);
        status = genericI2CwriteToAddress(dev->commsChannel, dev->devAddr, LSM_CTRL2_G_REG, 1, &regVal);
    }
    return status;
}

esp_err_t LSM_setFIFOmode(LSM_DriverSettings_t *dev, LSM_FIFOMode_t mode)
{
    uint8_t regvalue = 0, writevalue = 0;
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
        status = genericI2CReadFromAddress(dev->commsChannel, dev->devAddr, LSM_FIFO_CTRL5_REG, 1, &regvalue);
        regvalue &= (0b11111000); /** clear low 3 bits **/
        writevalue |= regvalue;   /** set mode **/
        status = genericI2CwriteToAddress(dev->commsChannel, dev->devAddr, LSM_FIFO_CTRL5_REG, 1, &writevalue);
    }

    return status;
}

/** CORE FUNCTIONALITY **/

esp_err_t LSM_sampleLatest(LSM_DriverSettings_t *dev)
{

    esp_err_t status = ESP_OK;
    uint8_t regVal = 0, wait = 0;

    /** check the status bit */

    status = genericI2CReadFromAddress(dev->commsChannel, dev->devAddr, LSM_STATUS_REG, 1, &regVal);
    switch (dev->settings.opMode)
    {
    case LSM_OPMODE_ACCEL_ONLY:
        if (!(regVal & LSM_STATUS_ACCEL_AVAIL_BIT))
        {
            wait = 1;
            while (wait)
            {
                vTaskDelay(pdMS_TO_TICKS(GENERIC_I2C_COMMS_SHORTWAIT_MS));
                regVal = genericI2CReadFromAddress(dev->commsChannel, dev->devAddr, LSM_STATUS_REG, 1, &regVal);
                if (regVal & LSM_STATUS_ACCEL_AVAIL_BIT)
                {
                    wait = 0;
                }
            }
        }
        status = genericI2CReadFromAddress(dev->commsChannel, dev->devAddr, LSM_ACCELX_LSB_REG, 6, dev->measurements.rawAccel);
        LSM_processAccel(dev);
        break;

    case LSM_OPMODE_GYRO_ONLY:
        if (!(regVal & LSM_STATUS_GYRO_AVAIL_BIT))
        {
            wait = 1;
            while (wait)
            {
                vTaskDelay(pdMS_TO_TICKS(GENERIC_I2C_COMMS_SHORTWAIT_MS));
                regVal = genericI2CReadFromAddress(dev->commsChannel, dev->devAddr, LSM_STATUS_REG, 1, &regVal);
                if (regVal & LSM_STATUS_GYRO_AVAIL_BIT)
                {
                    wait = 0;
                }
            }
        }
        status = genericI2CReadFromAddress(dev->commsChannel, dev->devAddr, LSM_GYROX_LSB_REG, 6, dev->measurements.rawGyro);
        LSM_processGyro(dev);

    case LSM_OPMODE_GYRO_ACCEL:
        if (!(regVal & (LSM_STATUS_ACCEL_AVAIL_BIT & LSM_STATUS_GYRO_AVAIL_BIT)))
        {
            wait = 1;
            while (wait)
            {
                vTaskDelay(pdMS_TO_TICKS(GENERIC_I2C_COMMS_SHORTWAIT_MS));
                regVal = genericI2CReadFromAddress(dev->commsChannel, dev->devAddr, LSM_STATUS_REG, 1, &regVal);
                if (regVal & (LSM_STATUS_ACCEL_AVAIL_BIT & LSM_STATUS_GYRO_AVAIL_BIT))
                {
                    wait = 0; /** wait until both gyro and accel data available **/
                }
            }
        }
        status = genericI2CReadFromAddress(dev->commsChannel, dev->devAddr, LSM_GYROX_LSB_REG, 12, dev->measurements.rawGyro);
        LSM_processAccel(dev);
        LSM_processGyro(dev);

    default:
        break;
    }
}

esp_err_t LSM_getGyroX(LSM_DriverSettings_t *dev, float *x)
{
    esp_err_t status = ESP_OK;
    *x = dev->measurements.calibGyroX;
    return status;
}

esp_err_t LSM_getGyroY(LSM_DriverSettings_t *dev, float *y)
{
    esp_err_t status = ESP_OK;
    *y = dev->measurements.calibGyroY;
    return status;
}

esp_err_t LSM_getGyroZ(LSM_DriverSettings_t *dev, float *z)
{
    esp_err_t status = ESP_OK;
    *z = dev->measurements.calibGyroZ;
    return status;
}

esp_err_t LSM_getAccelX(LSM_DriverSettings_t *dev, float *x)
{
    esp_err_t status = ESP_OK;
    *x = dev->measurements.calibAccelX;
    return status;
}

esp_err_t LSM_getAccelY(LSM_DriverSettings_t *dev, float *y)
{
    esp_err_t status = ESP_OK;
    *y = dev->measurements.calibAccelY;
    return status;
}

esp_err_t LSM_getAccelZ(LSM_DriverSettings_t *dev, float *z)
{
    esp_err_t status = ESP_OK;
    *z = dev->measurements.calibAccelZ;
    return status;
}

/** FIFO SETTINGS **/

esp_err_t LSM_getFIFOmode(LSM_DriverSettings_t *dev, uint8_t *mode)
{
    esp_err_t status = ESP_OK;

    return status;
}

esp_err_t LSM_setFIFOwatermark(uint16_t watermark)
{
    esp_err_t status = ESP_OK;
    /** TODO: this **/
    return ESP_OK;
}

esp_err_t LSM_setFIFOpackets(LSM_DriverSettings_t *device, LSM_PktType_t pktType)
{
    esp_err_t status = ESP_OK;

    uint8_t writeA = 0, writeB = 0, regVal = 0, blank = 0;

    if (pktType & LSM_PKT1_GYRO)
    {
        writeA |= (1 << 3);
    }

    if (pktType & LSM_PKT2_ACCL)
    {
        writeA |= 1;
    }

    if (pktType & LSM_PKT3_SENSHUB)
    {
        writeB |= 1;
    }

    if (pktType & LSM_PKT4_STEP_OR_TEMP)
    {
        writeB |= (1 << 3);
    }

    if (writeA)
    {
        status = genericI2CwriteToAddress(device->commsChannel, device->devAddr, LSM_FIFO_CTRL3_REG, 1, &writeA);
    }
    if (writeB)
    {
        status = genericI2CwriteToAddress(device->commsChannel, device->devAddr, LSM_FIFO_CTRL4_REG, 1, &writeB);
    }

    /** restart the fifo (this clears old packets) **/
    status = genericI2CReadFromAddress(device->commsChannel, device->devAddr, LSM_FIFO_CTRL5_REG, 1, &regVal);
    /** clear fifo mode to zero, wait short time and restore **/
    status = genericI2CwriteToAddress(device->commsChannel, device->devAddr, LSM_FIFO_CTRL5_REG, 1, &blank);
    vTaskDelay(pdMS_TO_TICKS(GENERIC_I2C_COMMS_SHORTWAIT_MS));
    status = genericI2CwriteToAddress(device->commsChannel, device->devAddr, LSM_FIFO_CTRL5_REG, 1, &regVal);

    return status;
}

esp_err_t LSM_readFifoBlock(LSM_DriverSettings_t *device, uint16_t length)
{

    esp_err_t status = ESP_OK;

    return status;
}

/**** INTERUPT SETTINGS  ****/

esp_err_t LSM_configInt(LSM_DriverSettings_t *device, uint8_t intNum, LSM_interrupt_t intr)
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
        status = genericI2CReadFromAddress(device->commsChannel, device->devAddr, LSM_INT1_CTRL_REG, 1, &regval);
        writeval = (writeval == 0) ? 0 : writeval | regval;
        status = genericI2CwriteToAddress(device->commsChannel, device->devAddr, LSM_INT1_CTRL_REG, 1, &writeval);
    }
    else if (intNum == 2 && device->i2Pin > 0)
    {
        writeval = (intr == LSM_INT_TYPE_CLEAR) ? 0 : (1 << (intr - 1));
        status = genericI2CReadFromAddress(device->commsChannel, device->devAddr, LSM_INT2_CTRL_REG, 1, &regval);
        writeval = (writeval == 0) ? 0 : writeval | regval;
        status = genericI2CwriteToAddress(device->commsChannel, device->devAddr, LSM_INT2_CTRL_REG, 1, &writeval);
    }
    else
    {
        ESP_LOGE("LSM_DRIVER", "Error: Pin not configured!");
        status = ESP_ERR_NOT_FOUND;
    }

    return status;
}

esp_err_t LSM_getWhoAmI(LSM_DriverSettings_t *device, uint8_t *whoami)
{
    esp_err_t status = ESP_OK;

    status = genericI2CReadFromAddress(device->commsChannel, device->devAddr, LSM_WHOAMI_REG, 1, whoami);

    return status;
}
