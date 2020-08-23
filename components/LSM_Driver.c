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

#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/****** Function Prototypes ***********/

/************ ISR *********************/

void ISR_int1
{
    return;
}

void ISR_int2
{
    return;
}
/****** Private Data ******************/

static LSM_DeviceSettings_t *device = NULL;
/****** Private Functions *************/

/****** Global Data *******************/

/****** Global Functions *************/

/** \brief LSM_init()
 *      
 *          initilise the LSM driver. Assumes a single device. 
 *          takes a pointer to an LSM init struct
 * 
 *  \param LSM_initData_t initData 
 *  \return ESP_OK or error
 */
esp_err_t LSM_init(LSM_initData_t *initData)
{

    esp_err_t initStatus = ESP_OK;

    if (initData == NULL)
    {
        ESP_LOGE("LSM Driver", "Error - NULL ptr to init data");
        initStatus = ESP_ERR_INVALID_ARG;
    }
    else if ((initData->commMode = LSM_DEVICE_COMM_MODE_SPI && initData->commsChannel == 0) || initData->commsChannel > 2)
    {
        ESP_LOGE("LSM Driver", "Error - invalid comms channel");
        initStatus = ESP_ERR_INVALID_ARG;
    }
    else
    {
        /** allocate memory on the heap for the control structure */
        device = (LSM_DeviceSettings_t *)heap_caps_calloc(1, sizeof(LSM_DeviceSettings_t), MALLOC_CAP_8BIT);

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
                if (initStatus = ESP_ERR_INVALID_STATE)
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

        return initStatus;
    }

    /** 
 *  LSM_deinit() 
 *      tear down the LSM driver 
 *  
 *  \return ESP_OK or error
*/

    esp_err_t LSM_deInit();

    esp_err_t LSM_assignDMABuffer(LSM_DeviceSettings_t * device);
    esp_err_t LSM_setFIFOmode(LSM_FIFOMode_t mode);
    esp_err_t LSM_setFIFOwatermark(uint16_t watermark);
    esp_err_t LSM_getFIFOCount(uint16_t * count);
    esp_err_t LSM_setFIFOpackets(LSM_FifoPktCfg_t config, uint8_t fifoPacket);
    esp_err_t LSM_configInt(LSM_DeviceSettings_t * device, uint8_t intNum);

    esp_err_t LSM_readFifoBlock(LSM_DeviceSettings_t * device, uint16_t length);
    esp_err_t LSM_readWhoAmI(LSM_DeviceSettings_t * device);
