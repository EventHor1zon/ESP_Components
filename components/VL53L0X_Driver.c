/***************************************
* \file     VL53L0X_Driver.c
* \brief    My port of the VL53L0X API to the esp32
*           One fun error - 
*               in api_core.c, division by zero bug. 
*               added some fix code
*               Still don't know how to get CMake to build other dirs
*               so for now all the api files live in components
* \date     March 2021
* \author   RJAM
****************************************/

/********* Includes *******************/

#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "VL53L0X_Driver.h"
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"


/****** Function Prototypes ***********/

static VL53L0X_Dev_t dev = {0};
/************ ISR *********************/

/****** Private Data ******************/

const char *VL53_TAG = "VL53L0X";

/****** Private Functions *************/

static void vl53_driver_task(void *arg) {

    VL53L0X_DEV Dev = (VL53L0X_DEV)arg;

    uint8_t rdy;
    VL53L0X_RangingMeasurementData_t data = {0};

    while(1) {
        vl53_UpdateMeasurement(Dev);
        vTaskDelay(50);
    }

}

/****** Global Data *******************/

/****** Global Functions *************/



VL53L0X_DEV vl53_init() {

    VL53L0X_Error ret = VL53L0X_ERROR_NONE;
    VL53L0X_DEV Dev = &dev;
    dev.comms_speed_khz = 400;
    dev.comms_type = 1;
    dev.I2cDevAddr = 0x29;
    uint32_t spads = 0;
    uint8_t iaspads = 0, Vhv = 0, phase = 0;

    /** init the data - this init function should not be run more than once **/
    ret = VL53L0X_DataInit(Dev);
    if(ret != 0) {
        printf("DI: Ret was: %u\n", ret);
    } 
    if(ret == 0) {
        /** Get the device info **/
        VL53L0X_DeviceInfo_t info = {0};

        ret = VL53L0X_get_device_info(Dev, &info);
        if(ret != 0) {
            printf("GDI Ret was: %u\n", ret);
        } else {
            ESP_LOGI(VL53_TAG, "Got info -- ");
            ESP_LOGI(VL53_TAG, "Name: %s", info.Name);
            ESP_LOGI(VL53_TAG, "Type: %s", info.Type);
            ESP_LOGI(VL53_TAG, "ProductID: %s", info.ProductId);
            ESP_LOGI(VL53_TAG, "Product Type: %u", info.ProductType);
            ESP_LOGI(VL53_TAG, "ProductRev: %u-%u", info.ProductRevisionMajor, info.ProductRevisionMinor);
        }
    }
    if(ret == 0) {
        ret = VL53L0X_StaticInit(Dev);
        ESP_LOGI(VL53_TAG, "SI Ret was: %i\n", ret);

        ret = VL53L0X_PerformRefSpadManagement(Dev, &spads, &iaspads);
        if(ret != 0) {
            printf("PRSM Ret was: %i\n", ret);
        } else {
            ret = VL53L0X_PerformRefCalibration(Dev, &Vhv, &phase);
            if(ret != 0) {
                printf("PRC Ret was: %i\n", ret);
            }
        }
    }

    if(ret == 0) {
        ret = VL53L0X_SetOffsetCalibrationDataMicroMeter(Dev, 100); /** TODO: This, better **/
        if(ret != 0) {
            printf("PRC Ret was: %i\n", ret);
        }
    }

    if(ret == 0) {
        ret = VL53L0X_SetXTalkCompensationRateMegaCps(Dev, 0);
        ret |= VL53L0X_SetXTalkCompensationEnable(Dev, 1);
        if(ret != 0) {
            printf("XTK Ret was: %i\n", ret);
        }
    }

    if(ret == 0) {
        ret = VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
        if(ret != 0) {
            printf("SDM Ret was: %i\n", ret);
        }
    }

    if(ret == 0){
        ret = VL53L0X_StartMeasurement(Dev);
        if(ret != 0) {
            printf("SM Ret was: %i\n", ret);
        }
    }

    if(xTaskCreate(vl53_driver_task, "vl53_driver_task", 5012, Dev, 3, NULL) != pdTRUE) {
        ESP_LOGE(VL53_TAG, "Error creating driver task!");
    }

    return Dev;

}


esp_err_t vl53_getDeviceMode(VL53L0X_DEV Dev, uint8_t *val) {

    esp_err_t ret = VL53L0X_GetDeviceMode(Dev, val);
    return ret;
}

esp_err_t vl53_setDeviceMode(VL53L0X_DEV Dev, uint8_t *val) {

    uint8_t mode = *val;
    esp_err_t ret = ESP_OK;

    if( mode != VL53L0X_DEVICEMODE_SINGLE_RANGING &&
        mode != VL53L0X_DEVICEMODE_CONTINUOUS_RANGING &&
        mode != VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING && 
        mode != VL53L0X_DEVICEMODE_SINGLE_HISTOGRAM &&
        mode != VL53L0X_HISTOGRAMMODE_REFERENCE_ONLY &&
        mode != VL53L0X_HISTOGRAMMODE_RETURN_ONLY && 
        mode != VL53L0X_HISTOGRAMMODE_BOTH
    ) {
        ret = ESP_ERR_INVALID_ARG;
    }
    else {
        ret = VL53L0X_SetDeviceMode(Dev, mode); 
    }

    return ret;
}

esp_err_t vl53_UpdateMeasurement(VL53L0X_DEV Dev) {

    esp_err_t ret = ESP_OK;
    uint8_t rdy = 0;
    uint8_t notmeasured = VL53_MEASURE_WAIT_RETRIES;
    uint8_t mode = Dev->dev_mode;

    if(mode == VL53L0X_DEVICEMODE_CONTINUOUS_RANGING) {
        if(PALDevDataGet(Dev, PalState) != VL53L0X_STATE_RUNNING) {
            ret = VL53L0X_StartMeasurement(Dev);
        }
    }
    else if(mode == VL53L0X_DEVICEMODE_SINGLE_RANGING) {
        ret = VL53L0X_StartMeasurement(Dev);
    }
    while(notmeasured) {
        uint8_t ret = VL53L0X_GetMeasurementDataReady(Dev, &rdy);
        if(rdy) {
            ret = VL53L0X_GetRangingMeasurementData(Dev, &(Dev->rangeData));
            ESP_LOGI(VL53_TAG, "status [%u]\tRange (%u)\n", Dev->rangeData.RangeStatus, Dev->rangeData.RangeMilliMeter);

            if(Dev->rangeData.RangeStatus != 0) {
                ESP_LOGE(VL53_TAG, "Error in measurement: %i", ret);
                ret = ESP_ERR_INVALID_RESPONSE;
            }
            notmeasured = 0;       
        } else {
            vTaskDelay(5);
            ret = ESP_ERR_TIMEOUT;
            notmeasured--;
        }
    }
    
    return ret;
}


esp_err_t vl53_setDevicePwr(VL53L0X_DEV Dev, uint8_t *val) {

    esp_err_t ret = ESP_OK;
    uint8_t pwr = *val;

    if(pwr != VL53L0X_POWERMODE_STANDBY_LEVEL1 &&
        pwr != VL53L0X_POWERMODE_IDLE_LEVEL1) {
            ret = ESP_ERR_INVALID_ARG;
    }
    else {
        VL53L0X_SetPowerMode(Dev, pwr);
    }

    return ret;
}



esp_err_t vl53_getDevicePwr(VL53L0X_DEV Dev, uint8_t *val) {
    
    esp_err_t ret = ESP_OK;



    return ret;

}