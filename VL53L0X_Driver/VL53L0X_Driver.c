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
#include "./VL53L0X/include/vl53l0x_api.h"
#include "./VL53L0X/include/vl53l0x_platform.h"

#ifdef CONFIG_USE_PERIPH_MANAGER
const parameter_t vl53_param_map[vl53_param_len] = {
    {"Device Mode", 1, &vl53_getDeviceMode, &vl53_setDeviceMode, NULL, DATATYPE_UINT8, 3, (GET_FLAG | SET_FLAG) }, 
    {"Power Mode", 2, &vl53_getDevicePwr, &vl53_setDevicePwr, NULL, DATATYPE_UINT8, 2, (GET_FLAG | SET_FLAG) }, 
    {"Sample Config", 3, &vl53_getDeviceMode, &vl53_setDeviceMode, NULL, DATATYPE_UINT8, 3, (GET_FLAG | SET_FLAG) }, 
    {"Update Measurement", 4, NULL, NULL, &vl53_UpdateMeasurement, 0, 0, ( ACT_FLAG ) }, 
};
#endif

/****** Function Prototypes ***********/

/** Driver Task **/
static void vl53_driver_task(void *arg);

/** set device config profile **/
static esp_err_t SetSensorConfig(VL53L0X_DEV Dev, VL53L0X_config_t vl_config);

/************ ISR *********************/

/****** Private Data ******************/

static VL53L0X_Dev_t dev = {0};

const char *VL53_TAG = "VL53L0X";

/****** Private Functions *************/



/**************************************************************************/
/*! Configs Borrowed from Adafruit's VL53L0X driver. Thanks :)
    @brief  Configure the sensor for one of the ways the example ST
    sketches configure the sensors for different usages.
    @param  vl_config Which configureation you are trying to configure for
    It should be one of the following
        VL53L0X_SENSE_DEFAULT
        VL53L0X_SENSE_LONG_RANGE
        VL53L0X_SENSE_HIGH_SPEED,
        VL53L0X_SENSE_HIGH_ACCURACY
    @returns True if config was set successfully, False otherwise
*/
/**************************************************************************/
static esp_err_t SetSensorConfig(VL53L0X_DEV Dev, VL53L0X_config_t vl_config) {
    // All of them appear to configure a few things

    // Serial.print(F("VL53L0X: configSensor "));
    // Serial.println((int)vl_config, DEC);
    // Enable/Disable Sigma and Signal check
    esp_err_t Status = VL53L0X_SetLimitCheckEnable(
        Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(
            Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    }

    if (Status != VL53L0X_ERROR_NONE) {
        return false;
    }

    switch (vl_config) {
        
        case VL53L0X_SENSE_DEFAULT:
        // Taken directly from SDK vl5310x_SingleRanging_example.c
        // Maybe should convert to helper functions but...
            ESP_LOGI(VL53_TAG, "Configuring VL53L0X_SENSE_DEFAULT");
            
            if (Status == VL53L0X_ERROR_NONE) {
            Status = VL53L0X_SetLimitCheckEnable(
                Dev, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1);
            }

            if (Status == VL53L0X_ERROR_NONE) {
            Status = VL53L0X_SetLimitCheckValue(
                Dev, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
                (FixPoint1616_t)(1.5 * 0.023 * 65536));
            }
            break;
            
        case VL53L0X_SENSE_LONG_RANGE:
            Status = VL53L0X_SetLimitCheckValue(
                Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                (FixPoint1616_t)(0.1 * 65536));
            if (Status == VL53L0X_ERROR_NONE) {
            Status = VL53L0X_SetLimitCheckValue(Dev,
                                                VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                                                (FixPoint1616_t)(60 * 65536));
            }
            if (Status == VL53L0X_ERROR_NONE) {
            Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev, 33000);
            }

            if (Status == VL53L0X_ERROR_NONE) {
            Status = VL53L0X_SetVcselPulsePeriod(Dev,
                                                VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
            }
            if (Status == VL53L0X_ERROR_NONE) {
            Status = VL53L0X_SetVcselPulsePeriod(
                Dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
            }
            break;

        case VL53L0X_SENSE_HIGH_SPEED:
            ESP_LOGI(VL53_TAG, "Configuring VL53L0X_SENSE_HIGH_SPEED");
            Status = VL53L0X_SetLimitCheckValue(
                Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                (FixPoint1616_t)(0.25 * 65536));
            if (Status == VL53L0X_ERROR_NONE) {
                Status = VL53L0X_SetLimitCheckValue(Dev,
                                                    VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                                                    (FixPoint1616_t)(32 * 65536));
            }
            if (Status == VL53L0X_ERROR_NONE) {
                Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev, 30000);
            }
            break;

        case VL53L0X_SENSE_HIGH_ACCURACY:
        // increase timing budget to 200 ms

            if (Status == VL53L0X_ERROR_NONE) {
                VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                                    (FixPoint1616_t)(0.25 * 65536));
            }
            if (Status == VL53L0X_ERROR_NONE) {
                VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                                    (FixPoint1616_t)(18 * 65536));
            }
            if (Status == VL53L0X_ERROR_NONE) {
                VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev, 200000);
            }
            // Not sure about ignore threhold, try turnning it off...
            if (Status == VL53L0X_ERROR_NONE) {
                Status = VL53L0X_SetLimitCheckEnable(
                    Dev, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 0);
            }
            break;
    }

  return Status;
}


static void vl53_driver_task(void *arg) {

    VL53L0X_DEV Dev = (VL53L0X_DEV)arg;
    esp_err_t ret = ESP_OK;
    ESP_LOGI(VL53_TAG, "Starting VL53L0X Driver Task");

    while(1) {

        ret = vl53_UpdateMeasurement(Dev);

        if(ret != ESP_OK) {
            ESP_LOGE(VL53_TAG, "Update Measurements failed!");
        } else {
            ESP_LOGI(VL53_TAG, "[%u]\tRange (%u)\tMeasureT %u  (ts: %u)", Dev->rangeData.RangeStatus, Dev->rangeData.RangeMilliMeter, Dev->rangeData.MeasurementTimeUsec, Dev->rangeData.TimeStamp);
        }
        
        vTaskDelay(pdMS_TO_TICKS(500));
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
    dev.sample_mode = 0;
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

    // if(ret == 0) {
    //     ret = SetSensorConfig(Dev, VL53L0X_SENSE_HIGH_ACCURACY);
    //     if(ret != 0) {
    //         printf("SSC Ret was: %i\n", ret);
    //     }
    // }

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
        mode != VL53L0X_DEVICEMODE_SINGLE_HISTOGRAM 
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
        ret = VL53L0X_GetMeasurementDataReady(Dev, &rdy);
        /** a small delay loop **/
        if(rdy) {
            ret = VL53L0X_GetRangingMeasurementData(Dev, &(Dev->rangeData));
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

    ret = VL53L0X_GetPowerMode(Dev, val);

    return ret;
}


esp_err_t vl53_getDeviceSampleConfig(VL53L0X_DEV Dev, uint8_t *val) {
    
    esp_err_t ret = ESP_OK;

    *val = Dev->sample_mode;

    return ret;
}


esp_err_t vl53_setDeviceSampleConfig(VL53L0X_DEV Dev, uint8_t *val) {
    
    esp_err_t ret = ESP_OK;
    uint8_t conf = *val;

    if(conf != VL53L0X_SENSE_DEFAULT &&
        conf != VL53L0X_SENSE_LONG_RANGE &&
        conf != VL53L0X_SENSE_HIGH_SPEED &&
        conf != VL53L0X_SENSE_HIGH_ACCURACY
    ) {
        ESP_LOGE(VL53_TAG, "Invalid Sample config");
        ret = ESP_ERR_INVALID_ARG;
    } 
    else {
        ret = SetSensorConfig(Dev, conf);
    }

    return ret;
}