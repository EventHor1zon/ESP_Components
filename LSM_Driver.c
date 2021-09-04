/***************************************
* \file     LSM_Driver.c
* \brief    Driver for the LSM6DS3 Gyro and Accelerometer
*           Can be configured to use i2c or spi
*           Basic functionality for now, more complex later
*           This driver expects an initialised comms bus (spi/i2c)
*           but it will intiialise the gpio interrupt pins itself
*
*           There's so much stuff to do with fifo packets, maybe better to keep
*           the interface simple? 
*
*           Device datasheet: https://www.st.com/resource/en/datasheet/lsm6ds33.pdf
*           Device app note:  https://www.st.com/content/ccc/resource/technical/document/application_note/9c/d9/07/d0/d4/a9/45/00/DM00175930.pdf/files/DM00175930.pdf/jcr:content/translations/en.DM00175930.pdf
*
*           TODO: Add support for CBuffers
*           TODO: Finish control functions
*
* \date     Aug 2020
* \author   RJAM
****************************************/

/********* Includes *******************/
#include "esp_types.h"
#include <string.h>
#include <math.h>

#include "LSM_Driver.h"

#include "genericCommsDriver.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "Utilities.h"

/****** Global Data *******************/

LSM_DriverHandle_t *device = NULL;

#ifdef CONFIG_USE_PERIPH_MANAGER

#include "CommandAPI.h"

const parameter_t lsm_parameter_map[lsm_param_map_len] = {
    {"Gyro X", 1, &LSM_getGyroX, NULL, NULL, PARAMTYPE_FLOAT, 0, (GET_FLAG | STREAM_FLAG) },
    {"Gyro Y", 2, &LSM_getGyroY, NULL, NULL, PARAMTYPE_FLOAT, 0, (GET_FLAG | STREAM_FLAG) },
    {"Gyro Z", 3, &LSM_getGyroZ, NULL, NULL, PARAMTYPE_FLOAT, 0, (GET_FLAG | STREAM_FLAG) },
    {"Accel X", 4, &LSM_getAccelX, NULL, NULL, PARAMTYPE_FLOAT, 0, (GET_FLAG | STREAM_FLAG) },
    {"Accel Y", 5, &LSM_getAccelY, NULL, NULL, PARAMTYPE_FLOAT, 0, (GET_FLAG | STREAM_FLAG) },
    {"Accel Z", 6, &LSM_getAccelZ, NULL, NULL, PARAMTYPE_FLOAT, 0, (GET_FLAG | STREAM_FLAG) },
    {"Op Mode", 7, &LSM_getOpMode, &LSM_setOpMode, NULL, PARAMTYPE_UINT8, LSM_OPMODE_GYRO_ACCEL, (GET_FLAG | SET_FLAG)},
    {"Accel ODR", 8, &LSM_getAccelODRMode, &LSM_setAccelODRMode, NULL, PARAMTYPE_UINT8, LSM_ACCODR_6_66KHZ, (GET_FLAG | SET_FLAG)},
    {"Gyro ODR", 9, &LSM_getGyroODRMode, &LSM_setGyroODRMode, NULL, PARAMTYPE_UINT8, LSM_GYRO_ODR_1_66KHZ, (GET_FLAG | SET_FLAG)},
    {"Accel Fullscale", 10, &LSM_get_accel_full_scale, &LSM_set_accel_full_scale, NULL, PARAMTYPE_UINT8, LSM_ACCSCALE_8G, (GET_FLAG | SET_FLAG)},
    {"Gyro Fullscale", 11, &LSM_get_gyro_full_scale, &LSM_set_gyro_full_scale, NULL, PARAMTYPE_UINT8, LSM_GYRO_SCALE_125DPS, (GET_FLAG | SET_FLAG)},
    {"Accel BW mode", 12, &LSM_get_accel_bw_select_mode, &LSM_set_accel_bw_select_mode, NULL, PARAMTYPE_UINT8, LSM_ACC_BW_FROM_REG, (GET_FLAG | SET_FLAG)},
    {"Gyro Sleep", 13, &LSM_get_gyro_sleep_state, &LSM_set_gyro_sleep_state, NULL, PARAMTYPE_UINT8, LSM_GYRO_SLEEP_ENABLED, (GET_FLAG | SET_FLAG)},
    {"Merge Interrupts", 14, &LSM_get_int2_on_int1_state, &LSM_set_int2_on_int1_state, NULL, PARAMTYPE_UINT8, 1, (GET_FLAG | SET_FLAG)},
    {"Temp pkt en", 15, &LSM_get_temperature_pkt_en, &LSM_set_temperature_pkt_en, NULL, PARAMTYPE_BOOL, 1, (GET_FLAG | SET_FLAG)},
    {"Threshold stop", 16, &LSM_get_stop_on_fifo_thresh, &LSM_set_stop_on_fifo_thresh, NULL, PARAMTYPE_BOOL, 1, (GET_FLAG | SET_FLAG)},
    {"Gyro HighPower", 17, &LSM_get_gyro_hp_mode, &LSM_set_gyro_hp_mode, NULL, PARAMTYPE_BOOL, 1, (GET_FLAG | SET_FLAG)},
    {"G HPass mode", 18, &LSM_get_gyro_hpfilter_mode, &LSM_get_gyro_hpfilter_mode, NULL, PARAMTYPE_BOOL, 1, (GET_FLAG | SET_FLAG)},
    {"G HPass Cutoff", 19, &LSM_get_gyro_hp_filter_cutoff, &LSM_set_gyro_hp_filter_cutoff, NULL, PARAMTYPE_UINT8, LSM_GYRO_HPCUTOFF_16_3HZ, (GET_FLAG | SET_FLAG)},
    {"A LPass2 en", 20, &LSM_get_accel_lp2_filter_en, &LSM_set_accel_lp2_filter_en, NULL, PARAMTYPE_BOOL, 1, (GET_FLAG | SET_FLAG)},
    {"A HPass Cutoff", 21, &LSM_get_accel_hpfilter_mode, &LSM_set_accel_hpfilter_mode, NULL, PARAMTYPE_BOOL, 1, (GET_FLAG | SET_FLAG)},
    {"A HPass mode", 22, &LSM_get_accel_hp_filter_cutoff, &LSM_set_accel_hp_filter_cutoff, NULL, PARAMTYPE_BOOL, 1, (GET_FLAG | SET_FLAG)},
    {"6D LPass en", 23, &LSM_get_lowpass_on_6d_mode, &LSM_get_lowpass_on_6d_mode, NULL, PARAMTYPE_BOOL, 1, (GET_FLAG | SET_FLAG)},
    {"Adv Func en", 24, &LSM_get_function_en, &LSM_get_function_en, NULL, PARAMTYPE_BOOL, 1, (GET_FLAG | SET_FLAG)},
    {"A HPass Cutoff", 25, &LSM_get_accel_hp_filter_cutoff, &LSM_set_accel_hp_filter_cutoff, NULL, PARAMTYPE_BOOL, LSM_OPMODE_GYRO_ACCEL, (GET_FLAG | SET_FLAG)},
    {"Sample", 26, NULL, NULL, &LSM_sampleLatest, PARAMTYPE_NONE, 0, (ACT_FLAG)},
    {"Reset Memory", 27, NULL, NULL, &LSM_reboot_memory, PARAMTYPE_NONE, 0, (ACT_FLAG)},
    {"Reset Device", 28, NULL, NULL, &LSM_reset_device, PARAMTYPE_NONE, 0, (ACT_FLAG)},
    {"Intr1 Triggers", 29, NULL, &LSM_set_interrupt_1, NULL, PARAMTYPE_UINT8, UINT8_MAX, (GET_FLAG | SET_FLAG)},
    {"Intr2 Triggers", 30, NULL, &LSM_set_interrupt_2, NULL, PARAMTYPE_UINT8, UINT8_MAX, (GET_FLAG | SET_FLAG)},
    {"Gyro Fullscale", 31, &LSM_get_gyro_full_scale, &LSM_set_gyro_full_scale, NULL, PARAMTYPE_UINT8, LSM_GYRO_SCALE_125DPS, (GET_FLAG | SET_FLAG)},
    {"BDU en", 32, &LSM_get_block_data_update_en, &LSM_set_block_data_update_en, NULL, PARAMTYPE_BOOL, 1, (GET_FLAG | SET_FLAG) },
    {"MSB only en", 33, &LSM_get_only_msb_data, &LSM_set_only_msb_data, NULL, PARAMTYPE_BOOL, 1, (GET_FLAG | SET_FLAG) },
    {"Step Pkt en", 34, &LSM_get_fifo_step_en, &LSM_set_fifo_step_en, NULL, PARAMTYPE_BOOL, 1, (GET_FLAG | SET_FLAG) },
    {"Step DRDY", 35, &LSM_get_fifo_step_drdy, &LSM_set_fifo_step_drdy, NULL, PARAMTYPE_UINT8, 1, (GET_FLAG | SET_FLAG) },
    {"Fido ODR", 36, &LSM_get_fifo_odr, &LSM_set_fifo_odr, NULL, PARAMTYPE_UINT8, LSM_GYRO_ODR_1_66KHZ, (GET_FLAG | SET_FLAG) },
    {"Fifo Mode", 37, &LSM_getFIFOmode, &LSM_setFIFOmode, NULL, PARAMTYPE_UINT8, LSM_FIFO_MODE_CONTINUOUS, (GET_FLAG | SET_FLAG) },
    {"Fifo Threshold", 38, &LSM_getFIFOwatermark, &LSM_setFIFOwatermark, NULL, PARAMTYPE_UINT16, LSM_WATERMARK_MAX, (GET_FLAG | SET_FLAG) },
};


const peripheral_t lsm_periph_template = {
    .handle = NULL,
    .param_len = 38,
    .params = lsm_parameter_map,
    .periph_type = PTYPE_ACCEL_SENSOR,
    .peripheral_id = 0,
    .peripheral_name = "LSM6DS33",
};

#endif



/****** Function Prototypes ***********/


static IRAM_ATTR void ISR_int1(void *args);
static IRAM_ATTR void ISR_int2(void *args);

static void LSM_processAccel(LSM_DriverHandle_t *dev);
static void LSM_processGyro(LSM_DriverHandle_t *dev);
static uint16_t LSM_fifoPattern(LSM_DriverHandle_t *dev);
static esp_err_t LSM_getFIFOpktCount(LSM_DriverHandle_t *dev, uint16_t *count);
static esp_err_t LSM_waitSampleReady(LSM_DriverHandle_t *dev, uint8_t mask);
static esp_err_t LSM_getWhoAmI(LSM_DriverHandle_t *device, uint8_t *whoami);

void LSMDriverTask(void *args);



/************ ISR *********************/

static IRAM_ATTR void ISR_int1(void *args)
{
    LSM_DriverHandle_t *dev = (LSM_DriverHandle_t *)args;

    uint32_t intr_val = LSM_NOTIFY_INTR_PIN_1;
    BaseType_t higherPrio = pdFALSE;

    xTaskNotifyFromISR(dev->taskHandle, intr_val, eSetValueWithOverwrite, higherPrio);
    portYIELD_FROM_ISR();
}

static IRAM_ATTR void ISR_int2(void *args)
{
    LSM_DriverHandle_t *dev = (LSM_DriverHandle_t *)args;

    uint32_t intr_val = LSM_NOTIFY_INTR_PIN_2;
    BaseType_t higherPrio = pdFALSE;

    xTaskNotifyFromISR(dev->taskHandle, intr_val, eSetValueWithOverwrite, higherPrio);
    portYIELD_FROM_ISR();
}


/****** Private Data ******************/

/****** Private Functions *************/


static void print_i2c_register(LSM_DriverHandle_t *dev, uint8_t reg) {

    uint8_t val = 0;

    ESP_ERROR_CHECK(gcd_i2c_read_address(dev->commsChannel, dev->devAddr, reg, 1, &val));

    printByteBits(val);

    return;
}


static void LSM_processAccel(LSM_DriverHandle_t *dev)
{
    float factor = 0;

    switch (dev->settings.accelScale)
    {
    case LSM_ACCSCALE_2G:
        factor = 0.061;
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
        factor = 0.0;
        break;
    }

    int16_t s_ax = ((((int16_t)dev->measurements.rawAccel[1]) << 8) | ((int16_t)dev->measurements.rawAccel[0]));
    int16_t s_ay = ((((int16_t)dev->measurements.rawAccel[3]) << 8) | ((int16_t)dev->measurements.rawAccel[2]));
    int16_t s_az = ((((int16_t)dev->measurements.rawAccel[5]) << 8) | ((int16_t)dev->measurements.rawAccel[4]));

#ifdef DEBUG_MODE
    printf("Raw Accel Data: %u %u %u\n", sa_x, sa_y, sa_z);
    printf("Even rawer \n");
    for(uint8_t i=0;i<6;i++){
        printf("0x%02x ", dev->measurements.rawAccel[i]);
    }
    printf("\n");
#endif


    dev->measurements.calibAccelX = (float)s_ax * factor;
    dev->measurements.calibAccelY = (float)s_ay * factor;
    dev->measurements.calibAccelZ = (float)s_az * factor;

}


static void LSM_processGyro(LSM_DriverHandle_t *dev)
{
    float factor = 0;

    switch (dev->settings.gyroScale)
    {
    case LSM_GYRO_SCALE_125DPS:
        factor = 4.375;
        break;
    case LSM_GYRO_SCALE_250DPS:
        factor = 8.75;
        break;
    case LSM_GYRO_SCALE_500DPS:
        factor = 17.5;
        break;
    case LSM_GYRO_SCALE_1000DPS:
        factor = 35.0;
        break;
    case LSM_GYRO_SCALE_2000DPS:
        factor = 70.0;
        break;
    default:
        factor = 0.0;
        break;
    }

    int16_t x = ((((int16_t)dev->measurements.rawGyro[1]) << 8) | (int16_t)(dev->measurements.rawGyro[0]));
    int16_t y = ((((int16_t)dev->measurements.rawGyro[3]) << 8) | (int16_t)(dev->measurements.rawGyro[2]));
    int16_t z = ((((int16_t)dev->measurements.rawGyro[5]) << 8) | (int16_t)(dev->measurements.rawGyro[4]));

#ifdef DEBUG_MODE
    printf("Raw Gyro Data: %u %u %u\n", x, y, z);
    printf("Even rawer \n");
    for(uint8_t i=0;i<6;i++){
        printf("0x%02x ", dev->measurements.rawGyro[i]);
    }
    printf("\n");
#endif

    dev->measurements.calibGyroX = (((float)x) * factor);
    dev->measurements.calibGyroY = (((float)y) * factor);
    dev->measurements.calibGyroZ = (((float)z) * factor);

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


static esp_err_t LSM_getStatusRegister(LSM_DriverHandle_t *dev, uint8_t *regval) {
    return gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_STATUS_REG, 1, &regval);
}


static esp_err_t process_interrupts(LSM_DriverHandle_t *dev, uint32_t sources) {

    /** process interrupts 
     *  - if intr dataready:
     *          read status register to find dready signals
     *  - if intr  
     * 
     **/
    esp_err_t err = ESP_OK;

    bool check_drdy = false;
    bool check_fifo = false;
    bool check_steps = false;
    bool check_devrdy = false;
    bool check_sigmotion = false;

    uint8_t regval = 0;

    if(sources & LSM_NOTIFY_INTR_PIN_1) {

        if(dev->intr1_mask & LSM_INT1_TYPE_ACC_RDY ||
           dev->intr1_mask & LSM_INT1_TYPE_GYR_RDY ){
            check_drdy = true;
        }

        if(dev->intr1_mask & LSM_INT1_TYPE_BOOTSTAT) {
            check_devrdy = true;
        }

        if(dev->intr1_mask & LSM_INT1_TYPE_FIFO_THR || 
           dev->intr1_mask & LSM_INT1_TYPE_FIFO_OVR || 
           dev->intr1_mask & LSM_INT1_TYPE_FIFOFULL ){
            check_fifo = true;
        }
        
        if(dev->intr1_mask & LSM_INT1_TYPE_SIGMOTION) {
            check_sigmotion = true;
        }

        if(dev->intr1_mask & LSM_INT1_TYPE_STEPBASIC) {
            check_steps = true;
        }
    }


    if(sources & LSM_NOTIFY_INTR_PIN_2 || (sources & LSM_NOTIFY_INTR_PIN_1) && dev->settings.int2_on_int1) {
        if(dev->intr2_mask & LSM_INT2_TYPE_ACC_RDY ||
           dev->intr2_mask & LSM_INT2_TYPE_GYR_RDY || 
           dev->intr1_mask & LSM_INT2_TYPE_TMP_RDY ){
            check_drdy = true;
        }


        if(dev->intr1_mask & LSM_INT2_TYPE_FIFO_THR ||
           dev->intr1_mask & LSM_INT2_TYPE_FIFO_OVR ||
           dev->intr1_mask & LSM_INT2_TYPE_FIFOFULL ){
            check_fifo = true;
        }

        if(dev->intr1_mask & LSM_INT2_TYPE_STEPOVR ||
           dev->intr1_mask & LSM_INT2_TYPE_STEPDELTA) {
            check_steps = true;
        } 
    }


    if(check_drdy) {
        err = LSM_getStatusRegister(dev, &regval);
        if(!err) {
            dev->status.accel_drdy = (regval & LSM_STATUS_ACCEL_AVAIL_BIT) ? true : false;
            dev->status.gyro_drdy = (regval & LSM_STATUS_GYRO_AVAIL_BIT) ? true : false;
            dev->status.temp_drdy = (regval & LSM_STATUS_TEMP_AVAIL_BIT) ? true : false;
        }
    }

    if(check_devrdy) {
        err = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_CTRL3_C_REG, 1, &regval);
        if(regval & LSM_CTRL3_BOOT_BIT) {
            dev->status.device_rdy = true;
        }
    }

    if(check_fifo) {
        err = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_FIFO_STATUS2_REG, 1, &regval);
        if(!err) {
            dev->status.fifo_thresh = (regval & LSM_FIFO2_WATERMARK_BIT) ? true : false;
            dev->status.fifo_ovr = (regval & LSM_FIFO2_OVRRUN_BIT ) ? true : false;
            dev->status.fifo_full = (regval & LSM_FIFO2_FULL_BIT) ? true : false;
            dev->status.fifo_empty = (regval & LSM_FIFO2_EMPTY_BIT) ? true : false;
        }
    }

    if(check_steps) {
        err = ESP_ERR_NOT_SUPPORTED;
    }

    if(check_sigmotion) {
        err = ESP_ERR_NOT_SUPPORTED;
    }

    return err;
}


static float conversion_factor_accel(LSM_DriverHandle_t *dev) {
    float factor = 0;

    switch (dev->settings.accelScale)
    {
    case LSM_ACCSCALE_2G:
        factor = 0.061;
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
        factor = 0.0;
        break;
    }

    return factor;
}


static float conversion_factor_gyro(LSM_DriverHandle_t *dev) {
    float factor = 0;

    switch (dev->settings.gyroScale)
    {
    case LSM_GYRO_SCALE_125DPS:
        factor = 4.375;
        break;
    case LSM_GYRO_SCALE_250DPS:
        factor = 8.75;
        break;
    case LSM_GYRO_SCALE_500DPS:
        factor = 17.5;
        break;
    case LSM_GYRO_SCALE_1000DPS:
        factor = 35.0;
        break;
    case LSM_GYRO_SCALE_2000DPS:
        factor = 70.0;
        break;
    default:
        factor = 0.0;
        break;
    }

    return factor;
}


/** \brief reads fifo and discards until packet index = 0
 *          Also used to check if the packet is the expected size
 *          because why not?
 *  \return the number of packets dropped
 **/
static int16_t syncronise_packets(LSM_DriverHandle_t *dev, uint16_t packet_len, bool *expected_size) {

    uint16_t index = LSM_fifoPattern(dev);
    uint16_t i=0;
    uint16_t base = 0;
    uint8_t raw[2] = {0};
    bool ex = false;

    if(index > 0) {
        /** we are part way along the fifo pattern **/
        base = index;
        for(i=0; index > 0; i++) {
            /** read through the fifo until index becomes zero... **/
            gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_FIFO_DATA_LSB_REG, 2, raw);
            index = LSM_fifoPattern(dev);
            if(index == 0) {
                /** reached end of packets **/
                base += i;
                break;
            }
        }
    }

    ex = (base == packet_len) ? true : false;
    *expected_size = ex;

    printf("LSM Driver", "Checked packet length - found %u, expected %u\n", base, packet_len);

    return i;
}


/** \brief: Doesn't support crazy complex patterns OR 
 *          track any sort of order. 
 *          Totally alpha - tbh testing this would be exhaustive, so...
 *              Good luck.
 **/
static int16_t LSM_get_packet_length(LSM_DriverHandle_t *dev) {

    uint8_t a_mod = 1;
    uint8_t g_mod = 1;
    uint8_t dif = 0;
    uint8_t factor = 1;
    int16_t len = 0;

    printf("Calculating packet size!\n");

    if(dev->settings.accelRate != 0) {
        len += 3;
        printf("Accel enabled (len=%u)\n", len);

    }
    if(dev->settings.gyroRate != 0) {
        len += 3;
        printf("Gyro enabled (len=%u)\n", len);
    }

    if(dev->settings.accelRate != dev->settings.gyroRate && 
       (dev->settings.accelRate > 0 && dev->settings.gyroRate > 0)) {
        /** ahhh boy... luckily each enum is half the next, so hopefully... **/

        /** limit the ODR by the fifo ODR... this can limit to equal ODR  **/
        a_mod = dev->settings.accelRate > dev->settings.fifoODR ? dev->settings.fifoODR : dev->settings.accelRate;
        g_mod = dev->settings.gyroRate > dev->settings.fifoODR ? dev->settings.fifoODR : dev->settings.gyroRate; 


        if(a_mod > g_mod) {
            dif = a_mod - g_mod;
            factor = (uint8_t )pow(2, (double)dif);
            /** we have FACTOR * more accel packets than gyro packets **/
            printf("Accel more packets than gyro by factor of %u\n", factor);
        }
        else {
            dif = g_mod - a_mod;
            factor = (uint8_t )pow(2, (double)dif);
            /** we have FACTOR * more gyro packets than accel packets **/        
            printf("Gyro more packets than accel by factor of %u\n", factor);
        }
        
        /** dont care about order, this is length **/
        len += (3 * factor);
        printf("With this in mind, len is now %u\n", len);

    }

    if(dev->settings.temp_pkt_en) {
        len ++;
        printf("Third packet is temperature : len = %u\n", len);
    }

    if(dev->settings.pedo_fifo_en) {
        len++;
        printf("Third packet is step : len = %u\n", len);
    }

    // TODO: Timestamp
    //if(dev->)

    return len;

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
    uint32_t notify_val = 0;
    uint8_t regval = 0;
    esp_err_t err = ESP_OK;

    while (1)
    {

        /** if interrupts, wait for interrupt & process **/
        if(dev->int1En || dev->int2En) {
            xTaskNotifyWait(0, 0xFFFF, &notify_val, portMAX_DELAY);
    
            if(notify_val) {
                err = process_interrupts(dev, notify_val);
            
                if(!err) {
                    if(dev->status.fifo_full && dev->driver_settings.auto_read_fifo_full) {
                        uint16_t bytes_avail = LSM_FIFO_BUFFER_MEM_LEN;
                        LSM_readFifoBlock(dev, &bytes_avail);
                    }
                    if(dev->status.fifo_thresh) {
                        LSM_readFifoBlock(dev, &dev->settings.watermark);
                    }
                    if(dev->status.temp_drdy && dev->driver_settings.auto_read_sample_rdy) {
                        LSM_update_temperature(dev);
                    }
                    if((dev->status.accel_drdy || dev->status.gyro_drdy) &&
                        dev->driver_settings.auto_read_sample_rdy) {
                        LSM_sampleLatest(dev);
                    }
                }
            
            }
        }
        else if(dev->driver_settings.poll_sample_ready) {
            err = LSM_getStatusRegister(dev, &regval);

        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}





/****** Global Functions *************/


/**** MAIN CONFIG & ACTIONS *****/

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

            initStatus = gpio_config(&gpioInit);

            if (initStatus == ESP_OK)
            {
                initStatus = gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
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


        if(initStatus == ESP_OK && initData->use_cbuffer && initData->cbuff != NULL) {
            device->use_cbuffer = true;
            device->cbuff = initData->cbuff;
            device->cbuff_store_raw = initData->cbuff_store_raw;
        }
        else if (initStatus == ESP_OK && initData->assignFifoBuffer)
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
                    ESP_LOGE("LSM", "Error insufficient memory for buffer!");
                    initStatus = ESP_ERR_NO_MEM;
                }
            }
        }

        /** device settings - start these all at default values (unless specified in initData) **/
        if(initStatus == ESP_OK) {
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
        }

        /** try to get the device id to check the i2c & device **/
        if(initStatus == ESP_OK) {
            uint8_t who = 0;
            initStatus = LSM_getWhoAmI(device, &who);
            if(initStatus == ESP_OK && who != LSM_WHOAMI) {
                ESP_LOGE("LSM Driver", "Error - invalid device id! Expected %02x, Recevied: %02x", LSM_WHOAMI, who);
                initStatus = ESP_ERR_INVALID_RESPONSE;
            }
            else {
                ESP_LOGI("LSM_Driver", "Device ID read succesful!");
            }
        }

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


esp_err_t LSM_reset_device(LSM_DriverHandle_t *dev) {

    uint8_t regval = 0;
    esp_err_t err = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_CTRL3_C_REG, 1, &regval);
    if(!err) {
        regval |= 1;
        err = gcd_i2c_write_address(dev->commsChannel, dev->devAddr, LSM_CTRL3_C_REG, 1, &regval);
    }

    return err;
}


esp_err_t LSM_reboot_memory(LSM_DriverHandle_t *dev) {

    uint8_t regval = 0;
    esp_err_t err = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_CTRL3_C_REG, 1, &regval);
    if(!err) {
        regval |= LSM_CTRL3_BOOT_BIT;
        err = gcd_i2c_write_address(dev->commsChannel, dev->devAddr, LSM_CTRL3_C_REG, 1, &regval);
    }

    return err;
}


esp_err_t LSM_run_accel_self_test(LSM_DriverHandle_t *dev, bool *passed) {

    /** copying these from the appnote diagram - see there for explanation of commands (see above for app note) **/
    uint8_t regval = 0x30;
    uint8_t abort = 0;
    uint8_t temp_data[6];

    uint16_t x_no_st = 0;
    uint16_t y_no_st = 0;
    uint16_t z_no_st = 0;
    uint16_t x_st = 0;
    uint16_t y_st = 0;
    uint16_t z_st = 0;

    ESP_LOGI("LSM Driver", "INFO: Running Accelerometer Self-Test! \
             Please keep the device still for the next several seconds...");
    esp_err_t err = gcd_i2c_write_address(dev->commsChannel, dev->devAddr, LSM_CTRL1_XL_REG, 1, &regval);
    regval = 0;
    err += gcd_i2c_write_address(dev->commsChannel, dev->devAddr, LSM_CTRL2_G_REG, 1, &regval);
    regval = 0x44;
    err += gcd_i2c_write_address(dev->commsChannel, dev->devAddr, LSM_CTRL3_C_REG, 1, &regval);
    regval = 0;
    err += gcd_i2c_write_address(dev->commsChannel, dev->devAddr, LSM_CTRL4_C_REG, 1, &regval);
    err += gcd_i2c_write_address(dev->commsChannel, dev->devAddr, LSM_CTRL5_C_REG, 1, &regval);
    err += gcd_i2c_write_address(dev->commsChannel, dev->devAddr, LSM_CTRL6_C_REG, 1, &regval);
    err += gcd_i2c_write_address(dev->commsChannel, dev->devAddr, LSM_CTRL7_G_REG, 1, &regval);
    err += gcd_i2c_write_address(dev->commsChannel, dev->devAddr, LSM_CTRL8_XL_REG, 1, &regval);
    regval = 0x38;
    err += gcd_i2c_write_address(dev->commsChannel, dev->devAddr, LSM_CTRL9_XL_REG, 1, &regval);
    regval = 0;
    err += gcd_i2c_write_address(dev->commsChannel, dev->devAddr, LSM_CTRL10_C_REG, 1, &regval);

    /** wait for new data to arrive **/
    vTaskDelay(pdMS_TO_TICKS(LSM_SELF_TEST_DATA_QWR_MS));
    err += gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_STATUS_REG, 1, &regval);
    if(!(regval & 1)) {
        while(!(regval & 1)) {
            vTaskDelay(pdMS_TO_TICKS(LSM_SELF_TEST_DATA_QWR_MS));
            err += gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_STATUS_REG, 1, &regval);
            abort ++;
            if(abort > LSM_SELF_TEST_TIMEOUT_N) {
                ESP_LOGE("LSM_Driver", "Error: Unable to get data!");
                *passed = false;
                return ESP_ERR_INVALID_RESPONSE;         
            }  
        }
    }
    /** read & discard the first data sets **/
    err += gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_ACCELX_LSB_REG, 6, temp_data);

    /** wait for new data **/
    vTaskDelay(pdMS_TO_TICKS(LSM_SELF_TEST_DATA_QWR_MS));
    err += gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_STATUS_REG, 1, &regval);
    if(!(regval & 1)) {
        while(!(regval & 1)) {
            vTaskDelay(pdMS_TO_TICKS(LSM_SELF_TEST_DATA_QWR_MS));
            err += gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_STATUS_REG, 1, &regval);
            abort ++;
            if(abort > LSM_SELF_TEST_TIMEOUT_N) {
                *passed = false;
                return ESP_ERR_INVALID_RESPONSE;         
            }  
        }
    } /** appnote says something about... reading status bit 5 times? Surely just waiting for new data? **/

    memset(temp_data, '0', sizeof(uint8_t) * 6);
    /** read & store the new data set **/
    err += gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_ACCELX_LSB_REG, 6, temp_data);

    if(err) {
        ESP_LOGE("LSM Driver", "There has been a comms error - quitting self-test!");
        *passed = false;
        return ESP_ERR_INVALID_RESPONSE;
    }

    x_no_st = (((uint16_t )temp_data[1] << 8) | temp_data[0]);
    y_no_st = (((uint16_t )temp_data[3] << 8) | temp_data[2]);
    z_no_st = (((uint16_t )temp_data[5] << 8) | temp_data[4]);

    ESP_LOGI("LSM Driver", "Got Pre-SelfTest Data: x - %u y - %u z - %u", x_no_st, y_no_st, z_no_st);

    /** enable self test **/
    regval = 1;
    err += gcd_i2c_write_address(dev->commsChannel, dev->devAddr, LSM_CTRL5_C_REG, 1, &regval);

    vTaskDelay(pdMS_TO_TICKS(LSM_SELF_TEST_DATA_QWR_MS * 4));

    /** wait for new data **/
    err += gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_STATUS_REG, 1, &regval);
    if(!(regval & 1)) {
        while(!(regval & 1)) {
            vTaskDelay(pdMS_TO_TICKS(LSM_SELF_TEST_DATA_QWR_MS));
            err += gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_STATUS_REG, 1, &regval);
            abort ++;
            if(abort > LSM_SELF_TEST_TIMEOUT_N) {
                *passed = false;
                return ESP_ERR_INVALID_RESPONSE;         
            }  
        }
    }
    /** read & toss data **/
    err += gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_ACCELX_LSB_REG, 6, temp_data);

    vTaskDelay(pdMS_TO_TICKS(LSM_SELF_TEST_DATA_QWR_MS * 4));

    /** wait for new data **/
    err += gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_STATUS_REG, 1, &regval);
    if(!(regval & 1)) {
        while(!(regval & 1)) {
            vTaskDelay(pdMS_TO_TICKS(LSM_SELF_TEST_DATA_QWR_MS));
            err += gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_STATUS_REG, 1, &regval);
            abort ++;
            if(abort > LSM_SELF_TEST_TIMEOUT_N) {
                *passed = false;
                return ESP_ERR_INVALID_RESPONSE;         
            }  
        }
    }

    err += gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_ACCELX_LSB_REG, 6, temp_data);

    if(err) {
        ESP_LOGE("LSM Driver", "There has been a comms error - quitting self-test!");
        *passed = false;
        return ESP_ERR_INVALID_RESPONSE;      
    }

    x_st = (((uint16_t )temp_data[1] << 8) | temp_data[0]);
    y_st = (((uint16_t )temp_data[3] << 8) | temp_data[2]);
    z_st = (((uint16_t )temp_data[5] << 8) | temp_data[4]);

    ESP_LOGI("LSM Driver", "Got the selfTest measurements: x - %u y - %u z - %u", x_st, y_st, z_st);

     /** now, what the hell are the values for min(ST_X) and max(ST_X) ???? ?!??!???!?!??!? dammit ST why make this so difficult? **/

    return ESP_ERR_NOT_SUPPORTED;
}



/** Driver Behaviour Config **/

esp_err_t LSM_set_auto_read_fifo_full(LSM_DriverHandle_t *dev, bool *en) {
    dev->driver_settings.auto_read_fifo_full = *en;
    return ESP_OK;
}


esp_err_t LSM_get_auto_read_fifo_full(LSM_DriverHandle_t *dev, bool *en) {
    *en = dev->driver_settings.auto_read_fifo_full;
    return ESP_OK;
}


esp_err_t LSM_set_auto_sample_ready(LSM_DriverHandle_t *dev, bool *en) {
    dev->driver_settings.auto_read_sample_rdy = *en;
    return ESP_OK;
}


esp_err_t LSM_get_auto_sample_ready(LSM_DriverHandle_t *dev, bool *en) {
    *en = dev->driver_settings.auto_read_sample_rdy;
    return ESP_OK;
}


esp_err_t LSM_set_poll_sample_ready(LSM_DriverHandle_t *dev, bool *en) {
    dev->driver_settings.poll_sample_ready = *en;
    if(dev->driver_settings.sample_poll_ms == 0) {
        dev->driver_settings.sample_poll_ms = 1000;
    }
    return ESP_OK;
}


esp_err_t LSM_get_poll_sample_ready(LSM_DriverHandle_t *dev, bool *en) {
    *en = dev->driver_settings.poll_sample_ready;
    return ESP_OK;
}


esp_err_t LSM_set_poll_sample_period_ms(LSM_DriverHandle_t *dev, uint16_t *val) {
    esp_err_t err = ESP_OK;
    uint16_t v = *val;
    
    if(v > LSM_DRIVER_MAX_SAMPLE_PERIOD_MS || v < LSM_DRIVER_MIN_SAMPLE_PERIOD_MS) {
        ESP_LOGE("LSM Driver", "Error, invalid sample period (min 20, max 100,000 ms)");
        err = ESP_ERR_INVALID_ARG;
    }
    else {
        dev->driver_settings.sample_poll_ms = v;
    }
    return err;
}


esp_err_t LSM_get_poll_sample_period_ms(LSM_DriverHandle_t *dev, uint16_t *val) {
    *val = dev->driver_settings.sample_poll_ms;
    return ESP_OK;
}

/******* SAMPLE SETTINGS (BASIC) ********/


esp_err_t LSM_setOpMode(LSM_DriverHandle_t *dev, LSM_OperatingMode_t *mode) {
    esp_err_t status = ESP_OK;
    uint8_t regVals[2] = {0};
    uint8_t writeVals[2] = {0};
    uint8_t m = *mode;
    status = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_CTRL9_XL_REG, 2, regVals);

    /** enable all axis for eac dev **/
    if (m == LSM_OPMODE_ACCEL_ONLY)
    {
        writeVals[0] = ((LSM_CTRL9_ACCEL_Z_EN_BIT) | (LSM_CTRL9_ACCEL_Y_EN_BIT) | (LSM_CTRL9_ACCEL_X_EN_BIT) | (regVals[0] & 0b111));
    }
    else if (m == LSM_OPMODE_GYRO_ONLY)
    {
        writeVals[1] = ((LSM_CTRL10_GYRO_Z_EN_BIT) | (LSM_CTRL10_GYRO_Y_EN_BIT) | (LSM_CTRL10_GYRO_X_EN_BIT) | (regVals[1] & 0b111));
    }
    else if (m == LSM_OPMODE_GYRO_ACCEL)
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


esp_err_t LSM_getOpMode(LSM_DriverHandle_t *dev, LSM_OperatingMode_t *mode) {
    esp_err_t status = ESP_OK;
    *mode = dev->settings.opMode;
    return status;  
}


esp_err_t LSM_setAccelODRMode(LSM_DriverHandle_t *dev, LSM_AccelODR_t *m) {
    esp_err_t status = ESP_OK;
    uint8_t accelEn = 0, regVal = 0;
    uint8_t mode = *m;

    ESP_LOGI("LSM_Driver", "Setting Accel ODR to %u\n", mode);

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
            ESP_LOGI("LSM_Driver", "Writing %02x to Accel ctrl reg (Enabling accel axis)", accelEn);
            status = gcd_i2c_write_address(1, LSM_I2C_ADDR, LSM_CTRL9_XL_REG, 1, &accelEn);
        }

        /** get register value, clear mode & set new **/
        status = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_CTRL1_XL_REG, 1, &regVal);
        regVal &= 0b1111;

        regVal |= (mode << 4);
        status = gcd_i2c_write_address(dev->commsChannel, dev->devAddr, LSM_CTRL1_XL_REG, 1, &regVal);
    }
    return status;
}


esp_err_t LSM_getAccelODRMode(LSM_DriverHandle_t *dev, LSM_AccelODR_t *m) {
    esp_err_t status = ESP_OK;
    *m = dev->settings.accelRate;
    return status;
}


esp_err_t LSM_setGyroODRMode(LSM_DriverHandle_t *dev, LSM_GyroODR_t *m) {
    esp_err_t status = ESP_OK;
    uint8_t accelEn = 0, regVal = 0;
    uint8_t mode = *m;

    if (mode > LSM_GYRO_ODR_1_66KHZ)
    {
        status = ESP_ERR_INVALID_ARG;
    }
    else
    {
        ESP_LOGI("LSM_Driver", "Writing %02x to Accel ctrl reg", accelEn);
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
    }
    return status;
}


esp_err_t LSM_getGyroODRMode(LSM_DriverHandle_t *dev, LSM_GyroODR_t *m) {
    esp_err_t status = ESP_OK;
    *m = dev->settings.gyroRate;
    return status;
}


esp_err_t LSM_set_accel_full_scale(LSM_DriverHandle_t *dev, uint8_t *fs) {

    esp_err_t err = ESP_OK;
    uint8_t f = *fs;
    uint8_t regval = 0;

    if(f > LSM_ACCSCALE_8G) {
        err = ESP_ERR_INVALID_ARG;
    }
    else {
        err = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_CTRL1_XL_REG, 1, &regval);
    }

    if(!err) {
        regval &= 0b11110011;
        regval |= f << 2;
        err = gcd_i2c_write_address(dev->commsChannel, dev->devAddr, LSM_CTRL1_XL_REG, 1, &regval);
    }

    if(!err) {
        dev->settings.accelScale = f;
    }

    return err;
}


esp_err_t LSM_get_accel_full_scale(LSM_DriverHandle_t *dev, uint8_t *fs) {
    esp_err_t status = ESP_OK;
    *fs = dev->settings.accelScale;
    return status;
}


esp_err_t LSM_set_gyro_full_scale(LSM_DriverHandle_t *dev, uint8_t *fs) {

    esp_err_t err = ESP_OK;
    uint8_t f = *fs;
    uint8_t regval = 0;

    if(f > LSM_GYRO_SCALE_125DPS) {
        err = ESP_ERR_INVALID_ARG;
    }
    else {
        err = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_CTRL2_G_REG, 1, &regval);
    }

    if(!err) {
        regval &= 0b11110001;
        if(f == LSM_GYRO_SCALE_125DPS) {
            regval |= (1 << 1);            
        }
        else {
            regval |= (f << 2);
        }
        err = gcd_i2c_write_address(dev->commsChannel, dev->devAddr, LSM_CTRL1_XL_REG, 1, &regval);
    }

    if(!err) {
        dev->settings.gyroScale = f;
    }

    return err;
}


esp_err_t LSM_get_gyro_full_scale(LSM_DriverHandle_t *dev, uint8_t *fs) {
    esp_err_t status = ESP_OK;
    *fs = dev->settings.gyroScale;
    return status;
}



/** TODO: These things at some point maybe... **/

esp_err_t LSM_set_interrupt_pp_od(LSM_DriverHandle_t *dev, bool *val) {
    esp_err_t err = ESP_ERR_NOT_SUPPORTED;
    return err;
}


esp_err_t LSM_set_interrupt_active_level(LSM_DriverHandle_t *dev, bool *val) {
    esp_err_t err = ESP_ERR_NOT_SUPPORTED;
    return err;
}


esp_err_t LSM_set_spi_mode(LSM_DriverHandle_t *dev, bool *val) {
    esp_err_t err = ESP_ERR_NOT_SUPPORTED;
    return err; 
}


esp_err_t LSM_set_auto_increment_addr(LSM_DriverHandle_t *dev, bool *val) {
    esp_err_t err = ESP_ERR_NOT_SUPPORTED;
    return err; 
}



/** SPECIFIC SETTINGS  **/


esp_err_t LSM_set_block_data_update_en(LSM_DriverHandle_t *dev, bool *en) {

    bool e = *en;
    bool wr = false;
    uint8_t regval = 0;
    esp_err_t err = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_CTRL3_C_REG, 1, &regval);

    if(!err) {
        if(e && !(regval & LSM_CTRL3_BLOCKDATA_UPDATE_BIT)) {
            regval |= LSM_CTRL3_BLOCKDATA_UPDATE_BIT;
            wr = true;
        }
        else if(!e && (regval & LSM_CTRL3_BLOCKDATA_UPDATE_BIT)) {
            regval &= ~(LSM_CTRL3_BLOCKDATA_UPDATE_BIT);
            wr = true;
        }

        if(wr) {
            err = gcd_i2c_write_address(dev->commsChannel, dev->devAddr, LSM_CTRL3_C_REG, 1, &regval);
        }
    }

    if(!err) {
        dev->settings.block_data_update = e;
    }
    
    return err;
}


esp_err_t LSM_get_block_data_update_en(LSM_DriverHandle_t *dev, bool *en) {
    esp_err_t status = ESP_OK;
    *en = dev->settings.block_data_update;
    return status;
}


esp_err_t LSM_set_accel_bw_select_mode(LSM_DriverHandle_t *dev, LSM_AccelBandwidthMode_t *mode) {

    esp_err_t err = ESP_OK;
    uint8_t m = *mode;
    uint8_t regval = 0;
    bool wr = false;

    err = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_CTRL4_C_REG, 1, &regval);
    
    if(!err) {
        if(m && !(regval & LSM_CTRL4_ACCEL_BW_SEL_BIT)) {
            regval |= LSM_CTRL4_ACCEL_BW_SEL_BIT;
            wr = true;
        }
        else if(!m && (regval & LSM_CTRL4_ACCEL_BW_SEL_BIT)) {
            regval &= ~(LSM_CTRL4_ACCEL_BW_SEL_BIT);
            wr = true;
        }

        err = gcd_i2c_write_address(dev->commsChannel, dev->devAddr, LSM_CTRL4_C_REG, 1, &regval);
    }

    if(!err) {
        dev->settings.accel_bw_mode = m;
    }

    return err;
}


esp_err_t LSM_get_accel_bw_select_mode(LSM_DriverHandle_t *dev, LSM_AccelBandwidthMode_t *mode) {
    esp_err_t status = ESP_OK;
    *mode = dev->settings.accel_bw_mode;
    return status;
}


esp_err_t LSM_set_gyro_sleep_state(LSM_DriverHandle_t *dev, LSM_GyroSleepState_t *mode) {

    esp_err_t err = ESP_OK;
    uint8_t m = *mode;
    uint8_t regval = 0;
    bool wr = false;

    err = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_CTRL4_C_REG, 1, &regval);
    
    if(!err) {
        if(m && !(regval & LSM_CTRL4_GYRO_SLEEP_BIT)) {
            regval |= LSM_CTRL4_GYRO_SLEEP_BIT;
            wr = true;
        }
        else if(!m && (regval & LSM_CTRL4_GYRO_SLEEP_BIT)) {
            regval &= ~(LSM_CTRL4_GYRO_SLEEP_BIT);
            wr = true;
        }
        if(wr) {
            err = gcd_i2c_write_address(dev->commsChannel, dev->devAddr, LSM_CTRL4_C_REG, 1, &regval);
        }
    }

    if(!err) {
        dev->settings.gyro_sleep_state = m ? LSM_GYRO_SLEEP_ENABLED : LSM_GYRO_SLEEP_DISABLED;
    }

    return err;    
}


esp_err_t LSM_get_gyro_sleep_state(LSM_DriverHandle_t *dev, LSM_GyroSleepState_t *mode) {
    esp_err_t status = ESP_OK;
    *mode = dev->settings.gyro_sleep_state;
    return status;
}


esp_err_t LSM_set_int2_on_int1_state(LSM_DriverHandle_t *dev, uint8_t *mode) {

    esp_err_t err = ESP_OK;
    uint8_t m = *mode;
    uint8_t regval = 0;
    bool wr = false;

    err = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_CTRL4_C_REG, 1, &regval);
    
    if(!err) {
        if(m && !(regval & LSM_CTRL4_ALL_ISR_PAD1_BIT)) {
            regval |= LSM_CTRL4_ALL_ISR_PAD1_BIT;
            wr = true;
        }
        else if(!m && (regval & LSM_CTRL4_ALL_ISR_PAD1_BIT)) {
            regval &= ~(LSM_CTRL4_ALL_ISR_PAD1_BIT);
            wr = true;
        }

        if(wr) {
            err = gcd_i2c_write_address(dev->commsChannel, dev->devAddr, LSM_CTRL4_C_REG, 1, &regval);
        }
    }

    if(!err) {
        dev->settings.int2_on_int1 = m ? 1 : 0;
    }

    return err;
}


esp_err_t LSM_get_int2_on_int1_state(LSM_DriverHandle_t *dev, uint8_t *mode) {
    esp_err_t status = ESP_OK;
    *mode = dev->settings.int2_on_int1;
    return status;
}


esp_err_t LSM_set_temperature_pkt_en(LSM_DriverHandle_t *dev, bool *en) {

    esp_err_t err = ESP_OK;
    bool e = *en;
    uint8_t regval = 0;
    bool wr = false;

    err = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_CTRL4_C_REG, 1, &regval);
    
    if(!err) {
        if(e && !(regval & LSM_CTRL4_FIFO_TEMP_EN_BIT)) {
            regval |= LSM_CTRL4_FIFO_TEMP_EN_BIT;
            wr = true;
        }
        else if(!e && (regval & LSM_CTRL4_FIFO_TEMP_EN_BIT)) {
            regval &= ~(LSM_CTRL4_FIFO_TEMP_EN_BIT);
            wr = true;
        }

        if(wr) {
            err = gcd_i2c_write_address(dev->commsChannel, dev->devAddr, LSM_CTRL4_C_REG, 1, &regval);
        }
    }

    if(!err) {
        dev->settings.temp_pkt_en = e;
    }

    return err;
}


esp_err_t LSM_get_temperature_pkt_en(LSM_DriverHandle_t *dev, bool *en) {
    esp_err_t status = ESP_OK;
    *en = dev->settings.temp_pkt_en;
    return status;
}


esp_err_t LSM_set_stop_on_fifo_thresh(LSM_DriverHandle_t *dev, bool *en) {
    esp_err_t err = ESP_OK;
    bool e = *en;
    uint8_t regval = 0;
    bool wr = false;

    err = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_CTRL4_C_REG, 1, &regval);
    
    if(!err) {
        if(e && !(regval & LSM_CTRL4_FIFO_THRLD_EN_BIT)) {
            regval |= LSM_CTRL4_FIFO_THRLD_EN_BIT;
            wr = true;
        }
        else if(!e && (regval & LSM_CTRL4_FIFO_THRLD_EN_BIT)) {
            regval &= ~(LSM_CTRL4_FIFO_THRLD_EN_BIT);
            wr = true;
        }

        if(wr) {
            err = gcd_i2c_write_address(dev->commsChannel, dev->devAddr, LSM_CTRL4_C_REG, 1, &regval);
        }
    }

    if(!err) {
        dev->settings.stop_on_thresh = e;
    }

    return err; 
}


esp_err_t LSM_get_stop_on_fifo_thresh(LSM_DriverHandle_t *dev, bool *en) {
    esp_err_t status = ESP_OK;
    *en = dev->settings.stop_on_thresh;
    return status;   
}


esp_err_t LSM_set_rounding_mode(LSM_DriverHandle_t *dev, uint8_t *mode) {
    return ESP_ERR_NOT_SUPPORTED;
}


esp_err_t LSM_set_gyro_hp_mode(LSM_DriverHandle_t *dev, bool *mode) {
    esp_err_t err = ESP_OK;
    bool e = *mode;
    uint8_t regval = 0;
    bool wr = false;

    err = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_CTRL7_G_REG, 1, &regval);
    
    if(!err) {
        if(e && !(regval & LSM_CTRL7_GYRO_HPMODE_DISABLE_BIT)) {
            regval |= LSM_CTRL7_GYRO_HPMODE_DISABLE_BIT;
            wr = true;
        }
        else if(!e && (regval & LSM_CTRL7_GYRO_HPMODE_DISABLE_BIT)) {
            regval &= ~(LSM_CTRL7_GYRO_HPMODE_DISABLE_BIT);
            wr = true;
        }

        if(wr) {
            err = gcd_i2c_write_address(dev->commsChannel, dev->devAddr, LSM_CTRL7_G_REG, 1, &regval);
        }
    }

    if(!err) {
        dev->settings.gyro_hp_en = e;
    }

    return err; 
}


esp_err_t LSM_get_gyro_hp_mode(LSM_DriverHandle_t *dev, bool *mode) {

    esp_err_t status = ESP_OK;
    *mode = dev->settings.gyro_hp_en;
    return status; 
}


esp_err_t LSM_set_gyro_hpfilter_mode(LSM_DriverHandle_t *dev, bool *mode) {
    esp_err_t err = ESP_OK;
    bool e = *mode;
    uint8_t regval = 0;
    bool wr = false;

    err = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_CTRL7_G_REG, 1, &regval);
    
    if(!err) {
        if(e && !(regval & LSM_CTRL7_GYRO_HIPASS_EN_BIT)) {
            regval |= LSM_CTRL7_GYRO_HIPASS_EN_BIT;
            wr = true;
        }
        else if(!e && (regval & LSM_CTRL7_GYRO_HIPASS_EN_BIT)) {
            regval &= ~(LSM_CTRL7_GYRO_HIPASS_EN_BIT);
            wr = true;
        }

        if(wr) {
            err = gcd_i2c_write_address(dev->commsChannel, dev->devAddr, LSM_CTRL7_G_REG, 1, &regval);
        }
    }

    if(!err) {
        dev->settings.gyro_hp_filter_en = e;
    }

    return err; 
}


esp_err_t LSM_get_gyro_hpfilter_mode(LSM_DriverHandle_t *dev, bool *mode) {

    esp_err_t status = ESP_OK;
    *mode = dev->settings.gyro_hp_filter_en;
    return status; 
}


esp_err_t LSM_set_gyro_hp_filter_cutoff(LSM_DriverHandle_t *dev, LSM_GyroHighpassCutoff_t *cutoff) {

    esp_err_t err = ESP_OK;
    uint8_t m = *cutoff;
    uint8_t regval = 0;

    if(m > LSM_GYRO_HPCUTOFF_16_3HZ) {
        err = ESP_ERR_INVALID_ARG;
    }
    else {
        err = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_CTRL7_G_REG, 1, &regval);
    }
    
    if(!err) {
        regval &= 0b11111100;
        regval |= m;
        err = gcd_i2c_write_address(dev->commsChannel, dev->devAddr, LSM_CTRL7_G_REG, 1, &regval);
    }

    if(!err) {
        dev->settings.GHPcutoff = m;
    }

    return err;
}


esp_err_t LSM_get_gyro_hp_filter_cutoff(LSM_DriverHandle_t *dev, LSM_GyroHighpassCutoff_t *cutoff) {
    esp_err_t status = ESP_OK;
    *cutoff = dev->settings.GHPcutoff;
    return status; 
}


esp_err_t LSM_set_accel_lp2_filter_en(LSM_DriverHandle_t *dev, bool *en) {
    esp_err_t err = ESP_OK;
    bool e = *en;
    uint8_t regval = 0;
    bool wr = false;

    err = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_CTRL8_XL_REG, 1, &regval);
    
    if(!err) {
        if(e && !(regval & LSM_CTRL8_ACCEL_LPF2_EN_BIT)) {
            regval |= LSM_CTRL8_ACCEL_LPF2_EN_BIT;
            wr = true;
        }
        else if(!e && (regval & LSM_CTRL8_ACCEL_LPF2_EN_BIT)) {
            regval &= ~(LSM_CTRL8_ACCEL_LPF2_EN_BIT);
            wr = true;
        }

        if(wr) {
            err = gcd_i2c_write_address(dev->commsChannel, dev->devAddr, LSM_CTRL8_XL_REG, 1, &regval);
        }
    }

    if(!err) {
        dev->settings.accel_lp2_filter_en = e;
    }

    return err; 
}


esp_err_t LSM_get_accel_lp2_filter_en(LSM_DriverHandle_t *dev, bool *en) {

    esp_err_t status = ESP_OK;
    *en = dev->settings.accel_lp2_filter_en;
    return status; 
}


esp_err_t LSM_set_accel_hp_filter_cutoff(LSM_DriverHandle_t *dev, LSM_HighpassSlopeSettings_t *cutoff) {

    esp_err_t err = ESP_OK;
    uint8_t m = *cutoff;
    uint8_t regval = 0;

    if(m > LSM_HPLP_ODR_400) {
        err = ESP_ERR_INVALID_ARG;
    }
    else {
        err = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_CTRL8_XL_REG, 1, &regval);
    }
    
    if(!err) {
        regval &= 0b11111100;
        regval |= m;
        err = gcd_i2c_write_address(dev->commsChannel, dev->devAddr, LSM_CTRL8_XL_REG, 1, &regval);
    }

    if(!err) {
        dev->settings.highPass = m;
    }

    return err;
}


esp_err_t LSM_get_accel_hp_filter_cutoff(LSM_DriverHandle_t *dev, LSM_HighpassSlopeSettings_t *cutoff) {
    esp_err_t status = ESP_OK;
    *cutoff = dev->settings.highPass;
    return status; 
}


esp_err_t LSM_set_accel_hpfilter_mode(LSM_DriverHandle_t *dev, bool *mode) {
    esp_err_t err = ESP_OK;
    bool e = *mode;
    uint8_t regval = 0;
    bool wr = false;

    err = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_CTRL8_XL_REG, 1, &regval);
    
    if(!err) {
        if(e && !(regval & LSM_CTRL8_ACCEL_HPSLOPE_EN_BIT)) {
            regval |= LSM_CTRL8_ACCEL_HPSLOPE_EN_BIT;
            wr = true;
        }
        else if(!e && (regval & LSM_CTRL8_ACCEL_HPSLOPE_EN_BIT)) {
            regval &= ~(LSM_CTRL8_ACCEL_HPSLOPE_EN_BIT);
            wr = true;
        }

        if(wr) {
            err = gcd_i2c_write_address(dev->commsChannel, dev->devAddr, LSM_CTRL8_XL_REG, 1, &regval);
        }
    }

    if(!err) {
        dev->settings.accel_hp_filter_en = e;
    }

    return err; 
}


esp_err_t LSM_get_accel_hpfilter_mode(LSM_DriverHandle_t *dev, bool *mode) {

    esp_err_t status = ESP_OK;
    *mode = dev->settings.accel_hp_filter_en;
    return status; 
}


esp_err_t LSM_set_lowpass_on_6d_mode(LSM_DriverHandle_t *dev, bool *mode) {
    esp_err_t err = ESP_OK;
    bool e = *mode;
    uint8_t regval = 0;
    bool wr = false;

    err = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_CTRL8_XL_REG, 1, &regval);
    
    if(!err) {
        if(e && !(regval & LSM_CTRL8_ACCEL_6D_LOWPASS_EN_BIT)) {
            regval |= LSM_CTRL8_ACCEL_6D_LOWPASS_EN_BIT;
            wr = true;
        }
        else if(!e && (regval & LSM_CTRL8_ACCEL_6D_LOWPASS_EN_BIT)) {
            regval &= ~(LSM_CTRL8_ACCEL_6D_LOWPASS_EN_BIT);
            wr = true;
        }

        if(wr) {
            err = gcd_i2c_write_address(dev->commsChannel, dev->devAddr, LSM_CTRL8_XL_REG, 1, &regval);
        }
    }

    if(!err) {
        dev->settings.lowpass_on_6d_en = e;
    }

    return err; 
}


esp_err_t LSM_get_lowpass_on_6d_mode(LSM_DriverHandle_t *dev, bool *mode) {

    esp_err_t status = ESP_OK;
    *mode = dev->settings.lowpass_on_6d_en;
    return status; 
}


esp_err_t LSM_set_function_en(LSM_DriverHandle_t *dev, bool *mode) {
    esp_err_t err = ESP_OK;
    bool e = *mode;
    uint8_t regval = 0;
    bool wr = false;

    err = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_CTRL10_C_REG, 1, &regval);
    
    if(!err) {
        if(e && !(regval & LSM_CTRL10_EMBEDDED_FUNC_EN_BIT)) {
            regval |= LSM_CTRL10_EMBEDDED_FUNC_EN_BIT;
            wr = true;
        }
        else if(!e && (regval & LSM_CTRL10_EMBEDDED_FUNC_EN_BIT)) {
            regval &= ~(LSM_CTRL10_EMBEDDED_FUNC_EN_BIT);
            wr = true;
        }

        if(wr) {
            err = gcd_i2c_write_address(dev->commsChannel, dev->devAddr, LSM_CTRL10_C_REG, 1, &regval);
        }
    }

    if(!err) {
        dev->settings.embd_functions_en = e;
    }

    return err; 
}


esp_err_t LSM_get_function_en(LSM_DriverHandle_t *dev, bool *mode) {

    esp_err_t status = ESP_OK;
    *mode = dev->settings.embd_functions_en;
    return status; 
}




/** CORE FUNCTIONALITY **/

esp_err_t LSM_sampleLatest(LSM_DriverHandle_t *dev)
{

    esp_err_t status = ESP_OK;
    uint8_t regVal = 0, wait = 0;
    void *ptr = NULL;
    uint32_t len = 0;
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

        status = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_GYROX_LSB_REG, 6, dev->measurements.rawGyro);
        status = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_ACCELX_LSB_REG, 6, dev->measurements.rawAccel);
        LSM_processAccel(dev);
        LSM_processGyro(dev);
        break;

    default:
        break;
    }

    status = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_STATUS_REG, 1, &regVal);

    return status;
}


esp_err_t LSM_update_temperature(LSM_DriverHandle_t *dev) {
    esp_err_t status = ESP_OK;

    status = gcd_i2c_read_address(dev->commsHandle, dev->devAddr, LSM_TEMP_LSB_REG, 2, &dev->measurements.raw_temp[0]);    

    if(!status) {
        dev->measurements.calibTemperature = (((int16_t)dev->measurements.raw_temp[1] << 8) | (int16_t )dev->measurements.raw_temp[0]);
    }

    return status;
}


esp_err_t LSM_get_temperature(LSM_DriverHandle_t *dev, int16_t *temp) {
    *temp = dev->measurements.calibTemperature;
    return ESP_OK;
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


esp_err_t LSM_get_fifo_pkt_index(LSM_DriverHandle_t *dev, uint16_t *index) {

    esp_err_t err = ESP_OK;
    uint8_t raw[2] = {0};

    err = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_FIFO_STATUS3_REG, 2, raw);
    if(!err) {
        *index = ((((uint16_t)raw[1] << 8) & 0b11) | (uint16_t)raw[0]);
    }

    return err;
}


/** FIFO SETTINGS **/

esp_err_t LSM_set_fifo_step_en(LSM_DriverHandle_t *dev, bool *en) {

    esp_err_t err = ESP_OK;
    bool e = *en;
    uint8_t regval = 0;
    bool wr = false;

    err = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_FIFO_CTRL2_REG, 1, &regval);
    if(!err) {
        if(!e && (regval & LSM_FIFOCTRL2_PEDOTMR_FIFO_EN_BIT)) {
            regval &= ~(LSM_FIFOCTRL2_PEDOTMR_FIFO_EN_BIT);
            wr = true;
        }
        else if (e && !(regval & LSM_FIFOCTRL2_PEDOTMR_FIFO_EN_BIT)) {
            regval |= LSM_FIFOCTRL2_PEDOTMR_FIFO_EN_BIT;
            wr = true;
        }
    }

    if(!err && wr) {
        err = gcd_i2c_write_address(dev->commsChannel, dev->devAddr, LSM_FIFO_CTRL2_REG, 1, &regval);
    }

    if(!err) {
        dev->settings.pedo_fifo_en = e;
    }

    return err;
}


esp_err_t LSM_get_fifo_step_en(LSM_DriverHandle_t *dev, bool *en) {

    esp_err_t err = ESP_OK;
    *en = dev->settings.pedo_fifo_en;
    return err;
 }


esp_err_t LSM_set_only_msb_data(LSM_DriverHandle_t *dev, bool *en) {
    esp_err_t err = ESP_OK;
    bool e = *en;
    uint8_t regval = 0;
    bool wr = false;

    err = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_FIFO_CTRL4_REG, 1, &regval);
    if(!err) {
        if(!e && (regval & LSM_FIFOCTRL4_MSBONLY_EN_BIT)) {
            regval &= ~(LSM_FIFOCTRL4_MSBONLY_EN_BIT);
            wr = true;
        }
        else if (e && !(regval & LSM_FIFOCTRL4_MSBONLY_EN_BIT)) {
            regval |= LSM_FIFOCTRL4_MSBONLY_EN_BIT;
            wr = true;
        }
    }

    if(!err && wr) {
        err = gcd_i2c_write_address(dev->commsChannel, dev->devAddr, LSM_FIFO_CTRL4_REG, 1, &regval);
    }

    if(!err) {
        dev->settings.msb_only_en = e;
    }

    return err;
}


esp_err_t LSM_get_only_msb_data(LSM_DriverHandle_t *dev, bool *en)  {

    esp_err_t err = ESP_OK;
    *en = dev->settings.msb_only_en;
    return err;    
}


esp_err_t LSM_set_fifo_step_drdy(LSM_DriverHandle_t *dev, uint8_t *on_step_en) {

    esp_err_t err = ESP_OK;
    bool e = *on_step_en;
    uint8_t regval = 0;
    bool wr = false;

    err = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_FIFO_CTRL2_REG, 1, &regval);
    if(!err) {
        if(!e && (regval & LSM_FIFOCTRL2_PEDOTMR_DRDY_BIT)) {
            regval &= ~(LSM_FIFOCTRL2_PEDOTMR_DRDY_BIT);
            wr = true;
        }
        else if (e && !(regval & LSM_FIFOCTRL2_PEDOTMR_DRDY_BIT)) {
            regval |= LSM_FIFOCTRL2_PEDOTMR_DRDY_BIT;
            wr = true;
        }
    }

    if(!err && wr) {
        err = gcd_i2c_write_address(dev->commsChannel, dev->devAddr, LSM_FIFO_CTRL2_REG, 1, &regval);
    }

    if(!err) {
        dev->settings.step_drdy_en = e;
    }

    return err;
}


esp_err_t LSM_get_fifo_step_drdy(LSM_DriverHandle_t *dev, uint8_t *on_step_en) {
    esp_err_t err = ESP_OK;
    *on_step_en = dev->settings.step_drdy_en;
    return err;
}


esp_err_t LSM_set_fifo_step_decim(LSM_DriverHandle_t *dev, uint8_t *dec) {

    esp_err_t err = ESP_OK;
    uint8_t d = *dec;
    uint8_t regval = 0;

    if(d > LSM_FIFO_32_DECM) {
        err = ESP_ERR_INVALID_ARG;
    }

    else {
        err = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_FIFO_CTRL4_REG, 1, &regval);
        if(!err) {
            regval &= 0b11000111;
            regval |= (d << 3);
            err = gcd_i2c_write_address(dev->commsChannel, dev->devAddr, LSM_FIFO_CTRL4_REG, 1, &regval);
        }
    }

    if(!err) {
        dev->settings.stepDec = d;
    }


    return err;
}


esp_err_t LSM_get_fifo_step_decim(LSM_DriverHandle_t *dev, uint8_t *dec) {
    esp_err_t err = ESP_OK;
    *dec = dev->settings.stepDec;
    return err;
}


esp_err_t LSM_set_fifo_odr(LSM_DriverHandle_t *dev, uint8_t *odr) {

    esp_err_t err = ESP_OK;
    uint8_t o = *odr;
    uint8_t regval = 0;
    uint8_t ar = 0, gr = 0, dr = 0;

    if(o > LSM_FIFO_ODR_6_66KHZ) {
        err = ESP_ERR_INVALID_ARG;
    }
    
    else {
        /** put the enum into odr rate order **/
        ar = dev->settings.accelRate;
        gr = dev->settings.gyroRate;

        if(o > ar || o > gr) {
            ESP_LOGI("LSM_Driver", "Invalid ODR: FIFO data rate is > accel/gyro data rate \
                     - fifo will be automatically limited to slowest data rate");
        }
    }

    if(!err) {
        err = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_FIFO_CTRL4_REG, 1, &regval);
        if(!err) {
            regval &= 0b11000111;
            regval |= (o << 3);
            err = gcd_i2c_write_address(dev->commsChannel, dev->devAddr, LSM_FIFO_CTRL4_REG, 1, &regval);
        }
    }

    if(!err) {
        dev->settings.fifoODR = o;
    }


    return err;
}


esp_err_t LSM_get_fifo_odr(LSM_DriverHandle_t *dev, uint8_t *odr) {
    esp_err_t err = ESP_OK;
    *odr = dev->settings.fifoODR;
    return err;
}


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

    if(status == ESP_OK) {
        dev->settings.fifoMode = mode;
    }

    return status;
}


esp_err_t LSM_getFIFOmode(LSM_DriverHandle_t *dev, uint8_t *mode)
{
    esp_err_t status = ESP_OK;
    *mode = dev->settings.fifoMode;
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

    if(status == ESP_OK) {
        
    }

    return status;
}



/** \brief: This function reads length packets from the fifo
 *          
 *          - read pkt
 *          - add to temp buffer
 *          - do for entire set - only support concurrent measurements for now!
 *          - if cbuffer:
 *              - if cbuff use_packets 
 *                  - write packet
 *              -else
 *                  write raw
 *          -else if use_fifo 
 *              
 **/
esp_err_t LSM_readFifoBlock(LSM_DriverHandle_t *dev, uint16_t *num_samples)
{

    esp_err_t status = ESP_OK;
    uint16_t l = *num_samples;
    uint8_t raw[12] = {0};
    int16_t val = 0;

    int16_t comb[64] = {0};
    float proc[6] = {0};

    int8_t packet_len = 0;

    float accel_conv_factor =  
    float gyro_conv_factor = 

    if(l == 0 || l > LSM_FIFO_BUFFER_MEM_LEN) {
        status = ESP_ERR_INVALID_ARG;
    }
    else if(!dev->use_cbuffer && dev->fifoBuffer == NULL) {
        status = ESP_ERR_NO_MEM;
    }
    else {

        /** EXPERIMENTAL: Try to keep packets sensible! **/
        if(dev->use_cbuffer && dev->cbuff_store_packets) {
            /** read an entire set of apckets at a time - ideally, fifo pkt index should start at n and and at n-1 **/
            packet_len = LSM_get_packet_length(dev);
            if(packet_len > LSM_MAX_SUPPORTED_PACKET_LENGTH) {
                ESP_LOGE("LSM Driver", "Error, packet length is > 36 samples - not supported!");
                status = ESP_ERR_NOT_SUPPORTED;
            }
            else {
                uint16_t full_packets = l / packet_len;
                bool exp = false;
                uint16_t dropped = syncronise_packets(dev, packet_len, &exp);
                printf("Dropped %u packets whilst syncronising { pkt len expected?: %u }\n", dropped, exp);

                for(int j=1; j < full_packets+1; j++) {
                    for (int i=1; i < packet_len+1; i++) {
                        status = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_FIFO_DATA_LSB_REG, 2, raw);
                        if(!status) {
                            val = (((int16_t)raw[1] << 1) | (int16_t)raw[0]);
                            proc[i-1] = val;
                        }
                        status = cbuffer_write_packet(dev->cbuff, proc, packet_len);
                    }
                }
            }
        }
        else if(dev->use_cbuffer && dev->cbuff_store_raw) {
            status = ESP_ERR_NOT_SUPPORTED;
            /** TODO: This **/
        }

        for(uint16_t i=0; i < l && status == ESP_OK; i++) {
            status = gcd_i2c_read_address(dev->commsChannel, dev->devAddr, LSM_FIFO_DATA_LSB_REG, 2, raw);
            if(!status) {
                val = (((int16_t)raw[1] << 1) | (int16_t)raw[0]);
            }
        }


    }


    return status;
}



/**** INTERUPT SETTINGS  ****/

esp_err_t LSM_set_interrupt_1(LSM_DriverHandle_t *device, uint8_t *int_t)
{
    esp_err_t status = ESP_OK;
    uint8_t writeval = 0, regval = 0;
    uint8_t intr = *int_t;

    if (device->i1Pin > 0)
    {

        status = gcd_i2c_write_address(device->commsChannel, device->devAddr, LSM_INT1_CTRL_REG, 1, &writeval);
    }
    else
    {
        ESP_LOGE("LSM_DRIVER", "Error: Pin not configured!");
        status = ESP_ERR_NOT_FOUND;
    }

    if(status == ESP_OK) {
        device->intr1_mask = intr;
        device->int1En = intr ? true : false;
    }

    return status;
}


esp_err_t LSM_set_interrupt_2(LSM_DriverHandle_t *device, uint8_t *int_t)
{
    esp_err_t status = ESP_OK;
    uint8_t regval = 0;
    uint8_t intr = *int_t;

    if (device->i2Pin > 0)
    {
        status = gcd_i2c_write_address(device->commsChannel, device->devAddr, LSM_INT2_CTRL_REG, 1, &intr);
    }
    else
    {
        ESP_LOGE("LSM_DRIVER", "Error: Pin not configured!");
        status = ESP_ERR_NOT_FOUND;
    }

    if(status == ESP_OK) {
        device->intr2_mask = intr;
        device->int2En = intr ? true : false;
    }

    return status;
}


esp_err_t LSM_set_function_intr1(LSM_DriverHandle_t *dev, uint8_t *int_t) {

    return ESP_ERR_NOT_SUPPORTED;
}


esp_err_t LSM_set_function_intr2(LSM_DriverHandle_t *dev, uint8_t *int_t) {

    return ESP_ERR_NOT_SUPPORTED;
}