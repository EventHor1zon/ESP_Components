/****************************************
* \file     HMC5883_Driver.h
* \brief    Header file for Driver
* \date     March 2021
* \author   RJAM
****************************************/

#ifndef HMC5883_DRIVER_H
#define HMC5883_DRIVER_H

/********* Includes ********************/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_types.h"
#include "esp_err.h"
#include "driver/gpio.h"

/********* Definitions *****************/

#define HMC_I2C_ADDR            0x0D


#define HMC_REGADDR_XDATA_MSB   0x00
#define HMC_REGADDR_XDATA_LSB   0x01
#define HMC_REGADDR_YDATA_MSB   0x02
#define HMC_REGADDR_YDATA_LSB   0x03
#define HMC_REGADDR_ZDATA_MSB   0x04
#define HMC_REGADDR_ZDATA_LSB   0x05
#define HMC_REGADDR_STATUS      0x06
#define HMC_REGADDR_TEMP_LSB    0x07
#define HMC_REGADDR_TEMP_MSB    0x08
#define HMC_REGADDR_CTRL_A      0x09
#define HMC_REGADDR_CTRL_B      0x0A

#define HMC_INTR_EN_MASK        (1 << 0)
#define HMC_ROLLOVER_MASK       (1 << 6)
#define HMC_RESET_MASK          (1 << 7)

#define HMC_MAX_VALUE           32767
#define HMC_MIN_VALUE           -32768

/********** Types **********************/

typedef enum {
    HMC_MODE_STANDBY,
    HMC_MODE_CONTINUOUS,
} hmc_mode_t;


typedef enum {
    HMC_AVG_SAMPLES_1,
    HMC_AVG_SAMPLES_2,
    HMC_AVG_SAMPLES_4,
    HMC_AVG_SAMPLES_8,
    HMC_AVG_SAMPLES_MAX,
} hmc_avg_samples_t;

typedef enum {
    HMC_DOR_10HZ,
    HMC_DOR_50HZ,
    HMC_DOR_100HZ,
    HMC_DOR_200HZ,
} hmc_odr_t;

typedef enum {
    HMC_OSR_512,
    HMC_OSR_256,
    HMC_OSR_128,
    HMC_OSR_64,
} hmc_osr_t;

typedef enum {
    HMC_SCALE_2G,
    HMC_SCALE_8G,
} hmc_scale_t;

typedef struct HMC5883_init
{
    /* data */
    uint8_t  i2c_bus;
    gpio_num_t drdy_pin;
    
} hmc_init_t;


typedef struct HMC_Measurements {

    float x_val;
    float y_val;
    float z_val;

    int16_t x_raw;
    int16_t y_raw;
    int16_t z_raw;
} hmc_measure_t;


typedef struct HMC5883_Driver
{
    /* data */
    uint8_t i2c_bus;
    uint8_t i2c_address;
    bool isr_en;
    gpio_num_t drdy_pin;

    hmc_mode_t mode;
    hmc_scale_t scale;
    hmc_odr_t d_rate;
    hmc_osr_t osr;
    hmc_measure_t results;

    TaskHandle_t t_handle;



} hmc_driver_t;


typedef hmc_driver_t * HMC_DEV;


/******** Function Definitions *********/


HMC_DEV hmc_init(hmc_init_t *ini);


esp_err_t hmc_get_mode(HMC_DEV dev, uint8_t *val);

esp_err_t hmc_set_mode(HMC_DEV dev, uint8_t *val);

esp_err_t hmc_get_avg_samples(HMC_DEV dev, uint8_t *val);

esp_err_t hmc_set_avg_samples(HMC_DEV dev, uint8_t *val);

esp_err_t hmc_get_mode(HMC_DEV dev, uint8_t *val);

esp_err_t hmc_set_mode(HMC_DEV dev, uint8_t *val);

esp_err_t hmc_get_mode(HMC_DEV dev, uint8_t *val);

esp_err_t hmc_set_mode(HMC_DEV dev, uint8_t *val);

esp_err_t hmc_update_measurements(HMC_DEV dev);
#endif /* HMC5883_DRIVER_H */
