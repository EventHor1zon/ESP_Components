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

#define HMC_I2C_ADDR            0x1E

#define HMC_REGADDR_CONFIG_A    0x00
#define HMC_REGADDR_CONFIG_B    0x01
#define HMC_REGADDR_MODE        0x02
#define HMC_REGADDR_XDATA_MSB   0x03
#define HMC_REGADDR_XDATA_LSB   0x04
#define HMC_REGADDR_YDATA_MSB   0x05
#define HMC_REGADDR_YDATA_LSB   0x06
#define HMC_REGADDR_ZDATA_MSB   0x07
#define HMC_REGADDR_ZDATA_LSB   0x08
#define HMC_REGADDR_STATUS      0x09
#define HMC_REGADDR_ID_A        0x0A
#define HMC_REGADDR_ID_B        0x0B
#define HMC_REGADDR_ID_C        0x0C


#define HMC_AVG_SAMPLES_OFFSET      5
#define HMC_D_OUTPUT_RATE_OFFSET    2
#define HMC_GAIN_CONFIG_OFFSET      5

#define HMC_ERR_MEASURE_VAL     -4096


/********** Types **********************/

typedef enum {
    HMC_MODE_CONTINUOUS,
    HMC_MODE_SINGLE_MEASURE,
    HMC_MODE_IDLE,
    HMC_MODE_MAX,
} hmc_mode_t;


typedef enum {
    HMC_AVG_SAMPLES_1,
    HMC_AVG_SAMPLES_2,
    HMC_AVG_SAMPLES_4,
    HMC_AVG_SAMPLES_8,
    HMC_AVG_SAMPLES_MAX,
} hmc_avg_samples_t;

typedef enum {
    HMC_DOR_0_75HZ,
    HMC_DOR_1_5HZ,
    HMC_DOR_3HZ,
    HMC_DOR_7_5HZ,
    HMC_DOR_15HZ,   /** default **/
    HMC_DOR_30HZ,    
    HMC_DOR_75HZ,
    HMC_DOR_MAX,
} hmc_dor_t;

typedef enum {
    HMC_MEASURE_NORMAL,
    HMC_MEASURE_POS_BIAS,
    HMC_MEASURE_NEG_BIAS,
    HMC_MEASURE_MAX,    
} hmc_measure_t;

typedef enum {
    HMC_GAIN_0_88,
    HMC_GAIN_1_3,
    HMC_GAIN_1_9,
    HMC_GAIN_2_5,
    HMC_GAIN_4,
    HMC_GAIN_4_7,
    HMC_GAIN_5_6,
    HMC_GAIN_8_1,
    HMC_GAIN_MAX
} hmc_gain_t;

typedef struct HMC5883_init
{
    /* data */
    uint8_t  i2c_bus;
    gpio_num_t drdy_pin;
    
} hmc_init_t;



typedef struct HMC5883_Driver
{
    /* data */
    uint8_t i2c_bus;
    gpio_num_t drdy_pin;

    hmc_mode_t mode;
    
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
