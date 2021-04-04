/****************************************
* \file         RGB_Driver.c
* \brief        Header file 
* \date         April 2021
* \author       RJAM
****************************************/

#ifndef RGB_DRIVER_H
#define RGB_DRIVER_H

/********* Includes ********************/

#include "esp_err.h"
#include "esp_types.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/********* Definitions *****************/

/********** Types **********************/



typedef struct RGB_Settings {
    uint16_t min;
    uint16_t max;
} rgb_settings_t;


typedef struct RGB_DriverInit
{
    /* data */
    gpio_num_t r_pin;   /**< r pin **/
    gpio_num_t g_pin;   /**< g pin **/
    gpio_num_t b_pin;   /**< b pin **/
    uint32_t freq;
    bool active_level;
} rgb_init_t;


typedef struct RGB_Driver
{
    /* data */

    gpio_num_t r_channel;   /**< r channel **/
    gpio_num_t g_channel;   /**< g channel **/
    gpio_num_t b_channel;   /**< b channel **/

    bool active_level;  /**< active high(1)/low(0) **/

    uint32_t resolution;    /**< pwm resolution **/
    uint32_t frequency;     /**< pwm frequency **/
    uint32_t max_duty;      

    uint32_t r_duty;    /**< r duty **/
    uint32_t g_duty;    /**< g duty **/
    uint32_t b_duty;    /**< b duty **/

    TaskHandle_t t_handle;

} rgb_driver_t;


typedef rgb_driver_t * RGB_HANDLE;

/******** Function Definitions *********/


RGB_HANDLE rgb_driver_init(rgb_init_t *init);

esp_err_t set_r_duty(RGB_HANDLE *handle, uint32_t *val);

esp_err_t set_g_duty(RGB_HANDLE *handle, uint32_t *val);

esp_err_t set_b_duty(RGB_HANDLE *handle, uint32_t *val);




#endif /* RGB_DRIVER_H */
