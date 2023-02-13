/****************************************
* \file
* \brief
* \date
* \author
****************************************/

#ifndef MSGEQ7_DRIVER_H
#define MSGEQ7_DRIVER_H

/********* Includes ********************/

#include "esp_types.h"
#include "driver/adc_common.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"

/********* Definitions *****************/

#define MSG_SAMPLES 10
#define MSG_CHANNELS 7

/********** Types **********************/

typedef struct msg_init 
{
    /* data */
    gpio_num_t data;
    gpio_num_t rst;
    gpio_num_t strobe;
    adc_bits_width_t adc_width;
} msg_init_t;


typedef struct MSGEQ7_Measures
{
    /* data */
    int16_t band_1;
    int16_t band_2;
    int16_t band_3;
    int16_t band_4;
    int16_t band_5;
    int16_t band_6;
    int16_t band_7;

    uint16_t base_levels[7];
} msg_measure_t;



typedef struct msg_handle
{
    /* data */
    gpio_num_t data_pin;    /**< pin connected to data output of chip **/
    gpio_num_t rst_pin;     /**< pin connected to the reset pin of chip **/
    gpio_num_t strobe_pin;  /**< pin connected to the strobe pin of chip **/
    adc_channel_t adc_channel; /**< adc channel used on pin **/
    adc_bits_width_t adc_bits; /**< adc resolution  **/
    bool autosample;          /**< automatically sample at rate specified **/
    uint16_t autosample_period_ms;  /**< period between each sample **/
    TaskHandle_t task;  /**< driver task handle **/
    TimerHandle_t timer;    /**< driver timer handle **/
    uint16_t pulse_count;   /**< pulses since last reset **/
    uint16_t reset_n;   /**< number of reads between resets **/

    bool enable_oversample;
    uint8_t oversample_factor;

    msg_measure_t data; /**< latest sampled data from device **/
} msg_handle_t;


/******** Function Definitions *********/


#ifdef CONFIG_DRIVERS_USE_HEAP
msg_handle_t *msg_init(msg_init_t *init);
#else
msg_handle_t *msg_init(msg_handle_t *handle, msg_init_t *init);
#endif



esp_err_t msg_get_channel_1(msg_handle_t *handle, int16_t *val);

esp_err_t msg_get_channel_2(msg_handle_t *handle, int16_t *val);

esp_err_t msg_get_channel_3(msg_handle_t *handle, int16_t *val);

esp_err_t msg_get_channel_4(msg_handle_t *handle, int16_t *val);

esp_err_t msg_get_channel_5(msg_handle_t *handle, int16_t *val);

esp_err_t msg_get_channel_6(msg_handle_t *handle, int16_t *val);

esp_err_t msg_get_channel_7(msg_handle_t *handle, int16_t *val);


#endif /* MSGEQ7_DRIVER_H */
