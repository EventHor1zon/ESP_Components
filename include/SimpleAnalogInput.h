/****************************************
* \file
* \brief
* \date
* \author
****************************************/

#ifndef SIMPLE_ANALOG_INPUT_H
#define SIMPLE_ANALOG_INPUT_H

/********* Includes ********************/

#include "esp_err.h"
#include "esp_adc_cal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/********* Definitions *****************/

/********** Types **********************/

/******** Function Definitions *********/

typedef struct SimpleAnalogInit
{
    /* data */
    adc_channel_t a_chan;
    adc_channel_t b_chan;
    adc_channel_t c_chan;

    adc_bits_width_t width;
    adc_atten_t atten;
} sai_init_t;



typedef struct SimpleAnalogInput
{
    /* data */
    adc_channel_t adc_channel_a;
    adc_channel_t adc_channel_b;
    adc_channel_t adc_channel_c;

    adc_bits_width_t width;
    adc_atten_t atten;

    uint32_t a_last;
    uint32_t b_last;
    uint32_t c_last;


    uint8_t delta_thresh;

} sai_driver_t;


typedef sai_driver_t * SAI_HANDLE;




SAI_HANDLE sai_init(sai_init_t *init);


esp_err_t sai_update_channel(SAI_HANDLE handle, uint8_t *chan);

esp_err_t sai_update_all_channels(SAI_HANDLE handle);

esp_err_t sai_get_delta_thresh(SAI_HANDLE handle, uint8_t *thresh);

esp_err_t sai_set_delta_thresh(SAI_HANDLE handle, uint8_t *thresh);

esp_err_t sai_enable_delta_thresh_en(SAI_HANDLE handle);



#endif /* SIMPLE_ANALOG_INPUT_H */
