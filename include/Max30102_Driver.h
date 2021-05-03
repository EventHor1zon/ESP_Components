/****************************************
* \file
* \brief
* \date
* \author
****************************************/

#ifndef MAX30102_DRIVER_H
#define MAX30102_DRIVER_H

/********* Includes ********************/

#include "esp_types.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/********* Definitions *****************/

#define MAX31_REGADDR_INTR_STATUS1 0x00
#define MAX31_REGADDR_INTR_STATUS2 0x01
#define MAX31_REGADDR_INTR_EN1     0x02
#define MAX31_REGADDR_INTR_EN2     0x03
#define MAX31_REGADDR_FIFO_WRTPTR  0x04
#define MAX31_REGADDR_OVRFLW_CNTR  0x05
#define MAX31_REGADDR_FIFO_RDPTR   0x06
#define MAX31_REGADDR_FIFO_DATA    0x07
#define MAX31_REGADDR_FIFO_CONFIG   0x08
#define MAX31_REGADDR_MODE_CONFIG   0x09
#define MAX31_REGADDR_SP02_CONFIG   0x0A
#define MAX31_REGADDR_LED1PULSE_AMP 0x0C
#define MAX31_REGADDR_LED2PULSE_AMP 0x0D
#define MAX31_REGADDR_MULTILED12_CTRL 0x11
#define MAX31_REGADDR_MULTILED34_CTRL 0x12
#define MAX31_REGADDR_DIETEMP_INT       0x1F
#define MAX31_REGADDR_DIETEMP_FRC       0x20
#define MAX31_REGADDR_DIETEMP_SAMPLE    0x21
#define MAX31_REGADDR_REV_ID            0xFE 
#define MAX31_REGADDR_PART_ID            0xFF 


#define MAX31_SLAVE_ADDR 0x57

#define MAX31_PART_ID   0x15

#define MAX31_FIFO_SAMPLES  0x20
#define MAX31_ALMOSTFULL_MAX 0xf
#define MAX31_FIFO_SINGLE_SAMPLE_LEN 3
#define MAX31_FIFO_DOUBLE_SAMPLE_LEN 6

#define MAX31_FIFO_MAX_SIZE 200 /** ~192? **/

#define MAX31_INTR_TYPE_ALMFULL (1 << 7)
#define MAX31_INTR_TYPE_NEWSAMPLE (1 << 6)
#define MAX31_INTR_TYPE_AMBILITOVF (1 << 5)
#define MAX31_INTR_TYPE_PWRRDY (1)

#define MAX31_INTR_TYPE_DIETEMP_RDY (1 << 1)

/********** Types **********************/

typedef enum {
    MAX31_DRVRMODE_INTR_NEWMEASURE = 0x0,
    MAX31_DRVRMODE_INTR_FIFOFULLRD = 0x01,
    MAX31_DRVRMODE_INTR_POLLING = 0x02,

} max31_drivermode_t;

typedef enum {
    MAX31_MODE_STARTUP = 0,
    MAX31_MODE_HEARTRATE_RED = 0x02,
    MAX31_MODE_SPO2_RED_IR = 0x03,
    MAX31_MODE_MULTILED_RIR = 0x07,
} max31_mode_t;

typedef enum {
    MAX31_LED_PWM_69 = 0x0,
    MAX31_LED_PWM_118 = 0x01,
    MAX31_LED_PWM_215 = 0x02,
    MAX31_LED_PWM_411 = 0x11
} max31_ledpwm_t;

typedef enum {
    MAX31_SAMPLERATE_50 = 0,
    MAX31_SAMPLERATE_100,
    MAX31_SAMPLERATE_200,
    MAX31_SAMPLERATE_400,
    MAX31_SAMPLERATE_800,
    MAX31_SAMPLERATE_1000,
    MAX31_SAMPLERATE_1600,
    MAX31_SAMPLERATE_3200,
} max31_samplerate_t;

typedef enum {
    MAX31_ADC_RNG_2048,
    MAX31_ADC_RNG_4096,
    MAX31_ADC_RNG_8192,
    MAX31_ADC_RNG_16384,
} max31_adcrange_t;

typedef enum {
    MAX31_SAMPLE_AVG_1 = 0,
    MAX31_SAMPLE_AVG_2 = 1,
    MAX31_SAMPLE_AVG_4 = 2,
    MAX31_SAMPLE_AVG_8 = 3,
    MAX31_SAMPLE_AVG_16 = 4,
    MAX31_SAMPLE_AVG_32 = 5,
} max31_sampleavg_t;

typedef struct max31_initdata
{
    uint8_t i2c_bus;
    gpio_num_t intr_pin;

} max31_initdata_t;


typedef struct Max30102_Driver
{
    /* data */
    uint8_t dev_addr;
    uint8_t i2c_bus;
    gpio_num_t intr_pin;
    TaskHandle_t taskhandle;
    uint8_t intr1_mask;
    uint8_t intr2_mask;
    uint8_t ledIR_ampl;
    uint8_t ledRed_ampl;
    bool shutdown;
    bool use_fifo;
    bool fifo_ovr;
    max31_mode_t device_mode;
    max31_sampleavg_t smpavg;
    uint8_t almostfull;
    max31_samplerate_t samplerate;
    max31_ledpwm_t ledpwm;
    max31_adcrange_t adcrange;
    uint8_t red_lvl;
    bool temp_sampling;
    uint8_t *fifo_buffer;
    uint8_t bytes_in_buffer;
    max31_drivermode_t drivermode;
    bool configured;

} max31_driver_t;


/******** Function Definitions *********/

max31_driver_t *max31_init(max31_initdata_t *init);

esp_err_t max31_get_device_id(max31_driver_t*dev, uint8_t *val);


esp_err_t max31_get_device_id(max31_driver_t *dev, uint8_t *val);

esp_err_t max31_set_interrupt1(max31_driver_t *dev, uint8_t *intr_mask);

esp_err_t max31_set_interrupt2(max31_driver_t *dev, uint8_t *intr_mask);

esp_err_t max31_set_sample_average(max31_driver_t *dev, max31_sampleavg_t *val);

esp_err_t max31_set_fifo_rollover(max31_driver_t *dev, uint8_t *val);

esp_err_t max31_set_almost_full_val(max31_driver_t *dev, uint8_t *val);

esp_err_t max31_set_shutdown(max31_driver_t *dev, uint8_t *val);

esp_err_t max31_set_mode(max31_driver_t *dev, max31_mode_t *val);

esp_err_t max31_set_spo2_samplerate(max31_driver_t *dev, uint8_t *val);

esp_err_t max31_set_ledpwm(max31_driver_t *dev, uint8_t *val);

esp_err_t max31_set_redledamplitude(max31_driver_t *dev, uint8_t *val);

esp_err_t max31_set_irledamplitude(max31_driver_t *dev, uint8_t *val);


esp_err_t max31_read_fifo(max31_driver_t *dev);

esp_err_t max31_reset_device(max31_driver_t *dev);

esp_err_t max31_sample_temp(max31_driver_t *dev);






#endif /* MAX30102_DRIVER_H */
