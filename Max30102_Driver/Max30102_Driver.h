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

#define MAX31_FIFO_SAMPLES  32
#define MAX31_ALMOSTFULL_MAX 0xf
#define MAX31_FIFO_SINGLE_SAMPLE_LEN 3
#define MAX31_FIFO_DOUBLE_SAMPLE_LEN 6


#define MAX31_FIFO_MAX_SIZE (MAX31_FIFO_SAMPLES * MAX31_FIFO_DOUBLE_SAMPLE_LEN) /** ~192? **/

#define MAX31_INTR_TYPE_ALMFULL (1 << 7)
#define MAX31_INTR_TYPE_NEWSAMPLE (1 << 6)
#define MAX31_INTR_TYPE_AMBILITOVF (1 << 5)
#define MAX31_INTR_TYPE_PWRRDY (1)

#define MAX31_INTR_TYPE_DIETEMP_RDY (1 << 1)

#ifdef CONFIG_USE_PERIPH_MANAGER
#include "CommandAPI.h"

#define max31_param_length 13
const parameter_t max31_param_map[max31_param_length];
const peripheral_t max31_periph_template;

#endif


/********** Types **********************/

typedef enum {
    MAX31_INTR_PWRRDY = MAX31_INTR_TYPE_PWRRDY,
    MAX31_INTR_TEMP_RDY = (1 << 1),
    MAX31_INTR_AMBIENT_CANCL_OVR = MAX31_INTR_TYPE_AMBILITOVF,
    MAX31_INTR_FIFO_NEW_SAMPLE = MAX31_INTR_TYPE_NEWSAMPLE,
    MAX31_INTR_FIFO_ALMOST_FULL = MAX31_INTR_TYPE_ALMFULL,
} max31_intr_source_t;


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
    bool use_cbuffer;
    CBuff cbuffer;
} max31_initdata_t;


typedef struct Max30102_Driver
{
    /* data */
    uint8_t dev_addr;
    uint8_t i2c_bus;
    bool use_cbuff;
    bool ambi_ovr_invalidates;
    bool drop_next_fifo;
    gpio_num_t intr_pin;
    TaskHandle_t taskhandle;
    uint8_t intr_mask;
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
    uint8_t fifo_buffer[MAX31_FIFO_MAX_SIZE];
    uint8_t bytes_read;
    float red_buffer[MAX31_FIFO_SAMPLES];
    float ir_buffer[MAX31_FIFO_SAMPLES];
    uint8_t pkts_in_fifo;
    max31_drivermode_t drivermode;
    bool configured;
    float temperature;
    uint32_t ticks_since_temperature;
    CBuff cbuff;

} max31_driver_t;


/******** Function Definitions *********/


/** 
 * \brief intialise the max30102 driver
 * \param init - pointer to max31_initdata_t struct
 * \return handle or NULL on error
 **/
max31_driver_t *max31_init(max31_initdata_t *init);

/** 
 * \brief Get the device id from the max30102 chip
 * \param dev - device handle
 * \param val - value storage
 * \return handle or NULL on error
 **/
esp_err_t max31_get_device_id(max31_driver_t *dev, uint8_t *val);

/** 
 * \brief Clears the interrupt source registers
 * \param dev - device handle
 * \param val - value storage
 * \return handle or NULL on error
 **/
esp_err_t max31_clear_interrupt_sources(max31_driver_t *dev);

/** 
 * \brief Sets the interrupt source registers
 * \param dev - device handle
 * \param val - value: an OR'd byte of max31_intr_source_t 
 * \return handle or NULL on error
 **/
esp_err_t max31_set_interrupt_sources(max31_driver_t *dev, uint8_t *intr_mask);

/** 
 * \brief Gets the ambient light setting - when on
 *          the ambient overflow interrupt will reset the fifo
 *          - this setting does not activate the ambient light overflow interrupt
 * \param dev - device handle
 * \param val - treu - enabled, false - disabled
 * \return handle or NULL on error
 **/
esp_err_t max31_get_ambient_light_invalidates(max31_driver_t *dev, bool *val);

/** 
 * \brief Sets the ambient light setting - when on
 *          the ambient overflow interrupt will reset the fifo
 *          - this setting does not activate the ambient light overflow interrupt
 * \param dev - device handle
 * \param val - treu - enabled, false - disabled
 * \return handle or NULL on error
 **/
esp_err_t max31_set_ambient_light_invalidates(max31_driver_t *dev, bool *val);

/** 
 * \brief Gets the sample average setting
 * \param dev - device handle
 * \param val - value storage
 * \return handle or NULL on error
 **/
esp_err_t max31_get_sample_average(max31_driver_t *dev, max31_sampleavg_t *val);


/** 
 * \brief Gets the sample average setting
 * \param dev - device handle
 * \param val - value: one of max31_sampleavg_t
 * \return handle or NULL on error
 **/
esp_err_t max31_set_sample_average(max31_driver_t *dev, max31_sampleavg_t *val);


/** 
 * \brief Gets the fifo rollover value currently set
 * \param dev - device handle
 * \param val - value storage
 * \return handle or NULL on error
 **/
esp_err_t max31_get_fifo_rollover(max31_driver_t *dev, uint8_t *val);

/** 
 * \brief Sets the fifo rollover value currently set
 * \param dev - device handle
 * \param val - value 0 - off, 1+ on
 * \return handle or NULL on error
 **/
esp_err_t max31_set_fifo_rollover(max31_driver_t *dev, uint8_t *val);

/** 
 * \brief Gets the almost-full value
 * \param dev - device handle
 * \param val - value storage
 * \return handle or NULL on error
 **/
esp_err_t max31_get_almost_full_val(max31_driver_t *dev, uint8_t *val);


/** 
 * \brief Sets the almost-full value
 * \param dev - device handle
 * \param val - value 0 - 31 (measurements until full - i.e 0=fifo full)
 * \return handle or NULL on error
 **/
esp_err_t max31_set_almost_full_val(max31_driver_t *dev, uint8_t *val);

/** 
 * \brief Gets the shutdown status
 * \param dev - device handle
 * \param val - value storage
 * \return handle or NULL on error
 **/
esp_err_t max31_get_shutdown(max31_driver_t *dev, bool *val);


/** 
 * \brief Sets the shutdown status
 * \param dev - device handle
 * \param val - true - shutdown, false = active
 * \return handle or NULL on error
 **/
esp_err_t max31_set_shutdown(max31_driver_t *dev, bool *val);

/** 
 * \brief Gets the current device mode
 * \param dev - device handle
 * \param val - value storage
 * \return handle or NULL on error
 **/
esp_err_t max31_get_mode(max31_driver_t *dev, max31_mode_t *val);


/** 
 * \brief Sets the current device mode
 * \param dev - device handle
 * \param val - one of max31_mode_t
 * \return handle or NULL on error
 **/
esp_err_t max31_set_mode(max31_driver_t *dev, max31_mode_t *val);


/** 
 * \brief Gets the current sample rate
 * \param dev - device handle
 * \param val - storage
 * \return handle or NULL on error
 **/
esp_err_t max31_get_spo2_samplerate(max31_driver_t *dev, uint8_t *val);


/** 
 * \brief Sets the current sample rate
 * \param dev - device handle
 * \param val - one of max31_spo2_samplerate_t
 * \return handle or NULL on error
 **/
esp_err_t max31_set_spo2_samplerate(max31_driver_t *dev, uint8_t *val);


/** 
 * \brief Gets the current led pwm sample
 * \param dev - device handle
 * \param val - storage
 * \return handle or NULL on error
 **/
esp_err_t max31_get_ledpwm(max31_driver_t *dev, uint8_t *val);

/** 
 * \brief Sets the current led pwm sample
 * \param dev - device handle
 * \param val - one of max31_ledpwm_t
 * \return handle or NULL on error
 **/
esp_err_t max31_set_ledpwm(max31_driver_t *dev, uint8_t *val);

esp_err_t max31_get_redledamplitude(max31_driver_t *dev, uint8_t *val);

esp_err_t max31_set_redledamplitude(max31_driver_t *dev, uint8_t *val);

esp_err_t max31_get_irledamplitude(max31_driver_t *dev, uint8_t *val);

esp_err_t max31_set_irledamplitude(max31_driver_t *dev, uint8_t *val);

esp_err_t max31_get_fifo_ovr(max31_driver_t *dev, uint8_t *val);

esp_err_t max31_get_temperature(max31_driver_t *dev, float *val);

esp_err_t max31_read_temperature(max31_driver_t *dev);

esp_err_t max31_read_fifo(max31_driver_t *dev);

esp_err_t max31_reset_device(max31_driver_t *dev);

esp_err_t max31_enable_temperature_sensor(max31_driver_t *dev);





#endif /* MAX30102_DRIVER_H */
