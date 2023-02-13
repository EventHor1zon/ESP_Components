/***************************************
* \file     Max30102_Driver.c
* \brief    Driver for the Max30102 pulse osimeter 
*           and heart-rate sensor
*
* \date     Dec 2020
* \author   RJAM
****************************************/

/********* Includes *******************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

#include "Max30102_Driver.h"
#include "genericCommsDriver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "Utilities.h"

#define DEBUG_MODE 1

/****** Function Prototypes ***********/

void max31_interrupt_handler(void *args);

/****** Global Data *******************/
#ifdef CONFIG_USE_PERIPH_MANAGER

const parameter_t max31_param_map[max31_param_length] = {
    // name, getfunc, setfunc, actfunc, parameter type, max value, flags //
    {"device ID", 1, &max31_get_device_id, NULL, NULL, DATATYPE_UINT8, 0, (GET_FLAG)},
    {"Mode", 2, &max31_get_mode, &max31_set_mode, NULL, DATATYPE_UINT8, 7, (GET_FLAG | SET_FLAG)},
    {"Sample Avg", 3, &max31_get_sample_average, &max31_set_sample_average, NULL, DATATYPE_UINT8, 5, (GET_FLAG | SET_FLAG)},
    {"FIFO Rollover", 4, &max31_get_fifo_rollover, &max31_set_fifo_rollover, NULL, DATATYPE_UINT8, 1, (GET_FLAG | SET_FLAG)},
    {"Almost full", 5, &max31_get_almost_full_val, &max31_set_almost_full_val, NULL, DATATYPE_UINT8, 0x0f, (GET_FLAG | SET_FLAG)},
    {"Shutdown", 6, &max31_get_shutdown, &max31_set_shutdown, NULL, DATATYPE_BOOL, 1, (GET_FLAG | SET_FLAG)},
    {"Sample Rate", 7, &max31_get_spo2_samplerate, &max31_set_spo2_samplerate, NULL, DATATYPE_UINT8, 7, (GET_FLAG | SET_FLAG)},
    {"Led PWM", 8, &max31_get_ledpwm, &max31_get_ledpwm, NULL, DATATYPE_UINT8, MAX31_LED_PWM_411, (GET_FLAG | SET_FLAG)},
    {"Red Amplitude", 9, &max31_get_redledamplitude, &max31_set_redledamplitude, NULL, DATATYPE_UINT8, 0xFF, (GET_FLAG | SET_FLAG)},
    {"IR Amplitude", 10, &max31_get_irledamplitude, &max31_set_irledamplitude, NULL, DATATYPE_UINT8, 0xFF, (GET_FLAG | SET_FLAG)},
    {"Red Amplitude", 11, &max31_get_redledamplitude, &max31_set_redledamplitude, NULL, DATATYPE_UINT8, 0xFF, (GET_FLAG | SET_FLAG)},
    {"Reset", 12, NULL, NULL, &max31_reset_device, DATATYPE_NONE, 0, (ACT_FLAG)},
    {"Ambi ovr", 13, &max31_get_ambient_light_invalidates, &max31_set_ambient_light_invalidates, DATATYPE_BOOL, 1, (GET_FLAG | SET_FLAG)},
};

const peripheral_t max31_periph_template = {
    .handle = NULL,
    .param_len = max31_param_length,
    .params = max31_param_map,
    .peripheral_id = 0,
    .peripheral_name = "Max30102", 
};


#endif /** CONFIG_USE_PERIPH_MANAGER **/

const char *MX_TAG = "MAX31 Driver";

/************ ISR *********************/

void max31_interrupt_handler(void *args) {

    BaseType_t higher_prio;
    max31_driver_t *dev = (max31_driver_t *)args;
    if(dev != NULL) {
        vTaskNotifyGiveFromISR(dev->taskhandle, &higher_prio);
    }
    portYIELD_FROM_ISR();
}

/****** Private Data ******************/

static TaskHandle_t taskhandle;

/****** Private Functions *************/

static void clear_fifo_buffer(max31_driver_t *dev) {
    memset(dev->fifo_buffer, 0, MAX31_FIFO_MAX_SIZE);
}


static esp_err_t max31_reset_fifo(max31_driver_t *dev) {
    
    esp_err_t status = ESP_OK;
    uint32_t clear_val = 0;

    status = gcd_i2c_write_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_FIFO_WRTPTR, 3, &clear_val);

    return status;
}



/** processes the raw data in the fifo_buffer into
 *  real samples 
 **/
static void process_data(max31_driver_t *dev) {

    uint8_t adc_shift = 0;
    uint32_t data = 0;
    bool dual_sample = false;
    bool red_channel = true;    /** true = red data, false = ir data **/
    float converted_data = 0;
    float conversion_factor = 0;

    memset(dev->red_buffer, 0, sizeof(float) * MAX31_FIFO_SAMPLES);
    memset(dev->ir_buffer, 0, sizeof(float) * MAX31_FIFO_SAMPLES);

    if(dev->bytes_read > 0) {
        
        if(dev->device_mode == MAX31_MODE_SPO2_RED_IR || dev->device_mode == MAX31_MODE_MULTILED_RIR) {
            dual_sample = true;
        }
        /** there isnt an IR only mode without dual sampling, so don't
         * need to check for it **/

        switch(dev->ledpwm) {
            case MAX31_LED_PWM_69:
            /** adc resolution = 15 bits **/
                adc_shift = 3;
                break;
            case MAX31_LED_PWM_118:
            /** adc resolution 16 bits **/
                adc_shift = 2;
                break;
            case MAX31_LED_PWM_215:
            /** adc res 17 bits **/
                adc_shift = 1;
                break;
            case MAX31_LED_PWM_411:
            /** adc res 18 bits **/
                break;
            default:
                ESP_LOGE(MX_TAG, "Error - invalid ledpwm set");
                break;
        }

        switch (dev->adcrange)
        {
            case MAX31_ADC_RNG_2048:
                /* code */
                conversion_factor = 7.81;
                break;
            case MAX31_ADC_RNG_4096:
                /* code */
                conversion_factor = 15.63;
                break;
            case MAX31_ADC_RNG_8192:
                /* code */
                conversion_factor = 31.25;
                break;
            case MAX31_ADC_RNG_16384:
                /* code */
                conversion_factor = 62.5;
                break;
            default:
                ESP_LOGE(MX_TAG, "Error - invalid adcrange set");
                break;
        }

        /** convert the data **/
        for(uint16_t i=0; i<dev->bytes_read; i+=3) {
            /** data is msb first **/
            data = ((dev->fifo_buffer[i+2] << 16) | (dev->fifo_buffer[i+1] | (dev->fifo_buffer[i])));            
            /** data is left-justified, shift right **/
            data = (data >> adc_shift);
            converted_data = ((float)data * conversion_factor);
            /** write converted data into register - alternate for dual sample **/
            if(red_channel) {
                dev->red_buffer[(i / 3)] = converted_data; 
            }
            else {
                dev->ir_buffer[(i / 3)] = converted_data;
            }
            red_channel = (dual_sample) ? !(red_channel) : red_channel;
        }
    }

    return;
}



static esp_err_t test_mode(max31_driver_t *dev) {

    uint8_t val = 0;
    bool v = false;
    ESP_LOGI(MX_TAG, "rst dev");
    max31_reset_device(dev);
    esp_err_t status; // = max31_set_shutdown(dev, &val);
    vTaskDelay(pdMS_TO_TICKS(100));

    status = max31_set_sample_average(dev, &val);
    val = 1;
    ESP_LOGI(MX_TAG, "rollover [%u]", status);

    max31_set_fifo_rollover(dev, &val);
    val = 0x00;
    ESP_LOGI(MX_TAG, "Setting the almost full value [%u]", status);
    status =max31_set_almost_full_val(dev, &val);

    ESP_LOGI(MX_TAG, "Setting the samples per sec [%u]", status);
    val = MAX31_SAMPLERATE_100;
    status = max31_set_spo2_samplerate(dev, &val);

    ESP_LOGI(MX_TAG, "Setting the led PW [%u]", status);
    val = MAX31_LED_PWM_411;
    status = max31_set_ledpwm(dev, &val);

    ESP_LOGI(MX_TAG, "Setting the led amplitude RED [%u]", status);
    val = 0x0f;
    status = max31_set_redledamplitude(dev, &val);

    ESP_LOGI(MX_TAG, "Setting the led amp IR [%u]", status);
    val = 0x0f;
    status = max31_set_irledamplitude(dev, &val);

    ESP_LOGI(MX_TAG, "Setting the mode [%u]", status);
    val = MAX31_MODE_HEARTRATE_RED;
    ESP_LOGI(MX_TAG, "Mode: %u", val);
    status = max31_set_mode(dev, &val);

    max31_reset_fifo(dev);

    // ESP_LOGI(MX_TAG, "starting device");
    // status = max31_set_shutdown(dev, &v);

    ESP_LOGI(MX_TAG, "Setting the interrupt function to A_FULL_EN");
    val = (MAX31_INTR_TYPE_ALMFULL);
    status = max31_set_interrupt_sources(dev, &val);
    ESP_LOGI(MX_TAG, "done setting up [%u]", status);
    vTaskDelay(pdMS_TO_TICKS(100));

    // gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, MAX31_REGADDR_INTR_EN1, 1, &val);
    // ESP_LOGI(MX_TAG, "r 2 [%02x]", val);
    // vTaskDelay(pdMS_TO_TICKS(100));
    // gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, MAX31_REGADDR_FIFO_CONFIG, 1, &val);
    // ESP_LOGI(MX_TAG, "r 8 [%02x]", val);
    // vTaskDelay(pdMS_TO_TICKS(100));
    // gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, MAX31_REGADDR_MODE_CONFIG, 1, &val);
    // ESP_LOGI(MX_TAG, "r 9 [%02x]", val);
    // vTaskDelay(pdMS_TO_TICKS(100));
    // gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, MAX31_REGADDR_SP02_CONFIG, 1, &val);
    // ESP_LOGI(MX_TAG, "r a [%02x]", val);
    // vTaskDelay(pdMS_TO_TICKS(100));
    // gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, MAX31_REGADDR_LED1PULSE_AMP, 1, &val);
    // ESP_LOGI(MX_TAG, "r c [%02x]", val);
    // vTaskDelay(pdMS_TO_TICKS(100));
    // gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, MAX31_REGADDR_LED2PULSE_AMP, 1, &val);
    // ESP_LOGI(MX_TAG, "r d [%02x]", val);
    // vTaskDelay(pdMS_TO_TICKS(100));
    // gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, MAX31_REGADDR_REV_ID, 1, &val);
    // ESP_LOGI(MX_TAG, "r fe [%02x]", val);    
    // vTaskDelay(pdMS_TO_TICKS(100));
    // gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, MAX31_REGADDR_PART_ID, 1, &val);
    // ESP_LOGI(MX_TAG, "r ff [%02x]", val);
    // vTaskDelay(pdMS_TO_TICKS(100));


    // vTaskDelay(pdMS_TO_TICKS(100));
    // gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, MAX31_REGADDR_OVRFLW_CNTR, 1, &val);
    // ESP_LOGI(MX_TAG, "r 5 [%02x]", val);    
    // vTaskDelay(pdMS_TO_TICKS(100));
    // gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, MAX31_REGADDR_FIFO_RDPTR, 1, &val);
    // ESP_LOGI(MX_TAG, "r 6 [%02x]", val);
    // gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, MAX31_REGADDR_FIFO_DATA, 1, &val);
    // vTaskDelay(pdMS_TO_TICKS(100));
    // ESP_LOGI(MX_TAG, "r 7 [%02x]", val);    
    // vTaskDelay(pdMS_TO_TICKS(100));
    // gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, MAX31_REGADDR_FIFO_RDPTR, 1, &val);
    // ESP_LOGI(MX_TAG, "r 5 [%02x]", val);    
    // vTaskDelay(pdMS_TO_TICKS(100));
    // gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, MAX31_REGADDR_FIFO_DATA, 1, &val);
    // ESP_LOGI(MX_TAG, "r 7 [%02x]", val);
    // vTaskDelay(pdMS_TO_TICKS(100));
    // gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, MAX31_REGADDR_FIFO_RDPTR, 1, &val);
    // ESP_LOGI(MX_TAG, "r 5 [%02x]", val);    
    // vTaskDelay(pdMS_TO_TICKS(100));
    // gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, MAX31_REGADDR_FIFO_DATA, 1, &val);
    // ESP_LOGI(MX_TAG, "r 7 [%02x]", val); 
    // vTaskDelay(pdMS_TO_TICKS(100));
    // gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, MAX31_REGADDR_FIFO_RDPTR, 1, &val);
    // ESP_LOGI(MX_TAG, "r 5 [%02x]", val);    
    // vTaskDelay(pdMS_TO_TICKS(100));
    // gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, MAX31_REGADDR_FIFO_DATA, 1, &val);
    // ESP_LOGI(MX_TAG, "r 7 [%02x]", val); 
    // vTaskDelay(pdMS_TO_TICKS(100));
    // gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, MAX31_REGADDR_FIFO_RDPTR, 1, &val);
    // ESP_LOGI(MX_TAG, "r 5 [%02x]", val);    
    // vTaskDelay(pdMS_TO_TICKS(100));
    // gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, MAX31_REGADDR_FIFO_DATA, 1, &val);
    // ESP_LOGI(MX_TAG, "r 7 [%02x]", val);
    // vTaskDelay(pdMS_TO_TICKS(100));
    // gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, MAX31_REGADDR_INTR_STATUS1, 1, &val);
    // ESP_LOGI(MX_TAG, "r 0 [%02x]", val);
    dev->configured = true;
    return status;
}


static uint8_t get_interrupt_source(max31_driver_t *dev) {
    uint16_t isr_source = 0;
    esp_err_t err = gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, MAX31_REGADDR_INTR_STATUS1, 2, &isr_source);
    /** merge the two bytes into one (no sense wasting the space) **/
    uint8_t src = ((uint8_t )(isr_source >> 8) | (uint8_t )isr_source);
    return src;
}


static void max31_task(void *args) {

    max31_driver_t *dev = (max31_driver_t *)args;
    uint8_t val = 0;
    esp_err_t status = ESP_OK;

    /** this driver task waits for an interrupt notification, then takes
     * required action - either reading fifo, sampling temperature or 
     * something something ALC_OVF... 
     **/

    while(1) {
        if(dev->configured != true) {
            vTaskDelay(1000); 
        } 
        else {

            ESP_LOGI(MX_TAG, "In task");
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY); 
            ESP_LOGI(MX_TAG, "got notify");
            val = get_interrupt_source(dev);
        
            if(val & MAX31_INTR_TYPE_DIETEMP_RDY) {
                status = max31_read_temperature(dev);
            }
            else if (val & MAX31_INTR_TYPE_NEWSAMPLE) {
            /** if interrupting on single sample, read single fifo measurement **/
            }
            else if (val & MAX31_INTR_TYPE_AMBILITOVF) {
                if(dev->ambi_ovr_invalidates) {
                    dev->drop_next_fifo = true;
                    max31_read_fifo(dev);
                }
            }
            else if (val & MAX31_INTR_TYPE_ALMFULL) {
                status = max31_read_fifo(dev);
                if(status == ESP_OK) {
                    process_data(dev);
                    if(dev->use_cbuff) {
                        cbuffer_write(dev->cbuff, dev->red_buffer, sizeof(float) * dev->pkts_in_fifo);
                        if(dev->device_mode == MAX31_MODE_MULTILED_RIR || dev->device_mode == MAX31_MODE_SPO2_RED_IR) {
                            cbuffer_write(dev->cbuff, dev->ir_buffer, sizeof(float) * dev->pkts_in_fifo);
                        }
                    }
#ifdef DEBUG_MODE
                    for(uint8_t i=0;i<MAX31_FIFO_SAMPLES; i++) {
                        ESP_LOGI(MX_TAG, "[Red %u] %f", i, dev->red_buffer[i]);
                    }
#endif
                }
            }
            else if (val & MAX31_INTR_TYPE_PWRRDY) {
                ;
            }
        }
    }

}



/****** Global Functions *************/


max31_driver_t *max31_init(max31_initdata_t *init) {

    esp_err_t istatus = ESP_OK;

    max31_driver_t *handle = (max31_driver_t *)heap_caps_calloc(1, sizeof(max31_driver_t), MALLOC_CAP_8BIT);
    void *fifo_mem = NULL;

    if(handle == NULL){
        ESP_LOGE(MX_TAG, "Error initialising driver memory!");
        istatus = ESP_ERR_NO_MEM;
    }
    else {
        handle->use_cbuff = (init->use_cbuffer && init->cbuffer != NULL) ? 1 : 0;
        handle->cbuff = (handle->use_cbuff) ? init->cbuffer : NULL;
        /** set the device default settings **/
        handle->dev_addr = (uint8_t )MAX31_SLAVE_ADDR;
        handle->ledpwm = MAX31_LED_PWM_69;
        handle->samplerate = MAX31_SAMPLERATE_50;
        handle->adcrange = MAX31_ADC_RNG_2048;
        handle->ledIR_ampl = 0;
        handle->ledRed_ampl = 0;
        handle->device_mode = MAX31_MODE_STARTUP;
        handle->fifo_ovr = 0;
        handle->smpavg = MAX31_SAMPLE_AVG_1;
        handle->use_fifo = true;

        if(init->intr_pin) {
            gpio_config_t io_ini = {0};
            io_ini.intr_type = GPIO_INTR_NEGEDGE;
            io_ini.mode = GPIO_MODE_INPUT;
            io_ini.pin_bit_mask = (uint32_t)(1 << init->intr_pin);
            io_ini.pull_down_en = GPIO_PULLDOWN_DISABLE;
            io_ini.pull_up_en = GPIO_PULLUP_ENABLE;
            /** config the interrupt pin**/
            istatus = gpio_config(&io_ini);

            if(istatus == ESP_OK) {
                handle->intr_pin = init->intr_pin;
                istatus = gpio_isr_handler_add(handle->intr_pin, max31_interrupt_handler, (void *)handle);
                if(istatus != ESP_OK) {
                    ESP_LOGE(MX_TAG, "Error adding interrupt handler");
                } 
            } else {
                ESP_LOGE(MX_TAG, "Error configuring interrupt pin");
            }
        }

        if(istatus == ESP_OK) {
            /** create the driver task **/
            if(xTaskCreate(max31_task, "max31_task", 2056, (void *)handle, 3, &taskhandle) != pdTRUE) {
                ESP_LOGE(MX_TAG, "Error in creating driver task");
                istatus = ESP_ERR_NO_MEM;
            } 
            else {
                handle->taskhandle = taskhandle;
            }
        }

        if(istatus == ESP_OK) {
            if(init->i2c_bus > 3) {
                ESP_LOGE(MX_TAG, "Error: Invalid i2c bus");
                istatus = ESP_ERR_INVALID_ARG;
            } 
            else {
                handle->i2c_bus = init->i2c_bus;
            }
        }
    
        if(istatus != ESP_OK && handle != NULL) {
            /** dont leave trash if init fails **/
#ifdef CONFIG_DRIVERS_USE_HEAP
            heap_caps_free(handle);
#endif
        } else {
            ESP_LOGI(MX_TAG, "Max30102 driver started succesfully!");
#ifdef DEBUG_MODE
            test_mode(handle);
#endif
        }
    }

    return handle;
}



esp_err_t max31_get_device_id(max31_driver_t *dev, uint8_t *val) {

    esp_err_t status = ESP_OK;
    uint8_t reg= 0;
    status = gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, (uint8_t)MAX31_REGADDR_PART_ID, 1, &reg);
    *val = reg;
    return status;
}


/** PARAM SETS **/

esp_err_t max31_clear_interrupt_sources(max31_driver_t *dev) {
    uint8_t val = 0b11100000;
    esp_err_t err = gcd_i2c_write_address(dev->i2c_bus, dev->dev_addr, MAX31_REGADDR_INTR_EN1, 1, &val);
    val = 0b00000010;
    err += gcd_i2c_write_address(dev->i2c_bus, dev->dev_addr, MAX31_REGADDR_INTR_EN2, 1, &val);
    return err;
}


esp_err_t max31_set_interrupt_sources(max31_driver_t *dev, uint8_t *intr_mask) {

    esp_err_t status = ESP_OK;
    uint8_t regval= 0;
    uint8_t v = *intr_mask;

    if(v & 0b00011101) {
        status = ESP_ERR_INVALID_ARG;
    } else {
        /*** clear existing interrupts **/
        status = max31_clear_interrupt_sources(dev);
        if(!status) {
            /** write intr2 reg if set **/
            if(v & MAX31_INTR_TEMP_RDY) {
                regval = 2;
                status = gcd_i2c_write_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_INTR_EN2, 1, &regval);
                /** clear bit **/
                v &= ~(MAX31_INTR_TEMP_RDY);
            }
            /** write other interrupts to intr1 **/
            if(v) {
                ESP_LOGI(MX_TAG, "Intr1: 0x%02x, writing 0x%02x", regval, *intr_mask);
                status = gcd_i2c_write_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_INTR_EN1, 1, &v);
            }
        }
        else {
            ESP_LOGE("MAX31", "Error clearing interrupts!");
        }

    }

    if(status == ESP_OK) {
        dev->intr_mask = *intr_mask;
    }
    return status;
}


esp_err_t max31_get_ambient_light_invalidates(max31_driver_t *dev, bool *val) {
    *val = dev->ambi_ovr_invalidates;
    return ESP_OK;
}


esp_err_t max31_set_ambient_light_invalidates(max31_driver_t *dev, bool *val) {
    
    dev->ambi_ovr_invalidates = *val;
    return ESP_OK;
}


esp_err_t max31_get_sample_average(max31_driver_t *dev, max31_sampleavg_t *val) {
    
    esp_err_t status = ESP_OK; 
    *val = dev->smpavg;
    return status;
}


esp_err_t max31_set_sample_average(max31_driver_t *dev, max31_sampleavg_t *val) {
    
    esp_err_t status = ESP_OK; 
    uint8_t regval = 0;
    uint8_t v = *val;
    if(v > MAX31_SAMPLE_AVG_32) {
        status = ESP_ERR_INVALID_ARG;
    } else {
        status = gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_FIFO_CONFIG, 1, &regval);
        regval |= ((v) << 5);
        status = gcd_i2c_write_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_FIFO_CONFIG, 1, &regval);
        if(status == ESP_OK) {
            dev->smpavg = v;
        }
    }
    return status;
}


esp_err_t max31_get_fifo_rollover(max31_driver_t *dev, uint8_t *val) {
    
    esp_err_t status = ESP_OK;
    *val = dev->fifo_ovr;
    return status;
}


esp_err_t max31_set_fifo_rollover(max31_driver_t *dev, uint8_t *val) {
    
    esp_err_t status = ESP_OK;
    
    uint8_t write =0;
    uint8_t regval = 0;
    uint8_t v = *val;

    write = (1 << 5);
    status = gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_FIFO_CONFIG, 1, &regval);
    if(status == ESP_OK) {
        if(v) {
            regval |= write;
        } else {
            regval &= ~(write);
        }
        status = gcd_i2c_write_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_FIFO_CONFIG, 1, &regval);
        if(status == ESP_OK) {
            dev->fifo_ovr = write ? 1 : 0;
        }
    }
    return status;
}


esp_err_t max31_get_almost_full_val(max31_driver_t *dev, uint8_t *val) {
    esp_err_t status = ESP_OK;
    *val = dev->almostfull;
    return status;
}


esp_err_t max31_set_almost_full_val(max31_driver_t *dev, uint8_t *val) {
    esp_err_t status = ESP_OK;
    uint8_t regval = 0;
    uint8_t v = *val; 
    if(v > MAX31_ALMOSTFULL_MAX) {
        status = ESP_ERR_INVALID_ARG;
    } else {
        status = gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_FIFO_CONFIG, 1, &regval);
        regval |= v;
        status = gcd_i2c_write_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_FIFO_CONFIG, 1, &regval);
    }
    if(status == ESP_OK) {
        dev->almostfull = *val;
    }

    return status;
}


esp_err_t max31_get_shutdown(max31_driver_t *dev, bool *val) {
    esp_err_t status = ESP_OK;
    *val = dev->shutdown;
    return status;
}


esp_err_t max31_set_shutdown(max31_driver_t *dev, bool *val) {
    esp_err_t status = ESP_OK;
    uint8_t regval = 0;
    uint8_t write = 0;
    bool v = *val;

    write = (1 << 7);

    status = gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_MODE_CONFIG, 1, &regval);
    
    if(v) {
        regval |= write;
    } else {
        regval &= ~(write);
    }
    status = gcd_i2c_write_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_MODE_CONFIG, 1, &regval);
    if(status == ESP_OK) {
        dev->shutdown = v ? 1 : 0;
    }
    return status;
}


esp_err_t max31_get_mode(max31_driver_t *dev, max31_mode_t *val) {
    esp_err_t status = ESP_OK;
    *val = dev->device_mode;
    return status;
}


esp_err_t max31_set_mode(max31_driver_t *dev, max31_mode_t *val) {
    esp_err_t status = ESP_OK;
    uint8_t regval = 0;
    uint8_t write = 0;
    regval = *val;
    ESP_LOGI(MX_TAG, "setting mode %u", regval);

    if(regval == MAX31_MODE_HEARTRATE_RED ||
       regval == MAX31_MODE_SPO2_RED_IR   ||
       regval== MAX31_MODE_MULTILED_RIR) {
        write = regval;

        status = gcd_i2c_write_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_MODE_CONFIG, 1, &write);
        ESP_LOGI(MX_TAG, "wrote mode %u", write);
    } else {
        ESP_LOGI(MX_TAG, "failed to write mode %u", write);
        status = ESP_ERR_INVALID_ARG;
    }

    if(status == ESP_OK) {
        dev->device_mode = write;
    }

    return status;
}


esp_err_t max31_get_spo2_samplerate(max31_driver_t *dev, uint8_t *val) {
    esp_err_t status = ESP_OK;
    *val = dev->samplerate;
    return status;
}


esp_err_t max31_set_spo2_samplerate(max31_driver_t *dev, uint8_t *val) {
    esp_err_t status = ESP_OK;
    uint8_t regval = 0;
    if(*val > MAX31_SAMPLERATE_3200) {
        status = ESP_ERR_INVALID_ARG;
    } else {
        status = gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_SP02_CONFIG, 1, &regval);
        regval |= (*val << 2);
        status = gcd_i2c_write_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_SP02_CONFIG, 1, &regval);
    }

    if(status == ESP_OK) {
        dev->samplerate = *val;
    }

    return status;
}


esp_err_t max31_get_ledpwm(max31_driver_t *dev, uint8_t *val) {
    esp_err_t status = ESP_OK;
    *val = dev->ledpwm;
    return status;
}


esp_err_t max31_set_ledpwm(max31_driver_t *dev, uint8_t *val) {
    esp_err_t status = ESP_OK;
    uint8_t regval = 0;
    if(*val > MAX31_LED_PWM_411) {
        status = ESP_ERR_INVALID_ARG;
    } else {
        status = gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_SP02_CONFIG, 1, &regval);
        regval |= (*val);
        status = gcd_i2c_write_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_SP02_CONFIG, 1, &regval);
    }

    if(status == ESP_OK) {
        dev->ledpwm = *val;
    }

    return status;
}


esp_err_t max31_get_redledamplitude(max31_driver_t *dev, uint8_t *val) {
    esp_err_t status = ESP_OK;
    *val = dev->ledRed_ampl;
    return status;
}


esp_err_t max31_set_redledamplitude(max31_driver_t *dev, uint8_t *val) {
    esp_err_t status = ESP_OK;
    uint8_t regval = 0;

    regval = *val;
    status = gcd_i2c_write_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_LED1PULSE_AMP, 1, &regval);

    if(status == ESP_OK) {
        dev->red_lvl = *val;
    }

    return status;
}


esp_err_t max31_get_irledamplitude(max31_driver_t *dev, uint8_t *val) {
    esp_err_t status = ESP_OK;
    *val = dev->ledIR_ampl;
    return status;
}


esp_err_t max31_set_irledamplitude(max31_driver_t *dev, uint8_t *val) {
    esp_err_t status = ESP_OK;
    uint8_t regval = 0;

    regval = *val;
    status = gcd_i2c_write_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_LED2PULSE_AMP, 1, &regval);

    if(status == ESP_OK) {
        dev->red_lvl = *val;
    }

    return status;
}


esp_err_t max31_get_fifo_ovr(max31_driver_t *dev, uint8_t *val) {
    esp_err_t err = ESP_OK;
    uint8_t ovr = 0;
    err = gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_OVRFLW_CNTR, 1, &ovr);
    if(!err) {
        *val = ovr;
    }
    return err;
}


esp_err_t max31_get_temperature(max31_driver_t *dev, float *val) {
    *val = dev->temperature;
    return ESP_OK;
}


/** ACTIONS **/

esp_err_t max31_read_temperature(max31_driver_t *dev) {

    uint8_t regval[2] = {0};
    int8_t temp_int = 0;
    float temp_f = 0.0;
    esp_err_t err = gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, MAX31_REGADDR_DIETEMP_INT, 2, regval);

    if(!err) {
        temp_int = (int8_t)regval[0];
        temp_f = ((float) regval[1]) * 0.0625;
        temp_f = temp_f + ((float)temp_int);
        dev->temperature = temp_f;
    }
    return err;
}


esp_err_t max31_read_fifo(max31_driver_t *dev) {

    esp_err_t status = ESP_OK;

    uint8_t regvals[3] = {0};
    uint8_t rd_ptr_val = 0;
    uint8_t wr_ptr_val = 0;
    uint8_t avail_samples = 0;
    uint8_t avail_bytes = 0;
    uint8_t fifo_width = 3;

    /** point at the avail fifo mem **/
    uint8_t *ptr = dev->fifo_buffer;

    if(dev->use_fifo == false) {
        status = ESP_ERR_INVALID_STATE;
    }
    else if(dev->drop_next_fifo) {
        max31_reset_fifo(dev);
        dev->drop_next_fifo = false;
        status = ESP_ERR_INVALID_RESPONSE;
#ifdef DEBUG_MODE
        ESP_LOGI(MX_TAG, "ALC OVERFLOW - clearing fifo");
#endif
    }

    else {
        status = gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_FIFO_WRTPTR, 3, regvals);

        /** get number of bytes **/
        if(status == ESP_OK) {
            rd_ptr_val = (regvals[2] & 0b00011111);
            wr_ptr_val = (regvals[0] & 0b00011111);
            /** does read_ptr increment per byte or per sample?? **/
            avail_samples = wr_ptr_val > rd_ptr_val ? (wr_ptr_val - rd_ptr_val) : (MAX31_FIFO_SAMPLES - rd_ptr_val) + wr_ptr_val;
            
            /** if collecting both datas, fifo width = 6 **/
            if(dev->device_mode == MAX31_MODE_SPO2_RED_IR) {
                fifo_width = 6;
            }
            avail_bytes = avail_samples * fifo_width;
            dev->pkts_in_fifo = avail_samples * (fifo_width / 3);
#ifdef DEBUG_MODE
            ESP_LOGI(MX_TAG, "Write_ptr: %u Rd_Ptr: %u Packets: %u Avail bytes: %u", regvals[0], regvals[2], avail_samples, avail_bytes);
#endif
            if(avail_bytes > MAX31_FIFO_MAX_SIZE) {
                avail_bytes = MAX31_FIFO_MAX_SIZE;
                ESP_LOGI(MX_TAG, "Not enough space for all fifo!");
            }
        }
        if(status == ESP_OK && avail_bytes) {
            memset(dev->fifo_buffer, 0, MAX31_FIFO_MAX_SIZE);
            dev->bytes_read = 0;
            /** read the data from fifo **/
            status = gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_FIFO_DATA, avail_bytes, dev->fifo_buffer);

            if(status) {
                ESP_LOGE(MX_TAG, "Error reading %u bytes from fifo!", avail_bytes);
                showmem(dev->fifo_buffer, 200);
            }
            else {
                dev->bytes_read = avail_bytes;
            }
        }
    }

    return status;
}


esp_err_t max31_reset_device(max31_driver_t *dev) {
    esp_err_t status = ESP_OK;
    uint8_t regval = 0;
    uint8_t  write = (1 << 6);

    status = gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_MODE_CONFIG, 1, &regval);
    regval |= write;
    status = gcd_i2c_write_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_MODE_CONFIG, 1, &regval);

    return status;
}


esp_err_t max31_enable_temperature_sensor(max31_driver_t *dev) {
    esp_err_t status = ESP_OK;
    uint8_t  write = 1;

    status = gcd_i2c_write_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_DIETEMP_SAMPLE, 1, &write);
    dev->temp_sampling = true;
    return status;
}

