/***************************************
* \file     Max30102_Driver.c
* \brief    Driver for the Max30102 pulse osimeter 
*           and heart-rate sensor
*
*
*           Driver going well, still some todos. Need to decide how i'm handling the
*           the task/event system. If the user isn't using events then we probably want
*           to have at least some automation in there, or what else is it doing?
*           Don't like to go much further than giving user access to the device functions 
*           (allowing people to build their own systems like a proper libraray), but we do have this
*           interrupt servicing task. 
*           Maybe go minimalist and implement a fifo auto-read setting or something?
*
*
*           Also, do we need to do floating-point maths to generate pico-amps? Maybe leave
*           that to the data processing functions, and instead use raw ADC counts instead to save
*           space. We only really care about the difference peak-to-peak in case of HB detect.
*
*           
*
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

#include "../Utils/Utilities.h"

// #define DEBUG_MODE 1

/****** Function Prototypes ***********/

gpio_isr_t max31_interrupt_handler(void *args);

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

gpio_isr_t max31_interrupt_handler(void *args) {

    BaseType_t higher_prio;
    MAX31_h dev = (MAX31_h )args;
    if(dev != NULL) {
        vTaskNotifyGiveFromISR(dev->taskhandle, &higher_prio);
    }
    portYIELD_FROM_ISR();
    return 0;
}

/****** Private Data ******************/

static TaskHandle_t taskhandle;

/****** Private Functions *************/

#ifdef CONFIG_ENABLE_MAX31_EVENTS

static esp_err_t emit_fifo_almostfull_event(MAX31_h dev) {
    return esp_event_post_to(dev->event_loop, 0, MAX31_EVENT_FIFO_ALMOST_FULL, dev, sizeof(void *), pdMS_TO_TICKS(CONFIG_SHORTWAIT_MS));
}

static esp_err_t emit_ambient_overflow_event(MAX31_h dev) {
    return esp_event_post_to(dev->event_loop, 0, MAX31_EVENT_AMBI_OVR, dev, sizeof(void *), pdMS_TO_TICKS(CONFIG_SHORTWAIT_MS));
}

static esp_err_t emit_new_reddata_event(MAX31_h dev) {
    return esp_event_post_to(dev->event_loop, 0, MAX31_EVENT_NEW_RED_DATA, dev, sizeof(void *), pdMS_TO_TICKS(CONFIG_SHORTWAIT_MS));
}

static esp_err_t emit_new_irdata_event(MAX31_h dev) {
    return esp_event_post_to(dev->event_loop, 0, MAX31_EVENT_NEW_IR_DATA, dev, sizeof(void *), pdMS_TO_TICKS(CONFIG_SHORTWAIT_MS));
}

static esp_err_t emit_fifo_read_done_event(MAX31_h dev) {
    return esp_event_post_to(dev->event_loop, 0, MAX31_EVENT_FIFO_READ_COMPLETE, dev, sizeof(void *), pdMS_TO_TICKS(CONFIG_SHORTWAIT_MS));
}

#endif /** CONFIG_ENABLE_MAX31_EVENTS **/


static void clear_fifo_buffer(MAX31_h dev) {
    memset(dev->fifo_buffer, 0, MAX31_FIFO_MAX_SIZE);
}


static uint8_t adc_shift_from_pwm(max31_ledpwm_t pwm) {
    uint8_t adc_shift = 0;

    switch(pwm) {
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

    return adc_shift;
}


static float conversion_factor_from_adcrange(max31_adcrange_t adc_range) {

    float conversion_factor = 0;

    switch (adc_range)
    {
        case MAX31_ADC_RNG_2048:
            conversion_factor = 7.81;
            break;
        case MAX31_ADC_RNG_4096:
            conversion_factor = 15.63;
            break;
        case MAX31_ADC_RNG_8192:
            conversion_factor = 31.25;
            break;
        case MAX31_ADC_RNG_16384:
            conversion_factor = 62.5;
            break;
        default:
            ESP_LOGE(MX_TAG, "Error - invalid adcrange set");
            break;
    }

    return conversion_factor;
}


static esp_err_t max31_reset_fifo(MAX31_h dev) {
    
    esp_err_t status = ESP_OK;
    uint32_t clear_val = 0;

    status = gcd_i2c_write_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_FIFO_WRTPTR, 3, &clear_val);

    return status;
}


static uint8_t get_interrupt_source(MAX31_h dev) {
    uint16_t isr_source = 0;
    esp_err_t err = gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, MAX31_REGADDR_INTR_STATUS1, 2, &isr_source);
    /** merge the two bytes into one (no sense wasting the space) **/
    uint8_t src = ((uint8_t )(isr_source >> 8) | (uint8_t )isr_source);
    return src;
}


static void convert_fifo_data(MAX31_h dev) {

    bool dual_sample = false;
    uint16_t counter=0;
    uint16_t increment=0;
    float converted;
    uint32_t raw;
    uint32_t raw_shifted;
    float factor;
    uint8_t adc_shift;
    
    if(dev->bytes_read == 0) {
        return;
    }

    adc_shift = adc_shift_from_pwm(dev->dev_settings.ledpwm);
    factor = conversion_factor_from_adcrange(dev->dev_settings.adcrange);    

    if(dev->dev_settings.device_mode == MAX31_MODE_SPO2_RED_IR || dev->dev_settings.device_mode == MAX31_MODE_MULTILED_RIR) {
        dual_sample = true;
    }

    /** TODO: How to tell which sample comes first? We should always be reading the 
     *  fifo with a read pointer of 0, which is IR data. Assume this for now.
     **/
    ESP_LOGI(MX_TAG, "Converting with adc shift= %u factor= %.3f", adc_shift, factor);
    memset(dev->red_buffer, 0, sizeof(float) * MAX31_FIFO_SAMPLES);
    memset(dev->ir_buffer, 0, sizeof(float) * MAX31_FIFO_SAMPLES);

    for(counter=0; counter < dev->bytes_read; counter+=3) {
#ifdef DEBUG_MODE
        printf("Data Raw[0] %02x Raw[1] %02x Raw[2] %02x ", dev->fifo_buffer[counter], dev->fifo_buffer[counter+1], dev->fifo_buffer[counter+2]); 
#endif
        raw = ((dev->fifo_buffer[counter] << 16) | (dev->fifo_buffer[counter+1] << 8) | (dev->fifo_buffer[counter+2]));
        raw_shifted = raw >> adc_shift;
        converted = ((float)raw_shifted) * factor;

#ifdef DEBUG_MODE
        printf("Converged %u Shifted %u Converted %f\n",  raw, raw_shifted, converted);
#endif

        /** if we're expecting IR and fifo# is even, store IR **/
        if(dual_sample && (counter % 2 == 0))  {
            dev->ir_buffer[increment] = converted;
        }
        /** else we're not expecting IR or # is odd **/
        else {
            dev->red_buffer[increment] = converted;
            printf("Red channel[%u] = %.3f (converted = %.3f)\n", increment, dev->red_buffer[increment], converted);
            increment++; /** only increment this counter once per pair **/
        
        }
    }

}


static esp_err_t test_mode(MAX31_h dev) {

    uint8_t val = 0;
    bool v = false;
    ESP_LOGI(MX_TAG, "rst dev");
    max31_reset_device(dev);
    esp_err_t status; // = max31_set_shutdown(dev, &val);

    ESP_LOGI(MX_TAG, "Setting the interrupt function to A_FULL_EN");
    val = (MAX31_INTR_TYPE_ALMFULL | MAX31_INTR_TYPE_AMBILITOVF | MAX31_INTR_TYPE_NEWSAMPLE);
    status = max31_set_interrupt_sources(dev, &val);

    status = max31_set_sample_average(dev, &val);
    val = 0;
    ESP_LOGI(MX_TAG, "rollover [%u]", status);

    status = max31_set_fifo_rollover(dev, &val);
    val = 0x0F;
    ESP_LOGI(MX_TAG, "Setting the almost full value [%u]", status);
    status = max31_set_almost_full_val(dev, &val);

    ESP_LOGI(MX_TAG, "Setting the samples per sec [%u]", status);
    val = MAX31_SAMPLERATE_100;
    status = max31_set_spo2_samplerate(dev, &val);

    ESP_LOGI(MX_TAG, "Setting the led PW [%u]", status);
    val = MAX31_LED_PWM_411;
    status = max31_set_ledpwm(dev, &val);

    ESP_LOGI(MX_TAG, "Setting the led amplitude RED [%u]", status);
    val = 0x3f;
    status = max31_set_redledamplitude(dev, &val);

    ESP_LOGI(MX_TAG, "Setting the led amp IR [%u]", status);
    status = max31_set_irledamplitude(dev, &val);

    ESP_LOGI(MX_TAG, "Setting the mode [%u]", status);
    val = MAX31_MODE_HEARTRATE_RED;
    ESP_LOGI(MX_TAG, "Mode: %u", val);
    status = max31_set_mode(dev, &val);

    max31_reset_fifo(dev);

    // ESP_LOGI(MX_TAG, "starting device");
    // status = max31_set_shutdown(dev, &v);


    ESP_LOGI(MX_TAG, "done setting up [%u]", status);
    vTaskDelay(pdMS_TO_TICKS(10));

    return status;
}


static void max31_task(void *args) {

    MAX31_h dev = (MAX31_h )args;
    uint8_t val = 0;
    esp_err_t status = ESP_OK;
    uint32_t notify;
    uint8_t tmp;


    /** this driver task waits for an interrupt notification, then takes
     * required action - either reading fifo, sampling temperature or 
     * something something ALC_OVF... 
     **/

    while(1) {
        if(dev->configured != true) {
            vTaskDelay(100); 
        } 
        else {

            ESP_LOGI(MX_TAG, "In task");
            notify = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100)); 
            if(!notify) {
                ESP_LOGE(MX_TAG, "Nothing occured timeout");
                val = get_interrupt_source(dev);
                ESP_LOGI(MX_TAG, "Interrupt status reg: %02x", val);
            }
            else {
                ESP_LOGI(MX_TAG, "got notify");
                val = get_interrupt_source(dev);
            
                if(val & MAX31_INTR_TYPE_DIETEMP_RDY) {
                    ESP_LOGI(MX_TAG, "Die temp ready interrupt");
                    status = max31_read_temperature(dev);
                }
                if (val & MAX31_INTR_TYPE_NEWSAMPLE) {
                /** if interrupting on single sample, read single fifo measurement **/
                    ESP_LOGI(MX_TAG, "New sample interrupt");
                    max31_get_fifo_ovr(dev, &tmp);
                    ESP_LOGI(MX_TAG, "Overflow = %u", tmp);
                    if(tmp) {
                        max31_read_fifo(dev);
                    }
                }
                if (val & MAX31_INTR_TYPE_AMBILITOVF) {
                    ESP_LOGI(MX_TAG, "Ambient overflow interrupt");
#ifdef CONFIG_ENABLE_MAX31_EVENTS
                    if(dev->use_events && \
                       dev->event_mask & MAX31_EVENT_AMBI_OVR
                    ) {
                        emit_ambient_overflow_event(handle);
                    }
#endif
                    if(dev->ambi_ovr_invalidates) {
                        dev->drop_next_fifo = true;
                        max31_read_fifo(dev);
                    }
                }
                if (val & MAX31_INTR_TYPE_ALMFULL) {
                    ESP_LOGI(MX_TAG, "Almost full interrupt");
#ifdef CONFIG_ENABLE_MAX31_EVENTS
    /** TODO: Events **/
                    if(dev->use_events && \
                       dev->event_mask & MAX31_EVENT_FIFO_ALMOST_FULL
                    ) {
                        emit_fifo_almostfull_event(handle);
                    }
#endif
                    if(status == ESP_OK && dev->read_fifo_on_almostfull) {
                        status = max31_read_fifo(dev);
#ifdef CONFIG_ENABLE_MAX31_EVENTS
                    if(dev->use_events && \
                       dev->event_mask & MAX31_EVENT_FIFO_READ_COMPLETE &&
                       status == ESP_OK
                    ) {
                        emit_fifo_read_done_event(handle);
                    }
#endif
                        convert_fifo_data(dev);
#ifdef CONFIG_SUPPORT_CBUFF
                        if(dev->use_cbuff) {
                            cbuffer_write(dev->cbuff, dev->red_buffer, sizeof(float) * dev->pkts_in_fifo);
                            if(dev->device_mode == MAX31_MODE_MULTILED_RIR || dev->device_mode == MAX31_MODE_SPO2_RED_IR) {
                                cbuffer_write(dev->cbuff, dev->ir_buffer, sizeof(float) * dev->pkts_in_fifo);
                            }
                        }
#endif
                        for(uint8_t i=0;i<MAX31_FIFO_SAMPLES; i++) {
                            ESP_LOGI(MX_TAG, "[Red %u] %f", i, dev->red_buffer[i]);
                        }
                    }
                }
                else if (val & MAX31_INTR_TYPE_PWRRDY) {
                    ;
                }
            }
        }
    }

}



/****** Global Functions *************/


MAX31_h max31_init(max31_initdata_t *init) {

    esp_err_t istatus = ESP_OK;

    MAX31_h handle = (MAX31_h )heap_caps_calloc(1, sizeof(max31_driver_t), MALLOC_CAP_8BIT);
    void *fifo_mem = NULL;

    if(handle == NULL){
        ESP_LOGE(MX_TAG, "Error initialising driver memory!");
        istatus = ESP_ERR_NO_MEM;
    }
    else {
#ifdef CONFIG_SUPPORT_CBUFF
        handle->use_cbuff = (init->use_cbuffer && init->cbuffer != NULL) ? 1 : 0;
        handle->cbuff = (handle->use_cbuff) ? init->cbuffer : NULL;
#endif /** CONFIG_SUPPORT_CBUFF **/

#ifdef CONFIG_ENABLE_MAX31_EVENTS
        handle->use_events = init->use_events;
        handle->event_loop = init->event_loop;
#endif /** CONFIG_ENABLE_MAX31_EVENTS **/

        /** set the device default settings **/
        handle->dev_addr = (uint8_t )MAX31_SLAVE_ADDR;
        handle->dev_settings.ledpwm = MAX31_LED_PWM_69;
        handle->dev_settings.samplerate = MAX31_SAMPLERATE_50;
        handle->dev_settings.adcrange = MAX31_ADC_RNG_2048;
        handle->dev_settings.ledIR_ampl = 0;
        handle->dev_settings.ledRed_ampl = 0;
        handle->dev_settings.device_mode = MAX31_MODE_STARTUP;
        handle->fifo_settings.fifo_ovr = 0;
        handle->dev_settings.smpavg = MAX31_SAMPLE_AVG_1;
        handle->fifo_settings.use_fifo = true;

        if(init->intr_pin > 0) {
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
            handle->configured = true;
            max31_reset_fifo(handle);
        }
    }

    return handle;
}



esp_err_t max31_get_device_id(MAX31_h dev, uint8_t *val) {

    esp_err_t status = ESP_OK;
    uint8_t reg= 0;
    status = gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, (uint8_t)MAX31_REGADDR_PART_ID, 1, &reg);
    *val = reg;
    return status;
}


/** PARAM SETS **/

esp_err_t max31_clear_interrupt_sources(MAX31_h dev) {
    uint16_t val = 0;
    esp_err_t err = gcd_i2c_write_address(dev->i2c_bus, dev->dev_addr, MAX31_REGADDR_INTR_EN1, 2, &val);
    return err;
}


esp_err_t max31_set_interrupt_sources(MAX31_h dev, uint8_t *intr_mask) {

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


esp_err_t max31_get_ambient_light_invalidates(MAX31_h dev, bool *val) {
    *val = dev->ambi_ovr_invalidates;
    return ESP_OK;
}


esp_err_t max31_set_ambient_light_invalidates(MAX31_h dev, bool *val) {
    
    dev->ambi_ovr_invalidates = *val;
    return ESP_OK;
}


esp_err_t max31_get_sample_average(MAX31_h dev, max31_sampleavg_t *val) {
    
    esp_err_t status = ESP_OK; 
    *val = dev->dev_settings.smpavg;
    return status;
}


esp_err_t max31_set_sample_average(MAX31_h dev, max31_sampleavg_t *val) {
    
    esp_err_t status = ESP_OK; 
    uint8_t regval = 0;
    uint8_t v = *val;
    if(v > MAX31_SAMPLE_AVG_32) {
        status = ESP_ERR_INVALID_ARG;
    } else {
        status = gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_FIFO_CONFIG, 1, &regval);
        ESP_LOGI(MX_TAG, "Read FIFO CONFIG reg : Val: %02x", regval);
        regval |= ((v) << 5);
        ESP_LOGI(MX_TAG, "Writing FIFO CONFIG reg : Val: %02x", regval);
        status = gcd_i2c_write_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_FIFO_CONFIG, 1, &regval);
        if(status == ESP_OK) {
            dev->dev_settings.smpavg = v;
        }
    }
    return status;
}


esp_err_t max31_get_fifo_rollover(MAX31_h dev, uint8_t *val) {
    
    esp_err_t status = ESP_OK;
    *val = dev->fifo_settings.fifo_ovr;
    return status;
}


esp_err_t max31_set_fifo_rollover(MAX31_h dev, uint8_t *val) {
    
    esp_err_t status = ESP_OK;
    
    uint8_t write =0;
    uint8_t regval = 0;
    uint8_t v = *val;

    write = (1 << 4);
    status = gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_FIFO_CONFIG, 1, &regval);
    if(status == ESP_OK) {
        if(v) {
            regval |= write;
        } else {
            regval &= ~(write);
        }
        status = gcd_i2c_write_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_FIFO_CONFIG, 1, &regval);
        if(status == ESP_OK) {
            dev->fifo_settings.fifo_ovr = write ? 1 : 0;
        }
    }
    return status;
}


esp_err_t max31_get_almost_full_val(MAX31_h dev, uint8_t *val) {
    esp_err_t status = ESP_OK;
    *val = dev->fifo_settings.almostfull;
    return status;
}


esp_err_t max31_set_almost_full_val(MAX31_h dev, uint8_t *val) {
    esp_err_t status = ESP_OK;
    uint8_t regval = 0;
    uint8_t v = *val; 
    if(v > MAX31_ALMOSTFULL_MAX) {
        status = ESP_ERR_INVALID_ARG;
    } else {
        ESP_LOGI(MX_TAG, "Writing %u to the fifo config", v);
        status = gcd_i2c_read_mod_write(dev->i2c_bus, dev->dev_addr, MAX31_REGADDR_FIFO_CONFIG, v, 0b1111);
    }
    if(status == ESP_OK) {
        dev->fifo_settings.almostfull = v;
    }

    return status;
}


esp_err_t max31_get_shutdown(MAX31_h dev, bool *val) {
    esp_err_t status = ESP_OK;
    *val = dev->shutdown;
    return status;
}


esp_err_t max31_set_shutdown(MAX31_h dev, bool *val) {
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


esp_err_t max31_get_mode(MAX31_h dev, max31_mode_t *val) {
    esp_err_t status = ESP_OK;
    *val = dev->dev_settings.device_mode;
    return status;
}


esp_err_t max31_set_mode(MAX31_h dev, max31_mode_t *val) {
    esp_err_t status = ESP_OK;
    uint8_t mode = *val;
    uint8_t write = 0;
    ESP_LOGI(MX_TAG, "setting mode %u", mode);

    if(mode == MAX31_MODE_HEARTRATE_RED ||
       mode == MAX31_MODE_SPO2_RED_IR   ||
       mode== MAX31_MODE_MULTILED_RIR) {
        write = mode;

        status = gcd_i2c_write_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_MODE_CONFIG, 1, &write);
        ESP_LOGI(MX_TAG, "wrote mode %u", write);
    } else {
        ESP_LOGI(MX_TAG, "invalid mode %u", write);
        status = ESP_ERR_INVALID_ARG;
    }

    if(status == ESP_OK) {
        dev->dev_settings.device_mode = write;
    }

    return status;
}


esp_err_t max31_get_spo2_samplerate(MAX31_h dev, uint8_t *val) {
    esp_err_t status = ESP_OK;
    *val = dev->dev_settings.samplerate;
    return status;
}


esp_err_t max31_set_spo2_samplerate(MAX31_h dev, uint8_t *val) {
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
        dev->dev_settings.samplerate = *val;
    }

    return status;
}


esp_err_t max31_get_ledpwm(MAX31_h dev, uint8_t *val) {
    esp_err_t status = ESP_OK;
    *val = dev->dev_settings.ledpwm;
    return status;
}


esp_err_t max31_set_ledpwm(MAX31_h dev, uint8_t *val) {
    esp_err_t status = ESP_OK;
    uint8_t regval = 0;
    uint8_t pwm = *val;
    if(pwm > MAX31_LED_PWM_411) {
        status = ESP_ERR_INVALID_ARG;
    } else {
        status = gcd_i2c_read_mod_write(dev->i2c_bus, dev->dev_addr, MAX31_REGADDR_SP02_CONFIG, pwm, 0b11);
    }

    if(status == ESP_OK) {
        dev->dev_settings.ledpwm = pwm;
    }

    return status;
}


esp_err_t max31_get_redledamplitude(MAX31_h dev, uint8_t *val) {
    esp_err_t status = ESP_OK;
    *val = dev->dev_settings.ledRed_ampl;
    return status;
}


esp_err_t max31_set_redledamplitude(MAX31_h dev, uint8_t *val) {
    esp_err_t status = ESP_OK;
    uint8_t regval = 0;

    regval = *val;
    status = gcd_i2c_write_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_LED1PULSE_AMP, 1, &regval);

    if(status == ESP_OK) {
        dev->red_lvl = *val;
    }

    return status;
}


esp_err_t max31_get_irledamplitude(MAX31_h dev, uint8_t *val) {
    esp_err_t status = ESP_OK;
    *val = dev->dev_settings.ledIR_ampl;
    return status;
}


esp_err_t max31_set_irledamplitude(MAX31_h dev, uint8_t *val) {
    esp_err_t status = ESP_OK;
    uint8_t regval = 0;

    regval = *val;
    status = gcd_i2c_write_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_LED2PULSE_AMP, 1, &regval);

    if(status == ESP_OK) {
        dev->red_lvl = *val;
    }

    return status;
}


esp_err_t max31_get_fifo_ovr(MAX31_h dev, uint8_t *val) {
    esp_err_t err = ESP_OK;
    uint8_t ovr = 0;
    err = gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_OVRFLW_CNTR, 1, &ovr);
    if(!err) {
        *val = ovr;
    }
    return err;
}


esp_err_t max31_get_temperature(MAX31_h dev, float *val) {
    *val = dev->temperature;
    return ESP_OK;
}


/** ACTIONS **/

esp_err_t max31_read_temperature(MAX31_h dev) {

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


esp_err_t max31_read_fifo(MAX31_h dev) {

    esp_err_t status = ESP_OK;

    uint8_t regvals[3] = {0};
    uint8_t rd_ptr_val = 0;
    uint8_t wr_ptr_val = 0;
    uint8_t avail_samples = 0;
    uint8_t avail_bytes = 0;
    uint8_t fifo_width = 3;

    /** point at the avail fifo mem **/
    uint8_t *ptr = dev->fifo_buffer;

    if(dev->fifo_settings.use_fifo == false) {
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
            if(dev->dev_settings.device_mode == MAX31_MODE_SPO2_RED_IR) {
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
            ESP_LOGI(MX_TAG, "Reading %u bytes from FIFO...", avail_bytes);
            memset(dev->fifo_buffer, 0, MAX31_FIFO_MAX_SIZE);
            dev->bytes_read = 0;
            /** read the data from fifo **/
            status = gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_FIFO_DATA, avail_bytes, dev->fifo_buffer);

            if(status) {
                ESP_LOGE(MX_TAG, "Error reading %u bytes from fifo!", avail_bytes);
            }
            else {
                dev->bytes_read = avail_bytes;
            }
        }
    }

    return status;
}


esp_err_t max31_reset_device(MAX31_h dev) {
    esp_err_t status = ESP_OK;
    uint8_t regval = 0;
    uint8_t  write = (1 << 6);

    status = gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_MODE_CONFIG, 1, &regval);
    regval |= write;
    status = gcd_i2c_write_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_MODE_CONFIG, 1, &regval);

    return status;
}


esp_err_t max31_enable_temperature_sensor(MAX31_h dev) {
    esp_err_t status = ESP_OK;
    uint8_t  write = 1;

    status = gcd_i2c_write_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_DIETEMP_SAMPLE, 1, &write);
    dev->temp_sampling = true;
    return status;
}


esp_err_t max31_read_fifo_on_almostfull(MAX31_h dev, bool *en) {
    dev->read_fifo_on_almostfull = *en;
    return ESP_OK;
}

#ifdef CONFIG_ENABLE_MAX31_EVENTS

esp_err_t max31_set_event_mask(MAX31_h dev, uint8_t *mask) {
    dev->event_mask = mask;
    return ESP_OK;
}

esp_err_t max31_get_event_mask(MAX31_h dev, uint8_t *mask) {
    *mask = dev->event_mask;
    return ESP_OK;
}


#endif /** CONFIG_ENABLE_MAX31_EVENTS **/