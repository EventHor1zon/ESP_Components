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

#include "include/Max30102_Driver.h"
#include "include/genericCommsDriver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "Utilities.h"

/****** Function Prototypes ***********/

void max31_interrupt_handler(void *args);

/****** Global Data *******************/


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
    uint8_t clear_val = 0;

    status = gcd_i2c_write_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_FIFO_WRTPTR, 3, &clear_val);

    return status;
}


static esp_err_t max31_read_temperature(max31_driver_t *dev, float *val) {

    esp_err_t status = ESP_OK;
    uint8_t temp[2];
    int8_t int_temp;
    float fract_temp;
    status = gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_DIETEMP_INT, 2, temp);

    int_temp = temp[0];
    fract_temp = (float)temp[1] * 0.0625;
    *val = (float)int_temp + fract_temp;

     return status;
}


static esp_err_t test_mode(max31_driver_t *dev) {

    uint8_t val = 0;
    ESP_LOGI(MX_TAG, "rst dev");
    max31_reset_device(dev);
    esp_err_t status; // = max31_set_shutdown(dev, &val);
    // vTaskDelay(pdMS_TO_TICKS(100));
    status = max31_set_sample_average(dev, &val);
    val = 1;
    ESP_LOGI(MX_TAG, "rollover [%u]", status);

    max31_set_fifo_rollover(dev, &val);
    val = 0x0f;
    ESP_LOGI(MX_TAG, "Setting the almost full value [%u]", status);
    status =max31_set_almost_full_val(dev, &val);
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(MX_TAG, "Setting the samples per sec [%u]", status);
    val = MAX31_SAMPLERATE_100;
    status = max31_set_spo2_samplerate(dev, &val);
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(MX_TAG, "Setting the led PW [%u]", status);
    val = MAX31_LED_PWM_411;
    status = max31_set_ledpwm(dev, &val);
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(MX_TAG, "Setting the led amplitude RED [%u]", status);
    val = 0x7F;
    status = max31_set_redledamplitude(dev, &val);
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(MX_TAG, "Setting the led amp IR [%u]", status);
    val = 0x7f;
    status = max31_set_irledamplitude(dev, &val);
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(MX_TAG, "Setting the mode [%u]", status);
    val = MAX31_MODE_SPO2_RED_IR;
    ESP_LOGI(MX_TAG, "Mode: %u", val);
    status = max31_set_mode(dev, &val);
    vTaskDelay(pdMS_TO_TICKS(100));

    max31_reset_fifo(dev);

    ESP_LOGI(MX_TAG, "starting device");
    val = 0;
    status = max31_set_shutdown(dev, &val);
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(MX_TAG, "Setting the interrupt function to A_FULL_EN");
    val = (MAX31_INTR_TYPE_ALMFULL);
    status = max31_set_interrupt1(dev, &val);
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



static void max31_task(void *args) {

    max31_driver_t *dev = (max31_driver_t *)args;
    uint8_t val = 0;
    while(1) {
        if(dev->configured != true) {
            vTaskDelay(1000); 
        } 
        else {

            ESP_LOGI(MX_TAG, "In task");
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY); 
            ESP_LOGI(MX_TAG, "got notify");
            gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, MAX31_REGADDR_INTR_STATUS1, 1, &val);
            ESP_LOGI(MX_TAG, "r 0 [%02x]", val); 
            max31_read_fifo(dev);
        // showmem(dev->fifo_buffer, 200);
        }
    }
}



/****** Global Functions *************/


max31_driver_t *max31_init(max31_initdata_t *init) {

    esp_err_t istatus = ESP_OK;

    max31_driver_t *handle = (max31_driver_t *)heap_caps_calloc(1, sizeof(max31_driver_t), MALLOC_CAP_8BIT);
    uint8_t *fifo_mem = (uint8_t *) heap_caps_calloc(1, MAX31_FIFO_MAX_SIZE, MALLOC_CAP_8BIT);
    if(handle == NULL || fifo_mem == NULL){
        ESP_LOGE(MX_TAG, "Error initialising driver memory!");
        istatus = ESP_ERR_NO_MEM;
    }
    else {

        handle->fifo_buffer= fifo_mem;
        handle->dev_addr = (uint8_t )MAX31_SLAVE_ADDR;
        /** set the device default settings **/
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
            heap_caps_free(handle);
            handle = NULL;
        } else {
            ESP_LOGI(MX_TAG, "Max30102 driver started succesfully!");
            test_mode(handle);
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

esp_err_t max31_set_interrupt1(max31_driver_t *dev, uint8_t *intr_mask) {

    esp_err_t status = ESP_OK;
    uint8_t regval= 0;
    uint8_t v = *intr_mask;
    /** idiotproof **/
    if(v & 0b00011111) {
        status = ESP_ERR_INVALID_ARG;
    } else {
        /** get the existing interrupts **/
        status = gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_INTR_EN1, 1, &regval);
        ESP_LOGI(MX_TAG, "Intr1: 0x%02x, writing 0x%02x", regval, *intr_mask);
        status = gcd_i2c_write_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_INTR_EN1, 1, &v);
    }

    if(status == ESP_OK) {
        dev->intr1_mask = *intr_mask;
    }
    return status;
}


esp_err_t max31_set_interrupt2(max31_driver_t *dev, uint8_t *intr_mask) {

    esp_err_t status = ESP_OK;
    uint8_t regval= 0;
    uint8_t val = *intr_mask;
    /** idiotproof? **/
    if(val & 0b11111101) {
        status = ESP_ERR_INVALID_ARG;
    } else {
        /** get the existing interrupts **/
        status = gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_INTR_EN2, 1, &regval);
        regval |= val;
        status = gcd_i2c_write_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_INTR_EN2, 1, &regval);
    }

    if(status == ESP_OK) {
        dev->intr2_mask = val;
    }
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


esp_err_t max31_set_shutdown(max31_driver_t *dev, uint8_t *val) {
    esp_err_t status = ESP_OK;
    uint8_t regval = 0;
    uint8_t write = 0;
    uint8_t v = *val;

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


/** ACTIONS **/

esp_err_t max31_read_fifo(max31_driver_t *dev) {

    esp_err_t status = ESP_OK;

    uint8_t regvals[3];
    uint8_t rd_ptr_val = 0;
    uint8_t wr_ptr_val = 0;
    uint8_t avail_bytes = 0;
    uint8_t data = 0;

    /** point at the avail fifo mem **/
    uint8_t *ptr = dev->fifo_buffer + dev->bytes_in_buffer;

    if(dev->use_fifo == false) {
        status = ESP_ERR_INVALID_STATE;
    } else {
        status = gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_FIFO_WRTPTR, 1, &regvals[0]);
        status = gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_FIFO_RDPTR, 1, &regvals[2]);

        /** get number of bytes **/
        if(status == ESP_OK) {
            rd_ptr_val = (regvals[2] & 0b00011111);
            wr_ptr_val = (regvals[0] & 0b00011111);
            avail_bytes = wr_ptr_val > rd_ptr_val ? (wr_ptr_val - rd_ptr_val) : (MAX31_FIFO_SAMPLES - rd_ptr_val) + wr_ptr_val;
            ESP_LOGI(MX_TAG, "Write_ptr: %u Rd_Ptr: %u Avail: %u", regvals[0], regvals[2], avail_bytes);
        }
        if(avail_bytes) {
            for(uint8_t i=0; i<avail_bytes; i++) {
                status = gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_FIFO_DATA, 1, &data);
                status = gcd_i2c_read_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_FIFO_RDPTR, 1, &regvals[2]);        
                ESP_LOGI(MX_TAG, "Got %02x; rd_ptr %02x", data, regvals[2]);
                //dev->bytes_in_buffer++;
                if(dev->bytes_in_buffer > 192) {
                    /** reset tp start of fifo **/
                    /** TODO: SEND EVENT FOR FULL FIFO **/
                    dev->bytes_in_buffer = 0;
                    ptr = dev->fifo_buffer;
                }
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


esp_err_t max31_sample_temp(max31_driver_t *dev) {
    esp_err_t status = ESP_OK;
    uint8_t  write = 1;

    status = gcd_i2c_write_address(dev->i2c_bus, dev->dev_addr, (uint8_t )MAX31_REGADDR_DIETEMP_SAMPLE, 1, &write);
    dev->temp_sampling = true;
    return status;
}

