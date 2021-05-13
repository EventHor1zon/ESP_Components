/***************************************
* \file     HMC5883_Driver.c
* \brief    Driver for the HMC5883L Magneto-meter! 
*           Follows standard pattern of other drivers
*           - task
*           - gets/sets
*           - modes, etc
*           Hahaha little gotcha... Couldn't understand why the chip ID wasn't matching - 
*               went searching & found there's 2 different types of HMC5883L 
*               A honeywell version & a QST version - This driver is for the QST Version
* \date     March 2021
* \author   RJAM
****************************************/

/********* Includes *******************/
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_types.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "genericCommsDriver.h"
#include "HMC5883_Driver.h"


const static int8_t HMC_CONFIG_INTR_LEVEL = ESP_INTR_FLAG_LEVEL3;

const char *HMC_TAG = "HMC5883";



/****** Function Prototypes ***********/



static void hmc_driver_task(void *args);



/************ ISR *********************/


void data_ready_isr(void *args) {

    HMC_DEV dev = args;
    BaseType_t higherPrioWoken = pdFALSE;

    /** notify the task **/
    vTaskNotifyGiveFromISR(dev->t_handle, &higherPrioWoken);
    
}


/****** Private Data ******************/

/****** Private Functions *************/

static void hmc_driver_task(void *args) {

    HMC_DEV dev = args;


    while(1) {

        if(dev->isr_en && dev->isr_pin_configured) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            hmc_update_measurements(dev);
        }

        else {
            hmc_update_measurements(dev);
            ESP_LOGI(HMC_TAG, "[>] X: %0.2f\t[%d]\tY: %0.2f\t[%d]\tZ: %0.2f\t[%d]",
                     dev->results.x_val, dev->results.x_raw, 
                     dev->results.y_val, dev->results.y_raw, 
                     dev->results.z_val, dev->results.z_raw
            ); 

            vTaskDelay(pdMS_TO_TICKS(200));

        }
    }
    /** here be dragons **/
}

/****** Global Data *******************/

/****** Global Functions *************/


HMC_DEV hmc_init(hmc_init_t *ini) {

    esp_err_t err = ESP_OK;
    HMC_DEV dev = NULL;
    TaskHandle_t t_handle = NULL;

    /** EC the i2c bus **/
    if(!gcd_i2c_check_bus(ini->i2c_bus)) {
        ESP_LOGE(HMC_TAG, "Error: Invalid i2c Bus");
        err = ESP_ERR_INVALID_ARG;
    }
    /** EC/Config the GPIO pin **/
    if(!err) {
        if(ini->drdy_pin > 0) {
            gpio_config_t conf = {0};
            conf.intr_type = GPIO_INTR_NEGEDGE;
            conf.mode = GPIO_MODE_INPUT;
            conf.pin_bit_mask = (1 << ini->drdy_pin);
            err = gpio_config(&conf);

            if(err != ESP_OK) {
               ESP_LOGE(HMC_TAG, "Error: in configuring DRDY pin [%u]", err);
            }
            else if(gpio_isr_handler_add(ini->drdy_pin, (gpio_isr_t)data_ready_isr, dev) == ESP_ERR_INVALID_STATE) {
                gpio_install_isr_service(HMC_CONFIG_INTR_LEVEL);
            }
        } 
    }
    /** create driver handle **/
    if(!err) {
        dev = heap_caps_calloc(1, sizeof(hmc_driver_t), MALLOC_CAP_DEFAULT);
        if(dev == NULL) {
            err= ESP_ERR_NO_MEM;
            ESP_LOGE(HMC_TAG, "Error: Could not create driver handle!");
        }
        else {
            /** set default values in the handle **/
            dev->drdy_pin = ini->drdy_pin;
            dev->i2c_bus = ini->i2c_bus;
            dev->i2c_address = HMC_I2C_ADDR;
            dev->mode = HMC_MODE_STANDBY;
        }
    }
    /** create the driver task ***/
    if(!err && xTaskCreate(hmc_driver_task, "hmc_driver_task", configMINIMAL_STACK_SIZE, dev, 3, &t_handle) != pdTRUE) {
        ESP_LOGE(HMC_TAG, "Error: Creating driver task!");
        err = ESP_ERR_NO_MEM;
    } 
    else {
        dev->t_handle = t_handle;
    }

    /** if setup fails and heap has been allocated, free now **/
    if(err){
        ESP_LOGE(HMC_TAG, "Error: Failed to initialise HMC device! [%u]", err);
        if(dev != NULL ){
            heap_caps_free(dev);
        }
    }
    else {
        ESP_LOGI(HMC_TAG, "Succesfully started the HMC Driver!");
    }    

    return dev;

}


esp_err_t hmc_get_mode(HMC_DEV dev, uint8_t *val) {

    esp_err_t err = ESP_OK;
    *val = dev->mode;
    return err;
}

esp_err_t hmc_set_mode(HMC_DEV dev, uint8_t *val) {
    
    esp_err_t err = ESP_OK;
    uint8_t byte = *val;
    uint8_t regval = 0;
    if(byte > 1) {
        ESP_LOGE(HMC_TAG, "Invalid mode");
        err = ESP_ERR_INVALID_ARG;
    }
    else {
        err = gcd_i2c_read_address(dev->i2c_bus, dev->i2c_address, HMC_REGADDR_CTRL_A, 1, &regval);
        if(!err) {
            regval &= ~(0b11); // clear the lowest 2 bits
            regval |= byte;
            err = gcd_i2c_write_address(dev->i2c_bus, dev->i2c_address, HMC_REGADDR_CTRL_A, 1, &regval);
        }
    }
    if(!err) {
        dev->mode = byte;
    }
    return err;
}


esp_err_t hmc_get_scale(HMC_DEV dev, uint8_t *val) {
   esp_err_t status = ESP_OK;
   *val = dev->scale;
   return status;
}


esp_err_t hmc_set_scale(HMC_DEV dev, uint8_t *val) {
   esp_err_t err = ESP_OK;
   
    uint8_t regval = 0;
    uint8_t byte = *val;
    if(byte > 1) {
        ESP_LOGE(HMC_TAG, "Invalid scale value");
        err = ESP_ERR_INVALID_ARG;
    }
    else {
        err = gcd_i2c_read_address(dev->i2c_bus, dev->i2c_address, HMC_REGADDR_CTRL_A, 1, &regval);
        if(!err) {
            regval &= ~(0b110000); // clear the lowest 2 bits
            regval |= (byte << 4);
            err = gcd_i2c_write_address(dev->i2c_bus, dev->i2c_address, HMC_REGADDR_CTRL_A, 1, &regval);
        }
    }
    
    if(!err) {
        dev->scale = byte;
    }

   return err;
}


esp_err_t hmc_get_data_rate(HMC_DEV dev, uint8_t *val) {
   esp_err_t status = ESP_OK;
   *val = dev->d_rate;
   return status;
}


esp_err_t hmc_set_data_rate(HMC_DEV dev, uint8_t *val) {
    esp_err_t err = ESP_OK;
    
    uint8_t regval = 0;
    uint8_t byte = *val;
    if(byte > HMC_DOR_200HZ) {
        ESP_LOGE(HMC_TAG, "Invalid scale value");
        err = ESP_ERR_INVALID_ARG;
    }
    else {
        err = gcd_i2c_read_address(dev->i2c_bus, dev->i2c_address, HMC_REGADDR_CTRL_A, 1, &regval);
        if(!err) {
            regval &= ~(0b1100); // clear the lowest 2 bits
            regval |= (byte << 2);
            err = gcd_i2c_write_address(dev->i2c_bus, dev->i2c_address, HMC_REGADDR_CTRL_A, 1, &regval);
        }
    }
    if(!err) {
        dev->d_rate = byte;
    }

   return err;
}


esp_err_t hmc_get_oversample_ratio(HMC_DEV dev, uint8_t *val) {
   esp_err_t status = ESP_OK;
   *val = dev->d_rate;
   return status;
}


esp_err_t hmc_set_oversample_register(HMC_DEV dev, uint8_t *val) {
   esp_err_t err = ESP_OK;
   
   uint8_t regval = 0;
   uint8_t byte = *val;
   if(byte > HMC_DOR_200HZ) {
       ESP_LOGE(HMC_TAG, "Invalid scale value");
       err = ESP_ERR_INVALID_ARG;
   }
   else {
        err = gcd_i2c_read_address(dev->i2c_bus, dev->i2c_address, HMC_REGADDR_CTRL_A, 1, &regval);
        if(!err) {
            regval &= ~(0b11000000); // clear the lowest 2 bits
            regval |= (byte << 6);
            err = gcd_i2c_write_address(dev->i2c_bus, dev->i2c_address, HMC_REGADDR_CTRL_A, 1, &regval);
        }

   }
    if(!err) {
        dev->osr = byte;
    }

   return err;
}



esp_err_t hmc_get_interupt_enabled(HMC_DEV dev, uint8_t *val) {
   esp_err_t status = ESP_OK;
   *val = dev->isr_en;
   return status;
}


esp_err_t hmc_set_interrupt_enabled(HMC_DEV dev, bool *val) {
    esp_err_t err = ESP_OK;
    
    uint8_t regval = 0; 
    bool en = *val;
    err = gcd_i2c_read_address(dev->i2c_bus, dev->i2c_address, HMC_REGADDR_CTRL_B, 1, &regval);
    if(!err) {
        if(en && (regval & HMC_INTR_EN_MASK) == 0) {
            regval |= HMC_INTR_EN_MASK;
            err = gcd_i2c_write_address(dev->i2c_bus, dev->i2c_address, HMC_REGADDR_CTRL_B, 1, &regval);        
        }
        else if(!en && (regval & HMC_INTR_EN_MASK)) {
            regval &= ~(0b1);
            err = gcd_i2c_write_address(dev->i2c_bus, dev->i2c_address, HMC_REGADDR_CTRL_B, 1, &regval);        
        }
    }
    if(!err) {
        dev->isr_en = en;
    }

   return err;
}

esp_err_t hmc_get_x_gaus(HMC_DEV dev, float *val) {
   esp_err_t status = ESP_OK;
   *val = dev->results.x_val;
   return status;
}

esp_err_t hmc_get_y_gaus(HMC_DEV dev, float *val) {
   esp_err_t status = ESP_OK;
   *val = dev->results.y_val;
   return status;
}

esp_err_t hmc_get_z_gaus(HMC_DEV dev, float *val) {
   esp_err_t status = ESP_OK;
   *val = dev->results.z_val;
   return status;
}

esp_err_t hmc_get_x_raw(HMC_DEV dev, int16_t *val) {
   esp_err_t status = ESP_OK;
   *val = dev->results.x_raw;
   return status;
}

esp_err_t hmc_get_y_raw(HMC_DEV dev, int16_t *val) {
   esp_err_t status = ESP_OK;
   *val = dev->results.y_raw;
   return status;
}

esp_err_t hmc_get_z_raw(HMC_DEV dev, int16_t *val) {
   esp_err_t status = ESP_OK;
   *val = dev->results.z_raw;
   return status;
}


esp_err_t hmc_read_measurements(HMC_DEV dev) {

    esp_err_t err = ESP_OK;
    float mod = 0.0;
    uint8_t buffer[6] = {0};

    err = gcd_i2c_read_address(dev->i2c_bus, dev->i2c_address, HMC_REGADDR_XDATA_LSB, 6, buffer);

    if(!err) {
        dev->results.x_raw = ((int16_t )(buffer[1] << 8) || ((int16_t ) buffer[0]));
        dev->results.y_raw = ((int16_t )(buffer[3] << 8) || ((int16_t ) buffer[2]));
        dev->results.z_raw = ((int16_t )(buffer[5] << 8) || ((int16_t ) buffer[4]));
    
        if(dev->scale) {
            mod = 2 / HMC_MAX_VALUE;
        }
        else {
            mod = 8 / HMC_MAX_VALUE;
        }
        dev->results.x_val = mod * (float)dev->results.x_raw;
        dev->results.y_val = mod * (float)dev->results.y_raw;
        dev->results.z_val = mod * (float)dev->results.z_raw;
    }

    return err;
}