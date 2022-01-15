/***************************************
* \file     ADPS9960_Driver.c
* \brief    Driver for the RGB, Gesture & proximity detector
*           Going to cover most of the device's features
*           TBH not going to do the offsets unless needed.
*           (Aint nobody got time for that)
*           https://ww1.microchip.com/downloads/en/DeviceDoc/21419D.pdf
*
* \date     March 2021
* \author   RJAM
****************************************/

/********* Includes *******************/

#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "Utilities.h"
#include "genericCommsDriver.h"
#include "APDS9960_Driver.h"


#ifdef CONFIG_USE_PERIPH_MANAGER
const peripheral_t apds_parameter_map[apds_param_len] {
    { "Power status", 1, &apds_get_pwr_on_status, &apds_set_pwr_on_status, NULL, PARAMTYPE_BOOL, 1, (GET_FLAG | SET_FLAG)},
    { "Proximity status", 2, &apds_get_proximity_status, &apds_set_proximity_status, NULL, PARAMTYPE_BOOL, 1, (GET_FLAG | SET_FLAG)},
    { "RGB status", 3, &apds_get_als_status, &apds_set_als_status, NULL, PARAMTYPE_BOOL, 1, (GET_FLAG | SET_FLAG)},
    { "Gesture status", 4, &apds_get_gesture_status, &apds_set_gesture_status, NULL, PARAMTYPE_BOOL, 1, (GET_FLAG | SET_FLAG)},
    { "RGB ADC time", 5, &apds_get_adc_time, &apds_set_adc_time, NULL, PARAMTYPE_UINT8, 255, (GET_FLAG | SET_FLAG)},
    { "Wait time", 6, &apds_get_wait_time, &apds_set_wait_time, NULL, PARAMTYPE_UINT8, 255, (GET_FLAG | SET_FLAG)},
    { "RGB Low thresh", 7, &apds_get_alsintr_low_thr, &apds_set_alsintr_low_thr, NULL, PARAMTYPE_UINT16, 0xffff, (GET_FLAG | SET_FLAG)},
    { "RGB High thresh", 8, &apds_get_alsintr_hi_thr, &apds_set_alsintr_hi_thr, NULL, PARAMTYPE_UINT16, 0xffff, (GET_FLAG | SET_FLAG)},
    { "Prox Low thresh", 9, &apds_get_prxintr_low_thr, &apds_set_prxintr_low_thr, NULL, PARAMTYPE_UINT8, 0xff, (GET_FLAG | SET_FLAG)},
    { "Prox High thresh", 10, &apds_get_prxintr_high_thr, &apds_set_prxintr_high_thr, NULL, PARAMTYPE_UINT8, 0xff, (GET_FLAG | SET_FLAG)},
    { "Long Wait", 11, &apds_get_longwait_en, &apds_set_longwait_en, NULL, PARAMTYPE_BOOL, 1, (GET_FLAG | SET_FLAG)},
    { "Prox Intr perist", 12, &apds_get_prx_intr_persistence, &apds_set_prx_intr_persistence, NULL, PARAMTYPE_UINT8, 16, (GET_FLAG | SET_FLAG)},
    { "RGB Intr perist", 13, &apds_get_als_intr_persistence, &apds_set_als_intr_persistence, NULL, PARAMTYPE_UINT8, 16, (GET_FLAG | SET_FLAG)},
    { "Prox Pulse time", 14, &apds_get_prx_ledpulse_t, &apds_set_prx_ledpulse_t, NULL, PARAMTYPE_UINT8, 3, (GET_FLAG | SET_FLAG)}, 
    { "Prox Pulse count", 15, &apds_get_prx_pulses, &apds_set_prx_pulses, NULL, PARAMTYPE_UINT8, 64, (GET_FLAG | SET_FLAG)}, 
    { "Led Drive str", 16, &apds_get_led_drive_strength, &apds_set_led_drive_strength, NULL, PARAMTYPE_UINT8, 3, (GET_FLAG | SET_FLAG)}, 
    { "Prox Gain", 17, &apds_get_prx_gain, &apds_set_prx_gain, NULL, PARAMTYPE_UINT8, 4, (GET_FLAG | SET_FLAG)}, 
    { "RGB Gain", 18, &apds_get_als_gain, &apds_set_als_gain, NULL, PARAMTYPE_UINT8, 4, (GET_FLAG | SET_FLAG)}, 

};

const peripheral_t apds_periph_template = {
    .handle = NULL,
    .param_len = apds_param_len,
    .params = apds_parameter_map,
    .peripheral_name = "APDS9960",
    .peripheral_id = 0
};
#endif


const char *APDS_TAG = "APDS Driver";

/****** Function Prototypes ***********/

static IRAM_ATTR void apds_intr_handler(void *args);

static esp_err_t regSetMask(APDS_DEV dev, uint8_t regaddr, uint8_t mask);

static esp_err_t regUnsetMask(APDS_DEV dev, uint8_t regaddr, uint8_t mask);

static uint8_t get_interrupt_source_and_clear(APDS_DEV dev);

static bool is_als_valid(APDS_DEV dev);

static bool is_prx_valid(APDS_DEV dev);

/************ ISR *********************/


IRAM_ATTR void apds_intr_handler(void *args) {
    APDS_DEV dev = (adps_handle_t *)args;
    BaseType_t higherPrio = pdFALSE;
    xTaskNotifyFromISR(dev->t_handle, 0, eIncrement, &higherPrio);
    portYIELD_FROM_ISR();
}



/****** Private Data ******************/

/****** Private Functions *************/

/** Sets the BITS in mask **/
static esp_err_t regSetMask(APDS_DEV dev, uint8_t regaddr, uint8_t mask) {

    esp_err_t err = ESP_OK;
    uint8_t regval = 0; 

    err = gcd_i2c_read_address(dev->bus, dev->addr, regaddr, 1, &regval);

    if(!err) {
        // check if mask is already set
        if((regval & mask) != mask) {
            regval |= mask;
            err = gcd_i2c_write_address(dev->bus, dev->addr, regaddr, 1, &regval);
        }
    }

    return err;
}


static esp_err_t regUnsetMask(APDS_DEV dev, uint8_t regaddr, uint8_t mask) {

    esp_err_t err = ESP_OK;
    uint8_t regval = 0; 

    err = gcd_i2c_read_address(dev->bus, dev->addr, regaddr, 1, &regval);

    if(!err) {
        regval &= ~(mask);
        err = gcd_i2c_write_address(dev->bus, dev->addr, regaddr, 1, &regval);
    }

    return err;   
}


static uint8_t get_interrupt_source_and_clear(APDS_DEV dev) {

    esp_err_t err = ESP_OK;
    uint8_t regval = 0;
    uint8_t retval = 0;
    uint8_t clear = 1;
    err = gcd_i2c_read_address(dev->bus, dev->addr, APDS_REGADDR_STATUS, 1, &regval);
    if(!err) {
        if(regval & APDS_REGBIT_PRX_INT) {
            retval |= APDS_REGBIT_PRX_INT;
            gcd_i2c_write_address(dev->bus, dev->addr, APDS_REGADDR_PRX_ISR_CLR, 1, &clear);
        }
        if(regval & APDS_REGBIT_ALS_INT) {
            retval |= APDS_REGBIT_ALS_INT;
            gcd_i2c_write_address(dev->bus, dev->addr, APDS_REGADDR_ALS_ISR_CLR, 1, &clear);

        }
        if(regval & APDS_REGBIT_GST_INT) {
            retval |= APDS_REGBIT_GST_INT;
        }
    }

    return retval;
}


static bool is_als_valid(APDS_DEV dev) {
    uint8_t val = 0;
    gcd_i2c_read_address(dev->bus, dev->addr, APDS_REGADDR_STATUS, 1, &val);
    return ((val & APDS_REGBIT_ALS_VALID) ? true : false);
}


static bool is_prx_valid(APDS_DEV dev) {
    uint8_t val = 0;
    gcd_i2c_read_address(dev->bus, dev->addr, APDS_REGADDR_STATUS, 1, &val);
    return ((val & APDS_REGBIT_PRX_VALID) ? true : false);
}


static esp_err_t apds_initialise_device(APDS_DEV dev) {

    uint8_t pwr_on = 1;
    uint8_t id = 0;

    esp_err_t err = gcd_i2c_write_address(dev->bus, dev->addr, APDS_REGADDR_ENABLE, 1, &pwr_on);

    if(!err) {
        err = gcd_i2c_read_address(dev->bus, dev->addr, APDS_REGADDR_ID, 1, &id);
        if(!err) {
            ESP_LOGI(APDS_TAG, "Got ID: 0x%02x", id);
        }
    }

    /** set some default values - looks like rest are 0 default? **/

    dev->als_settings.adc_intg_time = 0xff;
    dev->als_settings.wait_time = 0xff;
    return err;
}


static esp_err_t apds_config_intr_pin(APDS_DEV dev) {

    gpio_config_t conf = {0};

    conf.pin_bit_mask = (1 << dev->intr_pin);
    conf.intr_type = GPIO_INTR_NEGEDGE;
    conf.mode = GPIO_MODE_INPUT;
    conf.pull_up_en = GPIO_PULLUP_ENABLE;
    conf.pull_down_en = 0;
 
    esp_err_t err = gpio_config(&conf);
    if(err) {
        ESP_LOGE(APDS_TAG, "Error configuring GPIO pin!");
    }
    else {
        err = gpio_isr_handler_add(dev->intr_pin, (gpio_isr_t)apds_intr_handler, dev);
    }
    return err;
}


static esp_err_t apds_gst_clr_fifo(APDS_DEV dev) {
    esp_err_t err = regSetMask(dev, APDS_REGADDR_GST_CONFIG_3, APDS_REGBIT_GST_FIFO_CLR);
    return err;
}


static esp_err_t testmode(APDS_DEV dev) {

    /** set the prx en & als en **/

    uint8_t byte = 0; 

    uint8_t  prx = 0;
    uint8_t crgbraw[8] = {0};



    /** set the wait time to ~100ms **/
    byte = 200;
    ESP_ERROR_CHECK(gcd_i2c_write_address(dev->bus, dev->addr, APDS_REGADDR_WAITTIME, 1, &byte));
    
    /** set the prox led drive strength **/

    byte = (APDS_GST_LEDDRIVE_50MA << APDS_REG_OFFSET_LDRIVE);
    ESP_ERROR_CHECK(gcd_i2c_write_address(dev->bus, dev->addr, APDS_REGADDR_GAIN_CTRL, 1, &byte));

    byte = 220;
    ESP_ERROR_CHECK(gcd_i2c_write_address(dev->bus, dev->addr, APDS_REGADDR_ADCTIME, 1, &byte));

    byte = (APDS_REGBIT_ALS_EN | APDS_REGBIT_PRX_EN | APDS_REGBIT_PWR_ON);
    ESP_ERROR_CHECK(gcd_i2c_write_address(dev->bus, dev->addr, APDS_REGADDR_ENABLE, 1, &byte));

    for(uint16_t i=0; i < 200; i++) {

        ESP_ERROR_CHECK(gcd_i2c_read_address(dev->bus, dev->addr, APDS_REGADDR_PROX_DATA, 1, &prx));
        ESP_ERROR_CHECK(gcd_i2c_read_address(dev->bus, dev->addr, APDS_REGADDR_CLRCHAN_DATA_LSB, 8, crgbraw));
        uint16_t c = (crgbraw[0] | crgbraw[1] << 8);
        uint16_t r = (crgbraw[2] | crgbraw[3] << 8);
        uint16_t g = (crgbraw[4] | crgbraw[5] << 8);
        uint16_t b = (crgbraw[6] | crgbraw[7] << 8);

        ESP_LOGI(APDS_TAG, "Results: PRX: %u  C: %u R: %u G: %u B: %u ", prx, c, r, g, b);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    return ESP_OK;
}


static uint8_t get_gst_fifo_pkts(APDS_DEV dev) {
    uint8_t val = 0;
    gcd_i2c_read_address(dev->bus, dev->addr, APDS_REGADDR_GST_FIFO_LVL, 1, &val);
    return val;
}


static void apds_driver_task(void *args) {
 
    APDS_DEV dev = (adps_handle_t *)args;    
    uint32_t events = 0;

    while(1) {

        events = ulTaskNotifyTake(pdFALSE, portMAX_DELAY);


        if(events) {
            ESP_LOGI(APDS_TAG, "Received interrupt");        
        }

        /** get the isr source **/
        uint8_t interrupt_source = get_interrupt_source_and_clear(dev);
        ESP_LOGI(APDS_TAG, "Got interrupt value 0x%02x", interrupt_source);

        switch (interrupt_source)
        {
        case APDS_REGBIT_CP_SAT:
            break;
        case APDS_REGBIT_PRX_GST_SAT:
            break;
        case APDS_REGBIT_PRX_INT:
            break;
        case APDS_REGBIT_ALS_INT:
            break;
        case APDS_REGBIT_GST_INT:
            break;
        case APDS_REGBIT_PRX_VALID:
            break;
        case APDS_REGBIT_ALS_VALID:
            break;
        
        default:
            break;
        }


        vTaskDelay(pdMS_TO_TICKS(100));
    }
    /** here be dragons **/
}



/****** Global Data *******************/

/****** Global Functions *************/

APDS_DEV apds_init(apds_init_t *init) {

    esp_err_t err = ESP_OK;
    APDS_DEV dev = NULL;
    TaskHandle_t t_handle = NULL;

    if(!gcd_i2c_check_bus(init->i2c_bus)) {
        err = ESP_ERR_INVALID_ARG;
        ESP_LOGE(APDS_TAG, "Error - invalid i2c bus");
    }

    if(!err) {
        dev = heap_caps_calloc(1, sizeof(adps_handle_t), MALLOC_CAP_DEFAULT);
        if(dev == NULL) {
            ESP_LOGE(APDS_TAG, "Error: Error assigning heap mem [%u]", err);
            err = ESP_ERR_NO_MEM;
        }
        else {
            dev->addr = APDS_I2C_ADDRESS;
            dev->bus = init->i2c_bus;
        }
    }

    if(!err && apds_initialise_device(dev)) {
        ESP_LOGE(APDS_TAG, "Error: Error assigning heap mem [%u]", err);
        err = ESP_ERR_TIMEOUT;
    }

    if(init->intr_pin != 0) {
        dev->intr_pin = init->intr_pin;
        err = apds_config_intr_pin(dev);
    }

    if(!err && xTaskCreate(apds_driver_task, "apds_driver_task", 5012, dev, 3, &t_handle) != pdTRUE) {
        ESP_LOGE(APDS_TAG, "Error creating driver task");
        err = ESP_ERR_NO_MEM;
    }

    if(!err) {
        testmode(dev);
    }
    
    return dev;
}


 
esp_err_t apds_get_pwr_on_status(APDS_DEV dev, uint8_t *on) {
    *on = dev->gen_settings.pwr_on;
    return ESP_OK;
}


esp_err_t apds_set_pwr_on_status(APDS_DEV dev, uint8_t *on) {
    esp_err_t err = ESP_OK;
    uint8_t val = *on, regval = 0;

    if(val ) {
        err = regSetMask(dev, APDS_REGADDR_ENABLE, APDS_REGBIT_PWR_ON);
    }
    else {
        err = regUnsetMask(dev, APDS_REGADDR_ENABLE, APDS_REGBIT_PWR_ON);
    }
 
    if(!err) {
        dev->gen_settings.pwr_on = (val) ? 1 : 0;
    }
    return err;
}


esp_err_t apds_get_proximity_status(APDS_DEV dev, uint8_t *on) {
    *on = dev->prx_settings.prx_en;
    return ESP_OK;
}


esp_err_t apds_set_proximity_status(APDS_DEV dev, uint8_t *on) {
    esp_err_t err = ESP_OK;
    uint8_t val = *on;
    if(val ) {
        err = regSetMask(dev, APDS_REGADDR_ENABLE, APDS_REGBIT_PRX_EN);
    }
    else {
        err = regUnsetMask(dev, APDS_REGADDR_ENABLE, APDS_REGBIT_PRX_EN);
    }

    if(!err) {
        dev->prx_settings.prx_en = (val) ? 1 : 0;
    }
    return err;
}


esp_err_t apds_get_als_status(APDS_DEV dev, uint8_t *on) {
    *on = dev->als_settings.asl_en;
    return ESP_OK;
}


esp_err_t apds_set_als_status(APDS_DEV dev, uint8_t *on) {
    esp_err_t err = ESP_OK;
    uint8_t val = *on;
    if(val ) {
        err = regSetMask(dev, APDS_REGADDR_ENABLE, APDS_REGBIT_ALS_EN);
    }
    else {
        err = regUnsetMask(dev, APDS_REGADDR_ENABLE, APDS_REGBIT_ALS_EN);
    }

    if(!err) {
        dev->als_settings.asl_en = (val) ? 1 : 0;
    }
    return err;
}


esp_err_t apds_get_gesture_status(APDS_DEV dev, uint8_t *on) {
    *on = dev->gst_settings.gst_en;
    return ESP_OK;
}


esp_err_t apds_set_gesture_status(APDS_DEV dev, uint8_t *on) {
    uint8_t state = *on;
    uint8_t regval = 0;
    esp_err_t err = ESP_OK;

    if(state && dev->gst_settings.gst_en) {
        ESP_LOGI(APDS_TAG, "Gesture engine alreay enabled");
        return ESP_OK;
    }
    else if (!state && !(dev->gst_settings.gst_en)) {
        ESP_LOGI(APDS_TAG, "Gesture engine alreay disabled");
        return ESP_OK;
    }
    else {
        err = gcd_i2c_read_address(dev->bus, dev->addr, APDS_REGADDR_ENABLE, 1, &regval);
        if(state) {
            regval |= APDS_REGBIT_GST_EN;
        }
        else {
            regval &= ~APDS_REGBIT_GST_EN;
        }
        if(!err) {
            gcd_i2c_write_address(dev->bus, dev->addr, APDS_REGADDR_ENABLE, 1, &regval);
            if(!err) {
                dev->gst_settings.gst_en = (state ? 1 : 0);
            }
        }
    }

    return ESP_OK;
}


/**************** General Settings ***************/

esp_err_t apds_get_wait_time(APDS_DEV dev, uint8_t *wait){
    *wait = dev->als_settings.wait_time;
    return ESP_OK;
}


esp_err_t apds_set_wait_time(APDS_DEV dev, uint8_t *wait) {
    uint8_t byte = *wait;
    esp_err_t err = regSetMask(dev, APDS_REGADDR_WAITTIME, byte);
    return err;
}


esp_err_t apds_get_longwait_en(APDS_DEV dev, uint8_t *thr) {
    *thr = dev->als_settings.wait_long_en;
    return ESP_OK;
}


esp_err_t apds_set_longwait_en(APDS_DEV dev, uint8_t *thr) {
    uint8_t val = *thr;
    esp_err_t err = ESP_OK;
    if(val) {
        err = regSetMask(dev, APDS_REGADDR_CONFIG_1, APDS_REGBIT_WLON_EN);
    }
    else {
        err = regUnsetMask(dev, APDS_REGADDR_CONFIG_1, APDS_REGBIT_WLON_EN);
    }

    if(!err) {
        dev->als_settings.wait_long_en = val;
    }

    return ESP_OK;
}


esp_err_t apds_set_led_drive_strength(APDS_DEV dev, gst_led_drive_t *drive) {
    uint8_t d = *drive;
    esp_err_t err = ESP_OK;
    if(d > APDS_GST_LEDDRIVE_MAX) {
        err = ESP_ERR_INVALID_ARG;
        ESP_LOGE(APDS_TAG, "Invalid args");
    }
    else {
        d = (d << 6);
        err = regUnsetMask(dev, APDS_REGADDR_GAIN_CTRL, (3 << 6));
        if(!err && d) {
            err = regSetMask(dev, APDS_REGADDR_GAIN_CTRL, d);
        }
    }
    if(!err) {
        dev->gst_settings.gst_led_drive_str = d;
    }
    return err;
}


esp_err_t apds_get_led_drive_strength(APDS_DEV dev, gst_led_drive_t *drive) {
    *drive = dev->gst_settings.gst_led_drive_str;
    return ESP_OK;
}



/***************  ALS Settings ***********************/

esp_err_t apds_get_adc_time(APDS_DEV dev, uint8_t *adct) {
    *adct = dev->als_settings.adc_intg_time;
    return ESP_OK;
}


esp_err_t apds_set_adc_time(APDS_DEV dev, uint8_t *adct) {
    uint8_t byte = *adct;
    esp_err_t err = regSetMask(dev, APDS_REGADDR_ADCTIME, byte);
    return err;
}


esp_err_t apds_get_alsintr_low_thr(APDS_DEV dev, uint16_t *thr) {
    *thr = dev->als_settings.als_thresh_l;
    return ESP_OK;
}


esp_err_t apds_set_alsintr_low_thr(APDS_DEV dev, uint16_t *thr) {
    uint16_t val = *thr;
    uint8_t bytes[2] = {
        (uint8_t )val,
        ((uint8_t )val >> 8),
    };
    esp_err_t err = gcd_i2c_write_address(dev->bus, dev->addr, APDS_REGADDR_ALS_THR_LOW_LSB, 2, bytes);
    if(!err) {
        dev->als_settings.als_thresh_l = val;
    }
    return err;
}


esp_err_t apds_get_alsintr_hi_thr(APDS_DEV dev, uint16_t *thr) {
    *thr = dev->als_settings.als_thresh_h;
    return ESP_OK;
}


esp_err_t apds_set_alsintr_hi_thr(APDS_DEV dev, uint16_t *thr) {
    uint16_t val = *thr;
    uint8_t bytes[2] = {
        (uint8_t )val,
        ((uint8_t )val >> 8),
    };
    esp_err_t err = gcd_i2c_write_address(dev->bus, dev->addr, APDS_REGADDR_ALS_THR_HIGH_LSB, 2, bytes);
    if(!err) {
        dev->als_settings.als_thresh_h = val;
    }
    return err;
}


esp_err_t apds_get_als_intr_persistence(APDS_DEV dev, uint8_t *cnt) {
    *cnt = dev->als_settings.als_persist;
    return ESP_OK;
}


esp_err_t apds_set_als_intr_persistence(APDS_DEV dev, uint8_t *cnt) {
    esp_err_t err = ESP_OK;
    uint8_t val = *cnt;
    if(val > 15) {
        err= ESP_ERR_INVALID_ARG;
    }
    else {
        err = regUnsetMask(dev, APDS_REGADDR_ISR_PERSIST_FLTR, 0x0f);
        if(!err && val) {
            err =regSetMask(dev, APDS_REGADDR_ISR_PERSIST_FLTR, val);
        }
    }
    if(!err) {
        dev->als_settings.als_persist = val;
    }

    return err;
}


esp_err_t apds_get_als_gain(APDS_DEV dev, als_gain_t *g) {
    *g = dev->als_settings.als_gain;
    return ESP_OK;  
}


esp_err_t apds_set_als_gain(APDS_DEV dev, als_gain_t *g) {
    uint8_t val = *g;
    esp_err_t err = ESP_OK;
    if(val > APDS_GST_GAIN_MAX) {
        err = ESP_ERR_INVALID_ARG;
        ESP_LOGE(APDS_TAG, "Invalid args");
    }
    else {
        err = regUnsetMask(dev, APDS_REGADDR_GAIN_CTRL, 3);
        if(!err && val) {
            err = regSetMask(dev, APDS_REGADDR_GAIN_CTRL, val);
        }
    }

    if(!err) {
        dev->als_settings.als_gain = val;
    }
    return err;
}



/***************** Proximity Settings **********************/

esp_err_t apds_get_prxintr_low_thr(APDS_DEV dev, uint8_t *thr) {
    *thr = dev->prx_settings.prox_thresh_l;
    return ESP_OK;
}


esp_err_t apds_set_prxintr_low_thr(APDS_DEV dev, uint8_t *thr) {
    uint8_t val = *thr;
    esp_err_t err = gcd_i2c_write_address(dev->bus, dev->addr, APDS_REGADDR_PRX_THR_LOW, 1, &val);
    if(!err) {
        dev->prx_settings.prox_thresh_l = val;
    }
    return err;
}


esp_err_t apds_get_prxintr_high_thr(APDS_DEV dev, uint8_t *thr) {
    *thr = dev->prx_settings.prox_thresh_h;
    return ESP_OK;
}


esp_err_t apds_set_prxintr_high_thr(APDS_DEV dev, uint8_t *thr) {
    uint8_t val = *thr;
    esp_err_t err = gcd_i2c_write_address(dev->bus, dev->addr, APDS_REGADDR_PRX_THR_HIGH, 1, &val);
    if(!err) {
        dev->prx_settings.prox_thresh_h = val;
    }
    return err;
}


esp_err_t apds_get_prx_intr_persistence(APDS_DEV dev, uint8_t *cnt) {
    *cnt = dev->prx_settings.prox_perist_cycles;
    return ESP_OK;
}


esp_err_t apds_set_prx_intr_persistence(APDS_DEV dev, uint8_t *cnt) {
    esp_err_t err = ESP_OK;
    uint8_t val = *cnt;
    if(val > 15) {
        err= ESP_ERR_INVALID_ARG;
    }
    else {
        err = regUnsetMask(dev, APDS_REGADDR_ISR_PERSIST_FLTR, 0xf0);
        if(!err && val) {
            err =regSetMask(dev, APDS_REGADDR_ISR_PERSIST_FLTR, (val << 4));
        }
    }
    if(!err) {
        dev->prx_settings.prox_perist_cycles = val;
    }

    return err;
}


esp_err_t apds_get_prx_ledpulse_t(APDS_DEV dev, prx_ledtime_t *t) {
    *t = dev->prx_settings.ledtime;
    return ESP_OK;
}


esp_err_t apds_set_prx_ledpulse_t(APDS_DEV dev, prx_ledtime_t *t) {
    uint8_t val = *t;
    esp_err_t err = ESP_OK;

    if(val >= APDS_PRX_LEDTIME_MAX) {
        ESP_LOGE(APDS_TAG, "Error: Invalid value [%u]", val);
        err = ESP_ERR_INVALID_ARG;
    }
    else {
        err = regUnsetMask(dev, APDS_REGADDR_PRX_PULSE_LEN, 0xC0);
        err = regSetMask(dev, APDS_REGADDR_PRX_PULSE_LEN, (val << 6));    
    }

    if(!err) {
        dev->prx_settings.ledtime = val;
    }

    return err;
}


esp_err_t apds_get_prx_pulses(APDS_DEV dev, uint8_t *cnt) {
    *cnt = dev->prx_settings.led_pulse_n + 1;
    return ESP_OK;
}


esp_err_t apds_set_prx_pulses(APDS_DEV dev, uint8_t *cnt) {

    uint8_t val = *cnt;
    esp_err_t err = ESP_OK;

    if(val > 63) {
        err = ESP_ERR_INVALID_ARG;
        ESP_LOGE(APDS_TAG, "Invalid value");
    }
    else {
        uint8_t  regval = val | (dev->prx_settings.ledtime << 6);
        err = gcd_i2c_write_address(dev->bus, dev->addr, APDS_REGADDR_PRX_PULSE_LEN, 1, &regval); 
    }
    if(!err) {
        dev->prx_settings.led_pulse_n = val; 
    }

    return err;

}


esp_err_t apds_get_prx_gain(APDS_DEV dev, gst_gain_t *g) {
    *g = dev->prx_settings.prox_gain;
    return ESP_OK;
}


esp_err_t apds_set_prx_gain(APDS_DEV dev, gst_gain_t *g) {

    uint8_t val = *g;
    esp_err_t err = ESP_OK;
    if(val > APDS_GST_GAIN_MAX) {
        err = ESP_ERR_INVALID_ARG;
        ESP_LOGE(APDS_TAG, "Invalid args");
    }
    else {
        err = regUnsetMask(dev, APDS_REGADDR_GAIN_CTRL, (3 << APDS_REG_OFFSET_PGAIN));
        if(!err && val) {
            err = regSetMask(dev, APDS_REGADDR_GAIN_CTRL, (val << APDS_REG_OFFSET_PGAIN));
        }
    }

    if(!err) {
        dev->prx_settings.prox_gain = val;
    }
    return err;
}




/******************** Gesture Settings ***********************/


esp_err_t apds_get_gst_proximity_ent_thr(APDS_DEV dev, uint8_t *d) {
    *d = dev->gst_settings.gst_thresh_entr;
    return ESP_OK;
}


esp_err_t apds_set_gst_proximity_ent_thr(APDS_DEV dev, uint8_t *d) {
    uint8_t val = *d;
    esp_err_t err = gcd_i2c_write_address(dev->bus, dev->addr, APDS_REGADDR_GST_ENTR_THR, 1, &val);
    if(!err) {
        dev->gst_settings.gst_thresh_entr = val;
    }

    return err;
}


esp_err_t apds_get_gst_proximity_ext_thr(APDS_DEV dev, uint8_t *d) {
    *d = dev->gst_settings.gst_thresh_exit;
    return ESP_OK;
}


esp_err_t apds_set_gst_proximity_ext_thr(APDS_DEV dev, uint8_t *d) {
    uint8_t val = *d;
    esp_err_t err = gcd_i2c_write_address(dev->bus, dev->addr, APDS_REGADDR_GST_EXT_THR, 1, &val);
    if(!err) {
        dev->gst_settings.gst_thresh_exit = val;
    }

    return err;
}


esp_err_t apds_set_prx_direction_mode(APDS_DEV dev, uint8_t *mode) {

    uint8_t regval = *mode;
    if(regval > APDS_DIR_LEFTRIGHT_ONLY) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = gcd_i2c_write_address(dev->bus, dev->addr, APDS_REGADDR_GST_CONFIG_3, 1, &regval);
    if(!err) {
        dev->gst_settings.gst_dir_select = regval;  
    }
    return err;
}


esp_err_t apds_get_prx_direction_mode(APDS_DEV dev, uint8_t *mode) {
    *mode = dev->gst_settings.gst_dir_select;
    return ESP_OK;
}


esp_err_t apds_get_gst_pulse_len(APDS_DEV dev, uint8_t *cnt) {
    esp_err_t status = ESP_OK;
    *cnt = dev->gst_settings.gst_pulse_len;
    return status;
}


esp_err_t apds_set_gst_pulse_len(APDS_DEV dev, uint8_t *cnt) {
    esp_err_t err = ESP_OK;
    uint8_t val = *cnt;
    uint8_t regval = 0;
    if(val > 3) {
        err = ESP_ERR_INVALID_ARG;
    }
    else {
        err = gcd_i2c_read_address(dev->bus, dev->addr, APDS_REGADDR_GST_PULSE_LEN, 1, &regval);
    }

    if(!err) {
        regval &= ~(0b11000000);
        regval |= (val << 6);
        err = gcd_i2c_write_address(dev->bus, dev->addr, APDS_REGADDR_GST_PULSE_LEN, 1, &regval);
        if(!err) {
            dev->gst_settings.gst_pulse_len = val;
        }
    }
    
    return err;
}


esp_err_t apds_get_gst_pulse_cnt(APDS_DEV dev, uint8_t *cnt) {
    esp_err_t status = ESP_OK;
    *cnt = dev->gst_settings.gst_pulse_len;
    return status;
}


esp_err_t apds_set_gst_pulse_cnt(APDS_DEV dev, uint8_t *cnt) {
    esp_err_t err = ESP_OK;
    uint8_t val = *cnt;
    uint8_t regval = 0;
    if(val > 64) {
        err = ESP_ERR_INVALID_ARG;
    }
    else {
        err = gcd_i2c_read_address(dev->bus, dev->addr, APDS_REGADDR_GST_PULSE_LEN, 1, &regval);
    }

    if(!err) {
        regval &= ~(0b11111);
        regval |= val;
        err = gcd_i2c_write_address(dev->bus, dev->addr, APDS_REGADDR_GST_PULSE_LEN, 1, &regval);
        if(!err) {
            dev->gst_settings.gst_pulse_cnt = val;
        }
    }
    
    return err;
}



/*** INTERRUPT SETTINGS **/


esp_err_t apds_get_sleep_after_intr(APDS_DEV dev, uint8_t *en) {
    *en = dev->gen_settings.sleep_after_intr;
    return ESP_OK;
}


esp_err_t apds_set_sleep_after_intr(APDS_DEV dev, uint8_t *en) {
    esp_err_t err = ESP_OK;
    uint8_t val = *en;

    if(val ) {
        err = regSetMask(dev, APDS_REGADDR_CONFIG_3, APDS_REGBIT_SLP_POST_INT);
    }
    else {
        err = regUnsetMask(dev, APDS_REGADDR_CONFIG_3, APDS_REGBIT_SLP_POST_INT);
    }
 
    if(!err) {
        dev->gen_settings.sleep_after_intr = (val) ? 1 : 0;
    }
    return err;
}


esp_err_t apds_get_als_intr(APDS_DEV dev, uint8_t *en) {
    *en = dev->als_settings.asl_intr_en;
    return ESP_OK;
}


esp_err_t apds_set_als_intr(APDS_DEV dev, uint8_t *en) {
    esp_err_t err = ESP_OK;
    uint8_t val = *en;

    if(val ) {
        err = regSetMask(dev, APDS_REGADDR_ENABLE, APDS_REGBIT_ALS_INT_EN);
    }
    else {
        err = regUnsetMask(dev, APDS_REGADDR_ENABLE, APDS_REGBIT_ALS_INT_EN);
    }
 
    if(!err) {
        dev->als_settings.asl_intr_en = (val) ? 1 : 0;
    }
    return err;
}


esp_err_t apds_get_prox_intr(APDS_DEV dev, uint8_t *en) {
    *en = dev->prx_settings.prox_intr_en;
    return ESP_OK;
}


esp_err_t apds_set_prox_intr(APDS_DEV dev, uint8_t *en) {
    esp_err_t err = ESP_OK;
    uint8_t val = *en;

    if(val ) {
        err = regSetMask(dev, APDS_REGADDR_ENABLE, APDS_REGBIT_PRX_INT_EN);
    }
    else {
        err = regUnsetMask(dev, APDS_REGADDR_ENABLE, APDS_REGBIT_PRX_INT_EN);
    }
 
    if(!err) {
        dev->prx_settings.prox_intr_en = (val) ? 1 : 0;
    }
    return err;
}


esp_err_t apds_get_gst_intr(APDS_DEV dev, uint8_t *en) {
    *en = dev->gst_settings.gst_int_en;
    return ESP_OK;
}


esp_err_t apds_set_gst_intr(APDS_DEV dev, uint8_t *en) {
    esp_err_t err = ESP_OK;
    uint8_t val = *en;

    if(val ) {
        err = regSetMask(dev, APDS_REGADDR_GST_CONFIG_4, APDS_REGBIT_GST_INT_EN);
    }
    else {
        err = regUnsetMask(dev, APDS_REGADDR_GST_CONFIG_4, APDS_REGBIT_GST_INT_EN);
    }
 
    if(!err) {
        dev->prx_settings.prox_intr_en = (val) ? 1 : 0;
    }
    return err;
}




/*************** FIFO Settings **********************/

esp_err_t apds_get_gst_fifo_lvl(APDS_DEV dev, uint8_t *level) {
    esp_err_t status = ESP_OK;
    uint8_t  regval = 0;
    status = gcd_i2c_read_address(dev->bus, dev->addr, APDS_REGADDR_GST_FIFO_LVL, 1, &regval);
    *level = regval;
    return status;
}


esp_err_t apds_read_fifo_data_set(APDS_DEV dev, uint8_t *index) {
    esp_err_t err = ESP_OK;
    uint8_t data[4] = {0};
    uint8_t pkt_index = *index;

    if(pkt_index > 8) {
        return ESP_ERR_INVALID_ARG;
    }

    err = gcd_i2c_read_address(dev->bus,dev->addr, APDS_REGADDR_GST_FIFO_U, 4, data);
    if(!err) {
        dev->data.gst_fifo_u_data[pkt_index] = data[0];
        dev->data.gst_fifo_d_data[pkt_index] = data[1];
        dev->data.gst_fifo_l_data[pkt_index] = data[2];
        dev->data.gst_fifo_r_data[pkt_index] = data[3];
    }

    return err;
}


esp_err_t apds_read_fifo_full(APDS_DEV dev) {
    esp_err_t err = ESP_OK;
    uint8_t data[128] = {0};
    uint8_t num_bytes = 0;

    err = apds_get_gst_fifo_lvl(dev, &num_bytes);

    if(err || num_bytes == 0) {
        ESP_LOGE(APDS_TAG, "Invalid fifo count (%u)", err);
        return ESP_ERR_INVALID_RESPONSE;
    }
    else {
        num_bytes *= 4;
        err = gcd_i2c_read_address(dev->bus,dev->addr, APDS_REGADDR_GST_FIFO_U, num_bytes, data);
    }
    if(!err) {
        /** copy/sort data **/
        memset(dev->data.gst_fifo_u_data, 0, (sizeof(uint8_t) * 8 * 4));
        uint8_t j=0;
        for(uint8_t i=0; i<num_bytes; i++) {
            switch (i % 4)
            {
            case 0:
                dev->data.gst_fifo_u_data[j] = data[i];
                break;
            case 1:
                dev->data.gst_fifo_d_data[j] = data[i];
                break;
            case 2:
                dev->data.gst_fifo_l_data[j] = data[i];
                break;
            case 3:
                dev->data.gst_fifo_r_data[j] = data[i];
                j++;
                break;
            default:
                break;
            }            
        }
    }

    return err;
}


esp_err_t apds_get_fifo_valid(APDS_DEV dev, bool *valid) {

    esp_err_t err = ESP_OK;
    uint8_t regval = 0; 
    err = gcd_i2c_read_address(dev->bus, dev->addr, APDS_REGADDR_GST_STATUS, 1, &regval);

    if(!err) {
        if(regval & APDS_REGBIT_GST_DATA_VALID) {
            *valid = true;
        }
        else {
            *valid = false;
        }
    }
    return err;
}


esp_err_t apds_get_fifo_overflow(APDS_DEV dev, bool *ov) {

    esp_err_t err = ESP_OK;
    uint8_t regval = 0; 
    err = gcd_i2c_read_address(dev->bus, dev->addr, APDS_REGADDR_GST_STATUS, 1, &regval);

    if(!err) {
        if(regval & APDS_REGBIT_GST_FIFO_OVR) {
            *ov = true;
        }
        else {
            *ov = false;
        }
    }
    return err;
}


esp_err_t apds_get_fifo_thresh(APDS_DEV dev, uint8_t *thr) {
    *thr = dev->gst_settings.gst_fifo_thresh;
    return ESP_OK;
}


esp_err_t apds_set_fifo_thresh(APDS_DEV dev, uint8_t *thr) {
    uint8_t thresh = *thr;
    esp_err_t err = ESP_OK;
    uint8_t regval = 0;
    
    if(thresh > 3) {
        return ESP_ERR_INVALID_ARG;
    }
    
    err = gcd_i2c_read_address(dev->bus, dev->addr, APDS_REGADDR_GST_CONFIG_1, 1, &regval);
    
    if(!err) {
        regval &= ~(0b11000000);
        regval |= (thresh << 6);
        err = gcd_i2c_read_address(dev->bus, dev->addr, APDS_REGADDR_GST_CONFIG_1, 1, &regval);
        if(!err) {
            dev->gst_settings.gst_fifo_thresh = thresh;
        }
    }

    return err;
}



