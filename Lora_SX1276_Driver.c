/***************************************
* \file     Lora_SX1276_Driver.c
* \brief    A driver for the SX1276 Lora Chipset 
*              This one is going to be fun...
*               Thanks to https://github.com/sandeepmistry/arduino-LoRa for guidance
*               via codesurfing
* \date     March 2021
* \author   RJAM
****************************************/

/********* Includes *******************/

#include "esp_err.h"
#include "esp_types.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "Lora_SX1276_Driver.h"


#ifdef CONFIG_USE_PERIPH_MANAGER

// #include "CommandAPI.h"

// const parameter_t lora_parameter_map[LORA_PERIPH_LEN] = {

//     {"Operating Mode", 1, sx1276_get_opmode_LoRa, sx1276_set_opmode_LoRa, 


// };
// const peripheral_t lora_peripheral_template;


#endif

/****** Function Prototypes ***********/

/************ ISR *********************/

/****** Private Data ******************/

const char *LORA_TAG = "SX1276";



/****** Private Functions *************/


static esp_err_t sx_spi_write_address_byte(SX1276_DEV Dev, uint8_t addr, uint8_t byte) {

    esp_err_t err = ESP_OK;
    uint8_t cmd = (addr | SX_READWRITE_BIT);
    spi_transaction_t trx = {0};

    trx.length = 16;
    trx.tx_data[0] = cmd; 
    trx.tx_data[1] = byte;
    trx.flags = SPI_TRANS_USE_TXDATA;

    // err = spi_device_acquire_bus(Dev->spi_handle, SX_SPI_TIMEOUT_DEFAULT);
    if(err != ESP_OK) {
       ESP_LOGE(LORA_TAG, "Error: Unable to aquire bus [%u]", err);
    }
    else {
        err = spi_device_transmit(Dev->spi_handle, &trx);
    }

    return err;
}


static esp_err_t sx_read_address_byte(SX1276_DEV Dev, uint8_t addr, uint8_t *byte) {

    esp_err_t err = ESP_OK;
    uint8_t cmd = (addr & ~(SX_READWRITE_BIT));
    spi_transaction_t trx = {0};

    trx.length = 8;
    trx.rxlength = 8;
    trx.tx_data[0] = cmd; 
    trx.flags = (SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA);

    err = spi_device_acquire_bus(Dev->spi_handle, SX_SPI_TIMEOUT_DEFAULT);
    if(err != ESP_OK) {
       ESP_LOGE(LORA_TAG, "Error: Unable to aquire bus [%u]", err);
    }
    else {
        err = spi_device_transmit(Dev->spi_handle, &trx);
    }
    if(err != ESP_OK) {
        ESP_LOGE(LORA_TAG, "Error performing SPI transaction! [%u]", err);
    }
    else {
        ESP_LOGI(LORA_TAG, "Read the following data: 0x%02x 0x%02x 0x%02x 0x%02x", trx.rx_data[0], trx.rx_data[1], trx.rx_data[2], trx.rx_data[3]);
        *byte = trx.rx_data[0];
    }

    return err;
}


static esp_err_t sx_spi_read_mod_write_byte(SX1276_DEV Dev, uint8_t addr, uint8_t or_data) {
    esp_err_t err = ESP_OK;

    uint8_t reg = 0;
    err = sx_read_address_byte(Dev, addr, &reg);
    if(!err) {
        reg |= or_data;
        err = sx_spi_write_address_byte(Dev, addr, reg);
    }
    return err;
}


static void reset_device(SX1276_DEV Dev) {

    ESP_LOGI(LORA_TAG, "Resetting device");
    gpio_set_level(Dev->rst_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(Dev->rst_pin, 1);

}


static esp_err_t sx_modem_status(SX1276_DEV Dev, uint8_t *val) {

    esp_err_t err = ESP_OK;
    uint8_t regval = 0;
    err = sx_read_address_byte(Dev, SX1276_REGADDR_MODEM_STAT, &regval);
    if(!err) {
        *val = regval;
    }
    return err;
}


static esp_err_t set_overcurrent_protection(SX1276_DEV Dev, uint8_t mA) {
    
    esp_err_t err = ESP_OK;
    uint8_t trim = 27; /** TODO: find out where this comes from **/
    if(mA <= 120) {
        trim = (mA - 45) / 5;
    } else if(mA <= 240) {
        trim = (mA + 30) / 10;
    }
    else {
        ESP_LOGI(LORA_TAG, "Current limit too damn high!");
        err = ESP_ERR_INVALID_ARG;
    }

    if(!err) {
        err = sx_spi_write_address_byte(Dev, SX1276_REGADDR_OCURRENT_PROT, (0x20 | (0x1F & trim)));
    }

    return err;
}


static esp_err_t set_tx_pwr(SX1276_DEV Dev, uint16_t pwr) {
    // SX1276_REGADDR_PA_CONFIG
    esp_err_t err = ESP_OK;
    /** TODO: Investigate these pins ! **/

    if (pwr > 17) {
        if (pwr > 20) {
            pwr = 20;
        }
        pwr -= 3;

        /** this is in high power mode **/
        err = sx_spi_write_address_byte(Dev, SX1276_REGADDR_PA_DAC, RE_PA_HP_ENABLE);
        err += set_overcurrent_protection(Dev, 140); /** TODO: Investigate moar! **/

    } else {
        if (pwr < 2) {
            pwr = 2;
        }
        err = sx_spi_write_address_byte(Dev, SX1276_REGADDR_PA_DAC, RE_PA_STD_PWR);
        err += set_overcurrent_protection(Dev, 100);
    }

    if(!err) {
        err = sx_spi_write_address_byte(Dev, SX1276_REGADDR_PA_CONFIG, (SX1276_PA_SELECT_BIT | (pwr-2)));
    }

    return err;
}


static esp_err_t device_sleep(SX1276_DEV Dev) {
    esp_err_t err = ESP_OK;
    ESP_LOGI(LORA_TAG, "Setting device to sleep");
    err = sx_spi_write_address_byte(Dev, SX1276_REGADDR_OPMODE, (SX1276_LORA_MODE_BIT | 0));
    return err;
}


static esp_err_t device_idle(SX1276_DEV Dev) {
    esp_err_t err = sx_spi_write_address_byte(Dev, SX1276_REGADDR_OPMODE, (SX1276_LORA_MODE_BIT | SX1276_MODE_STDBY));
    return err;
}


static esp_err_t get_device_id(SX1276_DEV Dev, uint8_t *ver) {
    uint8_t version = 0;
    esp_err_t err = sx_read_address_byte(Dev, SX1276_REGADDR_VERSION, &version);
    if(!err) {
        ESP_LOGI(LORA_TAG, "Got response : 0x%02x", version);
        *ver = version;
    }
    else {
        ESP_LOGE(LORA_TAG, "Error: %u", err);
    }
    return err;
}


static esp_err_t set_frequency(SX1276_DEV Dev, uint32_t frq) {

    uint64_t frf = (uint64_t)((frq << 19) / 32000000);
    esp_err_t err = sx_spi_write_address_byte(Dev, SX1276_REGADDR_CARRFREQ_MSB, (uint8_t )(frf >> 16));
    err += sx_spi_write_address_byte(Dev, SX1276_REGADDR_CARRFREQ_MIDB, (uint8_t )(frf >> 8));
    err += sx_spi_write_address_byte(Dev, SX1276_REGADDR_CARRFREQ_LSB, (uint8_t )(frf));

    return err;
}


static esp_err_t spreadingfactor_set(SX1276_DEV Dev, uint8_t sf) {
   esp_err_t err = ESP_OK;
   if(sf >= SX1276_SPREADF_MAX || sf <= SX1276_SPREADF_MIN) {
       err = ESP_ERR_INVALID_ARG;
   } else {
        uint8_t reg = 0;
        err = sx_read_address_byte(Dev, SX1276_REGADDR_MODEM_CONFIG2, &reg);
        if(!err) {
            reg |= (sf << 4);
            err = sx_spi_write_address_byte(Dev, SX1276_REGADDR_MODEM_CONFIG2, reg);
        }
   }
    if(!err) {
        Dev->settings.sf = sf;
    }

   return err;
}


static esp_err_t set_lora_bw(SX1276_DEV Dev, lora_bw_t bw) {
    esp_err_t err = ESP_OK;
    uint8_t reg = 0, val = 0;
    err = sx_read_address_byte(Dev, SX1276_REGADDR_MODEM_CONFIG1, &reg);
    if(!err) {
        val = reg;
        val |= (bw << 4);
        err = sx_spi_write_address_byte(Dev, SX1276_REGADDR_MODEM_CONFIG1, val);
    }
    if(!err) {
        Dev->settings.bw = bw;
    }
    return err;
}


static esp_err_t  device_init(SX1276_DEV Dev) {

    esp_err_t err = ESP_OK;
    uint8_t ver = 0;
    uint8_t lna = 0;
    ESP_LOGI(LORA_TAG, "Initialising device...");

    if(get_device_id(Dev, &ver) != ESP_OK) {
        return 1;
    }

    else if(device_sleep(Dev) != ESP_OK) {
        return 1;
    }
    /** set LoRa mode **/
    else if(sx_spi_write_address_byte(Dev, SX1276_REGADDR_OPMODE, SX1276_LORA_MODE_BIT) != ESP_OK) {
        return 1;
    }

    /** set the operating frequency - made this settable within limits **/
    else if(set_frequency(Dev, SX1276_LORA_FREQ_EURO) != ESP_OK) {
        return 1;
    }
    /** set the fifo off **/
    else if(sx_spi_write_address_byte(Dev, SX1276_REGADDR_FIFO_TXBASE, 0) != ESP_OK ||
            sx_spi_write_address_byte(Dev, SX1276_REGADDR_FIFO_RXBASE, 0) != ESP_OK) {
        return 1;
    }
    /** set the LNA gain **/
    else if(sx_read_address_byte(Dev, SX1276_REGADDR_LNA_CONFIG, &lna) != ESP_OK ||
            sx_spi_write_address_byte(Dev, SX1276_REGADDR_LNA_CONFIG, (lna | SX1276_LNA_BOOST_EN)) != ESP_OK) {
        return 1;
    }
    /** set modem mode **/
    else if(sx_spi_write_address_byte(Dev, SX1276_REGADDR_MODEM_CONFIG3, SX1276_AGC_AUTO_ON_BIT) != ESP_OK) {
        return 1;
    }
    /** Set TX power **/
    else if(set_tx_pwr(Dev, 12) != ESP_OK) {
        return 1;
    }
    else {
        device_idle(Dev);

        Dev->settings.frequency = SX1276_LORA_FREQ_EURO;
        Dev->settings.current_mode = SX1276_MODE_STDBY;
        Dev->settings.gain = SX1276_MAX_GAIN;
        Dev->settings.lna_boost = true;
        Dev->settings.agc_auto = true;
    }

    return err;
}


static void sx1276_driver_task(void *args) {


    SX1276_DEV Dev = (sx1276_driver_t *)args;

    while(1) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    /** here be dragons **/
}

/****** Global Data *******************/

/****** Global Functions *************/

/** 
 *  intitialise the device, assume SPI already init. 
 *  Pins for TTGO Lora32 are 
 *  - MISO	19
 *  - CS	17
 *  - MOSI	27
 *  - SCK	5
 *  - RST	14
 *  - IRQ	26
 **/

#define LORA_RST_PIN 14
#define LORA_IRQ_PIN 26
#define LORA_CS_PIN 17

SX1276_DEV sx1276_init(sx1276_init_t *init) {
    

    esp_err_t err = ESP_OK;
    spi_device_handle_t spi_handle;
    gpio_config_t conf = {0};
    SX1276_DEV dev_handle = NULL;
    TaskHandle_t t_handle = NULL;
    conf.mode = GPIO_MODE_OUTPUT;
    conf.pin_bit_mask = (1 << init->rst_pin);
    conf.pull_up_en = GPIO_PULLUP_DISABLE;

    err = gpio_config(&conf);
    if (err != ESP_OK) {
        ESP_LOGE(LORA_TAG, "Error configuring GPIO");
    }

    if(!err) {
        gpio_set_level(LORA_RST_PIN, 1);

        /** initialise the spi device **/
        if(init->spi_bus != SPI1_HOST && init->spi_bus != SPI2_HOST) {
            ESP_LOGE(LORA_TAG, "Error, invalid SPI Bus");
            err = ESP_ERR_INVALID_ARG;
        } 
        if(!err) {
            spi_host_device_t dev = init->spi_bus;
            spi_device_interface_config_t dconf = {0};
            dconf.address_bits = 0;
            dconf.clock_speed_hz = 100000;
            dconf.command_bits = 0;
            dconf.cs_ena_posttrans = 2;
            dconf.cs_ena_pretrans = 2;
            dconf.dummy_bits = 0;
            dconf.duty_cycle_pos = 128;
            dconf.mode = 0;
            dconf.spics_io_num = init->cs_pin;
            err = spi_bus_add_device(dev, &dconf, &spi_handle);
        }

        if(err != ESP_OK) {
            ESP_LOGE(LORA_TAG, "Error adding device to the bus!");
        }
    }

    /** create the device handle **/
    if(!err) {
        dev_handle = (sx1276_driver_t *) heap_caps_calloc(1, sizeof(sx1276_driver_t), MALLOC_CAP_DEFAULT);
        if(dev_handle == NULL) {
            ESP_LOGE(LORA_TAG, "Error allocating memory for driver handle!");
            err = ESP_ERR_NO_MEM;
        }
        else {
            dev_handle->cs_pin = LORA_CS_PIN;
            dev_handle->rst_pin = LORA_RST_PIN;
            dev_handle->spi_handle = spi_handle;
        }
    }

        /** start the driver task **/
    if(!err && xTaskCreate(sx1276_driver_task, "lora_driver_task", 5012, (void *)dev_handle, 3, &t_handle) != pdTRUE) {
        err = ESP_ERR_NO_MEM;
        ESP_LOGE(LORA_TAG, "Error starting driver task [%u]", err);
    }

    /** initialise the device in std lora mode **/
    if(!err && device_init(dev_handle) != ESP_OK) {
        err = ESP_FAIL;
        ESP_LOGE(LORA_TAG, "Error configuring device [%u]", err);   
    } 

    if(!err) {
        ESP_LOGI(LORA_TAG, "Succesfully intialised SX1276 Device!");
    }
    else {
        ESP_LOGI(LORA_TAG, "Failed to intialise SX1276 Device! :( [%u]", err);
        if(dev_handle != NULL) {
            heap_caps_free(dev_handle);
        }
    }

    return dev_handle;
}


/** lora getters / setters 
 * 
 * TODO: 
 *  - spreading factor
 *  - bandwidth
 *  - header_mode (implicit/explicit)
 *  - LDR Optimisation
 *  - status
 *  - DIO function
 **/

esp_err_t sx1276_get_opmode_LoRa(sx1276_driver_t *dev, uint8_t *mode);

esp_err_t sx1276_get_modtype(sx1276_driver_t *dev, uint8_t *mode);

esp_err_t sx1276_get_lowfreq_mode(sx1276_driver_t *dev, uint8_t *mode);

esp_err_t sx_get_mode(SX1276_DEV Dev, uint8_t mode) {
   esp_err_t status = ESP_OK;
   
   return status;
}


esp_err_t sx_get_lora_spreading_factor(SX1276_DEV dev, uint8_t *val) {
   esp_err_t status = ESP_OK;
   
   return status;
}

esp_err_t sx_set_lora_spreading_factor(SX1276_DEV dev, uint8_t *val) {
   esp_err_t status = ESP_OK;
   
   return status;
}

esp_err_t sx_get_lora_bandwidth(SX1276_DEV dev, uint8_t *val) {
   esp_err_t status = ESP_OK;
   
   return status;
}

esp_err_t sx_set_lora_bandwidth(SX1276_DEV dev, uint8_t *val) {
   esp_err_t status = ESP_OK;
   
   return status;
}

esp_err_t sx_get_lora_headermode(SX1276_DEV dev, uint8_t *val) {
   esp_err_t status = ESP_OK;
   
   return status;
}

esp_err_t sx_set_lora_headermode(SX1276_DEV dev, uint8_t *val) {
   esp_err_t status = ESP_OK;
   
   return status;
}

esp_err_t sx_get_lora_ldo_optmz(SX1276_DEV dev, uint8_t *val) {
   esp_err_t status = ESP_OK;
   
   return status;
}

esp_err_t sx_set_lora_ldo_optmz(SX1276_DEV dev, uint8_t *val) {
   esp_err_t status = ESP_OK;
   
   return status;
}

esp_err_t sx_get_lora_dio0_func(SX1276_DEV dev, uint8_t *val) {
   esp_err_t status = ESP_OK;
   
   return status;
}

esp_err_t sx_set_lora_dio0_func(SX1276_DEV dev, uint8_t *val) {
   esp_err_t status = ESP_OK;
   
   return status;
}
