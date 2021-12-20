/***************************************
* \file     MFRC522_Driver.c
* \brief    A driver for the MFRC RFID read/write IC
*
* \date     November 2021
* \author   RJAM
****************************************/

/********* Includes *******************/
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_types.h"
#include "esp_err.h"
#include "esp_log.h"
#include "Utilities.h"
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "genericCommsDriver.h"
#include "Utilities.h"

#include "MFRC522_Driver.h"

/****** Global Data *******************/

const char *MFRC_TAG = "MFRC Driver";

const mfrc_register_t init_reg_state = {
    .command_reg.regval = 0x20,
    .command_en_reg.regval = 0x80,
    .irq_passing_reg.regval = 0x00,
    .com_irq_reg.regval = 0x14,
    .div_irq_reg.regval = 0x00,
    .error_reg.regval = 0x00,
    .status_1_reg.regval = 0x21,
    .status_2_reg.regval = 0x00,
    .fifo_data_reg.regval = 0x00,
    .fifo_lvl_reg.regval = 0x00,
    .fifo_watermark_reg.regval = 0x08,
    .control_reg.regval = 0x10,
    .bitframe_reg.regval = 0x00,
    .collision_reg.regval = 0x00,
    .reserved_1 = 0x00,
    .reserved_2 = 0x00,
    .mode_reg.regval = 0x3F,
    .tx_mode_reg.regval = 0x00,
    .rx_mode_reg.regval = 0x00,
    .txcontrol_reg.regval = 0x80,
    .tx_ask_reg.regval = 0x00,
    .tx_select_reg.regval = 0x10,
    .rx_select_reg.regval = 0x84,
    .rx_threshold_reg.regval = 0x84,
    .demod_reg.regval = 0x4D,
    .reserved_3 = 0x00,
    .reserved_4 = 0x00,
    .mifare_tx_reg.regval = 0x62,
    .mifare_rx_reg.regval = 0x00,
    .reserved_5 = 0x00,
    .serial_speed_reg.regval = 0xEB,
    .reserved_7 = 0,
    .crc_result_msb = 0,
    .crc_result_lsb = 0,
    .reserved_8 = 0,
    .mod_width = 0,
    .reserved_9 = 0,
    .rf_cfg_reg.regval = 0x48,
    .gsn_reg.regval = 0x88,
    .cw_gsp_reg.regval = 0x20,
    .mod_gs_reg.regval = 0x20,
    .tmr_mode_reg.regval = 0x00,
    .tmr_prescaler_lsb = 0,
    .tmr_reload_msb = 0,
    .tmr_reload_lsb = 0,
    .tmr_cnt_msb = 0,
    .tmr_cnt_lsb = 0,
    .reserved_10 = 0,
    .test_sel_1_reg.regval = 0,
    .test_sel_2_reg.regval = 0,
    .testpin_en_reg.regval = 0x80,
    .testpin_val_reg.regval = 0,
    .testbus = 0,
    .auto_test_reg.regval = 0,
    .version_reg.regval = 0,
    .analog_test_reg.regval = 0,
    .test_dac1_reg.regval = 0,
    .test_dac2_reg.regval = 0,
    .adc_test_reg.regval = 0,
    .reserved_11 = 0,
    .reserved_12 = 0,
    .reserved_13 = 0
};
/****** Function Prototypes ***********/

/************ ISR *********************/

void mfrc_irq_function(void *args) {


}

/****** Private Data ******************/

/****** Private Functions *************/

/*
 * Resets the driver's record of the registers to the post-reset state 
 */
static void mfrc_reset_registers(MFRC_DEV dev) {
    memcpy(&dev->registers, &init_reg_state, sizeof(mfrc_register_t));
    return;
}

// static esp_err_t mfrc_test_registers(MFRC_DEV dev) {

//     esp_err_t err = ESP_OK;

//     uint8_t tx_buffer[64] = {0};
//     uint8_t rx_buffer[64] = {0};

//     for(uint8_t i=0; i < 64; i++) {
//         tx_buffer[i] = (((i+1) << 1) | MFRC_SPI_READ_BIT);
//     }

//     spi_transaction_t trx = {0};
//     trx.length = 8 * 64;
//     trx.rxlength = 8 * 64;
//     trx.tx_buffer = tx_buffer;
//     trx.rx_buffer = rx_buffer;

//     err = spi_device_polling_transmit(dev->spi_handle, &trx);

//     if(!err) {
//         printf("Tx Block\n");
//         showmem(tx_buffer, 64);
//         printf("Rx Block\n");
//         showmem(rx_buffer, 64);
//     }

//     return err;
// }



static esp_err_t mfrc_flush_fifo(MFRC_DEV dev) {
    return mfrc_write_register_byte(dev, MFRC_REGADDR_FIFO_LEVEL, (1 << 7));
}


static esp_err_t mfrc_clear_interrupts(MFRC_DEV dev) {
    return mfrc_write_register_byte(dev, MFRC_REGADDR_COMM_IRQ, 0x7F);
}


static esp_err_t mfrc_transceive_data(MFRC_DEV dev, uint8_t *data, uint8_t len) {

    mfrc_clear_interrupts(dev);
    mfrc_flush_fifo(dev);

    mfrc_write_register_data(dev, MFRC_REGADDR_FIFO_DATA, data, len);


    return ESP_ERR_NOT_SUPPORTED;
}



static esp_err_t mfrc_poll_for_cards(MFRC_DEV dev, bool card_present) {

    uint8_t buffer[2] = {0};

    /** clear collision clr bit **/
    dev->registers.collision_reg.bits.clr_rx_on_collision = 0;
    esp_err_t err = mfrc_write_register_byte(dev, MFRC_REGADDR_COLLISION_DETECT, dev->registers.collision_reg.regval);

    if(!err) {

    }

    return err;

}


static esp_err_t mfrc_read_register_byte(MFRC_DEV dev, uint8_t reg_addr, uint8_t *data) {

    esp_err_t err = ESP_OK;

    spi_transaction_t trx = {0};
    trx.length = 16;
    trx.tx_data[0] = (reg_addr << MFRC_SPI_ADDR_SHIFT) | MFRC_SPI_READ_BIT;
    trx.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA ;

    err = spi_device_polling_transmit(dev->spi_handle, &trx);

    if(!err) {
        *data = trx.rx_data[1];
    }
    else {
        ESP_LOGE(MFRC_TAG, "Error during transmission {%u}", err);
    }

    return err;
}

static esp_err_t mfrc_write_register_byte(MFRC_DEV dev, uint8_t reg_addr, uint8_t data) {

    esp_err_t err = ESP_OK;

    spi_transaction_t trx = {0};
    trx.length = 16;
    trx.tx_data[0] = (reg_addr << MFRC_SPI_ADDR_SHIFT);
    trx.tx_data[1] = data;
    trx.flags = SPI_TRANS_USE_TXDATA;

    err = spi_device_polling_transmit(dev->spi_handle, &trx);

    if(err) {
        ESP_LOGE(MFRC_TAG, "Error during transmission {%u}", err);
    }

    return err;
}


static esp_err_t mfrc_write_register_data(MFRC_DEV dev, uint8_t reg_addr, uint8_t *data, uint8_t len) {

    esp_err_t err = ESP_OK;

    uint8_t send_buffer[64] = {0};
    spi_transaction_t trx = {0};

    if(len > 64) {
        return ESP_ERR_INVALID_ARG;
    }
    else {
        /** To make this a continuous transaction, copy into a temp buffer **
         * write address to byte 0 and copy in data afterwards 
         **/
        memset(&send_buffer[0], (reg_addr << MFRC_SPI_ADDR_SHIFT), sizeof(uint8_t));
        memcpy(&send_buffer[1], data, sizeof(uint8_t) * len);
    }

    trx.length = (len+1) * 8;
    trx.tx_buffer = send_buffer;
    err = spi_device_polling_transmit(dev->spi_handle, &trx);

    if(err) {
        ESP_LOGE(MFRC_TAG, "Error during transmission {%u}", err);
    }

    return err;
}


static esp_err_t mfrc_configure_spi(MFRC_DEV dev) {

    esp_err_t err = ESP_OK;

    if( 
        dev->select_pin< 1  ||
        dev->comms_mode != MFRC_SPI_COMMS_MODE ||
        dev->comms_bus > VSPI_HOST
    ) {
        ESP_LOGE(MFRC_TAG, "Error - configuring SPI 4-wire mode");
        err = ESP_ERR_INVALID_ARG;
    }
    else {
        spi_device_interface_config_t cfg = {0};
        cfg.clock_speed_hz = 100000;
        cfg.spics_io_num = dev->select_pin;
        cfg.queue_size = 10;
        cfg.mode = 0;

        err = spi_bus_add_device(dev->comms_bus, &cfg, (spi_device_handle_t *)&dev->spi_handle);
    }

    return err;
}


static esp_err_t mfrc_reset_device(MFRC_DEV dev) {

    if(dev->rst_en) {
        gpio_set_level(dev->rst_pin, 0);
        vTaskDelay(pdMS_TO_TICKS(1));
        gpio_set_level(dev->rst_pin, 1);
    }
    else {
        ESP_LOGE(MFRC_TAG, "Device Reset pin not configured, cannot reset device");
    }
    return ESP_OK;
}



static esp_err_t mfrc_read_from_address(MFRC_DEV dev, uint8_t address, uint8_t len, uint8_t *buffer) {

    esp_err_t err = ESP_OK;

    if(dev->comms_mode == MFRC_SPI_COMMS_MODE) {
        
    }
    else if (dev->comms_mode == MFRC_I2C_COMMS_MODE) {

    }
    else {
        err = ESP_ERR_INVALID_ARG;
    }

    return err;
}



static void mrfc_driver_task(void *args) {
 
   while(1) {
       vTaskDelay(pdMS_TO_TICKS(10));
   }
   /** here be dragons **/
}


/****** Global Functions *************/


MFRC_DEV mfrc_init(mfrc_init_t *init) {

    MFRC_DEV handle = NULL;
    gpio_config_t pin_config = {0};
    esp_err_t err = ESP_OK;
    uint8_t data = 0;


    if(init->comms_mode >= MFRC_COMMS_MODE_INVALID) {
        err = ESP_ERR_INVALID_ARG;
    }


    /** create handle **/
    if(!err) {
        handle = heap_caps_calloc(1, sizeof(MFRC522_Driver_t), MALLOC_CAP_DEFAULT);
        if(handle == NULL) {
            err = ESP_ERR_NO_MEM;
            ESP_LOGE(MFRC_TAG, "Error - unable to assign heap memory for handle [%u]", err);
        }
        else {
            if(handle->comms_mode == MFRC_SPI_COMMS_MODE) {
                handle->select_pin = init->sel;
            }
            else if(handle->comms_mode == MFRC_I2C_COMMS_MODE) {
                handle->data_pin = init->sda;
                handle->clock_pin = init->scl;
            }
            else {
                ESP_LOGE(MFRC_TAG, "Error - only I2C and SPI comms modes supported");
                err = ESP_ERR_INVALID_STATE;
            }

            if(!err) {
                handle->comms_bus = init->comms_bus;
                handle->comms_mode = init->comms_mode;
                handle->irq_pin = init->irq_pin;
                handle->rst_pin = init->rst_pin;
                mfrc_reset_registers(handle);
            }
        }
    }

    /** initialise the pins **/
    if(!err) {
    
        pin_config.intr_type = GPIO_INTR_DISABLE;
        pin_config.mode = GPIO_MODE_OUTPUT;
        pin_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
        pin_config.pull_up_en = GPIO_PULLUP_ENABLE;

        /** reset pin optional, init **/
        if(init->rst_pin > 0) {
            pin_config.pin_bit_mask = (1 << init->rst_pin);
            err = gpio_config(&pin_config);
            if(err) {
                ESP_LOGE(MFRC_TAG, "Error configuring reset pin {%u}", err);
            }
            else {
                gpio_set_level(init->rst_pin, 1);
                handle->rst_en = true;
            }
        }

        if(!err && init->irq_pin) {
            pin_config.pin_bit_mask = (1 << init->irq_pin);
            pin_config.intr_type = GPIO_INTR_NEGEDGE;
            err = gpio_config(&pin_config);
            if(err) {
                ESP_LOGE(MFRC_TAG, "Error configuring irq pin {%u}", err);
            }
            else {
                err = gpio_isr_handler_add(init->irq_pin, mfrc_irq_function, handle);
                if(err) {
                    ESP_LOGE(MFRC_TAG, "Error adding isr handler {%u}", err);
                }
                else {
                    handle->irq_en = true;
                }
            }
        }
    }
    

    /** create task **/
    if(!err) {
        if(xTaskCreate(mrfc_driver_task, "mrfc_driver_task", 5012, handle, 3, &handle->task) != pdTRUE) {
            ESP_LOGE(MFRC_TAG, "Error creating driver task!");
            err = ESP_ERR_NO_MEM;
        }
    }

    /** add the device to the spi bus **/
    if(!err && handle->comms_mode == MFRC_SPI_COMMS_MODE) {
        err = mfrc_configure_spi(handle);
    }

    /** test register read **/
    if(!err) {
        mfrc_read_register_byte(handle, 1, &data);
        mfrc_read_register_byte(handle, 1, &data);
        ESP_LOGI(MFRC_TAG, "Test register read Addr 0x01 : 0x%02x", data);
        mfrc_read_register_byte(handle, 2, &data);
        ESP_LOGI(MFRC_TAG, "Test register read Addr 0x02 : 0x%02x", data);
        mfrc_read_register_byte(handle, 3, &data);
        ESP_LOGI(MFRC_TAG, "Test register read Addr 0x03 : 0x%02x", data);
    }


    /** folling arduino driver's method - set tx timer to auto **/
    if(!err) {
        handle->registers.tmr_mode_reg.bits.tmr_auto = 1;
        err = mfrc_write_register_byte(handle, MFRC_REGADDR_TMR_MODE, handle->registers.tmr_mode_reg.regval);
    }
    if(!err) {
        handle->registers.tmr_prescaler_lsb = 0xA9;
        err = mfrc_write_register_byte(handle, MFRC_REGADDR_TMR_PRESCALE, handle->registers.tmr_prescaler_lsb);
    }
    if(!err) {
        handle->registers.tmr_reload_msb = 0x03;
        err = mfrc_write_register_byte(handle, MFRC_REGADDR_TCNTR_RELOAD_MSB, handle->registers.tmr_reload_msb);
    }
    if(!err) {
        handle->registers.tmr_reload_lsb = 0xE8;
        err = mfrc_write_register_byte(handle, MFRC_REGADDR_TCNTR_RELOAD_LSB, handle->registers.tmr_reload_lsb);
    }
    if(!err) {
        handle->registers.tx_ask_reg.bits.force_100_ask = 1;
        err = mfrc_write_register_byte(handle, MFRC_REGADDR_TX_ASK, handle->registers.tx_ask_reg.regval);
    }
    if(!err) {
        handle->registers.mode_reg.bits.crc_preset = 1;
        err = mfrc_write_register_byte(handle, MFRC_REGADDR_MODE, handle->registers.mode_reg.regval);
    }
    if(!err) {
        handle->registers.txcontrol_reg.bits.tx1_rf_en = 1;
        handle->registers.txcontrol_reg.bits.tx2_rf_en = 1;
        err = mfrc_write_register_byte(handle, MFRC_REGADDR_TX_CONTROL, handle->registers.txcontrol_reg.regval);
    }

    if(err) {
        ESP_LOGE(MFRC_TAG, "Error setting registers {%u}", err);
    }




    if(err) {
        ESP_LOGE(MFRC_TAG, "Error initialising MFRC522 driver [%u]", err);
        if(handle) {
            heap_caps_free(handle);
        }
    } 
    else {
        ESP_LOGI(MFRC_TAG, "Succesfully started MFRC522 Driver");
        if(handle->rst_en) {
            mfrc_reset_device(handle);
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }

    return handle;
}




void mfrc_deinit(MFRC_DEV dev) {

    if(dev != NULL) {
        heap_caps_free(dev);
    }

    return;
}



// esp_err_t mfrc_set_interrupt_mask(MFRC_DEV dev, uint8_t *intr);

// esp_err_t mfrc_get_interrupt_mask(MFRC_DEV dev, uint8_t *intr);

// esp_err_t mfrc_enable_interrupt_pin(MFRC_DEV dev);

// esp_err_t mfrc_disable_interrupt_pin(MFRC_DEV dev);

// esp_err_t mfrc_soft_power_down(MFRC_DEV dev);



