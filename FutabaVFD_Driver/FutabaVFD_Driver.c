/***************************************
* \file         FutabaVFD_Driver.c
* \brief        Driver for the Futaba 8-MD-06INKM C2CIG type VFD display
*               Good luck finding a datasheet lolz
*               Code adapted from https://github.com/positronicsengineer/rpi-spi-vfd
*               Cheers fella
* \date         Aug 2021
* \author       RJAM
****************************************/

/********* Includes *******************/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_types.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "Utilities.h"
#include "genericCommsDriver.h"
#include "FutabaVFD_Driver.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"

/****** Global Data *******************/

const char *VFD_TAG = "VFD DRIVER";

/****** Function Prototypes ***********/

static void vfd_driver_task(void *args);

static esp_err_t vfd_init_commands(VFD_HANDLE handle);


/************ ISR *********************/

/****** Private Data ******************/

/****** Private Functions *************/


static esp_err_t sx_read_address_byte(VFD_HANDLE dev, uint8_t addr, uint8_t *byte) {

    esp_err_t err = ESP_OK;
    uint8_t cmd = (addr & ~(SX_READWRITE_BIT));
    spi_transaction_t trx = {0};

    trx.length = 16;
    trx.rxlength = 16;
    trx.tx_data[0] = cmd;
    trx.tx_data[1] = 0x00;
    trx.flags = (SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA);

    err = spi_device_transmit(dev->spi_handle, &trx);
    
    if(err != ESP_OK) {
        ESP_LOGE(VFD_TAG, "Error performing SPI transaction! [%u]", err);
    }
    else {
#ifdef SPI_DEBUG
        ESP_LOGI(LORA_TAG, "Read the following data: 0x%02x 0x%02x", trx.rx_data[0], trx.rx_data[1]);
#endif
        *byte = trx.rx_data[1];
    }

    return err;
}


static esp_err_t sx_write_address_byte(VFD_HANDLE dev, uint8_t addr, uint8_t byte) {

    esp_err_t err = ESP_OK;
    uint8_t cmd = (addr | SX_READWRITE_BIT);
    spi_transaction_t trx = {0};

    trx.length = 16;
    trx.tx_data[0] = cmd;
    trx.tx_data[1] = byte;
    trx.flags = (SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA);

    // err = spi_device_acquire_bus(dev->spi_handle, SX_SPI_TIMEOUT_DEFAULT);

    err = spi_device_transmit(dev->spi_handle, &trx);
    if(err != ESP_OK) {
        ESP_LOGE(VFD_TAG, "Error performing SPI transaction! [%u]", err);
    }
#ifdef SPI_DEBUG
    else {
        ESP_LOGI(VFD_TAG, "Read the following data: 0x%02x 0x%02x", trx.rx_data[0], trx.rx_data[1]);
    }
#endif

    return err;
}


static esp_err_t sx_spi_burst_write(VFD_HANDLE dev, uint8_t addr, uint8_t *data, uint8_t len) {

    uint8_t cmd = (addr | SX_READWRITE_BIT);
    spi_transaction_t trx = {0};
    uint8_t buffer[256] = {0};

    buffer[0] = cmd;
    memcpy(&buffer[1], data, sizeof(uint8_t) * len);

    trx.length = (8 + (len * 8));
    trx.tx_buffer = buffer;
    trx.flags = 0;

    return spi_device_transmit(dev->spi_handle, &trx);

}



/** this operation clears the masked bits, then Or's with data, then writes **/
static esp_err_t sx_spi_read_mod_write_mask(VFD_HANDLE dev, uint8_t addr, uint8_t data, uint8_t mask, uint8_t *storage) {
    esp_err_t err = ESP_OK;

    uint8_t reg = 0;
    err = sx_read_address_byte(dev, addr, &reg);

    /** conditional write - check the masked bits aren't already set... */
    if(!err) {
        if((reg & mask) != data) {
            reg &= ~(mask);
            reg |= data;
#ifdef SPI_DEBUG
            ESP_LOGI(VFD_TAG, "Writing value 0x%02x to register %02x", reg, addr);
#endif /** SPI_DEBUG **/
            err = sx_write_address_byte(dev, addr, reg);
        }
#ifdef SPI_DEBUG
        else {
            ESP_LOGI(VFD_TAG, "Not writing to register - curr: %02x, data: %02x", reg, data);
        }
#endif /** SPI_DEBUG **/
    }

    /** store the new register value **/
    if(!err && storage != NULL) {
        *storage = reg;
    }

    return err;
}



static void vfd_driver_task(void *args) {
 
    VFD_HANDLE handle = (vfd_handle_t *)args;

    while(1) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    /** here be dragons **/
}


static void vfd_reset(VFD_HANDLE handle) {
    gpio_set_level(handle->rst_pin, 0);
    vTaskDelay(2);
    gpio_set_level(handle->rst_pin, 1);
}


static esp_err_t vfd_init_commands(VFD_HANDLE handle) {

    esp_err_t err = ESP_OK;
    spi_transaction_t trx = {0};
    trx.addr = 0;
    trx.cmd = 0;
    trx.flags = SPI_TRANS_USE_TXDATA;
    trx.length = 16;   /** remember - this is in BITS! **/
    trx.rxlength = 0;
    trx.rx_buffer = NULL;
    trx.user = NULL;

    /** display timing **/
    trx.tx_data[0] = 0xE0;
    trx.tx_data[1] = 0x07;

    vfd_reset(handle);
    printf("spi write %u\n", err);

    vTaskDelay(pdMS_TO_TICKS(VFD_CONFIG_CMD_DELAY_MS));
    err = spi_device_polling_transmit(handle->spi_handle, &trx);
    if(err) {
        printf("something bad happened %u\n", err);
    }
    /** display dimming **/
    if(!err) {
        vTaskDelay(pdMS_TO_TICKS(VFD_CONFIG_CMD_DELAY_MS));
        printf("Dimming %u\n", err);
        trx.rxlength = 0; /** this keeps getting set to 16 weirdly? **/
        trx.tx_data[0] = 0xE4;
        trx.tx_data[1] = VFD_DEFAULT_DIMMING;
        err = spi_device_polling_transmit(handle->spi_handle, &trx);
    }

    if(!err) {
        printf("Light Up %u\n", err);
        vTaskDelay(pdMS_TO_TICKS(VFD_CONFIG_CMD_DELAY_MS));
        trx.rxlength = 0;
        trx.length = 8;
        trx.tx_data[0] = 0xE8;
        err = spi_device_polling_transmit(handle->spi_handle, &trx);
    }

    if(!err) {
        vTaskDelay(pdMS_TO_TICKS(VFD_CONFIG_CMD_DELAY_MS));
        printf("Sending 'A' %u\n", err);
        trx.rxlength = 0;
        trx.length = 16;
        trx.tx_data[0] = ((0b00100000|1));
        trx.tx_data[1] = 0x41;
        err = spi_device_polling_transmit(handle->spi_handle, &trx); 
    }

    return err;
}

/****** Global Functions *************/


VFD_HANDLE vfd_init(vfd_init_t *init) {

    esp_err_t err = ESP_OK;
    spi_device_handle_t dev_handle = NULL;
    VFD_HANDLE handle = NULL;
    gpio_config_t ioconf = {0};

    if(!gcd_spi_check_bus(init->spi_bus)) {
        ESP_LOGE(VFD_TAG, "Error invalid SPI bus");
        err = ESP_ERR_INVALID_ARG;
    }

    if(!err) {

        ioconf.intr_type = GPIO_INTR_DISABLE,
        ioconf.mode = GPIO_MODE_OUTPUT,
        ioconf.pin_bit_mask = (1 << init->rst_pin),
        ioconf.pull_down_en = GPIO_PULLDOWN_DISABLE,
        ioconf.pull_up_en = GPIO_PULLUP_ENABLE,

        err = gpio_config(&ioconf);
        if(err) {
            ESP_LOGE(VFD_TAG, "Error initialising reset pin {%u}", err);
        }
    }

    if(!err) {
        spi_device_interface_config_t dev = {0};
            dev.address_bits = 0;
            dev.clock_speed_hz = init->clock_speed;
            dev.dummy_bits = 0;
            dev.mode = 3;
            dev.duty_cycle_pos = 0;
            dev.spics_io_num = init->cs_pin;
            dev.flags = (SPI_DEVICE_BIT_LSBFIRST);
            dev.queue_size = 1;

        err = spi_bus_add_device(VSPI_HOST, &dev, &dev_handle);
        if(err) {
            ESP_LOGE(VFD_TAG, "Error adding device to bus! {%u}", err);
        }
    }

    if(!err) {
        handle = heap_caps_calloc(1, sizeof(vfd_handle_t), MALLOC_CAP_DEFAULT);
        if(handle == NULL) {
            ESP_LOGE(VFD_TAG, "Error assigning memory for handle!");
            err = ESP_ERR_NO_MEM;
        }
        else {
            handle->cs_pin = init->cs_pin;
            handle->rst_pin = init->rst_pin;
            handle->spi_bus = init->spi_bus;
            handle->spi_handle = dev_handle;
        }
    }
    TaskHandle_t th;

    if(!err && xTaskCreate(vfd_driver_task, "vfd_driver_task", VFD_CONFIG_STACK_DEPTH, handle, 3, &th) != pdTRUE) {
        ESP_LOGE(VFD_TAG, "Error creating task!");
        err = ESP_ERR_NO_MEM;
    }
    else {
        handle->task_handle = th;
    }
    if(!err) {
        vfd_init_commands(handle);
    }
    return handle;

}