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