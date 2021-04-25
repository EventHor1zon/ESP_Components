/***************************************
* \file     ST7735_Screen.c
* \brief    A driver for a 1.8" TFT Screen
*           Developed for the 4-wire SPI           
* \date     Nov 2020
* \author   RJAM
****************************************/

/********* Includes *******************/
// #include <>
#include "ST7735_Screen.h"

#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "esp_err.h"
#include "esp_heap_caps.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

/****** Function Prototypes ***********/

/************ ISR *********************/

/****** Private Data ******************/

/****** Private Functions *************/

/****** Global Data *******************/

/****** Global Functions *************/

screen_handle_t *init_screen(uint8_t spi_bus, gpio_num_t cs)
{
    screen_handle_t *screen;
    spi_device_interface_config_t dev_cfg = {0};
    spi_device_handle_t handle;

    dev_cfg.command_bits = 1;
    dev_cfg.address_bits = 0;
    dev_cfg.clock_speed_hz = ST_SCREEN_CONFIG_SPI_SPEED;
    dev_cfg.mode = 0;
    dev_cfg.spics_io_num = cs;
    dev_cfg.queue_size = 1;
    dev_cfg.flags = (SPI_DEVICE_3WIRE);

    spi_bus_add_device(spi_bus, &dev_cfg, &handle);

    screen = (screen_handle_t *)heap_caps_calloc(1, sizeof(screen_handle_t), MALLOC_CAP_8BIT);

    return handle;
}

esp_err_t screen_write(screen_handle_t *screen, screen_transaction_t *trx)
{
    spi_transaction_t transaction = {0};

    transaction.cmd = trx->mode;
    transaction.length = trx->s_len;
    transaction.tx_buffer = trx->send;

    esp_err_t status = spi_device_polling_transmit(screen->devhandle, &transaction);

    return status;
}

esp_err_t screen_readwrite(screen_handle_t *screen, screen_transaction_t *trx)
{
    spi_transaction_t transaction = {0};
    transaction.cmd = trx->mode;
    transaction.length = trx->mode;
    transaction.rxlength = trx->r_len;
    transaction.rx_buffer = trx->recv;
    transaction.tx_buffer = trx->send;

    esp_err_t status = ESP_OK;

    return status;
}