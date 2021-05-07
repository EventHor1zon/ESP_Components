/***************************************
* \file     ST7735_Screen.c
* \brief    A driver for a 1.8" TFT Screen
*           Developed for the 4-wire SPI           
* \date     Nov 2020
* \author   RJAM
****************************************/

/********* Includes *******************/
#include <string.h>
#include "ST7735_Screen.h"

#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"


const char *SCRN_TAG="ST7735 Driver";

/****** Function Prototypes ***********/

/************ ISR *********************/

/****** Private Data ******************/

/** almost definitely use this **/
static uint8_t screenbuffer[TFT_ST7735_MEMSZ] = {0};

static void toggle_rst_pin(screen_handle_t *screen) {
    gpio_set_level(screen->rst_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(screen->rst_pin, 1);
    return;
}

static esp_err_t screen_write(screen_handle_t *screen, screen_transaction_t *trx)
{
    spi_transaction_t transaction = {0};

    transaction.length = trx->s_len;
    transaction.tx_buffer = trx->send;

    gpio_set_level(screen->cmd_pin, trx->mode);

    esp_err_t status = spi_device_polling_transmit(screen->devhandle, &transaction);
    if(status != ESP_OK) {
        ESP_LOGE("SPI", "Error in write [%u]", status);
    }
    return status;
}

static esp_err_t screen_writeread(screen_handle_t *screen, screen_transaction_t *trx)
{
    spi_transaction_t transaction = {0};
    transaction.cmd = trx->mode;
    transaction.length = trx->mode;
    transaction.rxlength = trx->r_len;
    transaction.rx_buffer = trx->recv;
    transaction.tx_buffer = trx->send;

    esp_err_t status = spi_device_polling_transmit(screen->devhandle, &transaction);
    if(status != ESP_OK) {
        ESP_LOGE("SPI", "Error in writeread [%u]", status);
    }

    return status;
}


static esp_err_t screen_set_all(screen_handle_t *screen, uint16_t col) {

    esp_err_t err = ESP_OK;
    uint8_t byte = 0;
    screen_transaction_t trx = {0};
    memset(screenbuffer, 0, (sizeof(uint8_t) * TFT_ST7735_TFTWIDTH * TFT_ST7735_TFTHEIGHT));
    
    byte = ST_CMD_CLMADRS;
    trx.mode = ST_CMD_TRX;
    trx.s_len = 1;
    trx.send = &byte;
    err = screen_write(screen, &trx);
    
    trx.mode = ST_DATA_TRX;
    trx.s_len = 4;
    trx.send = screenbuffer;
    err = screen_write(screen, &trx);

    byte = ST_CMD_PGEADRS;
    trx.mode = ST_CMD_TRX;
    trx.s_len = 1;
    trx.send = &byte;
    err = screen_write(screen, &trx);

    trx.mode = ST_DATA_TRX;
    trx.s_len = 4;
    trx.send = screenbuffer;
    err = screen_write(screen, &trx);

    byte = ST_CMD_RAMWR;
    trx.mode = ST_CMD_TRX;
    trx.s_len = 1;
    trx.send = &byte;
    err = screen_write(screen, &trx);

    for(uint16_t i=0; i<TFT_ST7735_MEMSZ; i+=2) {
        screenbuffer[i] = (uint8_t )(col >> 8);
        screenbuffer[i+1] = (uint8_t )(col);
    }

    trx.mode = ST_DATA_TRX;
    trx.s_len = TFT_ST7735_MEMSZ;
    trx.send = screenbuffer;
    err = screen_write(screen, &trx);

    return err;

}



static esp_err_t screen_init_commands(screen_handle_t *screen) {

    /** initialise some variables ! */
    esp_err_t err = ESP_OK;
    screen_transaction_t trx = {0};

    uint8_t byte = 0;
    uint8_t data[16] = {0};
    /** hardware reset **/

    toggle_rst_pin(screen);


    /** software reset **/
    byte = ST_CMD_SWRESET;
    trx.mode = ST_CMD_TRX;
    trx.s_len = 1;
    trx.send = &byte;
    err = screen_write(screen, &trx);
    ESP_LOGI(SCRN_TAG, "Resetting Screen [%u]", err);
    vTaskDelay(pdMS_TO_TICKS(500));

    /** exit sleep **/
    if(!err) {
        ESP_LOGI(SCRN_TAG, "Waking Screen");
        byte = ST_CMD_SLPOUT;
        err = screen_write(screen, &trx);
    }

    /** write frame rate control **/
    if(!err) {
        ESP_LOGI(SCRN_TAG, "Frame control... [%u]", err);
        byte = ST_CMD_FRMCTR1;
        err = screen_write(screen, &trx);
    }

    if(!err) {
        data[0] = TFT_ST7735_FRMCTR1[0];
        data[1] = TFT_ST7735_FRMCTR1[1];
        data[2] = TFT_ST7735_FRMCTR1[2];
        trx.mode = ST_DATA_TRX;
        trx.s_len = 3;
        trx.send = data;
        err = screen_write(screen, &trx);
    }

    if(!err) {
        byte = ST_CMD_FRMCTR2;
        trx.mode = ST_CMD_TRX;
        trx.s_len = 1;
        trx.send = &byte;
        err = screen_write(screen, &trx);
    }

    if(!err) {
        data[0] = TFT_ST7735_FRMCTR2[0];
        data[1] = TFT_ST7735_FRMCTR2[1];
        data[2] = TFT_ST7735_FRMCTR2[2];
        trx.mode = ST_DATA_TRX;
        trx.s_len = 3;
        trx.send = data;
        err = screen_write(screen, &trx);
    }

    if(!err) {
        byte = ST_CMD_FRMCTR3;
        trx.mode = ST_CMD_TRX;
        trx.s_len = 1;
        trx.send = &byte;
        err = screen_write(screen, &trx);
    }

    if(!err) {
        data[0] = TFT_ST7735_FRMCTR3[0];
        data[1] = TFT_ST7735_FRMCTR3[1];
        data[2] = TFT_ST7735_FRMCTR3[2];
        trx.mode = ST_DATA_TRX;
        trx.s_len = 3;
        trx.send = data;
        err = screen_write(screen, &trx);
    }

    if(!err) {
        byte = ST_CMD_DINVCTR;
        trx.mode = ST_CMD_TRX;
        trx.s_len = 1;
        trx.send = &byte;
        err = screen_write(screen, &trx);
    }

    if(!err) {
        byte = 0;
        trx.mode = ST_DATA_TRX;
        err = screen_write(screen, &trx);
    }

    /** power control **/
    if(!err) {
        ESP_LOGI(SCRN_TAG, "Power control [%u]", err);
        byte = ST_CMD_PWCTR1;
        trx.mode = ST_CMD_TRX;
        err = screen_write(screen, &trx);
    }
    
    if(!err) {
        data[0] = TFT_ST7735_PWCTR1[0];
        data[1] = TFT_ST7735_PWCTR1[1];
        data[2] = TFT_ST7735_PWCTR1[2];
        trx.mode = ST_DATA_TRX;
        trx.s_len = 3;
        trx.send = data;
        err = screen_write(screen, &trx);
    }

    if(!err) {
        byte = ST_CMD_PWCTR2;
        trx.mode = ST_CMD_TRX;
        trx.s_len = 1;
        trx.send = &byte;
        err = screen_write(screen, &trx);
    }

    if(!err) {
        byte = 0xC5;
        trx.mode = ST_DATA_TRX;
        err = screen_write(screen, &trx);
    }

    if(!err) {
        byte = ST_CMD_PWCTR3;
        trx.mode = ST_CMD_TRX;
        err = screen_write(screen, &trx);
    }
    
    if(!err) {
        data[0] = TFT_ST7735_PWCTR3[0];
        data[1] = TFT_ST7735_PWCTR3[1];
        trx.mode = ST_DATA_TRX;
        trx.s_len = 2;
        trx.send = data;
        err = screen_write(screen, &trx);
    }

    if(!err) {
        byte = ST_CMD_PWCTR4;
        trx.mode = ST_CMD_TRX;
        trx.s_len = 1;
        trx.send = &byte;
        err = screen_write(screen, &trx);
    }
    
    if(!err) {
        data[0] = TFT_ST7735_PWCTR4[0];
        data[1] = TFT_ST7735_PWCTR4[1];
        trx.mode = ST_DATA_TRX;
        trx.s_len = 2;
        trx.send = data;
        err = screen_write(screen, &trx);
    }

    if(!err) {
        byte = ST_CMD_PWCTR5;
        trx.mode = ST_CMD_TRX;
        trx.s_len = 1;
        trx.send = &byte;
        err = screen_write(screen, &trx);
    }
    
    if(!err) {
        data[0] = TFT_ST7735_PWCTR5[0];
        data[1] = TFT_ST7735_PWCTR5[1];
        trx.mode = ST_DATA_TRX;
        trx.s_len = 2;
        trx.send = data;
        err = screen_write(screen, &trx);
    }

    /** vcom control **/
    if(!err) {
        ESP_LOGI(SCRN_TAG, "VCOM control... [%u]", err);
        byte = ST_CMD_VCOMCTR1;
        trx.mode = ST_CMD_TRX;
        trx.s_len = 1;        
        trx.send = &byte;
        err = screen_write(screen, &trx);
    }
    
    if(!err) {
        data[0] = TFT_ST7735_VCOMCTR1[0];
        data[1] = TFT_ST7735_VCOMCTR1[1];
        trx.mode = ST_DATA_TRX;
        trx.s_len = 2;
        trx.send = data;
        err = screen_write(screen, &trx);
    }

    if(!err) {
        byte = ST_CMD_DINVOF;
        trx.mode = ST_CMD_TRX;
        trx.s_len = 1;
        trx.send = &byte;
        err = screen_write(screen, &trx);
    }   

    /** pixel format **/
    if(!err) {
        ESP_LOGI(SCRN_TAG, "Pixel format... [%u]", err);

        byte = ST_CMD_PIXFMT;
        err = screen_write(screen, &trx);
    }   

    if(!err) {
        byte = 0x05;
        trx.mode = ST_DATA_TRX;
        err = screen_write(screen, &trx);
    }   

     if(!err) {
        byte = ST_CMD_GAMMASET;
        trx.mode = ST_CMD_TRX;
        err = screen_write(screen, &trx);
    }   

    if(!err) {
        byte = 0x01;
        trx.mode = ST_DATA_TRX;
        err = screen_write(screen, &trx);
    }   

    /** Gamma corrections **/

    if(!err) {
        byte = ST_CMD_PGAMMAC;
        trx.mode = ST_CMD_TRX;
        err = screen_write(screen, &trx);
    }

    if(!err) {
        memset(data, 0, (sizeof(uint8_t) * 15));
        memcpy(data, pGammaSet, (sizeof(uint8_t) * 15));
        trx.s_len = 15;
        trx.send = data;
        trx.mode = ST_DATA_TRX;
        err = screen_write(screen, &trx);
    }

    if(!err) {
        byte = ST_CMD_NGAMMAC;
        trx.mode = ST_CMD_TRX;
        trx.s_len = 1;
        trx.send = &byte;
        err = screen_write(screen, &trx);
    }

    if(!err) {
        memset(data, 0, (sizeof(uint8_t) * 15));
        memcpy(data, nGammaSet, (sizeof(uint8_t) * 15));
        trx.s_len = 15;
        trx.send = data;
        trx.mode = ST_DATA_TRX;
        err = screen_write(screen, &trx);
    }

    /** exit idle mode **/
    if(!err) {
        byte = ST_CMD_IDLEOF;
        trx.s_len = 1;
        trx.send = &byte;
        trx.mode = ST_CMD_TRX;
        err = screen_write(screen, &trx);
    }

    /** set column/row address **/
    if(!err) {
        byte = ST_CMD_CLMADRS;
        err = screen_write(screen, &trx);
    }

    if(!err) {
        memset(data, 0, (sizeof(uint8_t) * 16));
        data[2] = (uint8_t )( TFT_ST7735_TFTWIDTH >> 8);
        data[3] = (uint8_t )(TFT_ST7735_TFTWIDTH); 
        trx.s_len = 4;
        trx.send = data;
        trx.mode = ST_DATA_TRX;
        err = screen_write(screen, &trx);
    }   

    if(!err) {
        byte = ST_CMD_PGEADRS;
        trx.s_len = 1;
        trx.send = &byte;
        trx.mode = ST_CMD_TRX;
        err = screen_write(screen, &trx);
    }

    if(!err) {
        memset(data, 0, (sizeof(uint8_t) * 16));
        data[2] = (uint8_t )( TFT_ST7735_TFTHEIGHT >> 8);
        data[3] = (uint8_t )(TFT_ST7735_TFTHEIGHT); 
        trx.s_len = 4;
        trx.send = data;
        trx.mode = ST_DATA_TRX;
        err = screen_write(screen, &trx);
    }

    ESP_LOGI(SCRN_TAG, "Screen on... [%u]", err);

    if(!err) {
        byte = ST_CMD_NORML;
        trx.s_len = 1;
        trx.send = &byte;
        trx.mode = ST_CMD_TRX;
        err = screen_write(screen, &trx);
    }

    if(!err) {
        byte = ST_CMD_DISPON;
        err = screen_write(screen, &trx);
    }

    vTaskDelay(pdMS_TO_TICKS(5));

    ESP_LOGI(SCRN_TAG, "Rotati on... [%u]", err);

    if(!err) {
        byte = ST_CMD_MADCTL;
        err = screen_write(screen, &trx);
    }

    if(!err) {
        byte = ST_MAC_CTRL_DATA;
        trx.mode = ST_DATA_TRX;
        err = screen_write(screen, &trx);
    }

    return err;
}


/****** Private Functions *************/


/****** Global Data *******************/


/****** Global Functions *************/



screen_handle_t *init_screen(st7735_init_t *init)
{
    esp_err_t err = ESP_OK;
    screen_handle_t *screen = NULL;
    spi_device_interface_config_t dev_cfg = {0};
    spi_device_handle_t handle = NULL;

    dev_cfg.command_bits = 1;
    dev_cfg.address_bits = 0;
    dev_cfg.clock_speed_hz = ST_SCREEN_CONFIG_SPI_SPEED;
    dev_cfg.mode = 0;
    dev_cfg.spics_io_num = init->spi_cs;
    dev_cfg.queue_size = 1;
    dev_cfg.flags = (SPI_DEVICE_3WIRE);

    err = spi_bus_add_device(init->spi_bus, &dev_cfg, &handle);

    if(err) {
        ESP_LOGE(SCRN_TAG, "Error intialising SPI device [%u]", err);
    }

    if(!err) {
        screen = (screen_handle_t *)heap_caps_calloc(1, sizeof(screen_handle_t), MALLOC_CAP_DEFAULT);

        if(screen == NULL) {
            ESP_LOGE(SCRN_TAG, "Error assigning memory for screen handle");
            err = ESP_ERR_NO_MEM;
        }
        else {
            screen->devhandle = handle;
            screen->pwr_state = 0;
            screen->spi_bus = init->spi_bus;
            screen->trx_count = 0;
        }
    }

    if(!err) {
        /** initialise the rst pin **/
        gpio_config_t pins = {0};
        pins.intr_type = GPIO_INTR_DISABLE;
        pins.mode = GPIO_MODE_OUTPUT;
        pins.pin_bit_mask = ((1 << init->rst_pin) | (1 << init->cmd_pin));
        pins.pull_down_en = GPIO_PULLDOWN_DISABLE;
        pins.pull_up_en = GPIO_PULLUP_DISABLE;

        err = gpio_config(&pins);
        if(err) {
            ESP_LOGE(SCRN_TAG, "Error setting up gpio! [%u]", err);
        }
        else {
            screen->rst_pin = init->rst_pin;
            gpio_set_level(screen->rst_pin, 1);
            gpio_set_level(screen->cmd_pin, 0);
        }
    }

    if(!err) {
        err = screen_init_commands(screen);
    }

    if(!err) {
        err = screen_set_all(screen, 0xFFE0);
    }


    if(err && screen != NULL) {
        heap_caps_free(screen);
        ESP_LOGE(SCRN_TAG, "Error starting ST7735 Driver :(");
    }
    else if (!err) {
        ESP_LOGI(SCRN_TAG, "Succesfully started ST7735 Driver :)");
    }

    return screen;
}

