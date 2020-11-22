/****************************************
* \file     ST7735_Screen.h
* \brief    Header file for the ST7735_Screen driver
* \date     Nov 2020
* \author   RJAM
****************************************/

#ifndef ST7735_SCREEN_H
#define ST7735_SCREEN_H

/********* Includes ********************/
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

/********* Definitions *****************/

#define ST_SCREEN_CONFIG_SPI_SPEED 100000

typedef struct
{
    bool mode;                      /** < 0 - command write 1 - memory write **/
    uint8_t addr;                   /** < address **/
    uint8_t *send;                  /** < pointer to send buffer **/
    uint8_t *recv;                  /** < pointer to recv buffer **/
    uint16_t s_len;                 /** < send length **/
    uint16_t r_len;                 /** < recv length **/
    spi_device_handle_t *devhandle; /** < screen's spi handle **/
    esp_err_t trx_status;           /** < Transaction success status **/
} screen_transaction_t;
/********** Types **********************/

/******** Function Definitions *********/

esp_err_t
init_screen(uint8_t spi_bus, gpio_num_t cs);

esp_err_t screen_write(screen_transaction_t *trx);
esp_err_t screen_write_data(screen_transaction_t *trx);

#endif /* ST7735_SCREEN_H */
