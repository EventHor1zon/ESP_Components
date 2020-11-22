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
#define ST_SCREEN_BUFFER_BIT_WIDTH 162
#define ST_SCREEN_BUFFER_BIT_HEIGHT 132

#define ST_SYSCMD_NOOP 0
#define ST_SYSCMD_SW_RST 1
#define ST_SYSCMD_ID 0x04
#define ST_SYSCMD_STATUS 0x09
#define ST_SYSCMD_POWER 0x0A
#define ST_SYSCMD_RD_DISPLAY 0x0B
#define ST_SYSCMD_RD_PIXEL 0x0C
#define ST_SYSCMD_RD_IMG 0x0D
#define ST_SYSCMD_RD_SIGNAL 0x0E
#define ST_SYSCMD_SLEEP_IN 0x10
#define ST_SYSCMD_SLEEP_OUT 0x11
#define ST_SYSCMD_PARTIAL_ON 0x12
#define ST_SYSCMD_PARTIAL_OFF 0x13
#define ST_SYSCMD_INVERT_OFF 0x20
#define ST_SYSCMD_INVERT_ON 0x21
#define ST_SYSCMD_GAMMA_SEL 0x26
#define ST_SYSCMD_DISPLAY_OFF 0x28
#define ST_SYSCMD_DISPLAY_ON 0x29
#define ST_SYSCMD_COL_ADDR_SET 0x2A
#define ST_SYSCMD_ROW_ADDR_SET 0x2B
#define ST_SYSCMD_MEM_WRITE 0x2C
#define ST_SYSCMD_MEM_READ 0x2E

#define ST_SYSCMD_PARTIAL_START 0x30
#define ST_SYSCMD_TEAR_OFF 0x34
#define ST_SYSCMD_TEAR_ON 0x35
#define ST_SYSCMD_MEMACCESS 0x36
#define ST_SYSCMD_IDLE_ON 0x38
#define ST_SYSCMD_IDLE_OFF 0x39
#define ST_SYSCMD_PIXL_FMT 0x3A
#define ST_SYSCMD_RD_ID1 0xDA
#define ST_SYSCMD_RD_ID2 0xDB
#define ST_SYSCMD_RD_ID3 0xDC

typedef struct
{
    uint8_t mnf_id; /**< manufacturer's id (0x5c) **/
    uint8_t ver_id; /**< version id "" ***/
    uint8_t drv_id; /**< driver id "" **/
    bool bston;     /**< booster voltage status **/
    bool my;        /**< row address order **/
    bool mx;        /**< col address order **/
    bool mv;        /**< row/col exchange **/
    bool ml;        /**< scan address order **/
    bool rgb;       /**< rgb order **/
    bool mh;        /**< horizontal order **/
    uint8_t ifpf;   /**< interface color puxel format def **/
    bool idmon;     /**< idle mode **/
    bool ptlon;     /**< paartial mode **/
    bool slpout;    /**< sleep out/in **/
    bool noron;     /**< normal mode **/
    bool invon;     /**< inverted mode **/
    bool dison;     /**< display on/off **/
    uint8_t gcs;    /**< gamma curve selection **/
    bool telon;     /**< tearing effect on/off **/
    bool telom;     /**< tearing effect mode **/
} screen_settings_t;

typedef struct
{
    uint8_t spi_bus;
    spi_device_handle_t *devhandle; /** < screen's spi handle **/
    uint32_t trx_count;
    bool pwr_state;
} screen_handle_t;

typedef struct
{
    bool mode;            /** < 0 - command write 1 - memory write **/
    uint8_t addr;         /** < address **/
    uint8_t *send;        /** < pointer to send buffer **/
    uint8_t *recv;        /** < pointer to recv buffer **/
    uint16_t s_len;       /** < send length **/
    uint16_t r_len;       /** < recv length **/
    esp_err_t trx_status; /** < Transaction success status **/
} screen_transaction_t;
/********** Types **********************/

/******** Function Definitions *********/

screen_handle_t *init_screen(uint8_t spi_bus, gpio_num_t cs);

esp_err_t screen_write(screen_handle_t *screen, screen_transaction_t *trx);
esp_err_t screen_write_data(screen_transaction_t *trx);
esp_err_t screen_readwrite(screen_handle_t *screen, screen_transaction_t *trx);
#endif /* ST7735_SCREEN_H */
