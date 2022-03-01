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

#define ST_CMD_NOP           0x00;//Non operation
#define ST_CMD_SWRESET       0x01;//Soft Reset
#define ST_CMD_SLPIN         0x10;//Sleep ON
#define ST_CMD_SLPOUT        0x11;//Sleep OFF
#define ST_CMD_PTLON         0x12;//Partial Mode ON
#define ST_CMD_NORML         0x13;//Normal Display ON                                _CMD_NORON
#define ST_CMD_DINVOF        0x20;//Display Inversion OFF                            _CMD_INVOFF
#define ST_CMD_DINVON        0x21;//Display Inversion ON                            _CMD_INVON
#define ST_CMD_GAMMASET      0x26;//Gamma Set (0x01[1],0x02[2],0x04[3],0x08[4])
#define ST_CMD_DISPOFF       0x28;//Display OFF
#define ST_CMD_DISPON        0x29;//Display ON
#define ST_CMD_IDLEON        0x39;//Idle Mode ON                                    _CMD_IDLEON
#define ST_CMD_IDLEOF        0x38;//Idle Mode OFF                                    _CMD_IDLEOFF
#define ST_CMD_CLMADRS       0x2A;//Column Address Set                                _CMD_CASET
#define ST_CMD_PGEADRS       0x2B;//Page Address Set                                _CMD_PASET
#define ST_CMD_RAMWR         0x2C;//Memory Write
#define ST_CMD_RAMRD         0x2E;//Memory Read
#define ST_CMD_PARTAREA      0x30;//Partial Area                                    _CMD_PTLAR
#define ST_CMD_VSCLLDEF      0x33;//Vertical Scroll Definition                        _CMD_VSCRLLD
#define ST_CMD_TEFXLON       0x35;//Tearing Effect Line ON                            _CMD_TEFXON
#define ST_CMD_TEFXLOF       0x34;//Tearing Effect Line OFF                        _CMD_TEFXOFF
#define ST_CMD_MADCTL        0x36;//Memory Access Control
#define ST_CMD_VSSTADRS      0x37;//Vertical Scrolling Start address                _CMD_VSCLLSA
#define ST_CMD_PIXFMT        0x3A;//Interface Pixel Format
#define ST_CMD_FRMCTR1       0xB1;//Frame Rate Control (In normal mode/Full colors)
#define ST_CMD_FRMCTR2       0xB2;//Frame Rate Control(In Idle mode/8-colors)
#define ST_CMD_FRMCTR3       0xB3;//Frame Rate Control(In Partial mode/full colors)
#define ST_CMD_DINVCTR       0xB4;//Display Inversion Control                        _CMD_INVCTR
#define ST_CMD_DFUNCTR       0xB6;//Display Fuction set 5
#define ST_CMD_PWCTR1        0xC0;//Power_Control1
#define ST_CMD_PWCTR2        0xC1;//Power_Control2
#define ST_CMD_PWCTR3        0xC2;//Power_Control3
#define ST_CMD_PWCTR4        0xC3;//Power_Control4
#define ST_CMD_PWCTR5        0xC4;//Power_Control5
#define ST_CMD_PWCTR6        0xFC;//Power_Control6
#define ST_CMD_VCOMCTR1      0xC5;//VCOM_Control 1                                    _CMD_VMCTR1
#define ST_CMD_VCOMCTR2      0xC7;//VCOM_Control 2                                    _CMD_VMCTR2
#define ST_CMD_PGAMMAC       0xE0;//Positive Gamma Correction Setting                _CMD_POSGAMUT
#define ST_CMD_NGAMMAC       0xE1;//Negative Gamma Correction Setting    

#define ST_CMD_TRX          0x0
#define ST_DATA_TRX         0x01

#define ST_MAC_CTRL_DATA    0b11000000

#define TFT_ST7735_TFTWIDTH        128
#define TFT_ST7735_TFTHEIGHT    160
#define TFT_ST7735_MEMSZ        20480

static const uint8_t TFT_ST7735_FRMCTR1[3] = {0x00,0x06,0x03};
static const uint8_t TFT_ST7735_FRMCTR2[3] = {0x01,0x2C,0x2D};
static const uint8_t TFT_ST7735_FRMCTR3[3] = {0x01,0x2C,0x2D};
static const uint8_t TFT_ST7735_PWCTR1[3]  = {0xA2,0x02,0x84};

static const uint8_t TFT_ST7735_PWCTR3[2]   = {0x0A,0x00};
static const uint8_t TFT_ST7735_PWCTR4[2]   = {0x8A,0x2A};
static const uint8_t TFT_ST7735_PWCTR5[2]   = {0x8A,0x2A};
static const uint8_t TFT_ST7735_VCOMCTR1[2] = {0x3C,0x38};

#define TFT_ST7735_GAMMASET 1

#if (TFT_ST7735_GAMMASET == 1)
    static const uint8_t pGammaSet[15]= {0x0F,0x31,0x2B,0x0C,0x0E,0x08,0x4E,0xF1,0x37,0x07,0x10,0x03,0x0E,0x09,0x00};
    static const uint8_t nGammaSet[15]= {0x00,0x0E,0x14,0x07,0x11,0x07,0x31,0xC1,0x48,0x08,0x0F,0x0C,0x31,0x36,0x0F};
#elif (TFT_ST7735_GAMMASET == 2)
    static const uint8_t pGammaSet[15]= {0x3F,0x21,0x12,0x22,0x1C,0x15,0x42,0xB7,0x2F,0x13,0x02,0x0A,0x01,0x00,0x00};
    static const uint8_t nGammaSet[15]= {0x09,0x18,0x2D,0x0D,0x13,0x15,0x40,0x48,0x53,0x0C,0x1D,0x25,0x2E,0x24,0x29};
#elif (TFT_ST7735_GAMMASET == 3)
    static const uint8_t pGammaSet[15]= {0x3F,0x26,0x23,0x30,0x28,0x10,0x55,0xB7,0x40,0x19,0x10,0x1E,0x02,0x01,0x00};
    static const uint8_t nGammaSet[15]= {0x09,0x18,0x2D,0x0D,0x13,0x15,0x40,0x48,0x53,0x0C,0x1D,0x25,0x2E,0x24,0x29};
#elif (TFT_ST7735_GAMMASET == 4)
    static const uint8_t pGammaSet[15]= {0x3F,0x25,0x1C,0x1E,0x20,0x12,0x2A,0x90,0x24,0x11,0x00,0x00,0x00,0x00,0x00};
    static const uint8_t nGammaSet[15]= {0x20,0x20,0x20,0x20,0x05,0x15,0x00,0xA7,0x3D,0x18,0x25,0x2A,0x2B,0x2B,0x3A};
#endif

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



typedef struct st7735_init
{
    /* data */
    gpio_num_t spi_clk;
    gpio_num_t spi_sda;
    gpio_num_t spi_cs;
    gpio_num_t rst_pin;
    gpio_num_t cmd_pin;
    uint8_t spi_bus;

} st7735_init_t;



typedef struct
{
    bool pwr_state;        
    gpio_num_t rst_pin;
    gpio_num_t cmd_pin;
    uint8_t spi_bus;
    spi_device_handle_t devhandle; /** < screen's spi handle **/

    uint32_t trx_count;
} screen_handle_t;



typedef struct
{
    bool mode;            /** < 0 - command write 1 - memory write **/
    uint8_t *send;        /** < pointer to send buffer **/
    uint8_t *recv;        /** < pointer to recv buffer **/
    uint16_t s_len;       /** < send length **/
    uint16_t r_len;       /** < recv length **/
    esp_err_t trx_status; /** < Transaction success status **/
} screen_transaction_t;


/********** Types **********************/

/******** Function Definitions *********/

screen_handle_t *init_screen(st7735_init_t *init);


#endif /* ST7735_SCREEN_H */
