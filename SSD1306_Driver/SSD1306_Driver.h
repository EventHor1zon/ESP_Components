/****************************************
* \file         SSD1306_Driver.h
* \brief        Header file for the SSd1306 driver
* \date         Jan 2021
* \author       RJAM
* \credit       Mostly copied from https://github.com/yanbe/ssd1306-esp-idf-i2c.git
****************************************/

#ifndef SSD1306_DRIVER_H
#define SSD1306_DRIVER_H

/********* Includes ********************/

#include "esp_err.h"
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#ifdef CONFIG_USE_PERIPH_MANAGER

#include "CommandAPI.h"
#define screen_param_len 6

const parameter_t ssd1306_param_map[screen_param_len];
const peripheral_t ssd1306_peripheral_template;


#endif

// SLA (0x3C) + WRITE_MODE (0x00) =  0x78 (0b01111000)
#define OLED_I2C_ADDRESS   0x3C

// Control byte
#define OLED_CONTROL_BYTE_CMD_SINGLE    0x80
#define OLED_CONTROL_BYTE_CMD_STREAM    0x00
#define OLED_CONTROL_BYTE_DATA_STREAM   0x40

// Fundamental commands (pg.28)
#define OLED_CMD_SET_CONTRAST           0x81    // follow with 0x7F
#define OLED_CMD_DISPLAY_RAM            0xA4
#define OLED_CMD_DISPLAY_ALLON          0xA5
#define OLED_CMD_DISPLAY_NORMAL         0xA6
#define OLED_CMD_DISPLAY_INVERTED       0xA7
#define OLED_CMD_DISPLAY_OFF            0xAE
#define OLED_CMD_DISPLAY_ON             0xAF

// Addressing Command Table (pg.30)
#define OLED_CMD_SET_MEMORY_ADDR_MODE   0x20    // follow with 0x00 = HORZ mode = Behave like a KS108 graphic LCD
#define OLED_CMD_SET_COLUMN_RANGE       0x21    // can be used only in HORZ/VERT mode - follow with 0x00 and 0x7F = COL127
#define OLED_CMD_SET_PAGE_RANGE         0x22    // can be used only in HORZ/VERT mode - follow with 0x00 and 0x07 = PAGE7

// Hardware Config (pg.31)
#define OLED_CMD_SET_DISPLAY_START_LINE 0x40
#define OLED_CMD_SET_SEGMENT_REMAP      0xA1    
#define OLED_CMD_SET_MUX_RATIO          0xA8    // follow with 0x3F = 64 MUX
#define OLED_CMD_SET_COM_SCAN_MODE      0xC8    
#define OLED_CMD_SET_DISPLAY_OFFSET     0xD3    // follow with 0x00
#define OLED_CMD_SET_COM_PIN_MAP        0xDA    // follow with 0x12
#define OLED_CMD_NOP                    0xE3    // NOP

// Timing and Driving Scheme (pg.32)
#define OLED_CMD_SET_DISPLAY_CLK_DIV    0xD5    // follow with 0x80
#define OLED_CMD_SET_PRECHARGE          0xD9    // follow with 0xF1
#define OLED_CMD_SET_VCOMH_DESELCT      0xDB    // follow with 0x30

// Charge Pump (pg.62)
#define OLED_CMD_SET_CHARGE_PUMP        0x8D    // follow with 0x14
#define OLED_CMD_SET_CHARGE_PUMP_EN     0x14

#define SSD1306_SEG_SELECT              0xB0
#define SSD_MAX_STRLEN                    64
#define SSD1306_MAX_LINES                  8






/********* Definitions *****************/

typedef enum {
    SCROLL_DIR_UP,
    SCROLL_DIR_DOWN,
    SCROLL_DIR_LEFT,
    SCROLL_DIR_RIGHT,
    SCROLL_DIR_INVALID,
} screen_scroll_dir_t;

typedef struct ssd1306_init
{
    /* data */
    uint8_t i2c_bus;
    uint8_t i2c_addr;
    gpio_num_t rst;
} ssd1306_init_t;



typedef struct ssd1306_handle
{
    /* data */
    uint8_t bus;
    uint8_t dev_addr;
    uint16_t pixel_width;
    uint16_t pixel_height;
    uint8_t current_page;
    char text[SSD_MAX_STRLEN];
    uint16_t text_len;
    uint8_t orientation; /**< 0, one way, 1> other way **/
    bool active;
    uint8_t scroll_speed;
    bool is_scrolling;
    uint8_t contrast;
    gpio_num_t rst;
    TimerHandle_t timer;
    TaskHandle_t task_handle;
} ssd1306_handle_t;


/********** Types **********************/

/******** Function Definitions *********/

esp_err_t ssd1306_set_contrast(ssd1306_handle_t *screen, uint8_t *val);

esp_err_t ssd1306_get_contrast(ssd1306_handle_t *screen, uint8_t *val);

esp_err_t ssd1306_scroll_screen_dir(ssd1306_handle_t *screen, screen_scroll_dir_t *dir);

esp_err_t ssd1306_set_display_orientation(ssd1306_handle_t *screen, uint8_t *val);

esp_err_t ssd1306_get_orientation(ssd1306_handle_t *screen, uint8_t *val);

esp_err_t ssd1306_set_text(ssd1306_handle_t *screen, char *text);

esp_err_t ssd1306_set_line(ssd1306_handle_t *screen, uint8_t *line);

esp_err_t ssd1306_get_line(ssd1306_handle_t *screen, uint8_t *line);

esp_err_t ssd1306_clear_current_line(ssd1306_handle_t *screen);

esp_err_t ssd1306_write_current_line(ssd1306_handle_t *screen);

esp_err_t ssd1306_clear_screen(ssd1306_handle_t *screen);

esp_err_t ssd1306_write_text(ssd1306_handle_t *screen);

#ifdef CONFIG_DRIVERS_USE_HEAP
    ssd1306_handle_t *ssd1306_init(ssd1306_init_t *init);
#else
    ssd1306_handle_t *ssd1306_init(ssd1306_handle_t *handle, ssd1306_init_t *init);
#endif


#endif /* SSD1306_DRIVER_H */
