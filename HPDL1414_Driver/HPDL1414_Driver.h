/****************************************
* \file     HPDL1414_Driver.h
* \brief    Header file
* \date     Dev 2020
* \author   RJAM
****************************************/

#ifndef HDPL_DRIVER_H
#define HDPL_DRIVER_H

/********* Includes ********************/

#include "esp_err.h"
#include "driver/gpio.h"
#include "sdkconfig.h"


/********* Definitions *****************/

#define DATA_ROW_MASK_0 (1 << 0)
#define DATA_ROW_MASK_1 (1 << 1)
#define DATA_ROW_MASK_2 (1 << 2)
#define DATA_ROW_MASK_3 (1 << 3)
#define DATA_COL_MASK_0 (1 << 0)
#define DATA_COL_MASK_1 (1 << 1)
#define DATA_COL_MASK_2 (1 << 2)

#define ADDRESS_LED_0 0
#define ADDRESS_LED_1 1
#define ADDRESS_LED_2 2
#define ADDRESS_LED_3 3

#define HPDL_NUMLEDS 4
#define HPDL_MAX_LED_INDEX 3

/********** Types **********************/

const char *charmap[4][16];

typedef struct HPDL1414_init
{
    gpio_num_t D0;
    gpio_num_t D1;
    gpio_num_t D2;
    gpio_num_t D3;
    gpio_num_t D4;
    gpio_num_t D5;
    gpio_num_t D6;
    gpio_num_t A0;
    gpio_num_t A1;
    gpio_num_t write;

} hpdl_initdata_t;

typedef struct {
    gpio_num_t DR0;
    gpio_num_t DR1;
    gpio_num_t DR2;
    gpio_num_t DR3;
} rowdata_pins_t;

typedef struct {
    gpio_num_t DC4;
    gpio_num_t DC5;
    gpio_num_t DC6;
} coldata_pins_t;

typedef struct 
{
    gpio_num_t A0;
    gpio_num_t A1; 
} address_pins_t;


typedef struct HPDL1414_Driver
{
    rowdata_pins_t rowpins;
    coldata_pins_t colpins;
    address_pins_t adrpins;
    gpio_num_t write_pin;

    uint8_t current_led;
} hpdl_driver_t;


/******** Function Definitions *********/


/** \brief hpdl_init - initialises the driver
 *  \param - pointer to a hpdl_initdata_t struct
 * 
 *  \return hpdl_driver_t driver handle or NULL
 **/
#ifdef CONFIG_DRIVERS_USE_HEAP
hpdl_driver_t *hpdl_init(hpdl_initdata_t *init);
#else
hpdl_driver_t *hpdl_init(hpdl_driver_t *dev, hpdl_initdata_t *init);
#endif


/** \brief hpdl_set_char - send a single char data
 *  \param dev - pointer to device handle
 *  \param c - pointer to an ui8 format character
 *  \return ESP_OK or error
 **/
esp_err_t hpdl_set_char(hpdl_driver_t *dev, uint8_t *c);


/** \brief hpdl_set_led - set the led to be written next
 *  \param dev - pointer to device handle 
 *  \param led - led index, 0-3
 * 
 *  \return ESP_OK
 **/
esp_err_t hpdl_set_led(hpdl_driver_t *dev, uint8_t *led);


/** \brief hpdl_set chars - set all 4 chars 
 *  \param dev - pointer to device handle 
 *  \param led - ui32_t with a character in each byte
 * 
 *  \return ESP_OK
 **/
esp_err_t hpdl_set_chars(hpdl_driver_t *dev, uint32_t *var);

#endif /* HDPL_DRIVER_H */
