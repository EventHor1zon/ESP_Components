/****************************************
* \file     PeripheralManager.h
* \brief    Header file for the Peripheral manager.c
*
*   PeripheralManager is one of the main components of ESP_Home system
*   It acts as a bridge btween the connected peripherals and the API manager
*   It creates the main command queue and the control task is blocked waiting for 
*   an incomming command from the API manager
*
*
*
* \date     Sept 2020
* \author   RJAM
****************************************/

#ifndef PERIPHERAL_MANAGER_H
#define PERIPHERAL_MANAGER_H

/********* Includes ********************/
#include "esp_types.h"
#include "esp_err.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_intr_alloc.h"
#include "CommandAPI.h"

extern QueueHandle_t command_queue; /** < Handle for the main command queue **/
extern QueueHandle_t respondq; /** < Handle for the main respond queue **/

/********* Definitions *****************/


#define PM_CONFIG_GPIO_ISR_PRIO     ESP_INTR_FLAG_LEVEL3
#define PM_I2C_BUS_PRIMARY          I2C_NUM_0

#define PM_MAX_PERIPHERALS          10
#define PM_MAX_QUEUE_LEN            10
#define PM_QUEUE_SEND_TIMEOUT       1000
#define PM_PIFO_NUM_TYPE            0xFF

#define PM_ERR_INVALID_ID           0x80
#define PM_ERR_INVALID_ARG          0x81
#define PM_ERR_INVALID_TYPE         0x82
#define PM_ERR_INVALID_CMD          0x83
#define PM_ERR_INVALID_PERIPH_ID    0x84
#define PM_ERR_INVALID_PARAM_ID     0x85
#define PM_ERR_INVALID_CMD_ARGS     0x86
#define PM_ERR_INVALID_METHOD       0x87
#define PM_ERR_SET_OUT_OF_BOUNDS    0x88

#define PM_ERR_GET_FAILED_BASE      0x90
#define PM_ERR_SET_FAILED_BASE      0xA0
#define PM_ERR_ACT_FAILED_BASE      0xB0

typedef struct peripheral_summary
{
    peripheral_t *peripherals;
    uint8_t perip_num;
} peripheral_summary_t;

typedef struct pm_init {
    bool init_spi;
    bool init_i2c;
    bool init_uart;
    bool init_gpio_isr;
    uint8_t uart_channel;
    uint8_t spi_channel;
    uint8_t i2c_channel;
    uint32_t i2c_speed;
    int16_t sda;
    int16_t scl;
    int16_t mosi;
    int16_t miso;
    int16_t sck;
    int16_t uart_tx;
    int16_t uart_rx;
    int16_t uart_cts;
    int16_t uart_rts;
    uint32_t uart_baud;
} pm_init_t;

/********** Types **********************/

/******** Function Definitions *********/

/** peripheral_manager_init()
 *  
 *  initialises the peripheral manager
 * 
 *  \return ESP_OK or error 
 **/
esp_err_t peripheral_manager_init(pm_init_t *init_data);


/** pm_add_new_peripheral();
 *  \brief register a new peripheral with the PM
 *  \param template - a pointer to the Peripheral_t template
 *  \param id - the peripheral ID 
 *  \param handle - a pointer to the periph handle
 *  \return ESP_OK or error
 **/
esp_err_t pm_add_new_peripheral(peripheral_t *template, uint8_t id, void *handle);

/** pm_handle_parameter_request
 *  \brief - returns a cmd_rsp+t response to a command request
 *  \param - pointer to a request to process
 **/
cmd_rsp_t pm_handle_parameter_request(cmd_request_t *request);

/**
 *  \brief: Returns pointer to a peripheral from the peripheral id
 *  \param periph_id - the peripheral id to look up
 *  \return peripheral_t * or NULL
 * 
 **/
peripheral_t *get_peripheral_from_id(uint32_t periph_id);


/**
 *  \brief: returns a pointer to a parameter_t struct from given peripheral and parameter id
 *  \param periph - pointer to the peripheral to search
 *  \param param_id - the parameter id to look up
 *  \return parameter pointer or NULL
 **/
parameter_t *get_parameter_from_id(peripheral_t *periph, uint8_t param_id);



#ifdef CONFIG_USE_EVENTS

/** \brief returns the peripheral managers' event loop
 *  \return handle to event loop
 */
esp_event_loop_handle_t pm_get_event_loop(void);

/** DEBUG: remove **/
int pm_test_print(void *args);

/** \brief adds an event to the linked list of events   
 *  \param map_init a pointer to an event_map_init_t struct
 *  \return ESP_OK or ERROR
 */
esp_err_t add_event_map(event_map_init_t *map_init);

#endif


#endif /* PERIPHERAL_MANAGER_H */
