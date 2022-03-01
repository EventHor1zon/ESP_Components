/****************************************
* \file     VEML6070_Driver.h
* \brief    Header file
* \date     Jun 21
* \author   RJAM
****************************************/

#ifndef VEML6070_DRIVER_H
#define VEML6070_DRIVER_H

/********* Includes ********************/

#include "esp_types.h"
#include "genericCommsDriver.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/********* Definitions *****************/

/********** Types **********************/

#define VEML_MAX_I2C_FREQ       400000

#define VEML_REGISTER_COMMAND   0x70
#define VEML_REGADDR_DATA_LSB   0x71
#define VEML_REGADDR_DATA_MSB   0x73

#define VEML_REGBIT_SHUTDOWN    (1)
#define VEML_REGBIT_INTGR_TIME  (1 << 2)
#define VEML_REGBIT_INTR_THRESH (1 << 4)
#define VEML_REGBIT_INTR_EN     (1 << 5)


typedef enum {

    VEML_STATE_IDLE,
    VEML_STATE_WAITING_RESPONSE,
    VEML_STATE_READING,

} veml_state_t;


typedef struct  veml_init
{
    /* data */
    uint8_t i2c_bus;
    gpio_num_t ack_pin;
} veml_init_t;



typedef struct VEML6070_Driver
{
    /* data */
    uint8_t bus;
    gpio_num_t ack;
    bool intr_en;
    TaskHandle_t task;

    veml_state_t state;

    uint16_t last_value;

} veml_driver_t;


typedef veml_driver_t * VEML_DEV;


/******** Function Definitions *********/


VEML_DEV veml_init(veml_init_t *init);


#endif /* VEML6070_DRIVER_H */
