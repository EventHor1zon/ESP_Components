/****************************************
* \file
* \brief
* \date
* \author
****************************************/

#ifndef HEADER_H
#define HEADER_H

/********* Includes ********************/


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_types.h"
#include "esp_err.h"
#include "esp_log.h"
#include "Utilities.h"
#include "genericCommsDriver.h"
#include "driver/gpio.h"

/********* Definitions *****************/


#define MRFC_REGADDR_COMMAND    0x01
#define MRFC_REGADDR_COMM_INTR_EN       0x02
#define MRFC_REGADDR_DEV_INTR_EN        0x03
#define MRFC_REGADDR_COMM_IRQ           0x04
#define MRFC_REGADDR_DIV_IRQ            0x05
#define MRFC_REGADDR_ERRORS             0x06
#define MRFC_REGADDR_STATUS_1           0x07
#define MRFC_REGADDR_STATUS_2           0x08
#define MRFC_REGADDR_FIFO_DATA          0x09
#define MRFC_REGADDR_FIFO_LEVEL         0x0A
#define MRFC_REGADDR_FIFO_WATERLEVEL    0x0B
#define MRFC_REGADDR_CONTROL            0x0C
#define MRFC_REGADDR_BIT_FRAMING        0x0D
#define MRFC_REGADDR_COLLISION_DETECT   0x0E
#define MRFC_REGADDR_MODE               0x11
#define MRFC_REGADDR_TX_MODE            0x12
#define MRFC_REGADDR_RX_MODE            0x13
#define MRFC_REGADDR_TX_CONTROL         0x14
#define MRFC_REGADDR_TX_ASK             0x15
#define MRFC_REGADDR_TX_SELECT          0x16
#define MRFC_REGADDR_RX_SELECT          0x17
#define MRFC_REGADDR_RX_THRESH          0x18
#define MRFC_REGADDR_DEMODULATOR        0x19
#define MRFC_REGADDR_MIFARE_TX          0x1C
#define MRFC_REGADDR_MIFARE_RX          0x1D
#define MRFC_REGADDR_SERIAL_SPEED       0x1F

#define MRFC_REGADDR_CRC_RESULT_MSB     0x21
#define MRFC_REGADDR_CRC_RESULT_LSB     0x22
#define MRFC_REGADDR_MOD_WIDTH          0x24
#define MRFC_REGADDR_RFC_GAIN           0x26
#define MRFC_REGADDR_GS_N               0x27
#define MRFC_REGADDR_CWF_SP             0x28
#define MRFC_REGADDR_MODG_SP            0x29
#define MRFC_REGADDR_TMR_MODE           0x2A
#define MRFC_REGADDR_TMR_PRESCALE       0x2B
#define MRFC_REGADDR_TCNTR_MSB          0x2C
#define MRFC_REGADDR_TCNTR_LSB          0x2D




/********** Types **********************/


typedef enum {
    MFRC_SPI_COMMS_MODE,
    MFRC_I2C_COMMS_MODE,
    MFRC_COMMS_MODE_INVALID,
} mfrc_comms_mode_t;


typedef struct MFRC_Init
{
    /* data */
    uint8_t comms_bus;
    mfrc_comms_mode_t comms_mode;
    gpio_num_t rst_pin;
    gpio_num_t irq_pin;


} mfrc_init_t;



typedef struct MFRC522_Driver
{
    /* data */
    uint8_t comms_bus;
    mfrc_comms_mode_t comms_mode;
    gpio_num_t rst_pin;
    gpio_num_t irq_pin;

    bool rst_en;
    bool irq_en;

    TaskHandle_t task;

} MFRC522_Driver_t;


typedef MFRC522_Driver_t * MFRC_DEV;

/******** Function Definitions *********/

#endif /* HEADER_H */
