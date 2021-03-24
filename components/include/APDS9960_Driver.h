/****************************************
* \file     
* \brief
* \date
* \author
****************************************/

#ifndef APDS9960_DRIVER_H
#define APDS9960_DRIVER_H

/********* Includes ********************/

#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


/********* Definitions *****************/

#define APDS_REGADDR_RAM_START      0x00
#define APDS_REGADDR_RAM_END        0x79
#define APDS_REGADDR_ENABLE         0x80
#define APDS_REGADDR_ADCTIME        0x81
#define APDS_REGADDR_WAITTIME       0x83
#define APDS_REGADDR_ALS_THR_LOW_LSB    0x84
#define APDS_REGADDR_ALS_THR_LOW_MSB    0x85
#define APDS_REGADDR_ALS_THR_HIGH_LSB   0x86
#define APDS_REGADDR_ALS_THR_HIGH_MSB   0x87
#define APDS_REGADDR_PRX_THR_LOW        0x89
#define APDS_REGADDR_PRX_THR_HIGH       0x8B
#define APDS_REGADDR_ISR_PERSIST_FLTR   0x8C
#define APDS_REGADDR_CONFIG_1           0x8D
#define APDS_REGADDR_PRX_PULSE_LEN      0x8E
#define APDS_REGADDR_GAIN_CTRL          0x8F
#define APDS_REGADDR_CONFIG_2           0x90
#define APDS_REGADDR_ID                 0x92
#define APDS_REGADDR_STATUS             0x93
#define APDS_REGADDR_CLRCHAN_DATA_LSB   0x94
#define APDS_REGADDR_CLRCHAN_DATA_MSB   0x95
#define APDS_REGADDR_REDCHAN_DATA_LSB   0x96
#define APDS_REGADDR_REDCHAN_DATA_MSB   0x97
#define APDS_REGADDR_GRNCHAN_DATA_LSB   0x98
#define APDS_REGADDR_GRNCHAN_DATA_MSB   0x99
#define APDS_REGADDR_BLUCHAN_DATA_LSB   0x9A
#define APDS_REGADDR_BLUCHAN_DATA_MSB   0x9B
#define APDS_REGADDR_PROX_DATA          0x9C
#define APDS_REGADDR_PROX_OFFSET_UR     0x9D
#define APDS_REGADDR_PROX_OFFSET_LD     0x9E
#define APDS_REGADDR_CONFIG_3           0x9F
#define APDS_REGADDR_GST_ENTR_THR       0xA0
#define APDS_REGADDR_GST_EXT_THR        0xA1
#define APDS_REGADDR_GST_CONFIG_1       0xA2
#define APDS_REGADDR_GST_CONFIG_2       0xA3
#define APDS_REGADDR_GST_OFFSET_U       0xA4
#define APDS_REGADDR_GST_OFFSET_D       0xA5
#define APDS_REGADDR_GST_OFFSET_L       0xA7
#define APDS_REGADDR_GST_OFFSET_R       0xA9
#define APDS_REGADDR_GST_PULSE_LEN      0xA6
#define APDS_REGADDR_GST_CONFIG_3       0xAA
#define APDS_REGADDR_GST_CONFIG_4       0xAB
#define APDS_REGADDR_GST_FIFO_LVL       0xAE
#define APDS_REGADDR_GST_STATUS         0xAF
#define APDS_REGADDR_FORCE_ISR          0xE4
#define APDS_REGADDR_PRX_ISR_CLR        0xE5
#define APDS_REGADDR_ALS_ISR_CLR        0xE6
#define APDS_REGADDR_NONGST_ISR_CLR     0xE7
#define APDS_REGADDR_GST_FIFO_U         0xFC
#define APDS_REGADDR_GST_FIFO_D         0xFD
#define APDS_REGADDR_GST_FIFO_L         0xFE
#define APDS_REGADDR_GST_FIFO_R         0xFF

/** ENABLE REG BITS **/
#define APDS_REGBIT_PWR_ON      (1 << 0)
#define APDS_REGBIT_ALS_EN      (1 << 1)
#define APDS_REGBIT_PRX_EN      (1 << 2)
#define APDS_REGBIT_WAIT_EN     (1 << 3)
#define APDS_REGBIT_ALS_INT_EN  (1 << 4)
#define APDS_REGBIT_PRX_INT_EN  (1 << 5)
#define APDS_REGBIT_GST_EN      (1 << 6)





/********** Types **********************/




typedef struct APDS_Init
{
    /* data */

    uint8_t i2c_bus;


} apds_init_t;



typedef struct APDS9960_Driver
{
    /* data */

    uint8_t bus;
    uint8_t addr;

} adps_handle_t;



typedef adps_handle_t * APDS_DEV;


/******** Function Definitions *********/


APDS_DEV apds_init(apds_init_t *init);





#endif /* APDS9960_DRIVER_H */
