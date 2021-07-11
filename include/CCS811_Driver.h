/****************************************
* \file
* \brief
* \date
* \author
****************************************/

#ifndef CCS811_DRIVER_H
#define CCS811_DRIVER_H

/********* Includes ********************/

#include "esp_err.h"
#include "esp_types.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "freertos/task.h"


/********* Definitions *****************/

#define CCS_MODE_IDLE   0
#define CCS_MODE_1SEC   1
#define CCS_MODE_10SEC  2
#define CCS_MODE_60SEC  3
#define CCS_MODE_CONST  4

#define CCS_DEVICE_ADDRESS_LOW 0x5A
#define CCS_DEVICE_ADDRESS_HIGH 0x5B

#define CCS_REG_ADDR_STATUS 0x00
#define CCS_REG_ADDR_MEAS_MODE 0x01
#define CCS_REG_ADDR_ALG_RES_DATA 0x02
#define CCS_REG_ADDR_RAW_DATA 0x03
#define CCS_REG_ADDR_ENV_DATA 0x05
#define CCS_REG_ADDR_THRESHOLD 0x10
#define CCS_REG_ADDR_BASELINE 0x11
#define CCS_REG_ADDR_HW_ID 0x20
#define CCS_REG_ADDR_HW_VERS 0x21
#define CCS_REG_ADDR_FW_BOOTVERS 0x23
#define CCS_REG_ADDR_FW_APPVERS 0x24
#define CCS_REG_ADDR_INTERNAL_STATUS 0xA0
#define CCS_REG_ADDR_ERROR_ID 0xE0
#define CCS_REG_ADDR_SW_RST 0xFF


#define CCS_STATUS_BIT_ERR (1)
#define CCS_STATUS_BIT_DRDY (1 << 3)
#define CCS_STATUS_BIT_APPVALID (1 << 4)
#define CCS_STATUS_BIT_APPVERIFY (1 << 5)
#define CCS_STATUS_BIT_APPERASE (1 << 6)
#define CCS_STATUS_BIT_FWMODE (1 << 7)

#define CCS_MODE_BIT_INTRTHRESH (1 << 2)
#define CCS_MODE_BIT_INTRDRDY (1 << 3)


#define CCS_DRIVE_MODE_OFFSET 4

#define CCS_EST_CO2_LEN 2
#define CCS_EST_TVOC_LEN 2
#define CCS_NEW_DATA_OFFSET 4
#define CCS_ERR_ID_OFFSET 5

const uint8_t rst_sequence[4] = {0x11, 0xE5, 0x72, 0x8A};

/********** Types **********************/

typedef enum {
    CCS_INTR_NONE = 0,
    CCS_INTR_THRESHOLD = 1,
    CCS_INTR_DATARDY = 2,
} ccs_intr_t;

typedef struct ccs811_Device {

    uint8_t comms_channel;
    uint8_t dev_addr;
    uint8_t mode;
    TaskHandle_t t_handle;

} ccs811_Device_t;


typedef struct ccs811_init
{
    uint8_t i2c_channel;
    uint8_t addr_pin_lvl;
    
    gpio_num_t gpio_nreset;
    gpio_num_t gpio_nwake;
    gpio_num_t gpio_intr;

    ccs_intr_t intr_type;

} ccs811_init_t;




/******** Function Definitions *********/

ccs811_Device_t *ccs811_init(ccs811_init_t);

esp_err_t ccs811_getMode(ccs811_Device_t *dev, uint8_t );

esp_err_t ccs811_setMode(ccs811_Device_t *dev, uint8_t );



#endif /* CCS811_DRIVER_H */
