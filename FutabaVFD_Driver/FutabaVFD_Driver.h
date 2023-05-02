/****************************************
* \file     FutabaVFD_Driver.h
* \brief    Header file 
* \date     Aug 2021 
* \author   RJAM
****************************************/

#ifndef VFD_DRIVER_H
#define VFD_DRIVER_H

/********* Includes ********************/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_types.h"
#include "esp_err.h"
#include "esp_log.h"
#include "Utilities.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

/********* Definitions *****************/

#define VFD_COMMAND_LEN         2
#define VFD_SEGMENTS            8
#define VFD_SEG_HEIGHT          7
#define VFD_SEG_WIDTH           5

#define VFD_CONFIG_STACK_DEPTH  5012
#define VFD_CONFIG_SPI_QUEUE_TIMEOUT 100
#define VFD_CONFIG_CMD_DELAY_MS      10
#define VFD_CONFIG_SHORT_DELAY  pdMS_TO_TICKS(10)

#define VFD_MAX_DATA_LEN 16

#define DCRAM_DATA_WRITE 0x20
#define DGRAM_DATA_CLAER 0x10
#define CGRAM_DATA_WRITE 0x40
#define SET_DISPLAY_TIMING 0xE0
#define SET_DIMMING_DATA 0xE4
#define SET_DISPLAT_LIGHT_ON 0xE8
#define SET_DISPLAT_LIGHT_OFF 0xEA
#define SET_STAND_BY_MODE 0xEC


#define VFD_DEFAULT_DIMMING 50
#define VFD_NUM_CGRAM_PAGES 18   /** I have no idea if this is correct **/


typedef enum {
    VFD_CMD_DGRAM_DATA_CLEAR = 0x10,
    VFD_CMD_DCRAM_DATA_WRITE = 0x20,
    VFD_CMD_CGRAM_DATA_WRITE = 0x40,
    VFD_CMD_DISPLAY_TIMING = 0xE0,
    VFD_CMD_DIM = 0xE4,
    VFD_CMD_BKLIGHT_ON = 0xE8,
    VFD_CMD_BKLIGHT_OFF = 0xEA,
    VFD_CMD_STANDBY = 0xEC
} vfd_cmd_t;

typedef struct VFD_Init
{
    /* data */
    uint8_t spi_bus;
    uint16_t clock_speed;
    gpio_num_t cs_pin;
    gpio_num_t rst_pin;
} vfd_init_t;


typedef struct FutabaVFD_Driver
{
    /* data */
    uint8_t spi_bus;
    char vfd_buff[VFD_SEGMENTS+1];
    uint8_t cgram_page;
    TaskHandle_t task_handle;
    gpio_num_t cs_pin;
    gpio_num_t rst_pin;
    spi_device_handle_t spi_handle;

} vfd_handle_t;


/********** Types **********************/

typedef vfd_handle_t * VFD_HANDLE;

/******** Function Definitions *********/

#ifdef CONFIG_DRIVERS_USE_HEAP
VFD_HANDLE vfd_init(vfd_init_t *init);
#else
VFD_HANDLE vfd_init(VFD_HANDLE handle, vfd_init_t *init);
#endif


#endif /* VFD_DRIVER_H */
