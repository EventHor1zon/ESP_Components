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

#ifdef CONFIG_USE_PERIPH_MANAGER

#include "PeripheralManager.h"
#define vfd_parameter_len 15
const parameter_t vfd_parameter_map[vfd_parameter_len];
const peripheral_t vfd_periph_template;

#endif /** CONFIG_USE_PERIPH_MANAGER **/

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

#define VFD_CONFIG_DEFAULT_DIMMING 50
#define VFD_CONFIG_MAX_BRIGHTNESS 255 /** no idea about this **/
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
    uint32_t clock_speed;
    gpio_num_t cs_pin;
    gpio_num_t rst_pin;
} vfd_init_t;


typedef struct FutabaVFD_Driver
{
    /* data */
    uint8_t spi_bus;
    char vfd_buff[VFD_SEGMENTS];
    uint8_t cgram_page;
    uint8_t brightness;
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

/**
 * @brief set character 0
 *         sets the character in the array. Will not be displayed
 *          until write_segment or write_all_segments
 *          is called
 * @param handle 
 * @param character - the ascii code of the character
 * @return ** esp_err_t 
 */
esp_err_t vfd_set_character0(VFD_HANDLE handle, uint8_t *character);

/**
 * @brief get character 0
 * 
 * @param handle 
 * @param character value storage
 * @return ** esp_err_t 
 */
esp_err_t vfd_get_character0(VFD_HANDLE handle, uint8_t *character);

/**
 * @brief set character 1
 *         sets the character in the array. Will not be displayed
 *          until write_segment or write_all_segments
 *          is called
 * @param handle 
 * @param character - the ascii code of the character
 * @return ** esp_err_t 
 */
esp_err_t vfd_set_character1(VFD_HANDLE handle, uint8_t *character);

/**
 * @brief get character 1
 * 
 * @param handle 
 * @param character value storage
 * @return ** esp_err_t 
 */
esp_err_t vfd_get_character1(VFD_HANDLE handle, uint8_t *character);

/**
 * @brief set character 2
 *         sets the character in the array. Will not be displayed
 *          until write_segment or write_all_segments
 *          is called
 * @param handle 
 * @param character - the ascii code of the character
 * @return ** esp_err_t 
 */
esp_err_t vfd_set_character2(VFD_HANDLE handle, uint8_t *character);

/**
 * @brief get character 2
 * 
 * @param handle 
 * @param character value storage
 * @return ** esp_err_t 
 */
esp_err_t vfd_get_character2(VFD_HANDLE handle, uint8_t *character);

/**
 * @brief set character 3
 *         sets the character in the array. Will not be displayed
 *          until write_segment or write_all_segments
 *          is called
 * @param handle 
 * @param character - the ascii code of the character
 * @return ** esp_err_t 
 */
esp_err_t vfd_set_character3(VFD_HANDLE handle, uint8_t *character);

/**
 * @brief get character 3
 * 
 * @param handle 
 * @param character value storage
 * @return ** esp_err_t 
 */
esp_err_t vfd_get_character3(VFD_HANDLE handle, uint8_t *character);

/**
 * @brief set character 4
 *         sets the character in the array. Will not be displayed
 *          until write_segment or write_all_segments
 *          is called
 * @param handle 
 * @param character - the ascii code of the character
 * @return ** esp_err_t 
 */
esp_err_t vfd_set_character4(VFD_HANDLE handle, uint8_t *character);

/**
 * @brief get character 4
 * 
 * @param handle 
 * @param character value storage
 * @return ** esp_err_t 
 */
esp_err_t vfd_get_character4(VFD_HANDLE handle, uint8_t *character);

/**
 * @brief set character 5
 *         sets the character in the array. Will not be displayed
 *          until write_segment or write_all_segments
 *          is called
 * @param handle 
 * @param character - the ascii code of the character
 * @return ** esp_err_t 
 */
esp_err_t vfd_set_character5(VFD_HANDLE handle, uint8_t *character);

/**
 * @brief get character 5
 * 
 * @param handle 
 * @param character value storage
 * @return ** esp_err_t 
 */
esp_err_t vfd_get_character5(VFD_HANDLE handle, uint8_t *character);

/**
 * @brief set character 6
 *         sets the character in the array. Will not be displayed
 *          until write_segment or write_all_segments
 *          is called
 * @param handle 
 * @param character - the ascii code of the character
 * @return ** esp_err_t 
 */
esp_err_t vfd_set_character6(VFD_HANDLE handle, uint8_t *character);

/**
 * @brief get character 6
 * 
 * @param handle 
 * @param character value storage
 * @return ** esp_err_t 
 */
esp_err_t vfd_get_character6(VFD_HANDLE handle, uint8_t *character);

/**
 * @brief set character 7
 *         sets the character in the array. Will not be displayed
 *          until write_segment or write_all_segments
 *          is called
 * @param handle 
 * @param character - the ascii code of the character
 * @return ** esp_err_t 
 */
esp_err_t vfd_set_character7(VFD_HANDLE handle, uint8_t *character);

/**
 * @brief get character 7
 * 
 * @param handle 
 * @param character value storage
 * @return ** esp_err_t 
 */
esp_err_t vfd_get_character7(VFD_HANDLE handle, uint8_t *character);

#if VFD_SEGMENTS == 16

esp_err_t vfd_set_character8(VFD_HANDLE handle, uint8_t *character);

esp_err_t vfd_get_character8(VFD_HANDLE handle, uint8_t *character);

esp_err_t vfd_set_character9(VFD_HANDLE handle, uint8_t *character);

esp_err_t vfd_get_character9(VFD_HANDLE handle, uint8_t *character);

esp_err_t vfd_set_character10(VFD_HANDLE handle, uint8_t *character);

esp_err_t vfd_get_character10(VFD_HANDLE handle, uint8_t *character);

esp_err_t vfd_set_character11(VFD_HANDLE handle, uint8_t *character);

esp_err_t vfd_get_character11(VFD_HANDLE handle, uint8_t *character);

esp_err_t vfd_set_character12(VFD_HANDLE handle, uint8_t *character);

esp_err_t vfd_get_character12(VFD_HANDLE handle, uint8_t *character);

esp_err_t vfd_set_character13(VFD_HANDLE handle, uint8_t *character);

esp_err_t vfd_get_character13(VFD_HANDLE handle, uint8_t *character);

esp_err_t vfd_set_character14(VFD_HANDLE handle, uint8_t *character);

esp_err_t vfd_get_character14(VFD_HANDLE handle, uint8_t *character);

esp_err_t vfd_set_character15(VFD_HANDLE handle, uint8_t *character);

esp_err_t vfd_get_character15(VFD_HANDLE handle, uint8_t *character);

#endif /** VFD_SEGMENTS == 16 **/

/**
 * @brief set brightness
 * 
 * @param dev 
 * @param br brightness level (max tbd)
 * @return ** esp_err_t 
 */
esp_err_t vfd_set_brightness(VFD_HANDLE dev, uint8_t *br);

/**
 * @brief get brightness
 * 
 * @param dev 
 * @param br value storage
 * @return ** esp_err_t 
 */
esp_err_t vfd_get_brightness(VFD_HANDLE dev, uint8_t *br);

/**
 * @brief update the brightness
 * 
 * @param dev device handle
 * @return ** esp_err_t 
 */
esp_err_t vfd_update_brightness(VFD_HANDLE dev);

/**
 * @brief set the current cgram (custom char storage)
 *          page
 * @param dev - device handle
 * @param page - page (max tbd)
 * @return ** esp_err_t 
 */
esp_err_t vfd_set_current_cgram_page(VFD_HANDLE dev, uint8_t *page);

/**
 * @brief get the current cgram (custom char storage)
 *          page
 * @param dev - device handle
 * @param page - value storage
 * @return ** esp_err_t 
 */
esp_err_t vfd_get_current_cgram_page(VFD_HANDLE dev, uint8_t *page);

/**
 * @brief display the custom char written to dev->cgram 
 *      
 * @param dev - device handle 
 * @param segment segment to display custom cgram in
 * @return ** esp_err_t 
 */
esp_err_t vfd_display_custom_segment(VFD_HANDLE dev, uint8_t *segment);

/**
 * @brief load a custom segment into cgram memory
 *          expects an array of uint8_t of length 5
 *          only lowest 7 bits are displayed
 * @param dev  - device handle
 * @param data - pointer to data to store
 * @return ** esp_err_t 
 */
esp_err_t vfd_load_custom_segment(VFD_HANDLE dev, uint8_t *data);

/**
 * @brief Write a single segment
 * 
 * @param dev - device handle
 * @param segment - segment to write
 * @return ** esp_err_t 
 */
esp_err_t vfd_write_segment(VFD_HANDLE dev, uint8_t *segment);

/**
 * @brief clear a single segment
 * 
 * @param dev - device handle
 * @param segment  - segment to clear
 * @return ** esp_err_t 
 */
esp_err_t vfd_clear_segment(VFD_HANDLE dev, uint8_t *segment);

/**
 * @brief update all segments with data stored
 *          in the character array
 * @param dev - device handle
 * @return ** esp_err_t 
 */
esp_err_t vfd_write_all_segments(VFD_HANDLE dev);

/**
 * @brief clear all the segments
 * 
 * @param dev - device handle
 * @return ** esp_err_t 
 */
esp_err_t vfd_clear_all_segments(VFD_HANDLE dev);

/**
 * @brief copies a string into the character buffer
 *          requires a call to write_all_segments to display
 *          this function only copies #segments worth of characters
 * @param dev  -    device handle
 * @param input -   character string
 * @return ** esp_err_t 
 */
esp_err_t vfd_copy_string(VFD_HANDLE dev, char *input);

/**
 * @brief send the backlight on command
 * 
 * @param dev = device handle
 * @return ** esp_err_t 
 */
esp_err_t vfd_set_backlight_on(VFD_HANDLE dev);

/**
 * @brief send the backlight off command
 * 
 * @param dev = device handle
 * @return ** esp_err_t 
 */
esp_err_t vfd_set_backlight_off(VFD_HANDLE dev);

#endif /* VFD_DRIVER_H */
