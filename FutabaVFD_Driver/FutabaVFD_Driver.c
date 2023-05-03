/***************************************
* \file         FutabaVFD_Driver.c
* \brief        Driver for the Futaba 8-MD-06INKM C2CIG type VFD display
*               Good luck finding a datasheet lolz
*               Code hints from https://github.com/positronicsengineer/rpi-spi-vfd
*               More hints: https://github.com/sfxfs/ESP32-VFD-8DM-Arduino/blob/oop/src/VFD.cpp#L256
*
*
*               Haha, it works!
*
* \date         Aug 2021
* \author       RJAM
****************************************/

/********* Includes *******************/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_types.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "Utilities.h"
#include "genericCommsDriver.h"
#include "FutabaVFD_Driver.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include <string.h>

/****** Global Data *******************/

#define DEBUG_MODE

const char *VFD_TAG = "VFD DRIVER";

/****** Function Prototypes ***********/

static void vfd_driver_task(void *args);

static esp_err_t vfd_init_commands(VFD_HANDLE handle);


/************ ISR *********************/

/****** Private Data ******************/

#ifdef DEBUG_MODE

const uint8_t test_char_data[VFD_SEG_WIDTH] = {
    0x55,
    0x55,
    0x55,
    0x55,
    0x55,
};

#endif

#ifdef CONFIG_USE_PERIPH_MANAGER

#include "PeripheralManager.h"

const parameter_t vfd_parameter_map[vfd_parameter_len] = {
    {"Character 0", 1, &vfd_get_character0, &vfd_set_character0, NULL, DATATYPE_UINT8, 0xFF, (GET_FLAG | SET_FLAG)},
    {"Character 1", 2, &vfd_get_character1, &vfd_set_character1, NULL, DATATYPE_UINT8, 0xFF, (GET_FLAG | SET_FLAG)},
    {"Character 2", 3, &vfd_get_character2, &vfd_set_character2, NULL, DATATYPE_UINT8, 0xFF, (GET_FLAG | SET_FLAG)},
    {"Character 3", 4, &vfd_get_character3, &vfd_set_character3, NULL, DATATYPE_UINT8, 0xFF, (GET_FLAG | SET_FLAG)},
    {"Character 4", 5, &vfd_get_character4, &vfd_set_character4, NULL, DATATYPE_UINT8, 0xFF, (GET_FLAG | SET_FLAG)},
    {"Character 5", 6, &vfd_get_character5, &vfd_set_character5, NULL, DATATYPE_UINT8, 0xFF, (GET_FLAG | SET_FLAG)},
    {"Character 6", 7, &vfd_get_character6, &vfd_set_character6, NULL, DATATYPE_UINT8, 0xFF, (GET_FLAG | SET_FLAG)},
    {"Character 7", 8, &vfd_get_character7, &vfd_set_character7, NULL, DATATYPE_UINT8, 0xFF, (GET_FLAG | SET_FLAG)},
#if VFD_SEGMENTS == 16
    {"Character 8", 9, &vfd_get_character8, &vfd_set_character8, NULL, DATATYPE_UINT8, 0xFF, (GET_FLAG | SET_FLAG)},
    {"Character 9", 10, &vfd_get_character9, &vfd_set_character9, NULL, DATATYPE_UINT8, 0xFF, (GET_FLAG | SET_FLAG)},
    {"Character 10", 11, &vfd_get_character10, &vfd_set_character10, NULL, DATATYPE_UINT8, 0xFF, (GET_FLAG | SET_FLAG)},
    {"Character 11", 12, &vfd_get_character11, &vfd_set_character11, NULL, DATATYPE_UINT8, 0xFF, (GET_FLAG | SET_FLAG)},
    {"Character 12", 13, &vfd_get_character12, &vfd_set_character12, NULL, DATATYPE_UINT8, 0xFF, (GET_FLAG | SET_FLAG)},
    {"Character 13", 14, &vfd_get_character13, &vfd_set_character13, NULL, DATATYPE_UINT8, 0xFF, (GET_FLAG | SET_FLAG)},
    {"Character 14", 15, &vfd_get_character14, &vfd_set_character14, NULL, DATATYPE_UINT8, 0xFF, (GET_FLAG | SET_FLAG)},
    {"Character 15", 16, &vfd_get_character15, &vfd_set_character15, NULL, DATATYPE_UINT8, 0xFF, (GET_FLAG | SET_FLAG)},
#endif /** VFD_SEGMENTS == 16 **/
    {"Brightness", 17, &vfd_get_brightness, &vfd_set_brightness, NULL, DATATYPE_UINT8, VFD_CONFIG_MAX_BRIGHTNESS, (GET_FLAG | SET_FLAG)},
    {"Update Bright", 18, NULL, NULL, &vfd_update_brightness, DATATYPE_NONE, 0, (ACT_FLAG)},
    {"CGram Page", 19, &vfd_get_current_cgram_page, &vfd_set_current_cgram_page, NULL, DATATYPE_UINT8, VFD_NUM_CGRAM_PAGES, (GET_FLAG | SET_FLAG)},
    {"Custom Char", 20, NULL, &vfd_display_custom_segment, NULL, DATATYPE_UINT8, VFD_SEGMENTS, (SET_FLAG)},
    {"Write Segments", 21, NULL, NULL, &vfd_write_all_segments, DATATYPE_NONE, 0, (ACT_FLAG)},
    {"Clear Segments", 22, NULL, NULL, &vfd_clear_all_segments, DATATYPE_NONE, 0, (ACT_FLAG)},
    {"Load String", 23, NULL, &vfd_copy_string, NULL, DATATYPE_STRING, VFD_SEGMENTS, (SET_FLAG)},
};

const peripheral_t vfd_periph_template = {
    .handle = NULL,
    .param_len = vfd_parameter_len,
    .params = vfd_parameter_map,
    .periph_type = "VFD Display",
    .peripheral_id = 0,
    .periph_type = PTYPE_DISPLAY
};

#endif /** CONFIG_USE_PERIPH_MANAGER **/

/****** Private Functions *************/


static esp_err_t spi_burst_write(VFD_HANDLE dev, uint8_t addr, uint8_t *data, uint8_t len) {

    uint8_t cmd = (addr | 1);
    spi_transaction_t trx = {0};
    uint8_t buffer[256] = {0};

    buffer[0] = cmd;
    memcpy(&buffer[1], data, sizeof(uint8_t) * len);

    trx.length = (8 + (len * 8));
    trx.tx_buffer = buffer;
    trx.flags = 0;

    return spi_device_transmit(dev->spi_handle, &trx);

}


static esp_err_t vfd_write_command_with_data(VFD_HANDLE dev, vfd_cmd_t cmd, uint8_t *data, uint8_t len) {

    esp_err_t err = ESP_OK;
    uint8_t data_buffer[VFD_MAX_DATA_LEN] = {0};

    if(len >= VFD_MAX_DATA_LEN) {
        return ESP_ERR_INVALID_ARG;
    }

    data_buffer[0] = cmd;
    memcpy(&data_buffer[1], data, len);

    spi_transaction_t trx = {0};
    trx.flags = 0;
    trx.length = (8 + (8 * len));
    trx.tx_buffer = data_buffer;

    err = spi_device_polling_transmit(dev->spi_handle, &trx);

    return err;

}


static esp_err_t vfd_write_command_with_data_byte(VFD_HANDLE dev, vfd_cmd_t cmd, uint8_t data) {

    esp_err_t err = ESP_OK;

    spi_transaction_t trx = {0};
    trx.flags = SPI_TRANS_USE_TXDATA;
    trx.length = 16;
    trx.tx_data[0] = cmd;
    trx.tx_data[1] = data;

    err = spi_device_polling_transmit(dev->spi_handle, &trx);

    return err;

}


static void vfd_driver_task(void *args) {
 
    VFD_HANDLE handle = (vfd_handle_t *)args;

    while(1) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    /** here be dragons **/
}


static void vfd_reset(VFD_HANDLE handle) {
    gpio_set_level(handle->rst_pin, 0);
    vTaskDelay(VFD_CONFIG_SHORT_DELAY);
    gpio_set_level(handle->rst_pin, 1);
    vTaskDelay(VFD_CONFIG_SHORT_DELAY);
}


static esp_err_t vfd_init_commands(VFD_HANDLE handle) {

    esp_err_t err = ESP_OK;
    uint8_t var = 0;

    vfd_reset(handle);

    vTaskDelay(pdMS_TO_TICKS(VFD_CONFIG_CMD_DELAY_MS));

    if(vfd_write_command_with_data_byte(handle, VFD_CMD_DISPLAY_TIMING, 7) != ESP_OK ||
       vfd_write_command_with_data_byte(handle, VFD_CMD_DIM, (uint8_t)VFD_CONFIG_DEFAULT_DIMMING) != ESP_OK ||
       vfd_write_command_with_data_byte(handle, VFD_CMD_BKLIGHT_ON, 0) != ESP_OK 
    ) {
        ESP_LOGE(VFD_TAG, "Error writing initialisation commands");
    }

#ifdef DEBUG_MODE
    for(uint8_t i=0; i < VFD_SEGMENTS; i++) {
        err = vfd_write_command_with_data_byte(handle, (VFD_CMD_DCRAM_DATA_WRITE|i), (0x41+i));
        if(err) {
            ESP_LOGE(VFD_TAG, "Error writing test data [%u]", err);
            break;
        }
    }
#endif /** DEBUG_MODE **/

    vTaskDelay(pdMS_TO_TICKS(1000));

    vfd_copy_string(handle, "Hello");
    vfd_write_all_segments(handle);

    vTaskDelay(pdMS_TO_TICKS(1000));

    vfd_copy_string(handle, "World!");
    vfd_write_all_segments(handle);

    vTaskDelay(3000);

    vfd_set_current_cgram_page(handle, &var);
    vfd_load_custom_segment(handle, &test_char_data[0]);

    vfd_clear_all_segments(handle);
    vfd_display_custom_segment(handle, &var);

    return err;
}

/****** Global Functions *************/

#ifdef CONFIG_DRIVERS_USE_HEAP
VFD_HANDLE vfd_init(vfd_init_t *init)
#else
VFD_HANDLE vfd_init(VFD_HANDLE handle, vfd_init_t *init)
#endif
{
    esp_err_t err = ESP_OK;
    spi_device_interface_config_t dev = {0};
    spi_device_handle_t dev_handle = NULL;
    gpio_config_t ioconf = {0};

    if(!gcd_spi_check_bus(init->spi_bus)) {
        ESP_LOGE(VFD_TAG, "Error invalid SPI bus");
        err = ESP_ERR_INVALID_ARG;
    }

    if(!err) {
#ifdef CONFIG_DRIVERS_USE_HEAP
        VFD_HANDLE handle = NULL;
        handle = heap_caps_calloc(1, sizeof(vfd_handle_t), MALLOC_CAP_DEFAULT);
        if(handle == NULL) {
            ESP_LOGE(VFD_TAG, "Error assigning memory for handle!");
            err = ESP_ERR_NO_MEM;
        }
#else
        memset(handle, 0, sizeof(vfd_handle_t));
#endif
    }

    if(!err) {
        handle->cs_pin = init->cs_pin;
        handle->rst_pin = init->rst_pin;
        handle->spi_bus = init->spi_bus;
        handle->spi_handle = dev_handle;
        handle->brightness = VFD_CONFIG_DEFAULT_DIMMING;
        handle->cgram_page = 0;
    }

    if(!err) {

        ioconf.intr_type = GPIO_INTR_DISABLE,
        ioconf.mode = GPIO_MODE_OUTPUT,
        ioconf.pin_bit_mask = (1 << init->rst_pin),
        ioconf.pull_down_en = GPIO_PULLDOWN_DISABLE,
        ioconf.pull_up_en = GPIO_PULLUP_ENABLE,

        err = gpio_config(&ioconf);
        if(err) {
            ESP_LOGE(VFD_TAG, "Error initialising reset pin {%u}", err);
        }
        else {
            gpio_set_level(handle->rst_pin, 1);
        }
    }

    if(!err) {
        dev.clock_speed_hz = init->clock_speed;
        dev.spics_io_num = init->cs_pin;
        dev.flags = (SPI_DEVICE_BIT_LSBFIRST | SPI_DEVICE_3WIRE);
        dev.queue_size = 4;

        err = spi_bus_add_device(handle->spi_bus, &dev, &handle->spi_handle);
        if(err) {
            ESP_LOGE(VFD_TAG, "Error adding device to bus! {%u}", err);
        }
    }

    if(!err && xTaskCreate(vfd_driver_task, "vfd_driver_task", VFD_CONFIG_STACK_DEPTH, handle, 3, &handle->task_handle) != pdTRUE) {
        ESP_LOGE(VFD_TAG, "Error creating task!");
        err = ESP_ERR_NO_MEM;
    }

    if(!err) {
        vfd_init_commands(handle);
    }


    return handle;

}


esp_err_t vfd_set_character0(VFD_HANDLE handle, uint8_t *character) {
    esp_err_t status = ESP_OK;
    handle->vfd_buff[0] = *character;
    return status;
}

esp_err_t vfd_get_character0(VFD_HANDLE handle, uint8_t *character) {
    esp_err_t status = ESP_OK;
    *character = handle->vfd_buff[0];
    return status;
}

esp_err_t vfd_set_character1(VFD_HANDLE handle, uint8_t *character) {
    esp_err_t status = ESP_OK;
    handle->vfd_buff[1] = *character;
    return status;
}

esp_err_t vfd_get_character1(VFD_HANDLE handle, uint8_t *character) {
    esp_err_t status = ESP_OK;
    *character = handle->vfd_buff[1];
    return status;
}

esp_err_t vfd_set_character2(VFD_HANDLE handle, uint8_t *character) {
    esp_err_t status = ESP_OK;
    handle->vfd_buff[2] = *character;
    return status;
}

esp_err_t vfd_get_character2(VFD_HANDLE handle, uint8_t *character) {
    esp_err_t status = ESP_OK;
    *character = handle->vfd_buff[2];
    return status;
}

esp_err_t vfd_set_character3(VFD_HANDLE handle, uint8_t *character) {
    esp_err_t status = ESP_OK;
    handle->vfd_buff[3] = *character;
    return status;
}

esp_err_t vfd_get_character3(VFD_HANDLE handle, uint8_t *character) {
    esp_err_t status = ESP_OK;
    *character = handle->vfd_buff[3];
    return status;
}

esp_err_t vfd_set_character4(VFD_HANDLE handle, uint8_t *character) {
    esp_err_t status = ESP_OK;
    handle->vfd_buff[4] = *character;
    return status;
}

esp_err_t vfd_get_character4(VFD_HANDLE handle, uint8_t *character) {
    esp_err_t status = ESP_OK;
    *character = handle->vfd_buff[4];
    return status;
}

esp_err_t vfd_set_character5(VFD_HANDLE handle, uint8_t *character) {
    esp_err_t status = ESP_OK;
    handle->vfd_buff[5] = *character;
    return status;
}

esp_err_t vfd_get_character5(VFD_HANDLE handle, uint8_t *character) {
    esp_err_t status = ESP_OK;
    *character = handle->vfd_buff[5];
    return status;
}

esp_err_t vfd_set_character6(VFD_HANDLE handle, uint8_t *character) {
    esp_err_t status = ESP_OK;
    handle->vfd_buff[6] = *character;
    return status;
}

esp_err_t vfd_get_character6(VFD_HANDLE handle, uint8_t *character) {
    esp_err_t status = ESP_OK;
    *character = handle->vfd_buff[6];
    return status;
}

esp_err_t vfd_set_character7(VFD_HANDLE handle, uint8_t *character) {
    esp_err_t status = ESP_OK;
    handle->vfd_buff[7] = *character;
    return status;
}

esp_err_t vfd_get_character7(VFD_HANDLE handle, uint8_t *character) {
    esp_err_t status = ESP_OK;
    *character = handle->vfd_buff[7];
    return status;
}

#if VFD_SEGMENTS == 16

esp_err_t vfd_set_character8(VFD_HANDLE handle, uint8_t *character) {
    esp_err_t status = ESP_OK;
    handle->vfd_buff[8] = *character;
    return status;
}

esp_err_t vfd_get_character8(VFD_HANDLE handle, uint8_t *character) {
    esp_err_t status = ESP_OK;
    *character = handle->vfd_buff[8];
    return status;
}

esp_err_t vfd_set_character9(VFD_HANDLE handle, uint8_t *character) {
    esp_err_t status = ESP_OK;
    handle->vfd_buff[9] = *character;
    return status;
}

esp_err_t vfd_get_character9(VFD_HANDLE handle, uint8_t *character) {
    esp_err_t status = ESP_OK;
    *character = handle->vfd_buff[9];
    return status;
}

esp_err_t vfd_set_character10(VFD_HANDLE handle, uint8_t *character) {
    esp_err_t status = ESP_OK;
    handle->vfd_buff[10] = *character;
    return status;
}

esp_err_t vfd_get_character10(VFD_HANDLE handle, uint8_t *character) {
    esp_err_t status = ESP_OK;
    *character = handle->vfd_buff[10];
    return status;
}

esp_err_t vfd_set_character11(VFD_HANDLE handle, uint8_t *character) {
    esp_err_t status = ESP_OK;
    handle->vfd_buff[11] = *character;
    return status;
}

esp_err_t vfd_get_character11(VFD_HANDLE handle, uint8_t *character) {
    esp_err_t status = ESP_OK;
    *character = handle->vfd_buff[11];
    return status;
}

esp_err_t vfd_set_character12(VFD_HANDLE handle, uint8_t *character) {
    esp_err_t status = ESP_OK;
    handle->vfd_buff[12] = *character;
    return status;
}

esp_err_t vfd_get_character12(VFD_HANDLE handle, uint8_t *character) {
    esp_err_t status = ESP_OK;
    *character = handle->vfd_buff[12];
    return status;
}

esp_err_t vfd_set_character13(VFD_HANDLE handle, uint8_t *character) {
    esp_err_t status = ESP_OK;
    handle->vfd_buff[13] = *character;
    return status;
}

esp_err_t vfd_get_character13(VFD_HANDLE handle, uint8_t *character) {
    esp_err_t status = ESP_OK;
    *character = handle->vfd_buff[13];
    return status;
}

esp_err_t vfd_set_character14(VFD_HANDLE handle, uint8_t *character) {
    esp_err_t status = ESP_OK;
    handle->vfd_buff[14] = *character;
    return status;
}

esp_err_t vfd_get_character14(VFD_HANDLE handle, uint8_t *character) {
    esp_err_t status = ESP_OK;
    *character = handle->vfd_buff[14];
    return status;
}

esp_err_t vfd_set_character15(VFD_HANDLE handle, uint8_t *character) {
    esp_err_t status = ESP_OK;
    handle->vfd_buff[15] = *character;
    return status;
}

esp_err_t vfd_get_character15(VFD_HANDLE handle, uint8_t *character) {
    esp_err_t status = ESP_OK;
    *character = handle->vfd_buff[15];
    return status;
}

#endif


esp_err_t vfd_set_brightness(VFD_HANDLE dev, uint8_t *br) {
    esp_err_t status = ESP_OK;
    uint8_t brt = *br;

    if(brt >= VFD_CONFIG_MAX_BRIGHTNESS) {
        return ESP_ERR_INVALID_ARG;
    }
    else {
       dev->brightness = brt;
    }
   
   return status;
}

esp_err_t vfd_get_brightness(VFD_HANDLE dev, uint8_t *br) {
   esp_err_t status = ESP_OK;
   *br = dev->brightness;
   return status;
}


esp_err_t vfd_update_brightness(VFD_HANDLE dev) {
    return vfd_write_command_with_data_byte(dev, VFD_CMD_DIM, dev->brightness);
}


esp_err_t vfd_set_current_cgram_page(VFD_HANDLE dev, uint8_t *page) {
    
    uint8_t p = *page;
    esp_err_t err = ESP_OK;

    if(p >= VFD_NUM_CGRAM_PAGES) {
        ESP_LOGE(VFD_TAG, "Error: Invalid cgram page");
        err = ESP_ERR_INVALID_ARG;
    }
    else {
        dev->cgram_page = p;
    }

    return err;
}

esp_err_t vfd_get_current_cgram_page(VFD_HANDLE dev, uint8_t *page) {
    *page = dev->cgram_page;
    return ESP_OK;
}


esp_err_t vfd_display_custom_segment(VFD_HANDLE dev, uint8_t *segment) {
    return vfd_write_command_with_data_byte(dev, (VFD_CMD_DCRAM_DATA_WRITE | *segment), dev->cgram_page);
}


esp_err_t vfd_load_custom_segment(VFD_HANDLE dev, uint8_t *data) {
    /** expects a pointer to a 5 element array
     *  characters are written by column, expect 1 bit to be
     *  lost. Assume MSB but might be LSB. Will find out soon
     **/
    esp_err_t err = ESP_OK;

    err = vfd_write_command_with_data(dev, (VFD_CMD_CGRAM_DATA_WRITE | dev->cgram_page), data, 5);
    
    if(err) {
        ESP_LOGE(VFD_TAG, "Error writing custom character data [%u]", err);
    }

    return err;
}


esp_err_t vfd_write_segment(VFD_HANDLE dev, uint8_t *segment) {
    esp_err_t err = ESP_OK;
    uint8_t seg = *segment;

    if(seg >= VFD_SEGMENTS) {
        return ESP_ERR_INVALID_ARG;
    }

    err = vfd_write_command_with_data_byte(dev, (VFD_CMD_DCRAM_DATA_WRITE | seg), dev->vfd_buff[seg]);
    if(err) {
        ESP_LOGE(VFD_TAG, "Error writing character %u data [%u]", seg, err);
    }

    return err;
}

esp_err_t vfd_clear_segment(VFD_HANDLE dev, uint8_t *segment) {
    esp_err_t err = ESP_OK;
    uint8_t seg = *segment;

    if(seg >= VFD_SEGMENTS) {
        return ESP_ERR_INVALID_ARG;
    }

    err = vfd_write_command_with_data_byte(dev, (VFD_CMD_DCRAM_DATA_WRITE | seg), VFD_CMD_DGRAM_DATA_CLEAR);
    if(err) {
        ESP_LOGE(VFD_TAG, "Error writing character %u data [%u]", seg, err);
    }

    return err;
}


esp_err_t vfd_write_all_segments(VFD_HANDLE dev) {
    esp_err_t err = ESP_OK;

    for(uint8_t i=0; i < VFD_SEGMENTS; i++) {
        err = vfd_write_command_with_data_byte(dev, (VFD_CMD_DCRAM_DATA_WRITE | i), dev->vfd_buff[i]);
        if(err) {
            ESP_LOGE(VFD_TAG, "Error writing character %u data [%u]", i, err);
            break;
        }
    }

    return err;
}

esp_err_t vfd_clear_all_segments(VFD_HANDLE dev) {
    esp_err_t err = ESP_OK;

    for(uint8_t i=0; i < VFD_SEGMENTS; i++) {
        err = vfd_write_command_with_data_byte(dev, (VFD_CMD_DCRAM_DATA_WRITE | i), VFD_CMD_DGRAM_DATA_CLEAR);
        if(err) {
            ESP_LOGE(VFD_TAG, "Error writing character %u data [%u]", i, err);
            break;
        }
    }

    return err;
}

esp_err_t vfd_copy_string(VFD_HANDLE dev, char *input) {

    memset(dev->vfd_buff, 0, sizeof(uint8_t) * VFD_SEGMENTS);

    strncpy(dev->vfd_buff, input, VFD_SEGMENTS);

    return ESP_OK;

}


esp_err_t vfd_set_backlight_on(VFD_HANDLE dev) {
    return vfd_write_command_with_data_byte(dev, VFD_CMD_BKLIGHT_ON, 0);
}

esp_err_t vfd_set_backlight_off(VFD_HANDLE dev) {
    return vfd_write_command_with_data_byte(dev, VFD_CMD_BKLIGHT_OFF, 0);
}