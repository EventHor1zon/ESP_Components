/***************************************
* \file .c
* \brief
*
* \date
* \author
****************************************/

/********* Includes *******************/
#include "esp_err.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

#include "SSD1306_Driver.h"

/****** Function Prototypes ***********/

/************ ISR *********************/

/****** Private Data ******************/

/****** Private Functions *************/

/****** Global Data *******************/


uint8_t screen_initconfig_cmds[6] = {
    OLED_CONTROL_BYTE_CMD_STREAM,
    OLED_CMD_SET_CHARGE_PUMP,
    OLED_CMD_SET_CHARGE_PUMP_EN,
    OLED_CMD_SET_SEGMENT_REMAP,
    OLED_CMD_SET_COM_SCAN_MODE,
    OLED_CMD_DISPLAY_ON,
};

uint8_t screen_clear_cmds[130] = {
    OLED_CONTROL_BYTE_CMD_SINGLE,
    OLED_CONTROL_BYTE_DATA_STREAM,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0
};


const char *SSD_TAG = "SSD";
/****** Global Functions *************/


ssd1306_handle_t *ssd1306_init(ssd1306_init_t *init) {

    esp_err_t err = ESP_OK;



    ssd1306_handle_t *handle = heap_caps_calloc(1, sizeof(ssd1306_handle_t), MALLOC_CAP_8BIT);
    if(handle == NULL) {
        err = ESP_ERR_NO_MEM;
        ESP_LOGE(SSD_TAG, "Error allocating driver memory");
    }

    if(err == ESP_OK) {

        if()
    }


}