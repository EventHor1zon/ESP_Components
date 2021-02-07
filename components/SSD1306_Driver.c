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



/****** Global Functions *************/
