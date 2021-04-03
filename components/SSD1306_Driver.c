/***************************************
* \file .c
* \brief
*
* \date
* \author
****************************************/

/********* Includes *******************/

#include "string.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "genericCommsDriver.h"

#include "font8x8_basic.h"

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

uint8_t screen_clear_cmds[131] = {
    OLED_CONTROL_BYTE_CMD_SINGLE,
    SSD1306_SEG_SELECT,
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

uint8_t screen_text_cmds[4] = {
    OLED_CONTROL_BYTE_CMD_STREAM,
    0,
    10,
    SSD1306_SEG_SELECT,
};

const char *SSD_TAG = "SSD";
/****** Global Functions *************/

static void ssd1306_task(void *args) {


    while(1) {
        vTaskDelay(100);
    }
}


esp_err_t ssd1306_set_text(ssd1306_handle_t *screen, char *text) {

    esp_err_t err = ESP_OK;

    uint16_t strlen = strlen(text); 
    if(strlen > SSD_MAX_STRLEN-1) {
        err = ESP_ERR_INVALID_SIZE;
    }
    else {
        memset(screen->text, '\0', SSD_MAX_STRLEN);
        memcpy(screen->text, text, strlen);
        screen->text_len = strlen;
    }
    return err;
}


esp_err_t ssd1306_clear_screen(ssd1306_handle_t *screen) {

    esp_err_t stat = ESP_OK;

    fo(uint8_t i=0; i<8; i++) {
        screen_clear_cmds[1] = (SSD1306_SEG_SELECT | i);
        stat = gcd_i2c_write_block(screen->bus, screen->dev_addr, sizeof(screen_clear_cmds), screen_clear_cmds);
    }
    return stat;
}

esp_err_t ssd1306_write_text(ssd1306_handle_t *screen) {

    esp_err_t stat = ESP_OK;
    uint8_t page = 0;

    /** reset the cursor position **/
    stat = gcd_i2c_write_block(screen->bus, screen->dev_addr, sizeof(screen_text_cmds), screen_text_cmds);

    if(stat == ESP_OK) {
        for(uint8_t i=0; i<screen->text_len; i++) {
            /** Process newline **/
            if(screen->text[i] == "\n") {
                uint8_t cmds[4] = 0;
                memcpy(cmds, screen_text_cmds, sizeof(screen_text_cmds));
                cmds[3] = ( SSD1306_SEG_SELECT | page++);
                stat = gcd_i2c_write_block(screen->bus, screen->dev_addr, sizeof(screen_text_cmds), cmds);
            }
            /** Print character **/
            else {
                uint8_t cmds[9] = {0};
                cmds[0] = OLED_CONTROL_BYTE_CMD_STREAM;
                memcpy(&cmds[1], font8x8_basic_tr[(uint8_t)screen->text[i]], 8);
                stat = gcd_i2c_write_block(screen->bus, screen->dev_addr, 9, cmds);
            }
        }
    }

    return stat;
}


ssd1306_handle_t *ssd1306_init(ssd1306_init_t *init) {

    esp_err_t err = ESP_OK;
    ssd1306_handle_t *handle = NULL;
    TaskHandle_t ssd_taskhandle = NULL;

    if(init->i2c_bus != I2C_NUM_0 && init->i2c_bus != I2C_NUM_1) {
        err = ESP_ERR_INVALID_ARG;
    }

    if(err == ESP_OK) {
        handle = heap_caps_calloc(1, sizeof(ssd1306_handle_t), MALLOC_CAP_8BIT);
        if(handle == NULL) {
            err = ESP_ERR_NO_MEM;
            ESP_LOGE(SSD_TAG, "Error allocating driver memory");
        }
        else {
            handle->bus = init->i2c_bus;
            handle->dev_addr = init->i2c_addr;
        }
    }

    if(err == ESP_OK && !(genericI2C_is_bus_init(init->i2c_bus))) {
        uint32_t wait_counter = 0;
        while(!(genericI2C_is_bus_init(init->i2c_bus))){
            vTaskDelay(pdMS_TO_TICKS(100));
            wait_counter++;
            if(wait_counter > 10) {
                ESP_LOGE(SSD_TAG, "Error - I2C bus was not initialised");
                err = ESP_ERR_TIMEOUT;
            }
        }
    }

    if(err == ESP_OK) {
        /** initialise the screen **/
        err = gcd_i2c_write_block(handle->bus, handle->dev_addr, sizeof(screen_initconfig_cmds), screen_initconfig_cmds);
        if(err != ESP_OK) {
            ESP_LOGE(COMMS_TAG, "Error sending initial commands");
        }
    }

    if(err==ESP_OK) {
        if(xTaskCreate(ssd1306_task, "ssd1306_task", configMINIMAL_STACK_SIZE, NULL, 3, &ssd_taskhandle) != pdTRUE) {
            ESP_LOGE(SSD_TAG, "Error creating task");
            err = ESP_ERR_NO_MEM;
        }
    }

    if(err != ESP_OK && handle != NULL) {
        heap_caps_free(handle);
    }

    if(err == ESP_OK) {
        ESP_LOGI(SSD_TAG, "Succesfully started SSD1306 Driver");
    } else {
        ESP_LOGE(SSD_TAG, "Error initialising SSD1306 Driver");
    }

    return handle;

}