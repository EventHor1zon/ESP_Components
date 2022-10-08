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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "Utilities.h"

#include "../Utils/font8x8_basic.h"

#include "SSD1306_Driver.h"



#ifdef CONFIG_USE_PERIPH_MANAGER

const parameter_t ssd1306_param_map[screen_param_len] = {
    {"text",        1, NULL, &ssd1306_set_text, NULL,               PARAMTYPE_STRING, (SSD_MAX_STRLEN-1), (SET_FLAG) },
    {"line",        2, &ssd1306_get_line, &ssd1306_set_line, NULL,  PARAMTYPE_UINT8, 8, (GET_FLAG | SET_FLAG) },
    {"clear line",  3, NULL, NULL, &ssd1306_clear_current_line,     PARAMTYPE_NONE,  0, (ACT_FLAG) },
    {"write line",  4, NULL, NULL, &ssd1306_write_current_line,     PARAMTYPE_NONE,  0, (ACT_FLAG) },
    {"clear all",   5, NULL, NULL, &ssd1306_clear_screen,           PARAMTYPE_NONE,  0, (ACT_FLAG) },
    {"write text",  6, NULL, NULL, &ssd1306_set_text,               PARAMTYPE_NONE,  0, (ACT_FLAG) },
};

const peripheral_t ssd1306_peripheral_template = {
    "SSD1306 Oled",
    NULL,
    &ssd1306_param_map, 
    screen_param_len,
    0,
    PTYPE_DISPLAY,
};

#endif


uint8_t screen_initconfig_cmds[5] = {
    OLED_CMD_SET_CHARGE_PUMP,
    OLED_CMD_SET_CHARGE_PUMP_EN,
    OLED_CMD_SET_SEGMENT_REMAP,
    OLED_CMD_SET_COM_SCAN_MODE,
    OLED_CMD_DISPLAY_ON,
};

/****** Function Prototypes ***********/

/************ ISR *********************/

/** TODO: Remember that thing tried to do with freertos
 * timers and passing a handle as the timer id? 
 * try it again - but rem this arg here VV is not the thing to
 * cast - cast the timer id!
 */

void screen_timer_callback(TimerHandle_t *arg) {

    printf("In timer callback - has scroll stopped?");

}

/****** Private Data ******************/

/****** Private Functions *************/


static uint16_t scroll_wait_time(ssd1306_handle_t *screen, screen_scroll_dir_t dir) {

    /** how far are we scrolling? **/
    uint8_t mod = 0;
    uint8_t frames = 0;
    uint16_t wait = 0;
    switch (screen->scroll_speed) 
    {
    case 0:
        frames = 5;
        break;
    case 1:
        frames = 64;
        break;
    case 2:
        frames = 128;
        break;
    case 3:
        frames = 255;
        break;
    case 4:
        frames = 3;
        break;
    case 5:
        frames = 4;
        break;
    case 6:
        frames = 25;
        break;
    case 7:
        frames = 2;
        break;
    default:
        frames = 2;
        break;
    }

    if(dir == SCROLL_DIR_DOWN || dir == SCROLL_DIR_UP) {
        /** 8 pages for full scroll **/
        mod = 8;
    }
    else if(dir == SCROLL_DIR_LEFT || dir == SCROLL_DIR_RIGHT) {
        mod = 16;
    } 
    wait = frames * mod * 10; /** this should give ms to wait + a little breating room **/

    return wait; 
}

static void ssd1306_task(void *args) {


    while(1) {
        vTaskDelay(100);
    }
}


/****** Global Data *******************/





const char *SSD_TAG = "SSD";


/****** Global Functions *************/



esp_err_t ssd1306_get_contrast(ssd1306_handle_t *screen, uint8_t *val) {
    *val = screen->contrast;
    return ESP_OK;
}

esp_err_t ssd1306_set_contrast(ssd1306_handle_t *screen, uint8_t *val) {

    esp_err_t err = ESP_OK;
    uint8_t c = *val;
    uint8_t cmds[2] = { OLED_CMD_SET_CONTRAST, c};
    err = gcd_i2c_write_address(screen->bus, screen->dev_addr, OLED_CONTROL_BYTE_CMD_STREAM, 2, cmds);
    if(!err) {
        screen->contrast = c;
    }
    return err;
}


esp_err_t ssd1306_get_display_active(ssd1306_handle_t *screen, bool *val) {
    *val = screen->active;
    return ESP_OK;
}


esp_err_t ssd1306_set_display_active(ssd1306_handle_t *screen, bool *val) {

    esp_err_t err = ESP_OK;
    bool o = *val;
    bool io = false;
    if(o && !screen->active) {
        io = true;
    }
    else if (!o && screen->active) {
        io = true;
    }

    if(io) {
        uint8_t cmd = OLED_CMD_DISPLAY_OFF | o;
        err = gcd_i2c_write_address(screen->bus, screen->dev_addr, OLED_CONTROL_BYTE_CMD_SINGLE, 1, &cmd);
        screen->active = o;
    }
    return err;
}


esp_err_t ssd1306_get_orientation(ssd1306_handle_t *screen, uint8_t *val) {
    *val = screen->orientation;
    return ESP_OK;
}


esp_err_t ssd1306_set_display_orientation(ssd1306_handle_t *screen, uint8_t *val) {

    esp_err_t err = ESP_OK;
    uint8_t o = *val;
    bool flip = false;
    if(o && !screen->orientation) {
        flip = true;
    }
    else if (!o && screen->orientation) {
        flip = true;
    }

    if(flip) {
        uint8_t cmd = OLED_CMD_DISPLAY_INVERTED | (o > 0 ? 1 : 0);
        err = gcd_i2c_write_address(screen->bus, screen->dev_addr, OLED_CONTROL_BYTE_CMD_SINGLE, 1, &cmd);
        screen->orientation = (o > 0 ? 1 : 0 );
    }
    return err;
}


esp_err_t ssd1306_scroll_screen_dir(ssd1306_handle_t *screen, screen_scroll_dir_t *dir) {

    esp_err_t err = ESP_OK;
    uint8_t d = *dir;
    uint8_t cmds[8] = {0}; 
    uint16_t write_len = 0;


    switch (d)
    {
        case SCROLL_DIR_UP:
        case SCROLL_DIR_DOWN:
            /* code */
            cmds[0] = 0x2A;
            cmds[3] = screen->scroll_speed;
            cmds[4] = 8;
            cmds[5] = 0x3F;
            cmds[6] = 0xFF; 
            cmds[7] = 0x2F;
            write_len = 8;
            break;
        case SCROLL_DIR_LEFT:
            /* code */
            cmds[0] = 0x27;
            cmds[3] = screen->scroll_speed; 
            cmds[4] = 8;
            cmds[5] = 0xFF;
            cmds[6] = 0x2F;
            write_len = 7;
            break;
        case SCROLL_DIR_RIGHT:
            cmds[0] = 0x26;
            cmds[3] = screen->scroll_speed; 
            cmds[4] = 8;
            cmds[5] = 0xFF;
            cmds[6] = 0x2F;
            write_len = 7;
            break;
        default:
            err = ESP_ERR_INVALID_ARG;
            break;
    }

    if(!err) {
        err = gcd_i2c_write_address(screen->bus, screen->dev_addr, OLED_CONTROL_BYTE_CMD_STREAM, write_len, cmds);
        if(!err) {
            screen->is_scrolling = true;
            xTimerChangePeriod(screen->timer, pdMS_TO_TICKS(scroll_wait_time(screen, d)), 0);
            xTimerStart(screen->timer, 0);
        }
    }

    return err;
}


esp_err_t ssd1306_set_text(ssd1306_handle_t *screen, char *text) {

    esp_err_t err = ESP_OK;

    uint16_t length = strlen(text); 
    if(length > SSD_MAX_STRLEN-1) {
        err = ESP_ERR_INVALID_SIZE;
    }
    else {
        memset(screen->text, '\0', SSD_MAX_STRLEN);
        memcpy(screen->text, text, length);
        screen->text_len = length;
    }
    printf("String set to %s\n", screen->text);
    return err;
}


esp_err_t ssd1306_set_line(ssd1306_handle_t *screen, uint8_t *line) {
    uint8_t val = *line;
    if(val > SSD1306_MAX_LINES) {
        return ESP_ERR_INVALID_ARG;
    }
    else {
        screen->current_page = val;
    }
    return ESP_OK;
}


esp_err_t ssd1306_get_line(ssd1306_handle_t *screen, uint8_t *line) {
   esp_err_t status = ESP_OK;
   *line = screen->current_page;
   return status;
}


esp_err_t ssd1306_clear_current_line(ssd1306_handle_t *screen) {
    esp_err_t err = ESP_OK;
    uint8_t commands[34] = {0};

    uint8_t i = screen->current_page;

    commands[0] = (SSD1306_SEG_SELECT | i);
    commands[1] = OLED_CONTROL_BYTE_DATA_STREAM;
    err = gcd_i2c_write_address(screen->bus, screen->dev_addr, OLED_CONTROL_BYTE_CMD_SINGLE, 34, commands); 
    return err;
}


esp_err_t ssd1306_write_current_line(ssd1306_handle_t *screen) {
    esp_err_t err = ESP_OK;
    printf("Printing %u bytes, string : %s\n", screen->text_len, screen->text);
    
    uint8_t cmds[3] = {
        0x00,
        0x10,
        (SSD1306_SEG_SELECT | screen->current_page),
    };
    err = gcd_i2c_write_address(screen->bus, screen->dev_addr, OLED_CONTROL_BYTE_CMD_STREAM, 3, cmds);

    for(uint8_t i=0; i<screen->text_len && i < (128/8); i++) {
        /** Process newline **/
        if(screen->text[i] == 0x0A) {
            break;
        }
        else {
            uint8_t txt_cmd[8] = {0};
            memcpy(txt_cmd, font8x8_basic_tr[(uint8_t)screen->text[i]], 8);
            err = gcd_i2c_write_address(screen->bus, screen->dev_addr, OLED_CONTROL_BYTE_DATA_STREAM, 8, txt_cmd);
        }
    }
    return err;
}


esp_err_t ssd1306_clear_screen(ssd1306_handle_t *screen) {

    esp_err_t stat = ESP_OK;

        ESP_LOGI(SSD_TAG, "Clearing screen");

    uint8_t commands[130] = {0};

    for(uint8_t i=0; i<8; i++) {
        commands[0] = (SSD1306_SEG_SELECT | i);
        commands[1] = OLED_CONTROL_BYTE_DATA_STREAM;
        stat = gcd_i2c_write_address(screen->bus, screen->dev_addr, OLED_CONTROL_BYTE_CMD_SINGLE, 130, commands);
    }
    return stat;
}


esp_err_t ssd1306_write_text(ssd1306_handle_t *screen) {

    esp_err_t stat = ESP_OK;
    uint8_t page = 0;

    // showmem((uint8_t)screen->text, screen->text_len);
    /** reset the cursor position **/
    uint8_t cmds[3] = {
        0x00,
        0x10,
        SSD1306_SEG_SELECT,
    };
    stat = gcd_i2c_write_address(screen->bus, screen->dev_addr, OLED_CONTROL_BYTE_CMD_STREAM, 3, cmds);

    if(stat == ESP_OK) {
        printf("Printing %u bytes, string : %s\n", screen->text_len, screen->text);
        for(uint8_t i=0; i<screen->text_len; i++) {
            /** Process newline **/
            if(screen->text[i] == 0x0A) {
                uint8_t nlcmds[3] = {0};
                memcpy(nlcmds, cmds, 2);
                page++;
                nlcmds[2] = ( SSD1306_SEG_SELECT | page);
                stat = gcd_i2c_write_address(screen->bus, screen->dev_addr, OLED_CONTROL_BYTE_CMD_STREAM, 3, nlcmds);
            }
            /** Print character **/
            else {
                uint8_t txt_cmd[8] = {0};
                memcpy(txt_cmd, font8x8_basic_tr[(uint8_t)screen->text[i]], 8);
                stat = gcd_i2c_write_address(screen->bus, screen->dev_addr, OLED_CONTROL_BYTE_DATA_STREAM, 8, txt_cmd);
            }
        }
    }

    return stat;
}


ssd1306_handle_t *ssd1306_init(ssd1306_init_t *init) {

    esp_err_t err = ESP_OK;
    ssd1306_handle_t *handle = NULL;
    TaskHandle_t ssd_taskhandle = NULL;
    
    ESP_LOGI(SSD_TAG, "Starting SSD1306 Driver");

    if(!(gcd_i2c_check_bus(init->i2c_bus))) {
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

    if(err == ESP_OK) {
        /** initialise the screen **/
        ESP_LOGI(SSD_TAG, "Writing Screen init commands to bus %u", handle->bus);
        err = gcd_i2c_write_address(handle->bus, handle->dev_addr, OLED_CONTROL_BYTE_CMD_STREAM, 5, screen_initconfig_cmds);
        if(err != ESP_OK) {
            ESP_LOGE(SSD_TAG, "Error sending initial commands");
            err = ESP_ERR_INVALID_ARG;
        }
    }

    if(err==ESP_OK) {
        if(xTaskCreate(ssd1306_task, "ssd1306_task", configMINIMAL_STACK_SIZE, NULL, 3, &ssd_taskhandle) != pdTRUE) {
            ESP_LOGE(SSD_TAG, "Error creating task");
            err = ESP_ERR_NO_MEM;
        }
        else {
            handle->task_handle = ssd_taskhandle;
        }
    }

    if(!err) {
        TimerHandle_t timer = xTimerCreate("screen timer", 1000, pdFALSE, NULL, (TimerCallbackFunction_t )screen_timer_callback);
        if(timer != NULL) {
            handle->timer = timer;
        }
        else {
            ESP_LOGE(SSD_TAG, "Error creating timer");
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