/**
*    @file    OV2460_Driver.c
*
*    @brief    header file for OV2460_Driver
*
*   This should be fun!
*   We're going to write a driver for the cameras which come attached to the 
*   ESP32-CAM, the OVO2460
*
*   This is going to involve multi-line SPI connection to the camera and 
*   gpio data lines. Borrow from other's code. Make it support my API.
*
*   See ESP32/esp32-cam github
*
*   Pin connections:
*   
*   GPIO  5 - D0        (Y0)
*   GPIO 18 - D1        (Y*)
*   GPIO 19 - D2
*   GPIO 21 - D3
*   GPIO SENSOR_VP - D4
*   GPIO SENSOR_VN - D5
*   GPIO 34 - D6
*   GPIO 35 - D7
*   GPIO 32 - CAM_PWR   
*   GPIO  0 - CAM_MCLK  
*   GPIO 25 - CAM_VSYNC
*   GPIO 23 - CAM_HSYNC
*   GPIO 22 - CAM_PCLK  (Pixel clock output?)
*   GPIO 26 - CAM_DATA
*   GPIO 27 - CAM_CLK
*
*   Notes:
*
*       Frame rate timing  
*           PCLK (MHz) 36 - 15 Framerate (fps)
*                      18 - 7.5
*                      6  - 6
*                      3  - 1.25
*
*
*
*    @author    RJAM
*    @created   Wed 31 May 23:27:08 BST 2023
*/



/** Includes **/

#include <stdio.h>
#include <string.h>
#include "esp_types.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "./OV2640_Driver.h"
#include "./driver/include/esp_camera.h"
#include "./driver/include/sensor.h"

/** Private Data **/

TaskHandle_t task_handle = NULL;
QueueHandle_t queue_handle = NULL;


/** Function Prototypes **/

void ovo_driver_task(void *args) {

    while(1) {
        ESP_LOGI("Cam", "Cam task");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    /** here be dragons **/
}

/** Static Functions **/

/** Tasks **/

/** Public Functions **/

esp_err_t ovo_init(OVO_H ovo, const camera_config_t *init) {

    esp_err_t err = ESP_OK;
    
    memset(ovo, 0, sizeof(ovo_handle_t));

    err = esp_camera_init(init);
    if(!err) {
        ESP_LOGE("Cam", "Error initialising camera %u", err);
    }

    if(!err) {
        ovo->cam_sensor = esp_camera_sensor_get();
        if(ovo->cam_sensor == NULL) {
            ESP_LOGE("Cam", "Error getting sensor pointer");
            err = ESP_ERR_NOT_SUPPORTED;
        }
    }

    return err;
}

esp_err_t ovo_get_init_status(OVO_H ovo, int *status) {
    *status = ovo->cam_sensor->init_status(ovo->cam_sensor);
    return ESP_OK;
}

esp_err_t ovo_reset_software(OVO_H ovo) {
    return (esp_err_t)ovo->cam_sensor->reset(ovo->cam_sensor);
}

esp_err_t ovo_get_jpeg_quality(OVO_H ovo, uint16_t *qual) {
    *qual = ovo->settings.jpeg_quality;
    return ESP_OK;
}

esp_err_t ovo_set_jpeg_quality(OVO_H ovo, uint16_t *qual) {
    esp_err_t err = ovo->cam_sensor->set_quality(ovo->cam_sensor, *qual);
    ovo->settings.jpeg_quality = (err == ESP_OK) ? 
                                  *qual : 
                                  0;
    return err;
}

esp_err_t ovo_get_brightness(OVO_H ovo, uint16_t *brt) {
    *brt = ovo->settings.brightness;
    return ESP_OK;
}

esp_err_t ovo_set_brightness(OVO_H ovo, uint16_t *brt) {
    
    esp_err_t err = ovo->cam_sensor->set_brightness(ovo->cam_sensor, *brt);
    if(!err) {
        ovo->settings.brightness = *brt;
    }

    return err;
}

esp_err_t ovo_get_contrast(OVO_H ovo, int8_t *con) {
    *con = ovo->settings.contrast;
    return ESP_OK;
}

esp_err_t ovo_set_contrast(OVO_H ovo, int8_t *con) {
    esp_err_t err = ovo->cam_sensor->set_contrast(ovo->cam_sensor, *con);
    if(!err) {
        ovo->settings.contrast = *con;
    }
    return err;
}


esp_err_t ovo_get_pixformat(OVO_H ovo, pixformat_t *fmt) {
    *fmt = ovo->settings.pixformat;
    return ESP_OK;
}

esp_err_t ovo_set_pixformat(OVO_H ovo, pixformat_t *fmt) {
    
    esp_err_t err = ovo->cam_sensor->set_contrast(ovo->cam_sensor, *fmt);

    if(!err) {
        ovo->settings.pixformat = *fmt;
    }
    
    return err;
}


esp_err_t ovo_get_framesize(OVO_H ovo, framesize_t *fs) {
    *fs = ovo->settings.framesize;
    return ESP_OK;
}

esp_err_t ovo_set_framesize(OVO_H ovo, framesize_t *fs) {
    esp_err_t err = ovo->cam_sensor->set_framesize(ovo->cam_sensor, *fs);
    if(!err) {
        ovo->settings.framesize = *fs;
    }
    return err;
}


esp_err_t ovo_get_saturation(OVO_H ovo, int8_t *sat) {
    *sat = ovo->settings.saturation;
    return ESP_OK;
}

esp_err_t ovo_set_saturation(OVO_H ovo, int8_t *sat) {
    esp_err_t err = ovo->cam_sensor->set_saturation(ovo->cam_sensor, *sat);
    if(!err) {
        ovo->settings.saturation = *sat;
    }
    return err;
}


esp_err_t ovo_get_colourbar_en(OVO_H ovo, bool *en) {
    *en = ovo->settings.colourbar_en;
    return ESP_OK;
}

esp_err_t ovo_set_colourbar_en(OVO_H ovo, bool *en) {
    esp_err_t err = ovo->cam_sensor->set_colorbar(ovo->cam_sensor, *en);
    if(!err) {
        ovo->settings.colourbar_en = *en;
    }
    return err;
}

esp_err_t ovo_get_hmirror_en(OVO_H ovo, bool *en) {
    *en = ovo->settings.hmirror_en;
    return ESP_OK;
}

esp_err_t ovo_set_hmirror_en(OVO_H ovo, bool *en) {
    esp_err_t err = ovo->cam_sensor->set_hmirror(ovo->cam_sensor, *en);
    if(!err) {
        ovo->settings.hmirror_en = *en;
    }
    return err;
}


esp_err_t ovo_get_vflip_en(OVO_H ovo, bool *en) {
    *en = ovo->settings.vflip_en;
    return ESP_OK;
}

esp_err_t ovo_set_vflip_en(OVO_H ovo, bool *en) {
    esp_err_t err = ovo->cam_sensor->set_vflip(ovo->cam_sensor, *en);
    if(!err) {
        ovo->settings.vflip_en = *en;
    }
    return err;
}

esp_err_t ovo_get_whitebalance_en(OVO_H ovo, bool *en) {
    *en = ovo->settings.whitebalance_en;
    return ESP_OK;
}

esp_err_t ovo_set_whitebalance_en(OVO_H ovo, bool *en) {
    esp_err_t err = ovo->cam_sensor->set_whitebal(ovo->cam_sensor, *en);
    if(!err) {
        ovo->settings.whitebalance_en = *en;
    }
    return err;
}

esp_err_t ovo_get_aec2_en(OVO_H ovo, bool *en) {
    *en = ovo->settings.aec2_en;
    return ESP_OK;
}

esp_err_t ovo_set_aec2_en(OVO_H ovo, bool *en) {
    esp_err_t err = ovo->cam_sensor->set_aec2(ovo->cam_sensor, *en);
    if(!err) {
        ovo->settings.aec2_en = *en;
    }
    return err;
}

esp_err_t ovo_get_exposure_ctrl_en(OVO_H ovo, bool *en) {
    *en = ovo->settings.exposurectrl_en;
    return ESP_OK;
}

esp_err_t ovo_set_exposure_ctrl_en(OVO_H ovo, bool *en) {
    esp_err_t err = ovo->cam_sensor->set_exposure_ctrl(ovo->cam_sensor, *en);
    if(!err) {
        ovo->settings.exposurectrl_en = *en;
    }
    return err;
}


esp_err_t ovo_get_gainceiling(OVO_H ovo, uint8_t *gainc) {
    *gainc = ovo->settings.gainceiling;
    return ESP_OK;
}

esp_err_t ovo_set_gainceiling(OVO_H ovo, uint8_t *gainc) {
    esp_err_t err = ovo->cam_sensor->set_gainceiling(ovo->cam_sensor, *gainc);
    if(!err) {
        ovo->settings.gainceiling = *gainc;
    }
    return err;
}



esp_err_t ovo_get_gain(OVO_H ovo, uint8_t *gain) {
    *gain = ovo->settings.gain;
    return ESP_OK;
}

esp_err_t ovo_set_gain(OVO_H ovo, uint8_t *gain) {
    esp_err_t err = ovo->cam_sensor->set_gain_ctrl(ovo->cam_sensor, *gain);
    if(!err) {
        ovo->settings.gain = *gain;
    }
    return err;
}


esp_err_t ovo_get_autoexpose_val(OVO_H ovo, uint8_t *exp) {
    *exp = ovo->settings.autoexpose_val;
    return ESP_OK;
}

esp_err_t ovo_set_autoexpose_val(OVO_H ovo, uint8_t *exp) {
    esp_err_t err = ovo->cam_sensor->set_aec_value(ovo->cam_sensor, *exp);
    if(!err) {
        ovo->settings.autoexpose_val = *exp;
    }
    return err;
}


esp_err_t ovo_get_sfx(OVO_H ovo, uint8_t *fx) {
    *fx = ovo->settings.sfx;
    return ESP_OK;
}

esp_err_t ovo_set_sfx(OVO_H ovo, uint8_t *fx) {
    esp_err_t err = ovo->cam_sensor->set_special_effect(ovo->cam_sensor, *fx);
    if(!err) {
        ovo->settings.sfx = *fx;
    }
    return err;
}

esp_err_t ovo_get_wbmode(OVO_H ovo, uint8_t *wb) {
    *wb = ovo->settings.wb_mode;
    return ESP_OK;
}

esp_err_t ovo_set_wbmode(OVO_H ovo, uint8_t *wb) {
    esp_err_t err = ovo->cam_sensor->set_wb_mode(ovo->cam_sensor, *wb);
    if(!err) {
        ovo->settings.wb_mode = *wb;
    }
    return err;
}

esp_err_t ovo_get_ae_level(OVO_H ovo, uint8_t *ae) {
    *ae = ovo->settings.ae_level;
    return ESP_OK;
}

esp_err_t ovo_set_ae_level(OVO_H ovo, uint8_t *ae) {
    esp_err_t err = ovo->cam_sensor->set_ae_level(ovo->cam_sensor, *ae);
    if(!err) {
        ovo->settings.ae_level = *ae;
    }
    return err;
}

/**
esp_err_t ovo_get_gainceiling(OVO_H ovo, uint8_t *gainc) {
    *gainc = ovo->settings.gainceiling;
    return ESP_OK;
}

esp_err_t ovo_set_gainceiling(OVO_H ovo, uint8_t *gainc) {
    esp_err_t err = ovo->cam_sensor->set_gainceiling(ovo->cam_sensor, *gainc);
    if(!err) {
        ovo->settings.gainceiling = *gainc;
    }
    return err;
}
**/

/** END **/
