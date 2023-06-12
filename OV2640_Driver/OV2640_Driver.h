/**
*    @file    OV2460_Driver.h
*
*    @brief    header file for OV2460_Driver
*
*
*
*    @author    RJAM
*    @created   Wed 31 May 23:27:02 BST 2023
*/


#ifndef OV2460_DRIVER_H
#define OV2460_DRIVER_H
#endif

/** Includes **/

#include "esp_types.h"
#include "esp_err.h"

#include "esp_camera.h"
#include "sensor.h"

#include "driver/gpio.h"


/** Defines **/

#define CAM_BRIGHTNESS_LOW -2
#define CAM_BRIGHTNESS_HI   2
#define CAM_SATURATION_LOW  0
#define CAM_SATURATION_HI   5
#define CAM_JQUALITY_LOW    0
#define CAM_JQUALITY_HI     63
#define CAM_GAIN_LOW        0
#define CAM_GAIN_HIGH       30
#define CAM_AUTOEXPOCTL_LOW 0
#define CAM_AUTOEXPOCTL_HI  1200
#define CAM_GAINCTRL_LOW    0
#define CAM_GAINCTRL_HI     8
#define CAM_SFX_LOW         0
#define CAM_SFX_HI          7
#define CAM_WB_MODE_LOW     0
#define CAM_WB_MODE_HI      4
#define CAM_AE_LOW          0
#define CAM_AE_HI           5



/** Typedefs **/

typedef struct {  
    gpio_num_t io_d0;
    gpio_num_t io_d1;
    gpio_num_t io_d2;
    gpio_num_t io_d3;
    gpio_num_t io_d4;
    gpio_num_t io_d5;
    gpio_num_t io_d6;
    gpio_num_t io_d7;
    gpio_num_t io_pwr;
    gpio_num_t io_mclk;
    gpio_num_t io_vsync;
    gpio_num_t io_hsync;
    gpio_num_t io_pclk;
    gpio_num_t io_data;
    gpio_num_t io_clk;
} io_t;

const io_t default_io = {
    .io_d0 = GPIO_NUM_5,
    .io_d1 = GPIO_NUM_18,
    .io_d2 = GPIO_NUM_19,
    .io_d3 = GPIO_NUM_21,
    .io_d4 = GPIO_NUM_36,
    .io_d5 = GPIO_NUM_39,
    .io_d6 = GPIO_NUM_34,
    .io_d7 = GPIO_NUM_35,
    .io_pwr = GPIO_NUM_32,
    .io_mclk = GPIO_NUM_0,
    .io_vsync = GPIO_NUM_25,
    .io_hsync = GPIO_NUM_23,
    .io_pclk = GPIO_NUM_22,
    .io_data = GPIO_NUM_26,
    .io_clk = GPIO_NUM_27,
};

typedef struct {
    uint16_t brightness;
    int8_t contrast;
    pixformat_t pixformat;
    int8_t etc;
    uint8_t jpeg_quality;
    uint8_t framesize;
    uint8_t saturation;
    uint8_t autoexpose_val;
    bool colourbar_en;
    bool hmirror_en;
    bool vflip_en;
    bool whitebalance_en;
    bool aec2_en;
    bool exposurectrl_en;
    uint8_t wb_mode;
    uint16_t gainceiling;
    uint16_t gain;
    uint8_t sfx;
    uint8_t ae_level;

} camera_settings_t;

typedef struct {
    camera_settings_t settings;
    sensor_t *cam_sensor;
    int a;
} ovo_handle_t;

typedef ovo_handle_t * OVO_H;

/** Function Declarations **/
esp_err_t ovo_get_init_status(OVO_H ovo, int *status);

esp_err_t ovo_reset_software(OVO_H ovo);

esp_err_t ovo_get_jpeg_quality(OVO_H ovo, uint16_t *qual);

esp_err_t ovo_set_jpeg_quality(OVO_H ovo, uint16_t *qual);

esp_err_t ovo_get_brightness(OVO_H ovo, uint16_t *brt);

esp_err_t ovo_set_brightness(OVO_H ovo, uint16_t *brt);

esp_err_t ovo_get_contrast(OVO_H ovo, int8_t *con);

esp_err_t ovo_set_contrast(OVO_H ovo, int8_t *con);

esp_err_t ovo_get_pixformat(OVO_H ovo, pixformat_t *fmt);

esp_err_t ovo_set_pixformat(OVO_H ovo, pixformat_t *fmt);

esp_err_t ovo_get_framesize(OVO_H ovo, framesize_t *fs);

esp_err_t ovo_set_framesize(OVO_H ovo, framesize_t *fs);

esp_err_t ovo_get_saturation(OVO_H ovo, int8_t *sat);

esp_err_t ovo_set_saturation(OVO_H ovo, int8_t *sat);



esp_err_t ovo_get_colourbar_en(OVO_H ovo, bool *en);

esp_err_t ovo_set_colourbar_en(OVO_H ovo, bool *en);

esp_err_t ovo_get_hmirror_en(OVO_H ovo, bool *en);

esp_err_t ovo_set_hmirror_en(OVO_H ovo, bool *en);

esp_err_t ovo_get_vflip_en(OVO_H ovo, bool *en);

esp_err_t ovo_set_vflip_en(OVO_H ovo, bool *en);

esp_err_t ovo_get_whitebalance_en(OVO_H ovo, bool *en);

esp_err_t ovo_set_whitebalance_en(OVO_H ovo, bool *en);

esp_err_t ovo_get_aec2_en(OVO_H ovo, bool *en);

esp_err_t ovo_set_aec2_en(OVO_H ovo, bool *en);

esp_err_t ovo_get_exposure_ctrl_en(OVO_H ovo, bool *en);

esp_err_t ovo_set_exposure_ctrl_en(OVO_H ovo, bool *en);



esp_err_t ovo_get_gainceiling(OVO_H ovo, uint8_t *gainc);

esp_err_t ovo_set_gainceiling(OVO_H ovo, uint8_t *gainc);

esp_err_t ovo_get_gain(OVO_H ovo, uint8_t *gain);

esp_err_t ovo_set_gain(OVO_H ovo, uint8_t *gain);

esp_err_t ovo_get_autoexpose_val(OVO_H ovo, uint8_t *exp);

esp_err_t ovo_set_autoexpose_val(OVO_H ovo, uint8_t *exp);

esp_err_t ovo_get_sfx(OVO_H ovo, uint8_t *fx);

esp_err_t ovo_set_sfx(OVO_H ovo, uint8_t *fx);

esp_err_t ovo_get_wbmode(OVO_H ovo, uint8_t *wb);

esp_err_t ovo_set_wbmode(OVO_H ovo, uint8_t *wb);

esp_err_t ovo_get_ae_level(OVO_H ovo, uint8_t *ae);

esp_err_t ovo_set_ae_level(OVO_H ovo, uint8_t *ae);



/** END **/
