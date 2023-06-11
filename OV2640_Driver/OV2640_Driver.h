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
    int8_t brightness;
    int8_t contrast;
    pixformat_t pixformat;
    int8_t etc;
    uint8_t jpeg_quality;
} camera_settings_t;

typedef struct {
    camera_settings_t settings;
    sensor_t *cam_sensor;
    int a;
} ovo_handle_t;

typedef ovo_handle_t * OVO_H;

/** Function Declarations **/

/** END **/
