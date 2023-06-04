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

/** Defines **/

/** Typedefs **/

typedef struct default_io {
    
    gpio_num_t io_d0 = GPIO_NUM_5;
    gpio_num_t io_d1 = GPIO_NUM_18;
    gpio_num_t io_d2 = GPIO_NUM_2;
    gpio_num_t io_d3 = GPIO_NUM_21;
    gpio_num_t io_d4 = GPIO_SENSOR_VP;
    gpio_num_t io_d5 = GPIO_SENSOR_VN;
    gpio_num_t io_d6 = GPIO_NUM_34;
    gpio_num_t io_d7 = GPIO_NUM_35;
    gpio_num_t io_pwr = GPIO_NUM_32;
    gpio_num_t io_mclk = GPIO_NUM_0;
    gpio_num_t io_vsyc = GPIO_NUM_25;
    gpio_num_t io_hsync = GPIO_NUM_23;
    gpio_num_t io_pclk = GPIO_NUM_22;
    gpio_num_t io_data = GPIO_NUM_26;
    gpio_num_t io_clk = GPIO_NUM_27;
} default_io_t;

typedef struct ovo_init {
    gpio_num_t pinmap[14];
    
} ovo_init_t;

/** Function Declarations **/

/** END **/
