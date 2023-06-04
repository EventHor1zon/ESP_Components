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

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "FreeRTOS/queue.h"

/** Private Data **/

TaskHandle_t task_handle = NULL;
QueueHandle_t queue_handle = NULL;


/** Function Prototypes **/

void ovo_driver_task(void *args) {

    while(1) {
    
    
    }

    /** here be dragons **/
}

/** Static Functions **/

/** Tasks **/

/** Public Functions **/

esp_err_t ovo_init(OVO_H ovo, ovo_init_t *init) {

    esp_err_t = ESP_OK;
    



}


/** END **/
