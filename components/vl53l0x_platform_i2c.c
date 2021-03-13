
#include "inc/vl53l0x_i2c_platform.h"
#include "inc/vl53l0x_platform.h"

#include "FreeRTOS/FreeRTOS.h"
#include "genericCommsDriver.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_sleep.h"
#include "rom/ets_sys.h"

#define VL_GPIO_PIN 10

int32_t VL53L0X_comms_initialise(uint8_t  comms_type,uint16_t comms_speed_khz) {

    return 0;
}


int32_t VL53L0X_comms_close(void) {
    return 0;
}


int32_t VL53L0X_write_multi(uint8_t address, uint8_t index, uint8_t  *pdata, int32_t count) {

    int32_t ret = 0;
    if(genericI2CwriteToAddress(I2C_NUM_0, address, index, count, pdata) != ESP_OK) {
        ret = 1;
    }
    return ret;
}


int32_t VL53L0X_read_multi(uint8_t address,  uint8_t index, uint8_t  *pdata, int32_t count) {
    int32_t ret = 0;
    if(genericI2CReadFromAddress(I2C_NUM_0, address, index, count, pdata) != ESP_OK) {
        ret = 1;
    }
    return ret;
}

int32_t VL53L0X_write_byte(uint8_t address,  uint8_t index, uint8_t data){

    int32_t ret = 0;
    uint8_t pdata = data;
    if(genericI2CReadFromAddress(I2C_NUM_0, address, index, 1, &pdata) != ESP_OK) {
        ret = 1;
    }
    return ret; 
}

int32_t VL53L0X_write_word(uint8_t address,  uint8_t index, uint16_t  data) {

    int32_t ret = 0;
    uint8_t pdata[2] = {0};
    pdata[0] = (uint8_t )(data >> 8);
    pdata[1] = (uint8_t )(data);
    if(genericI2CReadFromAddress(I2C_NUM_0, address, index, 2, pdata) != ESP_OK) {
        ret = 1;
    }
    return ret;   
}

int32_t VL53L0X_write_dword(uint8_t address, uint8_t index, uint32_t  data) {

    int32_t ret = 0;
    uint8_t pdata[4] = {0};
    pdata[0] = (uint8_t )(data >> 24);
    pdata[1] = (uint8_t )(data >> 16);
    pdata[2] = (uint8_t )(data >> 8);
    pdata[3] = (uint8_t )(data);
    if(genericI2CReadFromAddress(I2C_NUM_0, address, index, 4, pdata) != ESP_OK) {
        ret = 1;
    }
    return ret;   

}


int32_t VL53L0X_read_byte(uint8_t address,  uint8_t index, uint8_t  *pdata) {
    int32_t ret = 0;
    if(genericI2CReadFromAddress(I2C_NUM_0, address, index, 1, pdata) != ESP_OK) {
        ret = 1;
    }
    return ret;  
}

int32_t VL53L0X_read_word(uint8_t address,  uint8_t index, uint16_t *pdata) {
    int32_t ret = 0;
    if(genericI2CReadFromAddress(I2C_NUM_0, address, index, 2, pdata) != ESP_OK) {
        ret = 1;
    }
    return ret;  
}

int32_t VL53L0X_read_dword(uint8_t address, uint8_t index, uint32_t *pdata) {
    int32_t ret = 0;
    if(genericI2CReadFromAddress(I2C_NUM_0, address, index, 4, pdata) != ESP_OK) {
        ret = 1;
    }
    return ret;     
}

int32_t VL53L0X_platform_wait_us(int32_t wait_us) {
    ets_delay_us(wait_us);
    return 0;
}

int32_t VL53L0X_wait_ms(int32_t wait_ms) {
    vTaskDelay(pdMS_TO_TICKS(wait_ms));
    return 0;
}

int32_t VL53L0X_set_gpio(uint8_t  level) {
    gpio_set_level(VL_GPIO_PIN, level);
    return 0;
}

int32_t VL53L0X_get_gpio(uint8_t *plevel) {
    uint16_t l = gpio_get_level(VL_GPIO_PIN);
    *plevel = (uint8_t )l;
    return 0;
}

