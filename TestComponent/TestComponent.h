
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"



typedef struct {
    uint16_t uid;
    uint16_t wait;
    TaskHandle_t taskhandle;
} testcomponent_t;


typedef struct {
    uint16_t uid;
    uint16_t wait;
} testcomponent_init_t;


testcomponent_t *tc_init(testcomponent_init_t *init);

