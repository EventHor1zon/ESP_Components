
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"

#ifdef CONFIG_USE_PERIPH_MANAGER
#include "CommandAPI.h"
const parameter_t test_param_map[0];
const peripheral_t test_periph_template;
#endif

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

