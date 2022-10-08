
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "TestComponent.h"
#include "esp_log.h"


void testcomponent_driver_task(void *args) {
 
    testcomponent_t *tc = (testcomponent_t *)args;

    while(1) {
        printf("In task - UID: %u\n", tc->uid);
        vTaskDelay(pdMS_TO_TICKS(tc->wait));
    }
   /** here be dragons **/
}


testcomponent_t *tc_init(testcomponent_init_t *init) {

    testcomponent_t *tc = NULL;

    tc = (testcomponent_t *)heap_caps_calloc(1, sizeof(testcomponent_t), MALLOC_CAP_8BIT);

    if(tc == NULL) {
        ESP_LOGE("testcomponent", "Error alocating memory for handle!");
        return NULL;
    }
    else {
        tc->wait = init->wait;
        tc->uid = init->uid;
    }

    if(xTaskCreate(testcomponent_driver_task, "testcomponent_driver_task", 2048, tc, 3, &tc->taskhandle) != pdTRUE) {
        ESP_LOGE("testcomponent", "Error creating task!");
        return NULL;
    }

    return tc;
}