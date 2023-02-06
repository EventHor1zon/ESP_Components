/***************************************
* \file     SystemComponent.c
*
* \brief    System Component - contains system information
*           and functions for device control
* \date     Dec 2020
* \author   RJAM
****************************************/


/** Plans: 
 *      -- init function
 *      -- sleep functions
 *      -- watchdog?
 *      -- chip reset
 *      -- mac/ip addresses
 *      -- uptime/sys info
 **/


/********* Includes *******************/

#include "esp_err.h"
#include "esp_log.h"
#include "esp_types.h"
#include "esp_wifi.h"
#include "esp_pm.h"
#include "esp_heap_caps.h"
#include "esp_sleep.h"
#include "esp_task_wdt.h"

#include "driver/gpio.h"
#include "driver/rtc_io.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"
#include "freertos/task.h"

#include "SystemComponent.h"

const char  *SYS_TAG = "SYSTEM";


#ifdef CONFIG_USE_PERIPH_MANAGER
#include "../main/inc/CommandAPI.h"


const parameter_t sys_param_map[sys_cmd_len] = {
    {"total mem", 1, &sys_get_total_memory, NULL, DATATYPE_UINT32, 0, (GET_FLAG)},
    // {"WiFi MAC", 2, &sys_get_wifi_mac, NULL, DATATYPE_STRING, 0, (GET_FLAG)},
    // {"BT MAC", 3, &sys_get_bt_mac, NULL, DATATYPE_STRING, 0, (GET_FLAG)},
};


peripheral_t sys_peripheral_template = {

    .handle = NULL,
    .param_len = sys_cmd_len,
    .params = sys_param_map,
    .peripheral_name = "System",
    .peripheral_id = 0x55,
    .periph_type = PTYPE_NONE,
};

#endif



/****** Function Prototypes ***********/

/************ ISR *********************/

void uptime_timer_callback(void *timer) {
    /** use the freertos example to create a crude uptime counter **/ 
    // uint32_t counter;
    // configASSERT(timer);
    // counter = (uint32_t )pvTimerGetTimerID(timer);
    // counter++;
    // vTimerSetTimerID(timer, (void *)counter);
    return NULL; 
}

/****** Private Data ******************/

/****** Private Functions *************/
 
static void system_task(void *args) {

    esp_err_t kickstatus;

    while(1){

        /** kick the task watchdog **/
        kickstatus = esp_task_wdt_reset();
        if(kickstatus == ESP_ERR_NOT_FOUND ) {
            ESP_LOGI(SYS_TAG, "Could not reset WDT - Task not subscribed");
        } else if (kickstatus == ESP_ERR_INVALID_STATE) {
            ESP_LOGI(SYS_TAG, "Could not reset WDT - WDT not initialised");
        }

        vTaskDelay(1000);
    }
}

/****** Global Data *******************/

/****** Global Functions *************/


system_handle_t *system_init() {

    esp_err_t init = ESP_OK;
    system_handle_t *handle = (system_handle_t *)heap_caps_calloc(1, sizeof(system_handle_t), MALLOC_CAP_8BIT);

    if(handle == NULL) {
        ESP_LOGE(SYS_TAG, "Error assigning memory for system structure!");
    } else {
        TimerHandle_t uptimer = xTimerCreate("uptimer", pdMS_TO_TICKS(1000), pdTRUE, (void *)0, (TimerCallbackFunction_t)uptime_timer_callback);
        if(uptimer == NULL) {
            ESP_LOGE(SYS_TAG, "Error creating timer");
            init = ESP_FAIL;
        } else {
            handle->uptimer = uptimer;
        }
    
        /** make task & add to the task wdt **/
        if(init == ESP_OK) {
            TaskHandle_t system_task_handle;
            if(xTaskCreate(system_task, "system_task", configMINIMAL_STACK_SIZE, (void *)handle, 5, &system_task_handle) != pdTRUE) {
                ESP_LOGE(SYS_TAG, "Error creating the system task!");
                init = ESP_ERR_NO_MEM;
            } else {
                /** add the system task to the task_wdt **/
                init = esp_task_wdt_init(SYS_CONFIG_WDTT_TIMEOUT_S, true);
                if(init != ESP_OK) {
                    ESP_LOGE(SYS_TAG, "Error initialising task watchdog");
                } else {
                    init = esp_task_wdt_add(system_task_handle);
                    if(init != ESP_OK) {
                        ESP_LOGE(SYS_TAG, "Error adding system task to watchdog");
                    }
                }
            }
        }
    
    }

    return handle;

}


#ifdef CONFIG_USE_PERIPH_MANAGER

/** TODO: If this breaks shit, try using CONFIG_PM_DFS_INIT_AUTO **/
esp_err_t system_set_autosleep_wow(system_handle_t *handle, uint8_t *val) {


    esp_err_t status = ESP_OK;
    EventBits_t bits = xEventGroupGetBits(s_wifi_event_group);

    if(!(bits & WIFI_CONNECTED_BIT)) {
        ESP_LOGI(SYS_TAG, "Can't Wake on Wifi: Not connected");
        status = ESP_FAIL;
    } 
    else 
    {
        esp_pm_config_esp32_t pwr_cfg = {0};
        pwr_cfg.max_freq_mhz = CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ;
        pwr_cfg.min_freq_mhz = 10;

        if(*val) {
            ESP_LOGI(SYS_TAG, "Setting auto-sleep with wake-on-wifi");
            status = esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
            if(status != ESP_OK) {
                ESP_LOGE(SYS_TAG, "Error setting wifi modem-sleep [%u]", status);
            } 
            else 
            {
                pwr_cfg.light_sleep_enable = true;
                status = esp_pm_configure(&pwr_cfg);
                if(status != ESP_OK) {
                    ESP_LOGE(SYS_TAG, "Error configuring power manager [%u]", status);
                }
            }
        } 
        else 
        {
            ESP_LOGI(SYS_TAG, "Unsetting auto-sleep power save");
            status = esp_wifi_set_ps(WIFI_PS_NONE);
            if(status != ESP_OK) {
                ESP_LOGE(SYS_TAG, "Error setting wifi power-save off");
            } 
            else 
            {
                pwr_cfg.light_sleep_enable = false;
                status = esp_pm_configure(&pwr_cfg);
                if(status != ESP_OK) {
                    ESP_LOGE(SYS_TAG, "Error configuring power manager [%u]", status);
                }
            }
        }
    }

    if(status == ESP_OK) {
        if (val) {
            handle->sleep_state = SYS_AUTOLIGHT_WOW;
        }
        else{ 
            handle->sleep_state = SYS_NO_SLEEP;
        }
    }


    return status;
}
#endif  /** CONFIG_PM_ENABLE **/

void dump_system_info()
{

    esp_chip_info_t chip = {0};
    uint8_t wmac[6] = {0};
    uint8_t btmac[6] = {0};
    char *model, *cores, *embflash, *wifi, *ble, *btclassic;

    esp_chip_info(&chip);

    btclassic = (chip.features & CHIP_FEATURE_BT) ? "N" : "Y";
    ble = (chip.features & CHIP_FEATURE_BLE) ? "N" : "Y";
    wifi = (chip.features & CHIP_FEATURE_WIFI_BGN) ? "N" : "Y";
    embflash = (chip.features & CHIP_FEATURE_EMB_FLASH) ? "N" : "Y";

    esp_read_mac(wmac, ESP_MAC_WIFI_STA);
    esp_read_mac(btmac, ESP_MAC_BT);

    switch (chip.model)
    {
    case CHIP_ESP32:
        model = "ESP 32";
        break;
    case CHIP_ESP32S2:
        model = "ESP 32 s2";
        break;
    default:
        model = "Not recognised!";
        break;
    }

    switch (chip.cores)
    {
    case 1:
        cores = ": Single Core\n";
        break;
    case 2:
        cores = ": Dual Core\n";
        break;
    case 3:
        cores = ": Tri Core?\n";
        break;
    case 4:
        cores = ": Quad Core???\n";
        break;
    default:
        cores = ": godlike Core\n";
        break;
    }

    printf("\n[\t\tESP32 Info\t\t]\n");
    printf("[\t\t\t\t\t]\n");
    printf("[\tModel: %s %s\t]\n", model, cores);
    printf("[\tRev  : %d\t\t]\n", chip.revision);
    printf("[\tFeatures:\t\t\t]\n");
    printf("[\t\tBT:\t%s\t]\n", btclassic);
    printf("[\t\tBLE:\t%s\t]\n", ble);
    printf("[\t\tEmbedded Flash: %s\t]\n", embflash);
    printf("[\t\tWifi: \t%s\t]\n", wifi);
    printf("[\t\t\t\t\t]\n");
    printf("[\tWifi Mac: %02x:%02x:%02x:%02x%02x:%02x\t\t]\n", wmac[0], wmac[1], wmac[2], wmac[3], wmac[4], wmac[5]);
    printf("[\tBT Mac  : %02x:%02x:%02x:%02x%02x:%02x\t\t]\n", btmac[0], btmac[1], btmac[2], btmac[3], btmac[4], btmac[5]);
    printf("[\t\t\t\t\t]\n");
    printf("[====\t====\t====\t====\t]\n");
}



esp_err_t sys_get_total_memory(system_handle_t *handle, uint32_t *val)
{
    esp_err_t status = ESP_OK;

    uint32_t mem_total = 0, mem_dma = 0, mem_8bit = 0, mem_cap_iram = 0, mem_spi_ram;

    mem_dma = heap_caps_get_total_size(MALLOC_CAP_DMA);
    mem_8bit = heap_caps_get_total_size(MALLOC_CAP_8BIT);
    mem_cap_iram = heap_caps_get_total_size(MALLOC_CAP_IRAM_8BIT);
    mem_spi_ram = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);

    mem_total = mem_dma + mem_8bit + mem_cap_iram + mem_spi_ram;    
    *val = mem_total;
    return status;
}



esp_err_t sys_sleep_until_woken(system_handle_t *handle)
{
    esp_err_t status = ESP_OK;


    return status;
}

esp_err_t system_enable_ext0_io(system_handle_t *handle) {

    esp_err_t status = ESP_OK;

    if (sizeof(handle->wakepins) == 0) {
        ESP_LOGE(SYS_TAG, "Error: No pins assigned to wake!");
        status = ESP_FAIL;
    } else {
        for(uint8_t i=0; i<(sizeof(handle->wakepins)/sizeof(wake_pin_t)); i++) {
            status = esp_sleep_enable_ext0_wakeup(handle->wakepins[i].pin, handle->wakepins[i].level);
            ESP_LOGI(SYS_TAG, "Info: Adding gpio_num %u as wakeup on %u level", handle->wakepins[i].pin, handle->wakepins[i].level);
            if(status != ESP_OK) {
                ESP_LOGE(SYS_TAG, "Error adding pin %u as a wakeup source", handle->wakepins[i].pin);
                break;
            }
        }
    }

    return status;
}


esp_err_t system_enable_ext1_io(system_handle_t *handle) {

    esp_err_t status = ESP_OK;
    uint32_t mask = 0;
    if (sizeof(handle->wakepins) == 0) {
        ESP_LOGE(SYS_TAG, "Error: No pins assigned to wake!");
        status = ESP_FAIL;
    } 
    else {
        for(uint8_t i=0; i<(sizeof(handle->wakepins)/sizeof(wake_pin_t)); i++) {
            if(handle->wakepins[i].waketype == WAKETYPE_EXT1) {
                mask |= handle->wakepins[i].pin;
            }
        }
        ESP_LOGI(SYS_TAG, "Info: Adding gpio pins as ext1 wakeup on low");
        status = esp_sleep_enable_ext1_wakeup(mask, ESP_EXT1_WAKEUP_ALL_LOW);
    }

    return status;
}


esp_err_t system_register_wakeup_pin(system_handle_t *handle, wake_pin_t *pin_conf) {

    /** wakeup - ext0 pins can only wake from light/deep-with-rtc-io 
     *           ext1 pins can wake from deepsleep-rtc-sleeping
     *           gpio can only wake from lightsleep
     **/
    esp_err_t status = ESP_OK;

    if(pin_conf->waketype == WAKETYPE_EXT0 && rtc_gpio_is_valid_gpio(pin_conf->pin) > 0) {
        // handle->wakepins[handle->no_wakepins] = {
        //     .pin = pin_conf->pin,
        //     .level = pin_conf->level,
        //     .waketype = pin_conf->waketype,
        //     .pull = pin_conf->pull,
        // };
        handle->no_wakepins++;
    }
    else if (pin_conf->waketype == WAKETYPE_EXT1 && rtc_gpio_is_valid_gpio(pin_conf->pin) > 0) {
        // handle->wakepins[handle->no_wakepins] = {
        //     .pin = pin_conf->pin,
        //     .level = pin_conf->level,
        //     .waketype = pin_conf->waketype,
        //     .pull = pin_conf->pull,
        // };
        handle->no_wakepins++;
    }
    else if (pin_conf->waketype == WAKETYPE_GPIO) {
        // handle->wakepins[handle->no_wakepins] = {
        //     .pin = pin_conf->pin,
        //     .level = pin_conf->level,
        //     .waketype = pin_conf->waketype,
        //     .pull = pin_conf->pull,
        // };
        handle->no_wakepins++;       
    }
    else {
        ESP_LOGE(SYS_TAG, "Error invalid pin/sleep-mode");
        status = ESP_FAIL;
    }

    return status;

}


void system_deep_sleep(system_handle_t *handle) {

    /**
     *  for pin in wake sources (if wakeuptype= ext), activate the correct pullup, 
     *  config power-save settings 
     *  shutdown wifi/bluetooth if active?
     *  engage sleep
     **/

    esp_err_t status = ESP_OK;

    for(uint8_t i=0; i<handle->no_wakepins; i++) {

        wake_pin_t pin = handle->wakepins[i];

        if(pin.waketype == WAKETYPE_EXT0 && pin.pull != GPIO_FLOATING) {
            status = pin.pull ? rtc_gpio_pulldown_en(pin.pin) : rtc_gpio_pullup_en(pin.pin);
        }
        else if(pin.waketype == WAKETYPE_EXT1 && pin.pull != GPIO_FLOATING) {
            status = pin.pull ? rtc_gpio_pulldown_en(pin.pin) : rtc_gpio_pullup_en(pin.pin);
        }
    }

    /** don't really care about return here, if it's running, it gets stopped **/
    esp_wifi_stop();

    esp_deep_sleep_start();
}
