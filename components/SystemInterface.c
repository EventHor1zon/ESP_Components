/***************************************
* \file     SystemInterface.c
* \brief    A driver-like interface with the ESP system. 
*
* \date     Oct 2020
* \author   RJAM
****************************************/

/********* Includes *******************/
#include "esp_types.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_err.h"

/****** Function Prototypes ***********/

/************ ISR *********************/

/****** Private Data ******************/

/****** Private Functions *************/

/****** Global Data *******************/
static uint8_t wmac[6] = {0};
static uint8_t btmac[6] = {0};
static esp_chip_info_t chip = {0};
/****** Global Functions *************/

void dump_system_info()
{

    char *model, *cores, *embflash, *wifi, *ble, *btclassic;

    esp_chip_info(&chip);

    btclassic = (chip.features & CHIP_FEATURE_BT) ? "N" : "Y";
    ble = (chip.features & CHIP_FEATURE_BLE) ? "N" : "Y";
    wifi = (chip.features & CHIP_FEATURE_WIFI_BGN) ? "N" : "Y";
    embflash = (chip.features & CHIP_FEATURE_EMB_FLASH) ? "N" : "Y";

    esp_read_mac(wmac, 0);
    esp_read_mac(btmac, 2);

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

esp_err_t sys_get_total_memory(uint16_t *val)
{
    esp_err_t status = ESP_OK;

    uint32_t mem_total = 0, mem_dma = 0, mem_8bit = 0, mem_cap_iram = 0, mem_spi_ram;

    mem_dma = heap_caps_get_total_size(MALLOC_CAP_DMA);
    mem_8bit = heap_caps_get_total_size(MALLOC_CAP_8BIT);
    mem_cap_iram = heap_caps_get_total_size(MALLOC_CAP_IRAM_8BIT);
    mem_spi_ram = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);

    return status;
}