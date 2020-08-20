/***************************************
* \file .c
* \brief
*
* \date
* \author
****************************************/

/********* Includes *******************/

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "APA102_Driver.h"
#include "LedEffects.h"
#include "esp_err.h"
#include "esp_log.h"

#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"

/****** Function Prototypes ***********/

/************ ISR *********************/

/**
*   fxCallbackFunction
*   
*       Callback function for the timer expiry
*       set flag for Led Control task to deal with
*       Don't call the led effects functions here
**/
void timerExpiredCallback(TimerHandle_t timer)
{
    // StrandData_t *strand = NULL;

    // /** find the right strand from the timerHandle **/
    // for (uint8_t i = 0; i < ledControl.numStrands; i++)
    // {
    //     if (timer == allStrands[i]->refreshTimer)
    //     {
    //         strand = allStrands[i];
    //     }
    // }

    // if (strand != NULL)
    // {
    //     strand->updateLeds = 1;
    // }

    return;
}
/****** Private Data ******************/

/****** Private Functions *************/

/****** Global Data *******************/

const char *APA_TAG = "APA102 Driver";

/****** Global Functions *************/

static int
test_frame(spi_device_handle_t ledhandle);

static esp_err_t send_32bit_frame(spi_device_handle_t spi, uint32_t data)
{

    spi_transaction_t tx = {};
    memset(&tx, 0, sizeof(tx));

    tx.length = 32;
    tx.tx_buffer = &data;
    tx.user = (void *)0;

    esp_err_t ret = spi_device_transmit(spi, &tx);
#ifdef DEBUG_MODE
    assert(ret == ESP_OK);
#endif

    return ret;
}

esp_err_t APA102_init(uint8_t numleds, gpio_num_t clock_pin, gpio_num_t data_pin, uint8_t spi_bus, bool init_spi)
{

    esp_err_t init_status = ESP_OK;

    if (!(spi_bus == SPI2_HOST || spi_bus == SPI3_HOST))
    {
        ESP_LOGE(APA_TAG, "Error - invalid SPI bus. Please use SPI2_HOST or SPI3_HOST");
        init_status = ESP_ERR_INVALID_ARG;
    }

    if (init_spi && (init_status == ESP_OK))
    {
        ESP_LOGI("SPI_SETUP", "[+] Setting up SPI bus");

        spi_bus_config_t buscfg;
        buscfg.mosi_io_num = data_pin;
        buscfg.miso_io_num = -1;
        buscfg.sclk_io_num = clock_pin;
        buscfg.max_transfer_sz = ((numleds * APA_BYTES_PER_PIXEL) + (10 * APA_BYTES_PER_PIXEL)); // led data + up to 10 frames of start & end
        buscfg.quadhd_io_num = -1;
        buscfg.quadwp_io_num = -1;
        buscfg.flags = 0;
        buscfg.intr_flags = 0;
        ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &buscfg, 0));
    }

    if (init_status == ESP_OK)
    {
        ESP_LOGI("SPI_SETUP", "[+] Setting up LEDs as an SPI device");

        spi_device_interface_config_t leds;
        leds.command_bits = 0;
        leds.address_bits = 0;
        leds.dummy_bits = 0;
        leds.mode = 3;
        leds.duty_cycle_pos = 128;
        leds.cs_ena_posttrans = 0;
        leds.cs_ena_pretrans = 0;
        leds.spics_io_num = -1;
        leds.queue_size = 16;
        leds.clock_speed_hz = 200000; // APA claim to have refresh rate of 4KHz, start low.
        leds.input_delay_ns = 0;

        spi_device_handle_t ledSPIHandle;
        ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &leds, &ledSPIHandle));

        StrandData_t *strand = (StrandData_t *)heap_caps_calloc(1, sizeof(StrandData_t), MALLOC_CAP_8BIT);
        uint32_t *ledMem = (uint32_t *)heap_caps_calloc(1, (numleds * APA_BYTES_PER_PIXEL), MALLOC_CAP_DMA);
        ledEffectData_t *lfx = ledEffectInit(strand);

        if (strand != NULL && lfx != NULL && ledMem != NULL)
        {
            strand->numLeds = numleds;
            strand->spi_channel_no = spi_bus;
            strand->strandMemLength = APA_BYTES_PER_PIXEL * numleds;
            strand->strandMem = ledMem;
            strand->fxData = lfx;
        }
        else
        {
            ESP_LOGE(APA_TAG, "Error in assigning Driver memory");
            init_status = ESP_ERR_NO_MEM;
        }

        if (init_status == ESP_OK)
        {
            TimerHandle_t refreshTimer = xTimerCreate("refreshTimer", UINT16_MAX, pdTRUE, APA_TIMER_ID, timerExpiredCallback);
            SemaphoreHandle_t ledSemaphore = xSemaphoreCreateMutex();

            if (refreshTimer != NULL && ledSemaphore != NULL)
            {
                strand->refreshTimer = refreshTimer;
                strand->memSemphr = ledSemaphore;
            }
            else
            {
                ESP_LOGE(APA_TAG, "Error in setting up timer/semaphore");
                init_status = ESP_ERR_NOT_FOUND;
            }
        }
    }

    return init_status;
}

// int test_frame(spi_device_handle_t spi)
// {
//     ESP_LOGI("SPI_SETUP", "[+] Sending init data");
//     showmem(&init_frame, (24 * 4));

//     for (int i = 0; i < 24; i++)
//     {
//         uint32_t *data = &init_frame[i];
//         ESP_LOGI("SPI SEND", "[>] Sending %08x", *data);
//         send_32bit_frame(spi, *data);
//     }
//     ESP_LOGI("SPI_SETUP", "[+] Transaction done!");
//     return 1;
// }

uint8_t apa102_ctrl_generate_brt_segment(uint8_t bright)
{

    if (bright > APA_CTRL_MAX_BR)
    {
        return 0;
    }

    uint8_t segment = (uint8_t)APA_CTRL_BRT_MASK;
    segment |= bright;

    return segment;
}

uint32_t apa102_ctrl_generate_led_frame(void *rgb_data, uint8_t brt)
{

    uint8_t br = apa102_ctrl_generate_brt_segment(brt);
    if (!brt)
    {
        return 0;
    }

    uint8_t *data = (uint8_t *)rgb_data;
    /* color bit order for the APA102 is B, G, R in MSBF*/
    uint8_t r = *data;
    data++;
    uint8_t g = *data;
    data++;
    uint8_t b = *data;

    uint32_t frame = ((uint32_t)br << 24 | ((uint32_t)b << 16) | ((uint32_t)g << 8) | (uint32_t)r);
    return frame;
}