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
#include "Utilities.h"
#include "esp_err.h"
#include "esp_log.h"

#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"

/****** Function Prototypes ***********/
static int test_frame_polling(StrandData_t *strand);
// static esp_err_t send_32bit_frame(spi_device_handle_t spi, uint32_t *data);
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

static uint32_t init_frame[16] = {
    0xFF0000E1,
    0x00FF00E1,
    0x0000FFE1,
    0x110011E1,
    0x111100E1,
    0x001111E1,
    0x111111E1,
    0xFF0000E1,
    0xFF0000E1,
    0xFF0000E1,
    0xFF0000E1,
    0xFF0000E1,
    0xFF0000E1,
    0xFF0000E1,
    0xFF0000E1,
    0xFF0000E1,
};
/****** Private Functions *************/

/****** Global Data *******************/

StrandData_t *strand = NULL;

const char *APA_TAG = "APA102 Driver";
const uint32_t zerodata = 0x00000000;
const uint32_t onedata = 0xFFFFFFFF;

/****** Global Functions *************/

// static int test_frame(spi_device_handle_t ledhandle);

esp_err_t APA102_init(uint8_t numleds, int clock_pin, int data_pin, uint8_t spi_bus, bool init_spi)
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

        spi_bus_config_t buscfg = {0};
        buscfg.mosi_io_num = data_pin;
        buscfg.miso_io_num = -1;
        buscfg.sclk_io_num = clock_pin;
        buscfg.max_transfer_sz = 512; // led data + up to 10 frames of start & end
        buscfg.quadhd_io_num = -1;
        buscfg.quadwp_io_num = -1;
        buscfg.flags = 0;
        buscfg.intr_flags = 0;
        ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &buscfg, 0));
    }

    if (init_status == ESP_OK)
    {
        ESP_LOGI("SPI_SETUP", "[+] Setting up LEDs as an SPI device");

        spi_device_interface_config_t leds = {0};
        leds.command_bits = 0;
        leds.address_bits = 0;
        leds.dummy_bits = 0;
        leds.mode = 1;
        leds.duty_cycle_pos = 128;
        leds.cs_ena_posttrans = 0;
        leds.cs_ena_pretrans = 0;
        leds.spics_io_num = -1;
        leds.queue_size = 16;
        leds.clock_speed_hz = 200000; // APA claim to have refresh rate of 4KHz, start low.
        leds.input_delay_ns = 0;
        leds.queue_size = 10; // bit arbitrary...

        spi_device_handle_t ledSPIHandle;
        ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &leds, &ledSPIHandle));

        strand = (StrandData_t *)heap_caps_calloc(1, sizeof(StrandData_t), MALLOC_CAP_8BIT);
        uint32_t *ledMem = (uint32_t *)heap_caps_calloc(1, (numleds * APA_BYTES_PER_PIXEL), MALLOC_CAP_DMA);
        ledEffectData_t *lfx = ledEffectInit(strand);

        if (strand != NULL && lfx != NULL && ledMem != NULL)
        {
            strand->ledType = LEDTYPE_APA102;
            strand->numLeds = numleds;
            strand->spi_channel_no = spi_bus;
            strand->strandMemLength = APA_BYTES_PER_PIXEL * numleds;
            strand->strandMem = ledMem;
            strand->fxData = lfx;
            strand->ledSPIHandle = ledSPIHandle;
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

    if (init_status == ESP_OK)
    {
        test_frame_polling(strand);
    }

    return init_status;
}

static int test_frame_polling(StrandData_t *strand)
{
    ESP_LOGI("SPI_SETUP", "[+] Sending init data");

    printf("Gonna dump some shit:\n\n");
    printf("The strand: \n");
    showmem((uint8_t *)strand, sizeof(StrandData_t));

    spi_transaction_t tx = {0};
    uint16_t length = (sizeof(uint32_t) * 8);

    tx.length = 32;
    tx.flags = SPI_TRANS_USE_TXDATA;
    tx.addr = 0;
    tx.cmd = 0;
    tx.rxlength = 0;
    tx.rx_buffer = NULL;
    tx.user = NULL;

    // printf("Attempting to send 0x%02x%02x%02x%02x tx struct...\n", tx.tx_data[0], tx.tx_data[1], tx.tx_data[2], tx.tx_data[3]);
    printf("This is our spi handle... %p\n", strand->ledSPIHandle);
    showmem((uint8_t *)&tx, sizeof(spi_transaction_t));

    esp_err_t txStatus = spi_device_polling_transmit(strand->ledSPIHandle, &tx);

    if (txStatus != ESP_OK)
    {
        ESP_LOGE("SPI_TX", "Error in sending start frame %u", txStatus);
    }

    tx.length = 32;
    tx.flags = 0;
    for (int i = 0; i < strand->numLeds; i++)
    {
        printf("setting tx_buffer to %p which contains %zu\n", &init_frame[i], init_frame[i]);
        tx.tx_buffer = (void *)&init_frame[i];
        txStatus = spi_device_polling_transmit(strand->ledSPIHandle, &tx);
        if (txStatus != ESP_OK)
        {
            ESP_LOGE("SPI_TX", "Error in sending data frame number %d [%u]", i, txStatus);
        }
    }

    tx.length = sizeof(uint32_t) * 8;
    tx.flags = SPI_TRANS_USE_TXDATA;
    tx.tx_data[0] = 0xFF;
    tx.tx_data[1] = 0xFF;
    tx.tx_data[2] = 0xFF;
    tx.tx_data[3] = 0xFF;

    for (int i = 0; i < 3; i++)
    {
        txStatus = spi_device_polling_transmit(strand->ledSPIHandle, &tx);
    }
    if (txStatus != ESP_OK)
    {
        ESP_LOGE("SPI_TX", "Error in sending end frame %u", txStatus);
    }

    return txStatus;
}

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