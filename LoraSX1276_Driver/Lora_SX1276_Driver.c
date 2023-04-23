/***************************************
* \file     Lora_SX1276_Driver.c
* \brief    A driver for the SX1276 Lora Chipset 
*              This one is going to be fun...
*               Thanks to https://github.com/sandeepmistry/arduino-LoRa for guidance
*               via codesurfing
*
*           Transmitting data - don't be in sleep mode (fifo inaccesible)
*                        data is read/written to the pointer address. Pointer auto-increments
*                        RegRxNbBytes is rx len accepted
*                        RegPayloadLength is tx len
*           Lora interrupts - 
*                        RegIrqFlagsMask set bits to 1 to DEACTIVATE the interrupt
*                        RegIrqFlags bits set to 1 are triggered irq
*
*        Static configuration registers can only be accessed in Sleep mode, Standby mode or FSTX mode.
*        The LoRaTM FIFO can only be filled in Standby mode.
*        Data transmission is initiated by sending TX mode request.
*        Upon completion the TxDone interrupt is issued and the radio returns to Standby mode.
*        Following transmission the radio can be manually placed in Sleep mode or the FIFO refilled for a subsequent Tx
*
*           Driver plans: Don't break everything out at once, that will take forever. Focus on key settings
*                           required for functionality and stop there. 
*
*           Device Model - A model of the device registers is kept in the handle struct  
*                           This should be set to reset value on reset and updated when 
*                           either confirming update or in update function
*                           
*           Pins: For the TTGO Lora model ESP - 
*               + RST : IO23
*               + NSS : IO18
*               + SCK : IO05
*               + MOSI: IO27
*               + MISO: IO19
*               + IO0 : IO26
*
*           
* \date     March 2021
* \author   RJAM
****************************************/

/********* Includes *******************/

#include "esp_err.h"
#include "esp_types.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "Lora_SX1276_Driver.h"
#include "Utilities.h"

// #define SPI_DEBUG 1

#ifdef CONFIG_USE_PERIPH_MANAGER

#include "CommandAPI.h"

const parameter_t lora_parameter_map[LORA_PERIPH_LEN] = {

    {"Device Mode", 1, sx_get_device_mode, sx_set_device_mode, NULL, DATATYPE_INT8, SX_DEVICE_LORA_MODE, (GET_FLAG | SET_FLAG)},
    {"Device Version", 2, sx_get_version, NULL, NULL, DATATYPE_UINT8, 0, (GET_FLAG)},
    {"TRX Mode", 3, sx_get_trx_mode, sx_set_trx_mode, NULL, DATATYPE_UINT8, SX1276_TRXMODE_CAD, (GET_FLAG | SET_FLAG)},
    {"Frequency", 4, sx_get_frequency, sx_set_frequency, NULL, DATATYPE_UINT32, SX_LORA_MAX_FREQUENCY-1, (GET_FLAG | SET_FLAG)},
    {"PowerAmp Selected", 5, sx_get_pa_sel, sx_set_pa_sel, NULL, DATATYPE_BOOL, 1, (GET_FLAG | SET_FLAG) },
    {"LoRa Header mode", 6, sx_get_lora_headermode, sx_set_lora_headermode, DATATYPE_UINT8, 1, (GET_FLAG | SET_FLAG) },
    {"O-CurrentProt En", 7, sx_get_ocp_en, sx_set_ocp_en, NULL, DATATYPE_BOOL, 1, (GET_FLAG | SET_FLAG)},
    {"O-Current Trim", 8, sx_get_ocp_trim, sx_set_ocp_trim, NULL, DATATYPE_UINT8, 0x0F, (GET_FLAG | SET_FLAG)},
    {"LNA Gain", 9, sx_get_lna_gain, sx_get_lna_gain, NULL, DATATYPE_UINT8, 6, (GET_FLAG | SET_FLAG)},
    {"LNA HF Boost En", 10, sx_get_lna_boost_hf, sx_set_lna_boost_hf, DATATYPE_BOOL, 1, (GET_FLAG | SET_FLAG)},
    {"LoRa Bandwidth", 11, sx_get_signal_bandwidth, sx_set_signal_bandwidth, NULL, DATATYPE_UINT8, SX1276_LORA_BW_500KHZ, (GET_FLAG | SET_FLAG)},
    {"LoRa Spread Fc", 12, sx_get_lora_spreading_factor, sx_set_lora_spreading_factor, NULL, DATATYPE_UINT8, 12, (GET_FLAG | SET_FLAG)},
    {"Rx CRC En", 13, sx_get_rx_payload_crc_en, sx_set_rx_payload_crc_en, NULL,  DATATYPE_BOOL, 1, (GET_FLAG | SET_FLAG)},
    {"Low Rate Opt En", 14, sx_get_low_datarate_optimise, sx_set_low_datarate_optimise, NULL, DATATYPE_BOOL, 1, (GET_FLAG | SET_FLAG) },
    {"AGC Auto En", 15, sx_get_agc_auto, sx_set_agc_auto, NULL,  DATATYPE_BOOL, 1, (GET_FLAG | SET_FLAG)},
    {"Frequency Error", 16, sx_get_frequency_err, NULL, NULL, DATATYPE_UINT32, 0, (GET_FLAG )},
    {"LoRa Sync Word", 17, sx_get_lora_syncword, sx_set_lora_syncword, NULL, DATATYPE_UINT8, UINT8_MAX, (GET_FLAG | SET_FLAG)},
    {"Valid Hdrs Rx", 18, sx_get_valid_hdr_count, NULL, NULL, DATATYPE_UINT32, UINT32_MAX, (GET_FLAG)},
    {"Valid Pkts Rx", 19, sx_get_valid_pkt_count, NULL, NULL, DATATYPE_UINT32, UINT32_MAX, (GET_FLAG)},
    {"Last Rx Len", 20, sx_get_last_rx_len, NULL, NULL, DATATYPE_UINT32, UINT32_MAX, (GET_FLAG)},
    {"Last Rx CR", 21, sx_get_last_rx_coding_rate, NULL, NULL, DATATYPE_UINT32, UINT32_MAX, (GET_FLAG)},
    {"Last Rx SNR", 22, sx_get_last_pkt_snr, NULL, NULL, DATATYPE_UINT32, UINT32_MAX, (GET_FLAG)},
    {"Last Rx RSSI", 23, sx_get_last_pkt_rssi, NULL, NULL, DATATYPE_UINT32, UINT32_MAX, (GET_FLAG)},
    {"LoRa I/O 0 Fn", 24, sx_get_lora_dio0_func, sx_get_lora_dio0_func, NULL, DIO_FUNC_PAYLOAD_CRC_ERR, (GET_FLAG | SET_FLAG)},
    {"FIFO Tx Start", 25, NULL, sx_lora_set_fifo_tx_start, NULL, UINT8_MAX, (SET_FLAG)},
    {"FIFO Rx Start", 25, NULL, sx_lora_set_fifo_rx_start, NULL, UINT8_MAX, (SET_FLAG)},

};
// const peripheral_t lora_peripheral_template;


#endif


#define SX_MODE_CHECK(dev, mode) (dev->device_mode == mode ? true : false )
/****** Function Prototypes ***********/

/************ ISR *********************/

/****** Private Data ******************/

const char *LORA_TAG = "SX1276";

const Lora_Register_Map_t resetDefaults = {
    .regFifo = 0x00,
    .regOpMode.regByte = 0x01,
    .unused02 = 0x1A,
    .unused03 = 0x0B,
    .unused04 = 0x00,
    .unused05 = 0x52,
    .regRfCarrierFreqMsb = 0x6C,
    .regRfCarrierFreqMidsb = 0x80,
    .regRfCarrierFreqLsb = 0x00,
    .regPwrAmpCfg.regByte = 0x4F,
    .regPaRamp.regByte = 0x09,
    .regOcp.regByte = 0x2B,
    .regLNA.regByte = 0x20,
    .regFifoAddrPtr = 0x08,
    .regFifoTxBaseAddr = 0x02,
    .regFifoRxBaseAddr = 0x0A,
    .regFifoRxCurrAddr = 0xFF,
    .regIrqFlagsMask.regByte = 0x00,
    .regIrqFlags.regByte = 0x15,
    .regRxNumBytes = 0x0B,
    .regRxHdrCntMsb = 0x28,
    .regRxHdrCntLsb = 0x0C,
    .regRxPktCntMsb = 0x12,
    .regRxPktCntLsb = 0x47,
    .regModemStatus.regByte = 0x32,
    .regPktSnrValue = 0x3E,
    .regPktRssiValue = 0x00,
    .regRssiValue = 0x00,
    .regHopChannel.regByte = 0x00,
    .regModemCfg1.regByte = 0x00,
    .regModemCfg2.regByte = 0x00,
    .regSymbTimeout = 0x40,
    .regPreambleMsb = 0x00,
    .regPreambleLsb = 0x00,
    .regPayloadLen = 0x00,
    .regMaxPayloadLen = 0x00,
    .regHopPeriod = 0x05,
    .regFifoRxByteAddr = 0x00,
    .regModemConfig3.regByte = 0x03,
    .rsvd1 = 0x93,
    .regFeiMsb.regByte = 0x55,
    .regFeiMidsb = 0x55,
    .regFeiLsb = 0x55,
    .rsvd2b = 0x55,
    .regRssiWideband = 0x55,
    .rsvd2d = 0x55,
    .rsvd2e = 0x55,
    .rsvd2f = 0x55,
    .rsvd30 = 0x90,
    .regDetectOptmz.regByte = 0x40,
    .rsvd32 = 0x40,
    .regInvertIq.regByte = 0x00,
    .rsvd34 = 0x00,
    .rsvd35 = 0x0F,
    .rsvd36 = 0x00,
    .regDetectThresh = 0x00,
    .rsvd38 = 0x00,
    .regSyncWord = 0xF5,
    .rsvd3a = 0x20,
    .rsvd3b = 0x82,
    .rsvd3c = 0x00,
    .rsvd3d = 0x02,
    .rsvd3e = 0x80,
    .rsvd3f = 0x40,
    .regDioMapping1.regByte = 0x00,
    .regDioMapping2.regByte = 0x00,
    .regVersion = 0x12,
    .unused44 = 0x2D,
    .regTxco.regByte = 0x09,
    .regPaDac.regByte = 0x84,
    .regFormerTemp = 0x00,
    .unused45 = 0x00,
    .regAgcRef.regByte = 0x13,
    .regAcgThresh1.regByte= 0x0E,
    .regAgcThresh2.regByte = 0x5B,
    .regAgcThresh3.regByte = 0xDB,
    .regPll = 0xD0,

};


/****** Private Functions *************/

/** @brief Reads a byte of data from a device register
 *          address over SPI.
 *  @param dev device handle
 *  @param addr register address
 *  @param byte pointer to byte storage
 *  
 *  @return ESP_OK or error
 **/
static esp_err_t sx_read_address_byte(SX1276_DEV dev, uint8_t addr, uint8_t *byte);

/** @brief  Writes a byte of data to a device register
 *          address over SPI.
 *  @param dev device handle
 *  @param addr register address
 *  @param byte data to write
 *  
 *  @return ESP_OK or error
 **/
static esp_err_t sx_write_address_byte(SX1276_DEV dev, uint8_t addr, uint8_t byte);

/** @brief  Writes len byte of data to a device 
 *          address over SPI.
 *  @param dev device handle
 *  @param addr register address
 *  @param byte pointer to data 
 *  @param len length of data to write
 * 
 *  @return ESP_OK or error
 **/
static esp_err_t sx_spi_burst_write(SX1276_DEV dev, uint8_t addr, uint8_t *data, uint8_t len);



static IRAM_ATTR void irq_handler(void *args) {

    SX1276_DEV dev = (SX1276_DEV)args;
    BaseType_t pd = pdFALSE;
    vTaskNotifyGiveFromISR(dev->task, &pd);
    portYIELD_FROM_ISR();
}


static esp_err_t sx_read_address_byte(SX1276_DEV dev, uint8_t addr, uint8_t *byte) {

    esp_err_t err = ESP_OK;
    uint8_t cmd = (addr & ~(SX_READWRITE_BIT));
    spi_transaction_t trx = {0};

    trx.length = 16;
    trx.rxlength = 16;
    trx.tx_data[0] = cmd;
    trx.tx_data[1] = 0x00;
    trx.flags = (SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA);

    err = spi_device_transmit(dev->spi_handle, &trx);
    
    if(err != ESP_OK) {
        ESP_LOGE(LORA_TAG, "Error performing SPI transaction! [%u]", err);
    }
    else {
#ifdef SPI_DEBUG
        ESP_LOGI(LORA_TAG, "Read the following data: 0x%02x 0x%02x", trx.rx_data[0], trx.rx_data[1]);
#endif
        *byte = trx.rx_data[1];
    }

    return err;
}


static esp_err_t sx_write_address_byte(SX1276_DEV dev, uint8_t addr, uint8_t byte) {

    esp_err_t err = ESP_OK;
    uint8_t cmd = (addr | SX_READWRITE_BIT);
    spi_transaction_t trx = {0};

    trx.length = 16;
    trx.tx_data[0] = cmd;
    trx.tx_data[1] = byte;
    trx.flags = (SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA);

    // err = spi_device_acquire_bus(dev->spi_handle, SX_SPI_TIMEOUT_DEFAULT);

    err = spi_device_transmit(dev->spi_handle, &trx);
    if(err != ESP_OK) {
        ESP_LOGE(LORA_TAG, "Error performing SPI transaction! [%u]", err);
    }
#ifdef SPI_DEBUG
    else {
        ESP_LOGI(LORA_TAG, "Read the following data: 0x%02x 0x%02x", trx.rx_data[0], trx.rx_data[1]);
    }
#endif

    return err;
}


static esp_err_t sx_spi_burst_write(SX1276_DEV dev, uint8_t addr, uint8_t *data, uint8_t len) {

    uint8_t cmd = (addr | SX_READWRITE_BIT);
    spi_transaction_t trx = {0};
    uint8_t buffer[256] = {0};

    buffer[0] = cmd;
    memcpy(&buffer[1], data, sizeof(uint8_t) * len);

    trx.length = (8 + (len * 8));
    trx.tx_buffer = buffer;
    trx.flags = 0;

    return spi_device_transmit(dev->spi_handle, &trx);

}


static esp_err_t sx_spi_read_fifo_data(SX1276_DEV dev, uint8_t addr, uint8_t *buffer, uint8_t len) {


    esp_err_t err = ESP_OK;
    uint8_t cmd = (SX1276_REGADDR_REGFIFO & ~(SX_READWRITE_BIT));
    spi_transaction_t trx = {0};
    uint8_t dummy[256] = {0};
    uint8_t reg = 0;
    err = sx_read_address_byte(dev, SX1276_REGADDR_FIFOADDR_PTR, &reg);

    /** TODO: might need to transmit dummy data in order to read back
     *          fifo data - depends on spi mode...
     **/ 
    trx.length = len * 8;
    trx.rxlength = len * 8;
    trx.rx_buffer = buffer;
    dummy[0] = cmd;
    trx.tx_buffer = dummy;

    if(err != ESP_OK) {
       ESP_LOGE(LORA_TAG, "Error: Unable to aquire bus [%u]", err);
    }
    else {
        err = spi_device_transmit(dev->spi_handle, &trx);
    }
    if(err != ESP_OK) {
        ESP_LOGE(LORA_TAG, "Error performing SPI transaction! [%u]", err);
    }
#ifdef SPI_DEBUG
    else {
        printf("\n");
        showmem(buffer, len);
    }
#endif
    return err;

}


static esp_err_t sx_spi_read_tx_fifo_data(SX1276_DEV dev, uint8_t *buffer, uint8_t len) {


    esp_err_t err = ESP_OK;
    uint8_t cmd = (SX1276_REGADDR_REGFIFO & ~(SX_READWRITE_BIT));
    spi_transaction_t trx = {0};
    uint8_t dummy[256] = {0};
    uint8_t reg = 0;
    uint8_t tx_base = 0;

    err = sx_read_address_byte(dev, SX1276_REGADDR_FIFO_TXBASE, &tx_base);

    if(!err) {
        err = sx_read_address_byte(dev, SX1276_REGADDR_FIFOADDR_PTR, &reg);    
    }

    /** if the fifo ptr is not equal to tx base, set ptr to tx base **/
    if(!err && tx_base != reg) {
        err = sx_write_address_byte(dev, SX1276_REGADDR_FIFOADDR_PTR, tx_base);
    }


    if(!err) {
        err = sx_read_address_byte(dev, SX1276_REGADDR_FIFOADDR_PTR, &reg);
        if(reg != tx_base) {
            printf("Something went boogey!!\n\n\n");
        } 
    }

    if(!err) {
        ESP_LOGI(LORA_TAG, "Reading %u bytes from TxBase (0x%02x)", len, tx_base);
    }

    /** TODO: might need to transmit dummy data in order to read back
     *          fifo data - depends on spi mode...
     **/ 
    trx.length = len * 8;
    trx.rxlength = len * 8;
    trx.rx_buffer = buffer;
    dummy[0] = cmd;
    trx.tx_buffer = dummy;

    if(err != ESP_OK) {
       ESP_LOGE(LORA_TAG, "Error: Unable to aquire bus [%u]", err);
    }
    else {
        err = spi_device_transmit(dev->spi_handle, &trx);
    }
    if(err != ESP_OK) {
        ESP_LOGE(LORA_TAG, "Error performing SPI transaction! [%u]", err);
    }
#ifdef SPI_DEBUG
    else {
        printf("\n");
        showmem(buffer, len);
    }
#endif
    return err;

}


static esp_err_t sx_spi_write_fifo_data(SX1276_DEV dev, uint8_t *data, uint8_t len) {

    return sx_spi_burst_write(dev, SX1276_REGADDR_REGFIFO, data, len);

}

/** this operation clears the masked bits, then Or's with data, then writes **/
static esp_err_t sx_spi_read_mod_write_mask(SX1276_DEV dev, uint8_t addr, uint8_t data, uint8_t mask, uint8_t *storage) {
    esp_err_t err = ESP_OK;

    uint8_t reg = 0;
    err = sx_read_address_byte(dev, addr, &reg);

    /** conditional write - check the masked bits aren't already set... */
    if(!err) {
        if((reg & mask) != data) {
            reg &= ~(mask);
            reg |= data;
#ifdef SPI_DEBUG
            ESP_LOGI(LORA_TAG, "Writing value 0x%02x to register %02x", reg, addr);
#endif /** SPI_DEBUG **/
            err = sx_write_address_byte(dev, addr, reg);
        }
#ifdef SPI_DEBUG
        else {
            ESP_LOGI(LORA_TAG, "Not writing to register - curr: %02x, data: %02x", reg, data);
        }
#endif /** SPI_DEBUG **/
    }

    /** store the new register value **/
    if(!err && storage != NULL) {
        *storage = reg;
    }

    return err;
}


static void reset_device(SX1276_DEV dev) {

    ESP_LOGI(LORA_TAG, "Resetting device");
    gpio_set_level(dev->rst_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(dev->rst_pin, 1);

}


static esp_err_t sx_modem_status(SX1276_DEV dev, uint8_t *val) {

    esp_err_t err = ESP_OK;
    uint8_t regval = 0;
    err = sx_read_address_byte(dev, SX1276_REGADDR_MODEM_STAT, &regval);
    if(!err) {
        *val = regval;
    }
    return err;
}


static esp_err_t set_overcurrent_protection(SX1276_DEV dev, uint8_t mA) {
    
    esp_err_t err = ESP_OK;
    uint8_t trim = 27; /** TODO: find out where this comes from **/
    if(mA <= 120) {
        trim = (mA - 45) / 5;
    } else if(mA <= 240) {
        trim = (mA + 30) / 10;
    }
    else {
        ESP_LOGI(LORA_TAG, "Current limit too damn high!");
        err = ESP_ERR_INVALID_ARG;
    }

    if(!err) {
        err = sx_write_address_byte(dev, SX1276_REGADDR_OCURRENT_PROT, (0x20 | (0x1F & trim)));
    }

    return err;
}


static esp_err_t check_crc_on_payload(SX1276_DEV dev, bool *crc) {
    return ESP_ERR_NOT_SUPPORTED;
}


static esp_err_t spreadingfactor_set(SX1276_DEV dev, uint8_t sf) {
   esp_err_t err = ESP_OK;
   if(sf >= SX1276_SPREADF_MAX || sf <= SX1276_SPREADF_MIN) {
       err = ESP_ERR_INVALID_ARG;
   } else {
        uint8_t reg = 0;
        err = sx_read_address_byte(dev, SX1276_REGADDR_MODEM_CONFIG2, &reg);
        if(!err) {
            reg |= (sf << 4);
            err = sx_write_address_byte(dev, SX1276_REGADDR_MODEM_CONFIG2, reg);
        }
   }

   return err;
}


static esp_err_t set_lora_bw(SX1276_DEV dev, lora_bw_t bw) {
    esp_err_t err = ESP_OK;
    uint8_t reg = 0, val = 0;
    err = sx_read_address_byte(dev, SX1276_REGADDR_MODEM_CONFIG1, &reg);
    if(!err) {
        val = reg;
        val |= (bw << 4);
        err = sx_write_address_byte(dev, SX1276_REGADDR_MODEM_CONFIG1, val);
    }

    return err;
}


static esp_err_t check_isr_flags(SX1276_DEV dev, uint16_t *isr_flags) {
    esp_err_t err = ESP_OK;
    uint8_t reg[2] = {0};
    

    err = sx_read_address_byte(dev, SX1276_REGADDR_IRQFLAGS1, reg);

    if(!err) {
        err = sx_read_address_byte(dev, SX1276_REGADDR_IRQFLAGS2, &reg[1]);
    }

    if(!err) {
        *isr_flags = (((uint16_t )reg[1] << 8) | (uint16_t)reg[0]);
    }

    return err;

}


esp_err_t set_lora_payload_len(SX1276_DEV dev, uint8_t len) {
    esp_err_t err = ESP_OK;

    err = sx_write_address_byte(dev, SX1276_REGADDR_PAYLOAD_LEN, len);

    return err;
}


static esp_err_t clear_interrupts(SX1276_DEV dev) {

    esp_err_t err = ESP_OK;

    err = sx_write_address_byte(dev, SX1276_REGADDR_IRQ_FLAGS, 0xFF);

    return err;
}


static esp_err_t read_interrupts(SX1276_DEV dev, uint8_t *intr) {

    return sx_read_address_byte(dev, SX1276_REGADDR_IRQ_FLAGS, intr);
}


static void wait_tx_done(SX1276_DEV dev) {
    uint32_t ctr = 0;
    uint8_t reg = 0;
    while(1) {
        printf("Waiting tx [%02x]\n", ctr);
        sx_read_address_byte(dev, SX1276_REGADDR_IRQ_FLAGS, &reg);
        if(reg & SX1276_PKT_SENT_BIT) {
            printf("Pkt sent!\n\n");
            break;
        }
        if(ctr > 100) {
            printf("Breaking out, assume tx not done \n\n");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
        ctr++;
    }
}


static void test_transmit(SX1276_DEV dev) {

    char *msg = "hello";
    uint32_t l = strlen(msg);
    uint8_t mode = SX1276_TRXMODE_TX;

    sx_spi_write_fifo_data(dev, (uint8_t *)msg, l);

    esp_err_t err = sx_write_address_byte(dev, SX_LORA_REGADDR_PAYLOAD_LEN, l);

    printf("Setting tx mode");

    err = sx_set_trx_mode(dev, &mode);

    wait_tx_done(dev);

    clear_interrupts(dev);
}


static void sx1276_driver_task(void *args) {


    SX1276_DEV dev = (sx1276_driver_t *)args;
    uint32_t notify = 0;
    uint8_t reg = 0;

    while(1) {

        if(dev->irq_pin > 0) {
            ESP_LOGI(LORA_TAG, "Waiting for interrupt...");
            notify = ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(1000));
            if(notify) {
                ESP_LOGI(LORA_TAG, "Received Interrupt!!!!!!!!: %u", notify);

                read_interrupts(dev, &reg);
                ESP_LOGI(LORA_TAG, "ISR Register: ");
                printBytesOrderExplicit(reg);

                ESP_LOGI(LORA_TAG, "Clearing ISR register");
                clear_interrupts(dev);
            }
        }

        ESP_LOGI(LORA_TAG, "In driver task");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    /** here be dragons **/
}


/****** Global Data *******************/

/****** Global Functions *************/

/** 
 *  intitialise the device, assume SPI already init. 
 *  Pins for TTGO Lora32 are 
 *  - MISO    19
 *  - CS    17
 *  - MOSI    27
 *  - SCK    5
 *  - RST    14
 *  - IRQ    26
 **/

#ifdef CONFIG_DRIVERS_USE_HEAP
SX1276_DEV sx1276_init(sx1276_init_t *init) 
#else
SX1276_DEV sx1276_init(SX1276_DEV dev_handle, sx1276_init_t *init)
#endif
{


    esp_err_t err = ESP_OK;
    spi_device_handle_t spi_handle;
    gpio_config_t conf = {0};
    TaskHandle_t t_handle = NULL;

    if(!err) {

        /** initialise the spi device **/
        if(init->spi_bus != SPI1_HOST && init->spi_bus != SPI2_HOST) {
            ESP_LOGE(LORA_TAG, "Error, invalid SPI Bus");
            err = ESP_ERR_INVALID_ARG;
        }
        if(!err) {
            spi_host_device_t dev = init->spi_bus;
            spi_device_interface_config_t dconf = {0};
            dconf.address_bits = 0;
            dconf.clock_speed_hz = 500000;
            dconf.command_bits = 0;
            dconf.cs_ena_posttrans = 0;
            dconf.cs_ena_pretrans = 0;
            dconf.dummy_bits = 0;
            dconf.duty_cycle_pos = 128;
            dconf.mode = 0;
            dconf.queue_size = 4;
            dconf.spics_io_num = init->cs_pin;
            err = spi_bus_add_device(dev, &dconf, &spi_handle);
        }

        if(err != ESP_OK) {
            ESP_LOGE(LORA_TAG, "Error adding device to the bus!");
        }
    }

    /** create the device handle **/
#ifdef CONFIG_DRIVERS_USE_HEAP
    if(!err) {
            SX1276_DEV dev_handle = (sx1276_driver_t *) heap_caps_calloc(1, sizeof(sx1276_driver_t), MALLOC_CAP_DEFAULT);
        if(dev_handle == NULL) {
            ESP_LOGE(LORA_TAG, "Error allocating memory for driver handle!");
            err = ESP_ERR_NO_MEM;
        }
    }
#else
    memset(dev_handle, 0, sizeof(sx1276_driver_t));
#endif

    if(!err) {
        dev_handle->cs_pin = init->cs_pin;
        dev_handle->rst_pin = init->rst_pin;
        dev_handle->spi_handle = spi_handle;
        dev_handle->irq_pin = init->irq_pin;
        /** initialise the registers to their reset defaults **/
        memcpy(&dev_handle->registers, &resetDefaults, sizeof(Lora_Register_Map_t));
    }



    if(!err && dev_handle->rst_pin > 0) {
        conf.mode = GPIO_MODE_OUTPUT;
        conf.pin_bit_mask = (1 << dev_handle->rst_pin);
        conf.pull_up_en = GPIO_PULLUP_DISABLE;
        conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        conf.intr_type = GPIO_INTR_DISABLE;

        err = gpio_config(&conf);
    }

    /** configure the interrupt pin **/
    if(!err && dev_handle->irq_pin > 0) {
        conf.mode = GPIO_MODE_INPUT;
        conf.pin_bit_mask = (1 << dev_handle->irq_pin);
        conf.pull_up_en = GPIO_PULLUP_ENABLE;
        conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        conf.intr_type = GPIO_INTR_NEGEDGE;

        err = gpio_config(&conf);

        if(!err) {
            err = gpio_isr_handler_add(init->irq_pin, (gpio_isr_t)irq_handler, (void *)dev_handle);
        }
        
        if(!err) {
            ESP_LOGI(LORA_TAG, "ISR Pin enabled");
        }
    }


        /** start the driver task **/
    if(!err && xTaskCreate(sx1276_driver_task, "lora_driver_task", 5012, (void *)dev_handle, 3, &t_handle) != pdTRUE) {
        err = ESP_ERR_NO_MEM;
        ESP_LOGE(LORA_TAG, "Error starting driver task [%u]", err);
    }

    /** reset the device **/
    if(!err) {
        reset_device(dev_handle);
        vTaskDelay(pdMS_TO_TICKS(100));
    }


    if(!err) {
        ESP_LOGI(LORA_TAG, "Succesfully intialised SX1276 Device!");
        sx_setup_lora(dev_handle);
    }
    else {
        ESP_LOGI(LORA_TAG, "Failed to intialise SX1276 Device! :( [%u]", err);
#ifdef CONFIG_DRIVERS_USE_HEAP
        if(dev_handle != NULL) {
            heap_caps_free(dev_handle);
        }
#endif
    }

    return dev_handle;
}


/** lora getters / setters 
 * 
 * TODO: 
 *  - spreading factor
 *  - status
 **/

esp_err_t sx_get_version(SX1276_DEV dev, uint8_t *ver) {
    uint8_t version = 0;
    esp_err_t err = sx_read_address_byte(dev, SX1276_REGADDR_VERSION, &version);
    if(!err) {
        ESP_LOGI(LORA_TAG, "SX1276 Version Number: 0x%02x", version);
        *ver = version;
    }
    else {
        ESP_LOGE(LORA_TAG, "Error: %u", err);
    }
    return err;
}


/****************** REGISTER GETTER/SETTER FUNCTIONS **************/

esp_err_t sx_get_trx_mode(SX1276_DEV dev, uint8_t *mode) {
   esp_err_t status = ESP_OK;
   *mode = dev->registers.lora_reg.regOpMode.regBits.mode;
   return status;
}


esp_err_t sx_set_trx_mode(SX1276_DEV dev, sx_trxmode_t *mode) {
    
    esp_err_t err = ESP_OK;
    uint8_t m = *mode;

    if(m > SX1276_TRXMODE_CAD) {
        err = ESP_ERR_INVALID_ARG;
    }
    else {
        err = sx_spi_read_mod_write_mask(dev, SX1276_REGADDR_OPMODE, m, 0b111, &dev->registers.lora_reg.regOpMode.regByte);
    }

    if(!err) {
        dev->registers.lora_reg.regOpMode.regBits.mode = m;
    }
    return err;
}


esp_err_t sx_get_device_mode(SX1276_DEV dev, sx_device_mode_t *mode) {

    *mode = dev->device_mode;
    return ESP_OK;
}


esp_err_t sx_set_device_mode(SX1276_DEV dev, sx_device_mode_t *mode) {
    
    esp_err_t err = ESP_OK;
    uint8_t m = *mode;
    sx_trxmode_t sleep = SX1276_TRXMODE_SLEEP;
    if(m > SX_DEVICE_LORA_MODE) {
        err = ESP_ERR_INVALID_ARG;
    }
    else {
        err = sx_set_trx_mode(dev, &sleep);
    }
    
    if(!err) {
        err = sx_spi_read_mod_write_mask(dev, SX1276_REGADDR_OPMODE, (m << 7), (1<<7), &dev->registers.lora_reg.regOpMode.regByte);
    }

    return err;
}


esp_err_t sx_get_frequency(SX1276_DEV dev, uint32_t *frq) {

    uint32_t mod_frq = ((uint32_t )dev->registers.lora_reg.regRfCarrierFreqLsb | 
            ((uint32_t )dev->registers.lora_reg.regRfCarrierFreqMidsb << 8) |
            ((uint32_t )dev->registers.lora_reg.regRfCarrierFreqMsb << 16));
    
    float inter = SX_FRQ_HERTZ_PER_REGCOUNT * mod_frq;
    *frq = (uint32_t )inter;

    return ESP_OK;
}


esp_err_t sx_set_frequency(SX1276_DEV dev, uint32_t *frq) {
    /** the equation f(rf) = F(osc) * F(rf) / 2**19
     *  Gives a resolution in counts * SX_FRQ_HERTZ_PER_REGCOUNTHz
     * Instead of messing about with multiplication, 
     * just divide the desired frequency by SX_FRQ_HERTZ_PER_REGCOUNT and
     * write the result to the frequency registers
     **/

    esp_err_t err = ESP_OK;
    float intermediary;
    uint32_t val;
    uint8_t regvals[3] = {0};
    uint32_t f = *frq;

    if(f >= SX_LORA_MAX_FREQUENCY) {
        return ESP_ERR_INVALID_ARG;
    }

    intermediary = f / SX_FRQ_HERTZ_PER_REGCOUNT;
    val = (uint32_t )intermediary;
    /** load the bytes in reverse order as MSB is at addr 0x06 followed by MidSb and Lsb **/
    regvals[0] = (uint8_t )(val >> 16);
    regvals[1] = (uint8_t )(val >> 8);
    regvals[2] = (uint8_t )val;

    err = sx_spi_burst_write(dev, SX_LORA_REGADDR_CARRFREQ_MSB, regvals, 3);

    if(!err) {
        dev->registers.lora_reg.regRfCarrierFreqLsb = regvals[2];
        dev->registers.lora_reg.regRfCarrierFreqMidsb = regvals[1];
        dev->registers.lora_reg.regRfCarrierFreqMsb = regvals[0];
    }

    return err;
}


esp_err_t sx_get_pa_sel(SX1276_DEV dev, bool *pa_sel) {
    *pa_sel = dev->registers.lora_reg.regPwrAmpCfg.regBits.pAmpSel;
    return ESP_OK;
}


esp_err_t sx_set_pa_sel(SX1276_DEV dev, bool *pa_sel) {
    esp_err_t err = ESP_OK;
    uint8_t sel = (*pa_sel == true ? (1 << 7) : 0);
    err = sx_spi_read_mod_write_mask(dev, SX_LORA_REGADDR_PA_CONFIG, sel, (1 << 7), &dev->registers.lora_reg.regPwrAmpCfg.regByte);

    return err;
}


esp_err_t sx_get_ocp_en(SX1276_DEV dev, bool *en) {
    esp_err_t status = ESP_OK;
    *en = dev->registers.lora_reg.regOcp.regBits.ocpEn;
    return status;
}


esp_err_t sx_set_ocp_en(SX1276_DEV dev, bool *en) {
    esp_err_t status = ESP_OK;
    
    uint8_t _en = (*en == true ? (1 << 5) : 0);
    status = sx_spi_read_mod_write_mask(dev, SX_LORA_REGADDR_OCURRENT_PROT, _en, (1 << 5), &dev->registers.lora_reg.regOcp.regByte);

    return status;
}


esp_err_t sx_get_ocp_trim(SX1276_DEV dev, uint8_t *trim) {
    *trim = dev->registers.lora_reg.regOcp.regBits.ocpTrim;
    return ESP_OK;
}


esp_err_t sx_set_ocp_trim(SX1276_DEV dev, uint8_t *trim) {
   esp_err_t status = ESP_OK;
    uint8_t t = *trim;
    
    if(t > 0x0F) {
        return ESP_ERR_INVALID_ARG;
    }

    status = sx_spi_read_mod_write_mask(dev, SX_LORA_REGADDR_OCURRENT_PROT, t, 0x0F, &dev->registers.lora_reg.regOcp.regByte);

    return status;
}


// esp_err_t sx1276_get_modtype(sx1276_driver_t *dev, uint8_t *mode);

// esp_err_t sx1276_get_lowfreq_mode(sx1276_driver_t *dev, uint8_t *mode);


esp_err_t sx_set_lna_gain(SX1276_DEV dev, uint8_t *gain) {
    esp_err_t err = ESP_OK;
    uint8_t g = *gain;

    if(g >= 0b111) {
        err = ESP_ERR_INVALID_ARG;
    }
    else {
        err = sx_spi_read_mod_write_mask(dev, SX1276_REGADDR_LNA_CONFIG, (g << 5), (0b111 << 5), &dev->registers.lora_reg.regLNA.regByte);
    }

    return err;
}


esp_err_t sx_get_lna_gain(SX1276_DEV dev, uint8_t *gain) {
    *gain = dev->registers.lora_reg.regLNA.regBits.lnaGain;
    return ESP_OK;
}


esp_err_t sx_get_lna_boost_hf(SX1276_DEV dev, bool *io) {
   esp_err_t status = ESP_OK;
    *io = dev->registers.lora_reg.regLNA.regBits.lnaBoostHFrq;
   return status;
}


esp_err_t sx_set_lna_boost_hf(SX1276_DEV dev, bool *io) {
    esp_err_t err = ESP_OK;
    bool en = *io;
    uint8_t val = (en ? 0b11 : 0);

    err = sx_spi_read_mod_write_mask(dev, SX1276_REGADDR_LNA_CONFIG, val, 0b11, &dev->registers.lora_reg.regLNA.regByte);

    return err;
}


esp_err_t sx_get_signal_bandwidth(SX1276_DEV dev, lora_bw_t *bw) {
    *bw = dev->registers.lora_reg.regModemCfg1.regBits.bandwidth;
    return ESP_OK;
}


esp_err_t sx_set_signal_bandwidth(SX1276_DEV dev, lora_bw_t *bw) {

    uint8_t bdw = *bw;

    if(bdw >= SX1276_LORA_BW_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    else {
        bdw = (bdw << 4);
    }

    return sx_spi_read_mod_write_mask(dev, SX_LORA_REGADDR_MODEM_CONFIG1, bdw, 0xF0, &dev->registers.lora_reg.regModemCfg1.regByte);

}


esp_err_t sx_get_lora_spreading_factor(SX1276_DEV dev, uint8_t *val) {
   esp_err_t status = ESP_OK;
  *val = dev->registers.lora_reg.regModemCfg2.regBits.spreadingFactor;
   return status;
}


esp_err_t sx_set_lora_spreading_factor(SX1276_DEV dev, uint8_t *val) {
   esp_err_t status = ESP_OK;
   /** if spreading factor == 6 
    *   - set header to implicit mode
    *   - set bit field detectionOptimize reg to 0b101
    *   - write 0x0C in RegDetectionThreshold
    * **/
   uint8_t sf = *val;

    if(sf < 6 || sf > 12) {
        return ESP_ERR_INVALID_ARG;
    }
    else {
        sf = (sf << 4);
    }

    return sx_spi_read_mod_write_mask(dev, SX_LORA_REGADDR_MODEM_CONFIG2, sf, 0xF0, &dev->registers.lora_reg.regModemCfg2.regByte);
}


esp_err_t sx_get_rx_payload_crc_en(SX1276_DEV dev,  bool *en) {
    *en = dev->registers.lora_reg.regModemCfg2.regBits.rxPayloadCrcEn;
    return ESP_OK;
}


esp_err_t sx_set_rx_payload_crc_en(SX1276_DEV dev, bool *en) {
    esp_err_t err = ESP_OK;
    bool io = *en;
    uint8_t val = (io ? (1 << 2) : 0);

    err = sx_spi_read_mod_write_mask(dev, SX1276_REGADDR_MODEM_CONFIG2, val, (1 << 2), &dev->registers.lora_reg.regModemCfg2.regByte);

    return err;
}


/** TODO:   
 *          Symbol timeout
 *          preamble length
 *          payload length
 *          payload max length         
 *          frequency hopping period
 *
 **/


esp_err_t sx_get_lora_headermode(SX1276_DEV dev, uint8_t *val) {
   esp_err_t status = ESP_OK;
   *val = dev->registers.lora_reg.regModemCfg1.regBits.implicitHdrModeEn;
   return status;
}


esp_err_t sx_set_lora_headermode(SX1276_DEV dev, uint8_t *val) {
   esp_err_t status = ESP_OK;
    uint8_t v = *val;

    if(v) {
        v = 1;
    }    

    status = sx_spi_read_mod_write_mask(dev, SX_LORA_REGADDR_MODEM_CONFIG1, v, 1, &dev->registers.lora_reg.regModemCfg1.regByte);

    return status;
}


esp_err_t sx_get_low_datarate_optimise(SX1276_DEV dev,  bool *en) {
    *en = dev->registers.lora_reg.regModemConfig3.regBits.lowDataRateOptEn;
    return ESP_OK;
}


esp_err_t sx_set_low_datarate_optimise(SX1276_DEV dev, bool *en) {
    esp_err_t err = ESP_OK;
    bool io = *en;
    uint8_t val = (io ? (1 << 3) : 0);

    err = sx_spi_read_mod_write_mask(dev, SX1276_REGADDR_MODEM_CONFIG3, val, (1 << 3), &dev->registers.lora_reg.regModemConfig3.regByte);

    return err;
}


esp_err_t sx_get_agc_auto(SX1276_DEV dev,  bool *io) {
    *io = dev->registers.lora_reg.regModemConfig3.regBits.agcAutoEn;
    return ESP_OK;
}


esp_err_t sx_set_agc_auto(SX1276_DEV dev, bool *io) {
    esp_err_t err = ESP_OK;
    bool en = *io;
    uint8_t val = (en ? (1 << 2) : 0);

    err = sx_spi_read_mod_write_mask(dev, SX1276_REGADDR_MODEM_CONFIG3, val, 0b100, &dev->registers.lora_reg.regModemConfig3.regByte);

    return err;
}


esp_err_t sx_get_frequency_err(SX1276_DEV dev, uint32_t *frq) {
    esp_err_t err = ESP_OK;
    uint8_t data[3];
    uint32_t fe = 0;

    err = sx_read_address_byte(dev, SX_LORA_REGADDR_FEI_LSB, data);
    err += sx_read_address_byte(dev, SX_LORA_REGADDR_FEI_MIDB, &data[1]);
    err += sx_read_address_byte(dev, SX_LORA_REGADDR_FEI_MSB, &data[2]);

    if(err) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    fe = ((uint32_t )data[0] | ((uint32_t )data[1] << 8) | ((uint32_t )(data[2] & 0x0F) << 16));
    dev->registers.lora_reg.regFeiLsb = data[0];
    dev->registers.lora_reg.regFeiMidsb = data[1];
    dev->registers.lora_reg.regFeiMsb.regByte = (data[2] & 0x0F);

    *frq = fe;

    return err;
}


/**
 * TODO:    RSSI Wideband
 *          Detection Optimize
 *          InvertIQ
 *          Detection Threshold
 * 
 **/

esp_err_t sx_get_lora_syncword(SX1276_DEV dev, uint8_t *val) {
   esp_err_t status = ESP_OK;
   *val = dev->registers.lora_reg.regSyncWord;
   return status;
}


esp_err_t sx_set_lora_syncword(SX1276_DEV dev, uint8_t *val) {
   esp_err_t status = ESP_OK;
    uint8_t v = *val;

    if(v == 0x34) {
#ifdef SX_CONFIG_LORAWAN_EN
        ESP_LOGI(LORA_TAG, "Sync Word 0x34 is reserved for LoRaWAN Networks");
        status = sx_write_address_byte(dev, SX_LORA_REGADDR_MODEM_CONFIG1, v);
#else
        ESP_LOGE(LORA_TAG, "Sync Word 0x34 is reserved for LoRaWAN Networks");
        status = ESP_ERR_INVALID_ARG;
#endif
    }
    else {
        status = sx_write_address_byte(dev, SX_LORA_REGADDR_MODEM_CONFIG1, v);
    }

    if(status == ESP_OK) {
        dev->registers.lora_reg.regSyncWord = v;
    }

    return status;
}


esp_err_t sx_set_tx_pwr(SX1276_DEV dev, uint16_t pwr) {

    esp_err_t err = ESP_OK;

    if (pwr > 17) {
        if (pwr > 20) {
            pwr = 20;
        }
        pwr -= 3;

        /** this is in high power mode **/
        err = sx_write_address_byte(dev, SX1276_REGADDR_PA_DAC, RE_PA_HP_ENABLE);
        err += set_overcurrent_protection(dev, 140); /** TODO: Investigate moar! **/

    } else {
        if (pwr < 2) {
            pwr = 2;
        }
        err = sx_write_address_byte(dev, SX1276_REGADDR_PA_DAC, RE_PA_STD_PWR);
        err += set_overcurrent_protection(dev, 100);
    }

    if(!err) {
        err = sx_write_address_byte(dev, SX1276_REGADDR_PA_CONFIG, (SX1276_PA_SELECT_BIT | (pwr-2)));
    }

    return err;
}


esp_err_t sx_get_valid_hdr_count(SX1276_DEV dev, uint32_t *cnt) {
    esp_err_t err = ESP_OK;
    uint8_t data[2] = {0};

    err = sx_read_address_byte(dev, SX_LORA_REGADDR_RXHDR_CNT_LSB, &data[0]);
    err += sx_read_address_byte(dev, SX_LORA_REGADDR_RXHDR_CNT_MSB, &data[1]);

    *cnt = ((uint32_t )data[0] | ((uint32_t )data[1] << 8));

    return err; 
}


esp_err_t sx_get_valid_pkt_count(SX1276_DEV dev, uint32_t *cnt) {
    esp_err_t err = ESP_OK;
    uint8_t data[2] = {0};

    err = sx_read_address_byte(dev, SX_LORA_REGADDR_RXPKT_CNT_LSB, &data[0]);
    err += sx_read_address_byte(dev, SX_LORA_REGADDR_RXPKT_CNT_MSB, &data[1]);

    *cnt = ((uint32_t )data[0] | ((uint32_t )data[1] << 8));

    return err; 
}


esp_err_t sx_get_last_rx_len(SX1276_DEV dev, uint8_t *len) {
    return ESP_ERR_NOT_SUPPORTED;
}


esp_err_t sx_get_last_rx_coding_rate(SX1276_DEV dev, uint8_t *cr) {
    esp_err_t err = ESP_OK;
    uint8_t data = 0;

    err = sx_read_address_byte(dev, SX_LORA_REGADDR_MODEM_STAT, &data);

    *cr = ((data & 0b11100000) >> 5);

    return err; 
}


esp_err_t sx_get_last_pkt_snr(SX1276_DEV dev, int16_t *snr) {
    esp_err_t err = ESP_OK;
    uint8_t data = 0;

    err = sx_read_address_byte(dev, SX_LORA_REGADDR_PKT_SNR_VAL, &data);
    /**< 
     * Estimation of SNR on last packet received.In two’s compliment
     * format mutiplied by 4  (datasheet, pg111)
     *  **/
    *snr = ((int8_t)data / 4);

    return err; 
}


esp_err_t sx_get_last_pkt_rssi(SX1276_DEV dev, uint16_t *rssi) {
    esp_err_t err = ESP_OK;
    uint8_t data = 0;

    err = sx_read_address_byte(dev, SX_LORA_REGADDR_PKT_RSSI_VAL, &data);

    /**
     * RSSI[dBm] = -157 + Rssi (using HF output port, SNR >= 0)
     * or
     * RSSI[dBm] = -164 + Rssi (using LF output port, SNR >= 0)
     *  (datasheet, pg 112)
     * TODO: Record & make dependent on LF/HF port
     * **/
    *rssi = -157 + data;

    return err; 
}


/****************** DIO FUNCTIONS **************/

esp_err_t sx_get_lora_dio0_func(SX1276_DEV dev, sx_dio_func_t *val) {
    esp_err_t status = ESP_OK;

    uint8_t map = dev->registers.lora_reg.regDioMapping1.regBits.dio0mapping;
    if(map == 0b11) {
        *val = DIO_FUNC_NONE;
    }
    else if(map == 0b10) {
        *val = DIO_FUNC_CAD_DONE;
    }
    else if(map == 0b01) {
        *val = DIO_FUNC_TX_DONE;
    }
    else if(map == 0) {
        *val = DIO_FUNC_RX_DONE;
    }
    else {
        *val = DIO_FUNC_INVALID;
    }
    return status;
}


esp_err_t sx_set_lora_dio0_func(SX1276_DEV dev, sx_dio_func_t *val) {
    esp_err_t err = ESP_OK;
    uint8_t v = *val;

    if(v != DIO_FUNC_NONE    &&
       v != DIO_FUNC_RX_DONE && 
       v != DIO_FUNC_TX_DONE &&
       v != DIO_FUNC_CAD_DONE
    ) {
        ESP_LOGE(LORA_TAG, "Invalid mode selected for DIO 0 - valid modes: None, Rx done, Tx done, CAD done");
        err = ESP_ERR_INVALID_ARG;
    }
    if(!err) {
        if(v == DIO_FUNC_NONE) {
            v = (0b11 << 6);
        }
        else if (v == DIO_FUNC_RX_DONE) {
            v = 0;
        }
        else if (v == DIO_FUNC_TX_DONE) {
            v = (1 << 6);
        }
        else {
            v = (1 << 7);
        }
        err = sx_spi_read_mod_write_mask(dev, SX_LORA_REGADDR_DIO_MAP1, v, (0b11 << 6), &dev->registers.lora_reg.regDioMapping1.regByte);
    }
    return err;
}


/****************** GENERAL FUNCTIONS **************/

esp_err_t sx_lora_transmit_data(SX1276_DEV dev, uint8_t *data, uint8_t len) {

    /** get the current fifoTxPtrBase **/
    esp_err_t err = ESP_OK;
    uint8_t tx_base = 0;
    uint16_t fifo_space = 0;
    sx_trxmode_t mode = SX1276_TRXMODE_STDBY;
    uint8_t reg = 0;
    uint8_t buff[32] = {0};

    err = sx_read_address_byte(dev, SX1276_REGADDR_FIFO_TXBASE, &tx_base);

    if(err) {
        ESP_LOGE(LORA_TAG, "Error reading Tx ptr base");
        return err;
    }

    fifo_space = 256 - tx_base;
    /** make sure the length isn't longer than the fifo space **/
    ESP_LOGI(LORA_TAG, "Fifo Space: %u", fifo_space);

    if(len > fifo_space) {
        ESP_LOGI(LORA_TAG, "Only %u bytes available but %u requested, data will be truncated!", fifo_space, len);
        len = fifo_space;
    }

    /** set the device into standby mode **/

    err = sx_set_trx_mode(dev, &mode);

    if(err) {
        ESP_LOGE(LORA_TAG, "Error setting standby mode {%u}", err);
        return err;
    }

    /** set the fifo ptr to the startaddress **/

    err = sx_write_address_byte(dev, SX1276_REGADDR_FIFOADDR_PTR, tx_base);

    if(err) {
        ESP_LOGE(LORA_TAG, "Error writing fifo ptr value! {%u}", err);
        return err;
    }

    /** load the data into the fifo **/
#ifdef SPI_DEBUG
    ESP_LOGI(LORA_TAG, "Sending Following data to the fifo buffer: ");
    showmem(data, len);
#endif /** SPI_DEBUG **/
    
    err = sx_spi_write_fifo_data(dev, data, len);

    if(err) {
        ESP_LOGE(LORA_TAG, "Error writing data to fifo! {%u}", err);
        return err;
    }

    /** set the payload len **/

    if(!err) {
        err = set_lora_payload_len(dev, len);
        if(err) {
            ESP_LOGE(LORA_TAG, "Error setting payload length! {%u}", err);
        }
    
    }

    /** request mode Tx **/
    mode = SX1276_TRXMODE_TX;
    err = sx_set_trx_mode(dev, &mode);

    if(err) {
        ESP_LOGE(LORA_TAG, "Error requesting Tx mode! {%u}", err);
    }

    return err;

}


esp_err_t sx_setup_lora(SX1276_DEV dev) {

    esp_err_t err = ESP_OK;
    sx_trxmode_t mode = SX1276_TRXMODE_SLEEP;
    sx_device_mode_t dev_mode = SX_DEVICE_LORA_MODE;
    uint32_t freq = SX1276_LORA_FREQ_EURO;
    bool en = true;
    uint8_t byte = 0;

    err = sx_set_trx_mode(dev, &mode);
    ESP_LOGI(LORA_TAG, "Setting mode to 'sleep' [%u]", err);
    sx_read_address_byte(dev, SX1276_REGADDR_OPMODE, &byte);
    ESP_LOGI(LORA_TAG, "Confirm write (wrote %02x, read %02x)", dev_mode, byte);

    if(!err) {
        err = sx_set_device_mode(dev, &dev_mode);
        ESP_LOGI(LORA_TAG, "Setting dev mode to LoRa [%u]", err);
    }

    // if(!err) {
    //     err = sx_set_frequency(dev, &freq);
    //     ESP_LOGI(LORA_TAG, "Setting Frequency to %u [%u]", freq, err);
    // }

    if(!err) {
        byte = 0;
        err = sx_lora_set_fifo_tx_start(dev, &byte);
        ESP_LOGI(LORA_TAG, "Setting Fifo Tx base addr to 0");
    }

    if(!err) {
        byte = 3;
        err = sx_set_lna_gain(dev, &byte); 
        ESP_LOGI(LORA_TAG, "Setting lna enabled [%u]", err);
    }

    if(!err) {
        err = sx_set_agc_auto(dev, &en);
        ESP_LOGI(LORA_TAG, "Setting AGC Auto enabled [%u]", err);
    }

    if(!err) {
        err = sx_set_tx_pwr(dev, 17);
        ESP_LOGI(LORA_TAG, "Setting Tx power to 17 [%u]", err);
    }

    if(!err) {
        byte = 0;
        err = sx_set_lora_headermode(dev, &byte);
        ESP_LOGI(LORA_TAG, "Setting header mode explicit [%u]", err);
    }

    if(!err) {
        err = sx_read_address_byte(dev, SX_LORA_REGADDR_IRQFLAGS_MASK, &byte);
        ESP_LOGI(LORA_TAG, "IRQ Mask register %02x [%u]", byte, err);
    }

    if(!err) {
        err = clear_interrupts(dev);

        ESP_LOGI(LORA_TAG, "Cleared interrupts [%u]", err);
    }

    if(!err) {
        byte = 1;
        err = sx_spi_read_mod_write_mask(dev, SX1276_REGADDR_DIO_MAP1, (byte << 6), (3 << 6), NULL);
        ESP_LOGI(LORA_TAG, "Setting DIO0 function to tx done [%u]", err);
    }

    if(!err) {
        mode = SX1276_TRXMODE_STDBY;
        err = sx_set_trx_mode(dev, &mode);
        ESP_LOGI(LORA_TAG, "Setting Trx mode to Standby [%u]", err);
    }

    return err;

}


/****************** FIFO FUNCTIONS **************/

esp_err_t sx_lora_set_fifo_tx_start(SX1276_DEV dev, uint8_t *val) {
    esp_err_t err = ESP_OK;
    uint8_t v = *val;
    err = sx_write_address_byte(dev, SX1276_REGADDR_FIFO_TXBASE, v);

    return err;
}


esp_err_t sx_lora_set_fifo_rx_start(SX1276_DEV dev, uint8_t *val) {
    esp_err_t err = ESP_OK;
    uint8_t v = *val;
    err = sx_write_address_byte(dev, SX1276_REGADDR_FIFO_RXBASE, v);

    return err;
}