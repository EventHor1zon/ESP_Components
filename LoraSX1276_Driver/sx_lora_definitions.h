/****************************************
* \file     sx_lora_definitions.h
* \brief    Definitions file for the lora registers for 
*           SX lora/fsk chip
* \date     Oct 2021
* \author   RJAM
****************************************/

#ifndef SX_LORA_DEFINITIONS_H
#define SX_LORA_DEFINITIONS_H

/********* Includes ********************/

#include "esp_types.h"

/********* Definitions *****************/

/********** Types **********************/

typedef enum {

    SX_LORA_REGADDR_REGFIFO=            0x00,
    SX_LORA_REGADDR_OPMODE=             0x01,
    SX_LORA_REGADDR_CARRFREQ_MSB=       0x06,
    SX_LORA_REGADDR_CARRFREQ_MIDB=      0x07,
    SX_LORA_REGADDR_CARRFREQ_LSB=       0x08,
    SX_LORA_REGADDR_PA_CONFIG=          0x09,
    SX_LORA_REGADDR_PA_RAMP=            0x0A,
    SX_LORA_REGADDR_OCURRENT_PROT=      0x0B,
    SX_LORA_REGADDR_LNA_CONFIG=         0x0C,
    SX_LORA_REGADDR_FIFOADDR_PTR=       0x0D,
    SX_LORA_REGADDR_FIFO_TXBASE_ADDR=   0x0E,
    SX_LORA_REGADDR_FIFO_RXBASE_ADDR=   0x0F,
    SX_LORA_REGADDR_FIFO_RXCURR_ADDR=   0x10,
    SX_LORA_REGADDR_IRQFLAGS_MASK=      0x11,
    SX_LORA_REGADDR_IRQ_FLAGS=          0x12,
    SX_LORA_REGADDR_RX_N_BYTES=         0x13,
    SX_LORA_REGADDR_RXHDR_CNT_MSB=      0x14,
    SX_LORA_REGADDR_RXHDR_CNT_LSB=      0x15,
    SX_LORA_REGADDR_RXPKT_CNT_MSB=      0x16,
    SX_LORA_REGADDR_RXPKT_CNT_LSB=      0x17,
    SX_LORA_REGADDR_MODEM_STAT=         0x18,
    SX_LORA_REGADDR_PKT_SNR_VAL=        0x19,
    SX_LORA_REGADDR_PKT_RSSI_VAL=       0x1A,
    SX_LORA_REGADDR_RSSI_VAL=           0x1B,
    SX_LORA_REGADDR_HOPCHANNEL=         0x1C,
    SX_LORA_REGADDR_MODEM_CONFIG1=      0x1D,
    SX_LORA_REGADDR_MODEM_CONFIG2=      0x1E,
    SX_LORA_REGADDR_SYMB_TIMEOUT_LSB=   0x1F,
    SX_LORA_REGADDR_PREAMBLE_MSB=       0x20,
    SX_LORA_REGADDR_PREAMBLE_LSB=       0x21,
    SX_LORA_REGADDR_PAYLOAD_LEN=        0x22,
    SX_LORA_REGADDR_MAX_PAYLOAD_LEN=    0x23,
    SX_LORA_REGADDR_HOP_PERIOD=         0x24,
    SX_LORA_REGADDR_FIFO_RXBYTE_ADDR=   0x25,
    SX_LORA_REGADDR_MODEM_CONFIG3=      0x26,
    SX_LORA_REGADDR_FEI_MSB=            0x28,
    SX_LORA_REGADDR_FEI_MIDB=           0x29,
    SX_LORA_REGADDR_FEI_LSB=            0x2A,
    SX_LORA_REGADDR_RSSI_WIDEBAND=      0x2C,
    SX_LORA_REGADDR_DETECT_OPMZ=        0x31,
    SX_LORA_REGADDR_INVERT_IQ=          0x33,
    SX_LORA_REGADDR_DETECT_THRESH=      0x37,
    SX_LORA_REGADDR_SYNC_WORD=          0x39,
    SX_LORA_REGADDR_DIO_MAP1=           0x40,
    SX_LORA_REGADDR_DIO_MAP2=           0x41,
    SX_LORA_REGADDR_VERSION=            0x42,
    SX_LORA_REGADDR_TXCO=               0x4B,
    SX_LORA_REGADDR_PA_DAC=             0x4D,
    SX_LORA_REGADDR_FORMER_TEMP=        0x5B,
    SX_LORA_REGADDR_AGC_REF=            0x61,
    SX_LORA_REGADDR_AGC_THRESH1=        0x62,
    SX_LORA_REGADDR_AGC_THRESH2=        0x63,
    SX_LORA_REGADDR_AGC_THRESH3=        0x64,
    SX_LORA_REGADDR_PLL=                0x70,
    SX_LORA_REGADDR_INVALID
} Lora_regaddress_t;




typedef struct Lora_Register_Map
{
    /* Fifo  */
    uint8_t regFifo;    /**< 0x00 **/
    
    /** Common Settings Registers **/
    union {
        uint8_t regByte;
        struct {
            uint8_t mode : 3;
            uint8_t lowFreqModeEn : 1;
            uint8_t rsvd : 1;
            uint8_t modulationType : 2;
            uint8_t longRangeMode : 1;
        } regBits;
    } regOpMode;    /**< 0x01 **/

    uint8_t unused02;  /**< 0x02 **/
    uint8_t unused03;  /**< 0x03 **/
    uint8_t unused04;  /**< 0x04 **/
    uint8_t unused05;  /**< 0x05 **/


    uint8_t regRfCarrierFreqMsb;    /**< 0x06 **/
    uint8_t regRfCarrierFreqMidsb;  /**< 0x07 **/
    uint8_t regRfCarrierFreqLsb;    /**< 0x08 **/


    /** Transmitter registers **/
    union 
    {
        uint8_t regByte;
        struct {
            uint8_t outputPower : 4;
            uint8_t maxPower : 3;
            uint8_t pAmpSel : 1;
        } regBits;
    } regPwrAmpCfg;     /**< 0x09 **/

    union {
        uint8_t regByte;
        struct {
            uint8_t paRamp : 4;
            uint8_t rsvd : 1;
            uint8_t modulationShape : 2;
            uint8_t unused : 1;
        } regBits;
    } regPaRamp;    /**< 0x0A **/

    union {
        uint8_t regByte;
        struct {
            uint8_t ocpTrim : 4;
            uint8_t ocpEn : 1;
            uint8_t unsused : 2;
        } regBits;
    } regOcp;   /**< 0x0B **/
    /** Receiver Settings Registers **/

    union {
        uint8_t regByte;
        struct {
            uint8_t lnaBoostHFrq : 2;
            uint8_t rsvd : 1;
            uint8_t lnaBoostLFrq : 2;
            uint8_t lnaGain : 3;
        } regBits;
    } regLNA;       /**< 0x0C **/

    uint8_t regFifoAddrPtr;         /**< 0x0D **/
    uint8_t regFifoTxBaseAddr;      /**< 0x0E **/
    uint8_t regFifoRxBaseAddr;      /**< 0x0F **/
    uint8_t regFifoRxCurrAddr;      /**< 0x10 **/


    union {
        uint8_t regByte;
        struct {
            uint8_t cadDetectedMask : 1;
            uint8_t fhssChgChnlMask : 1;
            uint8_t cadDoneMask : 1;
            uint8_t txDoneMask : 1;
            uint8_t validHeaderMask : 1;
            uint8_t payloadCrcErrorMask : 1;
            uint8_t rxDoneMask : 1;
            uint8_t rxTimeoutMask : 1;
        } regBits;
    } regIrqFlagsMask;      /**< 0x11 **/

    union {
        uint8_t regByte;
        struct {
            uint8_t cadDetected : 1;
            uint8_t validHeader : 1;
            uint8_t payloadCrcError : 1;
            uint8_t txDone : 1;
            uint8_t rxDone : 1;
            uint8_t cadDone : 1;
            uint8_t fhssChgChnl : 1;
            uint8_t rxTimeout : 1;
        } regBits;
    } regIrqFlags;      /**< 0x12 **/

    uint8_t regRxNumBytes;      /**< 0x13 **/
    uint8_t regRxHdrCntMsb;     /**< 0x14 **/
    uint8_t regRxHdrCntLsb;     /**< 0x15 **/

    uint8_t regRxPktCntMsb;     /**< 0x16 **/
    uint8_t regRxPktCntLsb;     /**< 0x17 **/
    
    union {
        uint8_t regByte;
        struct {
            uint8_t modemStatus : 5;
            uint8_t rxCodingRate : 3;
        } regBits;
    } regModemStatus;               /**< 0x18 **/
    
    uint8_t regPktSnrValue;         /**< 0x19 **/

    uint8_t regPktRssiValue; /**< 0x1A **/

    uint8_t regRssiValue;  /**< 0x1B **/
    union {
        uint8_t regByte;
        struct {
            uint8_t fhssPresentChannel : 6;
            uint8_t crcOnPayload : 1;
            uint8_t pllTimeout : 1;
        } regBits;
    } regHopChannel;        /**< 0x1C **/

    union {
        uint8_t regByte;
        struct {
            uint8_t implicitHdrModeEn : 1;
            uint8_t codingRate : 3;
            uint8_t bandwidth : 4;
        } regBits;
    } regModemCfg1;         /**< 0x1D **/
    
    
    union {
        uint8_t regByte;
        struct {
            uint8_t symbTimeout : 2;
            uint8_t rxPayloadCrcEn : 1;
            uint8_t txContinuousMode : 1;
            uint8_t spreadingFactor : 4;
        } regBits;
    } regModemCfg2;      /**< 0x1E **/

    uint8_t regSymbTimeout;     /**< 0x1F **/
    uint8_t regPreambleMsb;     /**< 0x20 **/
    uint8_t regPreambleLsb;     /**< 0x21 **/
    uint8_t regMaxPayloadLen;   /**< 0x22 **/ 
    uint8_t regPayloadLen;      /**< 0x23 **/
    uint8_t regHopPeriod;       /**< 0x24 **/
    uint8_t regFifoRxByteAddr;  /**< 0x25 **/         

    union {
        uint8_t regByte;
        struct {
            uint8_t rsvd : 2;
            uint8_t agcAutoEn : 1;
            uint8_t lowDataRateOptEn : 1;
            uint8_t unused : 4;
        } regBits;
    } regModemConfig3;          /**< 0x26 **/

    uint8_t rsvd1;   /**< 0x27 **/

    union {
        uint8_t regByte;
        struct {
            uint8_t freqErrorMsb : 4;
            uint8_t rsvd : 4;
        } regBits;
    } regFeiMsb;            /**< 0x28 **/

    uint8_t regFeiMidsb;    /**< 0x29 **/
    uint8_t regFeiLsb;      /**< 0x2A **/
    uint8_t rsvd2b;         /**< 0x2B **/
    uint8_t regRssiWideband;    /**< 0x2C **/
    uint8_t rsvd2d;             /**< 0x2D **/
    uint8_t rsvd2e;
    uint8_t rsvd2f;
    uint8_t rsvd30;

    union {
        uint8_t regByte;
        struct {
            uint8_t detectOptmz : 3;
            uint8_t rsvd : 5;
        } regBits;
    } regDetectOptmz;                     /**< 0x31 **/

    uint8_t rsvd32;      /**< 0x32 **/

    union {
        uint8_t regByte;
        struct {
            uint8_t rsvd0 : 6;
            uint8_t invertIq : 1;
            uint8_t rsvd1 : 1;
        } regBits;
    } regInvertIq;          /**< 0x33 **/

    uint8_t rsvd34;    /**< 0x34 **/
    uint8_t rsvd35;    /**< 0x35 **/
    uint8_t rsvd36;    /**< 0x36 **/

    uint8_t regDetectThresh;    /**< 0x37 **/
    uint8_t rsvd38;             /**< 0x38 **/
    uint8_t regSyncWord;        /**< 0x39 **/

    uint8_t rsvd3a;    /**< 0x3A **/
    uint8_t rsvd3b;    /**< 0x3B **/
    uint8_t rsvd3c;    /**< 0x3C **/
    uint8_t rsvd3d;    /**< 0x3D **/
    uint8_t rsvd3e;    /**< 0x3E **/
    uint8_t rsvd3f;    /**< 0x3F **/

    /** IO Control Registers **/
    union {
        uint8_t regByte;
        struct {
            uint8_t dio3mapping : 2;
            uint8_t dio2mapping : 2;
            uint8_t dio1mapping : 2;
            uint8_t dio0mapping : 2;
        } regBits;
    } regDioMapping1;               /**< 0x40 **/

    union {
        uint8_t regByte;
        struct {
            uint8_t mapPreambleDetect : 1;
            uint8_t rsvd : 3;
            uint8_t dio5mapping : 2;
            uint8_t dio4mapping : 2;
        } regBits;
    } regDioMapping2;               /**< 0x41 **/

    /** Version Register **/

    uint8_t regVersion;             /**< 0x42 **/
    uint8_t reg20rsvd;              /**< 0x43 **/

    /** Additional registers **/

    uint8_t unused44;                    /**< 0x44 **/


    union {
        uint8_t regByte;
        struct {
            uint8_t rsvd0 : 4;
            uint8_t txcoInputEn : 1;
            uint8_t rsvd1 : 3;
        } regBits;
    } regTxco;                      /**< 0x4B **/

    union {
        uint8_t regByte;
        struct {
            uint8_t paDac : 3;
            uint8_t rsvd : 5;
        } regBits;
    } regPaDac;                     /**< 0x4D **/

    uint8_t regFormerTemp;

    uint8_t unused45;               /**< 0x5B **/

    union {
        uint8_t regByte;
        struct {
            uint8_t agcReferenceLvl : 6;
            uint8_t unused : 2;
        } regBits;
    } regAgcRef;                    /**< 0x61 **/

    union {
        uint8_t regByte;
        struct {
            uint8_t agcStep1 : 4;
            uint8_t unused : 4;
        } regBits;
    } regAcgThresh1;                /**< 0x62 **/

    union {
        uint8_t regByte;
        struct {
            uint8_t agcStep3 : 4;
            uint8_t agcStep2 : 4;
        } regBits;
    } regAgcThresh2;                /**< 0x63 **/

    union {
        uint8_t regByte;
        struct {
            uint8_t agcStep5 : 4;
            uint8_t agcStep4 : 4;
        } regBits;
    } regAgcThresh3;                /**< 0x64 **/

    uint8_t dummy[11];
    uint8_t regPll;

} Lora_Register_Map_t;


/******** Function Definitions *********/

#endif /* SX_LORA_DEFINITIONS_H */
