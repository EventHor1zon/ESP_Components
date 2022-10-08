/****************************************
* \file     sx_fsk_definitions.h
* \brief    Definitions header file for the FSK/OOK portion of the 
*           SX1276 lora/fsk chip
* \date     Oct 2021
* \author   RJAM
***************************************/

#ifndef SX_FSK_DEFINITIONS_H
#define SX_FSK_DEFINITIONS_H

/********* Includes ********************/

#include "esp_types.h"

/********* Definitions *****************/

/********** Types **********************/


typedef enum {
    SX_FSK_REGADDR_REGFIFO =            0x00,
    SX_FSK_REGADDR_OPMODE =             0x01,
    SX_FSK_REGADDR_BITRATE_MSB =        0x02,
    SX_FSK_REGADDR_BITRATE_LSB =        0x03,
    SX_FSK_REGADDR_DEV_MSB =            0x04,
    SX_FSK_REGADDR_DEV_LSB =            0x05,
    SX_FSK_REGADDR_CARRFREQ_MSB =       0x06,
    SX_FSK_REGADDR_CARRFREQ_MIDB =      0x07,
    SX_FSK_REGADDR_CARRFREQ_LSB =       0x08,
    SX_FSK_REGADDR_PA_CONFIG =          0x09,
    SX_FSK_REGADDR_PA_RAMP =            0x0A,
    SX_FSK_REGADDR_OCURRENT_PROT =      0x0B,
    SX_FSK_REGADDR_LNA_CONFIG =         0x0C,
    SX_FSK_REGADDR_RX_CONFIG =          0x0D,
    SX_FSK_REGADDR_RSSI_CONFIG =        0x0E,
    SX_FSK_REGADDR_RSSI_COLLSN =        0x0F,
    SX_FSK_REGADDR_RSSI_THRESH =        0x10,
    SX_FSK_REGADDR_RSSI_VALUE =         0x11,
    SX_FSK_REGADDR_RX_BW =              0x12,
    SX_FSK_REGADDR_AFC_BW =             0x13,
    SX_FSK_REGADDR_OOK_PEAK =           0x14,
    SX_FSK_REGADDR_OOK_FIX =            0x15,
    SX_FSK_REGADDR_OOK_AVG =            0x16,
    SX_FSK_REGADDR_RSVD_17 =            0x17,
    SX_FSK_REGADDR_RSVD_18 =            0x18,
    SX_FSK_REGADDR_RSVD_19 =            0x19,
    SX_FSK_REGADDR_AFC_FEI =            0x1A,
    SX_FSK_REGADDR_AFC_MSB =            0x1B,
    SX_FSK_REGADDR_AFC_LSB =            0x1C,
    SX_FSK_REGADDR_FEI_MSB =            0x1D,
    SX_FSK_REGADDR_FEI_LSB =            0x1E,
    SX_FSK_REGADDR_PREAMBLE_DETECT =    0x1F,
    SX_FSK_REGADDR_RX_TIMEOUT1 =        0x20,
    SX_FSK_REGADDR_RX_TIMEOUT2 =        0x21,
    SX_FSK_REGADDR_RX_TIMEOUT3 =        0x22,
    SX_FSK_REGADDR_RX_DELAY =           0x23,
    SX_FSK_REGADDR_OSC =                0x24,
    SX_FSK_REGADDR_PREAMBLE_MSB =       0x25,
    SX_FSK_REGADDR_PREAMBLE_LSB =       0x26,
    SX_FSK_REGADDR_SYNC_CONFIG =        0x27,
    SX_FSK_REGADDR_SYNC_VAL1 =          0x28,
    SX_FSK_REGADDR_SYNC_VAL2 =          0x29,
    SX_FSK_REGADDR_SYNC_VAL3 =          0x2A,
    SX_FSK_REGADDR_SYNC_VAL4 =          0x2B,
    SX_FSK_REGADDR_SYNC_VAL5 =          0x2C,
    SX_FSK_REGADDR_SYNC_VAL6 =          0x2D,
    SX_FSK_REGADDR_SYNC_VAL7 =          0x2E,
    SX_FSK_REGADDR_SYNC_VAL8 =          0x2F,
    SX_FSK_REGADDR_PKT_CONFIG1 =        0x30,
    SX_FSK_REGADDR_PKT_CONFIG2 =        0x31,
    SX_FSK_REGADDR_PAYLOAD_LEN =        0x32,
    SX_FSK_REGADDR_NODE_ADDR =          0x33,
    SX_FSK_REGADDR_BROADCAST_ADDR =     0x34,
    SX_FSK_REGADDR_FSKFIFO_THRESH =     0x35,
    SX_FSK_REGADDR_SEQ_CONFIG1 =        0x36,
    SX_FSK_REGADDR_SEQ_CONFIG2 =        0x37,
    SX_FSK_REGADDR_TIMER_RES =          0x38,
    SX_FSK_REGADDR_TIMER1_COEF =        0x39,
    SX_FSK_REGADDR_TIMER2_COEF =        0x3A,
    SX_FSK_REGADDR_IMAGE_CAL =          0x3B,
    SX_FSK_REGADDR_TEMP =               0x3C,
    SX_FSK_REGADDR_LOWBAT =             0x3D,
    SX_FSK_REGADDR_IRQFLAGS1 =          0x3E,
    SX_FSK_REGADDR_IRQFLAGS2 =          0x3F,
    SX_FSK_REGADDR_DIO_MAP1 =           0x40,
    SX_FSK_REGADDR_DIO_MAP2 =           0x41,
    SX_FSK_REGADDR_VERSION =            0x42,
    SX_FSK_REGADDR_PLL_HOP =            0x44,
    SX_FSK_REGADDR_TXCO =               0x4B,
    SX_FSK_REGADDR_PA_DAC =             0x4D,
    SX_FSK_REGADDR_FORMER_TEMP =        0x5B,
    SX_FSK_REGADDR_BITRATE_FRAC =       0x5D,
    SX_FSK_REGADDR_AGC_REF =            0x61,
    SX_FSK_REGADDR_AGC_THRESH1 =        0x62,
    SX_FSK_REGADDR_AGC_THRESH2 =        0x63,
    SX_FSK_REGADDR_AGC_THRESH3 =        0x64,
    SX_FSK_REGADDR_PLL =                0x70,
    SX_FSK_ADDRESS_INVALID,
} FSK_regaddress_t;


typedef struct FSK_Register_Map
{
    /* Fifo  */
    uint8_t regFifo;                    /**< 0x00 **/
    
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
    } regOpMode;                        /**< 0x01 **/

    uint8_t regBitrateMsb;              /**< 0x02 **/
    uint8_t regBitrateLsb;              /**< 0x03 **/

    union {
        uint8_t regByte;
        struct {
            uint8_t freqDevnMsb : 6;
            uint8_t resvd : 2;
        } regBits;
    } regFreqDevnMsb;                   /**< 0x04 **/

    uint8_t regFreqDevn;                /**< 0x05 **/

    uint8_t regRfCarrierFreqMsb;        /**< 0x06 **/
    uint8_t regRfCarrierFreqMidsb;      /**< 0x07 **/
    uint8_t regRfCarrierFreqLsb;        /**< 0x08 **/


    /** Transmitter registers **/
    union 
    {
        uint8_t regByte;
        struct {
            uint8_t outputPower : 4;
            uint8_t maxPower : 3;
            uint8_t pAmpSel : 1;
        } regBits;
    } regPwrAmpCfg;                     /**< 0x09 **/

    union {
        uint8_t regByte;
        struct {
            uint8_t paRamp : 4;
            uint8_t rsvd : 1;
            uint8_t modulationShape : 2;
            uint8_t unused : 1;
        } regBits;
    } regPaRamp;                        /**< 0x0A **/

    union {
        uint8_t regByte;
        struct {
            uint8_t ocpTrim : 4;
            uint8_t ocpEn : 1;
            uint8_t unsused : 2;
        } regBits;
    } regOcp;                           /**< 0x0B **/

    /** Receiver Settings Registers **/

    union {
        uint8_t regByte;
        struct {
            uint8_t lnaBoostHFrq : 2;
            uint8_t rsvd : 1;
            uint8_t lnaBoostLFrq : 2;
            uint8_t lnaGain : 3;
        } regBits;
    } regLNA;                           /**< 0x0C **/

    union {
        uint8_t regByte;
        struct {
            uint8_t rxTrigger : 3;
            uint8_t agcAutoEn : 1;
            uint8_t afcAutoEn : 1;
            uint8_t restartRxWithPLL : 1;
            uint8_t restartRxWoPLL : 1;
            uint8_t restartRxOnCollision : 1;
        } regBits;
    } regRxCfg;                         /**< 0x0D **/

    union {
        uint8_t regByte;
        struct {
            uint8_t rssiSmoothing : 3;
            uint8_t rssiOffset : 5;
        } regBits;
    } regRssiCfg;                       /**< 0x0E **/

    
    uint8_t rssiCollisionThresh;        /**< 0x0F **/
    uint8_t rssiThresh;                 /**< 0x10 **/
    uint8_t rssiValue;                  /**< 0x11 **/

    union {
        uint8_t regByte;
        struct {
            uint8_t rxBwExp : 3;
            uint8_t rxBwMan : 2;
            uint8_t rsvd : 2;
            uint8_t unused : 1;
        } regBits;
    } regRxBw;                          /**< 0x12 **/

    union {
        uint8_t regByte;
        struct {
            uint8_t rxBwExpAft : 3;
            uint8_t rxBwMantAfc : 2;
            uint8_t rsvd : 3;
        } regBits;
    } regAfcBw;                         /**< 0x13 **/

    union {
        uint8_t regByte;
        struct {
            uint8_t ookPeakThreahStep : 3;
            uint8_t ookThreshType : 2;
            uint8_t bitSyncEn : 1;
            uint8_t rsvd : 2;
        } regBits;
    } regOokPeak;                       /**< 0x14 **/
    
    uint8_t regOOKFix;                  /**< 0x15 **/

    union {
        uint8_t regByte;
        struct {
            uint8_t ookAvgThreshFit : 2;
            uint8_t ookAvgOffset : 2;
            uint8_t rsvd : 1;
            uint8_t ookPeakThreshDecrm : 3;
        } regBits;
    } regOOKAvg;                        /**< 0x16 **/

    uint8_t regRsvd17;                  /**< 0x17 **/
    uint8_t regRsvd18;                  /**< 0x18 **/
    uint8_t regRsvd19;                  /**< 0x19 **/

    union {
        uint8_t regByte;
        struct {
            uint8_t afcAutoCleanEn : 1;
            uint8_t afcClear : 1;
            uint8_t unused2 : 1;
            uint8_t rsvd : 1;
            uint8_t agcStart : 1;
            uint8_t unused : 2;
        } regBits;
    } regAfcFei;                        /**< 0x1A **/

    uint8_t regAfcMsb;                  /**< 0x1B **/
    uint8_t regAfcLsb;                  /**< 0x1C **/

    uint8_t regFeiMsb;                  /**< 0x1D **/
    uint8_t regFeiLsb;                  /**< 0x1E **/

    union {
        uint8_t regByte;
        struct {
            uint8_t preambleDetectorTln : 5;
            uint8_t preambleDetectorSize : 2;
            uint8_t preambleDetectorEn : 1;
        } regBits;
    } regPreambleDetect;                /**< 0x1F **/

    uint8_t regRxTimeout1;              /**< 0x20 **/
    uint8_t regRxTimeout2;              /**< 0x21 **/
    uint8_t regRxTimeout3;              /**< 0x22 **/
    uint8_t regRxDelay;                 /**< 0x23 **/

    /** RC Oscilator Registers **/
    union {
        uint8_t regByte;
        struct {
            uint8_t clkOut : 3;
            uint8_t rcCalStart : 1;
            uint8_t unused : 4;
        } regBits;
    } regOsc;                           /**< 0x24 **/

    /** Packet Handling Registers **/

    uint8_t regPreambleMsb;             /**< 0x25 **/
    uint8_t regPreambleLsb;             /**< 0x26 **/

    union {
        uint8_t regByte;
        struct {
            uint8_t syncSize : 3;
            uint8_t rsvd : 1;
            uint8_t syncEn : 1;
            uint8_t  preamblePolarity : 1;
            uint8_t autoRestartRxMode : 2;
        } regBits;
    } regSyncCfg;                       /**< 0x27 **/

    uint8_t regSyncValue1;              /**< 0x28 **/
    uint8_t regSyncValue2;
    uint8_t regSyncValue3;
    uint8_t regSyncValue4;
    uint8_t regSyncValue5;
    uint8_t regSyncValue6;
    uint8_t regSyncValue7;
    uint8_t regSyncValue8;              /**< 0x2F **/

    union {
        uint8_t regByte;
        struct {
            uint8_t crcWhiteType : 1;
            uint8_t addressFiltering : 2;
            uint8_t crcAutoClrOff : 1;
            uint8_t crcEn : 1;
            uint8_t dcFree : 2;
            uint8_t pktFormat : 1;
        } regBits;
    } regPacketCfg1;                    /**< 0x30 **/

    union {
        uint8_t regByte;
        struct {
            uint8_t payloadLenMsb : 3;
            uint8_t beaconEn : 1;
            uint8_t ioHomePwrFrame : 1;
            uint8_t ioHomeEn : 1;
            uint8_t dataMode : 1;
            uint8_t unused : 1;
        } regBits;
    } regPacketCfg2;                    /**< 0x31 **/

    uint8_t payloadLenLsb;              /**< 0x32 **/
    uint8_t regNodeAddress;             /**< 0x33 **/
    uint8_t regBroadcastAddress;        /**< 0x34 **/

    union {
        uint8_t regByte;
        struct {
            uint8_t fifoThresh : 6;
            uint8_t unused : 1;
            uint8_t txStartCondition : 1;
        } regBits;
    } regFifoThresh;                    /**< 0x35 **/

    /** Sequence Registers **/

    union {
        uint8_t regByte;
        struct {
            uint8_t fromTransmit : 1;
            uint8_t fromIdle : 1;
            uint8_t lowPwrSelect : 1;
            uint8_t fromStart : 2;
            uint8_t idleMode : 1;
            uint8_t sequencerStop : 1;
            uint8_t sequencerStart : 1;
        } regBits;
    } regSequencerConf1;                /**< 0x36 **/

    union {
        uint8_t regByte;
        struct {
            uint8_t fromPacketRcvd : 3;
            uint8_t fromRxTimeout : 2;
            uint8_t fromReceive : 3;
        } regBits;
    } regSequencerCfg2;                 /**< 0x37 **/

    union {
        uint8_t regByte;
        struct {
            uint8_t timer2Resolution : 2;
            uint8_t timer1Resolution : 2;
            uint8_t unused : 4;
        } regBits;
    } regTimerResolution;               /**< 0x38 **/

    uint8_t regTimer1Coeff;             /**< 0x39 **/
    uint8_t regTimer2Coeff;             /**< 0x3A **/

    /** Service Registers **/

    union {
        uint8_t regByte;
        struct {
            uint8_t tempMonitorOff : 1;
            uint8_t tempThresh : 2;
            uint8_t tempChange : 1;
            uint8_t unused : 1;
            uint8_t imageCalbrRunning : 1;
            uint8_t imageCalibrStart : 1;
            uint8_t autoImageCalEn : 1;
        } regBits;
    } regImageCal;                      /**< 0x3B **/

    uint8_t regTemp;                    /**< 0x3C **/

    union {
        uint8_t regByte;
        struct {
            uint8_t lowBatTrim : 3;
            uint8_t lowBatEn : 1;
            uint8_t unused : 4;
        } regBits;
    } regLowBat;                        /**< 0x3D **/

    union {
        uint8_t regByte;
        struct {
            uint8_t syncAddrMatch : 1;
            uint8_t preambleDetect : 1;
            uint8_t timeout : 1;
            uint8_t rssi : 1;
            uint8_t pllLock : 1;
            uint8_t txReady : 1;
            uint8_t rxReady : 1;
            uint8_t modeReady : 1;
        } regBits;
    } regIrqFlags1;                     /**< 0x3E **/

    union {
        uint8_t regByte;
        struct {
            uint8_t lowBat : 1;
            uint8_t crcOk : 1;
            uint8_t payloadready : 1;
            uint8_t packetSent : 1;
            uint8_t fifoOverrun : 1;
            uint8_t fifoLevel : 1;
            uint8_t fifoEmpty : 1;
            uint8_t fifoFull : 1; 
        } regBits;
    } regIrqFlags2;                     /**< 0x3F **/

    /** IO Control Registers **/
    union {
        uint8_t regByte;
        struct {
            uint8_t dio3mapping : 2;
            uint8_t dio2mapping : 2;
            uint8_t dio1mapping : 2;
            uint8_t dio0mapping : 2;
        } regBits;
    } regDioMapping1;                   /**< 0x40 **/

    union {
        uint8_t regByte;
        struct {
            uint8_t mapPreambleDetect : 1;
            uint8_t rsvd : 3;
            uint8_t dio5mapping : 2;
            uint8_t dio4mapping : 2;
        } regBits;
    } regDioMapping2;                   /**< 0x41 **/

    /** Version Register **/

    uint8_t regVersion;                 /**< 0x42 **/
    uint8_t reg20rsvd;                  /**< 0x43 **/

    /** Additional registers **/

    union {
        uint8_t regByte;
        struct {
            uint8_t rsvd : 7;
            uint8_t fastHopEn : 1;
        } regBits;
    } regPllHop;                        /**< 0x44 **/


    union {
        uint8_t regByte;
        struct {
            uint8_t rsvd0 : 4;
            uint8_t txcoInputEn : 1;
            uint8_t rsvd1 : 3;
        } regBits;
    } regTxco;                          /**< 0x4B **/

    union {
        uint8_t regByte;
        struct {
            uint8_t paDac : 3;
            uint8_t rsvd : 5;
        } regBits;
    } regPaDac;                         /**< 0x4D **/

    uint8_t regFormerTemp;

    union {
        uint8_t regByte;
        struct {
            uint8_t bitRateFractional : 4;
            uint8_t unused : 4;
        } regBits;
    } regBitrateFrac;                   /**< 0x5B **/

    union {
        uint8_t regByte;
        struct {
            uint8_t agcReferenceLvl : 6;
            uint8_t unused : 2;
        } regBits;
    } regAgcRef;                        /**< 0x61 **/

    union {
        uint8_t regByte;
        struct {
            uint8_t agcStep1 : 4;
            uint8_t unused : 4;
        } regBits;
    } regAcgThresh1;                    /**< 0x62 **/

    union {
        uint8_t regByte;
        struct {
            uint8_t agcStep3 : 4;
            uint8_t agcStep2 : 4;
        } regBits;
    } regAgcThresh2;                    /**< 0x63 **/

    union {
        uint8_t regByte;
        struct {
            uint8_t agcStep5 : 4;
            uint8_t agcStep4 : 4;
        } regBits;
    } regAgcThresh3;                    /**< 0x64 **/

    uint8_t dummy[11];
    uint8_t regPll;                     /**< 0x70 **/

} FSK_Register_Map_t;


/******** Function Definitions *********/

#endif /* FSK_DEFINITIONS_H */
