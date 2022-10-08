/****************************************
* \file
* \brief
* \date
* \author
****************************************/

#ifndef HEADER_H
#define HEADER_H

/********* Includes ********************/


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_types.h"
#include "esp_err.h"
#include "esp_log.h"
#include "Utilities.h"
#include "genericCommsDriver.h"
#include "driver/gpio.h"

/********* Definitions *****************/
#define DEBUG_MODE 1


/** TODO: Make this dynamic! **/
#define MFRC_I2C_ADDRESS                0x3A
#define MFRC_DEFAULT_TIMEOUT_MS         100

#define MRFC_REGADDR_COMMAND            0x01
#define MRFC_REGADDR_COMM_INTR_EN       0x02
#define MRFC_REGADDR_DEV_INTR_EN        0x03
#define MRFC_REGADDR_COMM_IRQ           0x04
#define MRFC_REGADDR_DIV_IRQ            0x05
#define MRFC_REGADDR_ERRORS             0x06
#define MRFC_REGADDR_STATUS_1           0x07
#define MRFC_REGADDR_STATUS_2           0x08
#define MRFC_REGADDR_FIFO_DATA          0x09
#define MRFC_REGADDR_FIFO_LEVEL         0x0A
#define MRFC_REGADDR_FIFO_WATERLEVEL    0x0B
#define MRFC_REGADDR_CONTROL            0x0C
#define MRFC_REGADDR_BIT_FRAMING        0x0D
#define MRFC_REGADDR_COLLISION_DETECT   0x0E
#define MRFC_REGADDR_MODE               0x11
#define MRFC_REGADDR_TX_MODE            0x12
#define MRFC_REGADDR_RX_MODE            0x13
#define MRFC_REGADDR_TX_CONTROL         0x14
#define MRFC_REGADDR_TX_ASK             0x15
#define MRFC_REGADDR_TX_SELECT          0x16
#define MRFC_REGADDR_RX_SELECT          0x17
#define MRFC_REGADDR_RX_THRESH          0x18
#define MRFC_REGADDR_DEMODULATOR        0x19
#define MRFC_REGADDR_MIFARE_TX          0x1C
#define MRFC_REGADDR_MIFARE_RX          0x1D
#define MRFC_REGADDR_SERIAL_SPEED       0x1F

#define MRFC_REGADDR_CRC_RESULT_MSB     0x21
#define MRFC_REGADDR_CRC_RESULT_LSB     0x22
#define MRFC_REGADDR_MOD_WIDTH          0x24
#define MRFC_REGADDR_RFC_GAIN           0x26
#define MRFC_REGADDR_GS_N               0x27
#define MRFC_REGADDR_CWG_SP             0x28
#define MRFC_REGADDR_MODG_SP            0x29
#define MRFC_REGADDR_TMR_MODE           0x2A
#define MRFC_REGADDR_TMR_PRESCALE       0x2B
#define MFRC_REGADDR_TCNTR_RELOAD_MSB   0x2C
#define MFRC_REGADDR_TCNTR_RELOAD_LSB   0x2D
#define MRFC_REGADDR_TCNTR_MSB          0x2E
#define MRFC_REGADDR_TCNTR_LSB          0x2F

#define MFRC_REGADDR_TEST_SEL_1         0x31
#define MFRC_REGADDR_TEST_SEL_2         0x32
#define MFRC_REGADDR_TEST_PIN_EN        0x33
#define MFRC_REGADDR_TEST_PIN_VAL       0x34
#define MFRC_REGADDR_TEST_BUS           0x35
#define MFRC_REGADDR_TEST_AUTO          0x36
#define MFRC_REGADDR_VERSION            0x37
#define MFRC_REGADDR_TEST_ANALOG        0x38
#define MFRC_REGADDR_TEST_DAC_1         0x39
#define MFRC_REGADDR_TEST_DAC_2         0x3A
#define MFRC_REGADDR_TEST_ADC           0x3B

#define MFRC_CMD_IDLE                   0
#define MFRC_CMD_MEM                    1
#define MFRC_CMD_GEN_RAND_ID            2
#define MFRC_CMD_CALC_CRC               3
#define MFRC_CMD_TRANSMIT               4
#define MFRC_CMD_NO_CHANGE              7
#define MFRC_CMD_RECEIVE                8
#define MFRC_CMD_TRANCEIVE              12
#define MFRC_CMD_MIFARE_AUTH            15
#define MFRC_CMD_SOFT_RESET             16

#define MFRC_FIFO_SIZE_BITS             8 * 64
#define MFRC_FIFO_SIZE_BYTES            64
#define MFRC_FIFO_MAX_WL                63

#define MFRC_TIMER_INPUT_FREQ_HZ        13560000

#define MFRC_ISR_TMR        (1 << 0)
#define MFRC_ISR_ERR        (1 << 1)
#define MFRC_ISR_FIFO_LO    (1 << 2)
#define MFRC_ISR_FIFO_HI    (1 << 3)
#define MFRC_ISR_IDLE       (1 << 4)
#define MFRC_ISR_RX_DONE    (1 << 5)
#define MFRC_ISR_TX_DONE    (1 << 6)

#define MFRC_DEVISR_MFIN    (1 << 4)
#define MFRC_DEVISR_CRC     (1 << 2)


/********** Types **********************/

/** 
 *  Try a macro based approach to register checking, 
 *  see how it works out?
 **/ 

/** register 1 macros **/
#define MFRC_CRC_RESULT_FROM_REG(x) (x & (1 << 6) ? true : false)
#define MFRC_CRC_READY_FROM_REG(x)  (x & (1 << 5) ? true : false)
#define MFRC_IRQ_READY_FROM_REG(x)  (x & (1 << 4) ? true : false)
#define MFRC_TMR_RUNNING_FROM_REG(x)(x & (1 << 3) ? true : false)
#define MFRC_FIFO_HI_FROM_REG(x)    (x & (1 << 1) ? true : false)
#define MFRC_FIFO_LO_FROM_REG(x)    (x & (1 << 0) ? true : false)

/** register 2 macros **/
#define MFRC_MODEMSTATE_FROM_REG(x) ((mfrc_modemstate_t)(x & 0b111))
#define MFRC_MFCRYPTO_EN_FROM_REG(x)(x & (1 << 3) ? true : false)
#define MFRC_I2C_HS_FROM_REG(x)     (x & (1 << 6) ? true : false)
#define MFRC_TEMPSENSE_FROM_REG(x)  (x & (1 << 7) ? true : false)

/** com irq macros **/
#define MFRC_IRQ_TIMER_SET(x)   (x & (1 << 0) ? true : false)
#define MFRC_IRQ_ERROR_SET(x)   (x & (1 << 1) ? true : false)
#define MFRC_IRQ_FIFO_LO_SET(x) (x & (1 << 2) ? true : false)
#define MFRC_IRQ_FIFO_HI_SET(x) (x & (1 << 3) ? true : false)
#define MFRC_IRQ_IDLE_SET(x)    (x & (1 << 4) ? true : false)
#define MFRC_IRQ_RX_SET(x)      (x & (1 << 5) ? true : false)
#define MFRC_IRQ_TX_SET(x)      (x & (1 << 6) ? true : false)

/** div irq macros **/
#define MFRC_IRQ_MFIN_SET(x)    (x & (1 << 4) ? true : false)
#define MFRC_IRQ_CRC_SET(x)     (x & (1 << 2) ? true : false)

/** control reg macros **/
#define MFRC_RX_LAST_BITS_VALID(x) ((x & 0b111) == 0 ? true : false)
#define MFRC_RCV_OFF_SET(x)     (x & (1 << 5) ? true : false)
#define MFRC_PWR_DOWN_SET(x)    (x & (1 << 4) ? true : false)

typedef enum {
    MFRC_COMMAND_IDLE = 0b0000,
    MFRC_COMMAND_MEM = 0b0001,
    MFRC_COMMAND_RAND_ID = 0b0010,
    MFRC_COMMAND_CALC_CRC = 0b0011,
    MFRC_COMMAND_TRANSMIT = 0b0100,
    MFRC_COMMAND_NO_CHANGE = 0b0111,
    MFRC_COMMAND_RECEIVE = 0b1000,
    MFRC_COMMAND_TRANSCEIVE = 0b1100,
    MFRC_COMMAND_MF_AUTHENT = 0b1110,
    MFRC_COMMAND_SOFT_RESET = 0b1111,
} mfrc_cmd_t;

typedef enum {
    MFRC_PWRMODE_ON,
    MFRC_PWRMODE_STARTING,
    MFRC_PWRMODE_POWER_DOWN,
    MFRC_PWRMODE_INVALID,
} mfrc_pwrmode_t;

typedef enum {
    MFRC_RECVR_PWR_ON,
    MFRC_RECVR_PWR_ANLG_OFF,
    MFRC_RECVR_PWR_INVALID,
} mfrc_rcvr_pwr_t;

typedef enum {
    MFRC_TX_PWR_ON,
    MFRC_TX_PWR_OFF,
    MFRC_TX_PWR_INVALID,
} mfrc_tx_pwr_t;

typedef enum {
    MFRC_UART_SEL_LOW,
    MFRC_UART_SEL_MANCHESTER,
    MFRC_UART_SEL_MODULATED_INTRN,
    MFRC_UART_SEL_NRZ_CODING,
    MFRC_UART_SEL_INVALID,
} mfrc_uart_sel_t;

typedef enum {
    MFRC_RX_GAIN_18_DB,
    MFRC_RX_GAIN_23_DB,
    MFRC_RX_GAIN_33_DB,
    MFRC_RX_GAIN_38_DB,
    MFRC_RX_GAIN_43_DB,
    MFRC_RX_GAIN_48_DB,
    MFRC_RX_GAIN_INVALID,
} mfrc_rx_gain_t;

typedef enum {
    MFRC_CRC_PRESET_0,
    MFRC_CRC_PRESET_6363,
    MFRC_CRC_PRESET_A671,
    MFRC_CRC_PRESET_FFFF,
    MFRC_CRC_PRESET_INVALID,
} mfrc_crc_preset_t;

typedef enum {
    MFRC_MODEMSTATE_IDLE,
    MFRC_MODEMSTATE_BF_START,
    MFRC_MODEMSTATE_TX_WAIT,
    MFRC_MODEMSTATE_TX,
    MFRC_MODEMSTATE_RX_WAIT,
    MFRC_MODEMSTATE_DATA_WAIT,
    MFRC_MODEMSTATE_RX,
} mfrc_modemstate_t;

typedef enum {
    MFRC_SPI_COMMS_MODE,
    MFRC_I2C_COMMS_MODE,
    MFRC_COMMS_MODE_INVALID,
} mfrc_comms_mode_t;

typedef enum {
    MFRC_INTR_SRC_IRQ,
    MFRC_INTR_SRC_TX_IRQ,
    MFRC_INTR_SRC_CRC_IRQ,
    MFRC_INTR_SRC_RX_IRQ,
    MFRC_INTR_SRC_FIFO_HI_IRQ,
    MFRC_INTR_SRC_FIFO_LO_IRQ,
    MFRC_INTR_SRC_ERR_IRQ,
} mfrc_intr_src_t;


typedef struct MFRC_Init
{
    /* data */
    uint8_t comms_bus;
    mfrc_comms_mode_t comms_mode;
    gpio_num_t rst_pin;
    gpio_num_t irq_pin;
    gpio_num_t ss_pin;

} mfrc_init_t;


// Commands sent to the PICC.
typedef enum {
    // The commands used by the PCD to manage communication with several PICCs (ISO 14443-3, Type A, section 6.4)
    PICC_CMD_REQA            = 0x26,        // REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
    PICC_CMD_WUPA            = 0x52,        // Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
    PICC_CMD_CT                = 0x88,        // Cascade Tag. Not really a command, but used during anti collision.
    PICC_CMD_SEL_CL1        = 0x93,        // Anti collision/Select, Cascade Level 1
    PICC_CMD_SEL_CL2        = 0x95,        // Anti collision/Select, Cascade Level 2
    PICC_CMD_SEL_CL3        = 0x97,        // Anti collision/Select, Cascade Level 3
    PICC_CMD_HLTA            = 0x50,        // HaLT command, Type A. Instructs an ACTIVE PICC to go to state HALT.
    PICC_CMD_RATS           = 0xE0,     // Request command for Answer To Reset.
    // The commands used for MIFARE Classic (from http://www.mouser.com/ds/2/302/MF1S503x-89574.pdf, Section 9)
    // Use PCD_MFAuthent to authenticate access to a sector, then use these commands to read/write/modify the blocks on the sector.
    // The read/write commands can also be used for MIFARE Ultralight.
    PICC_CMD_MF_AUTH_KEY_A    = 0x60,        // Perform authentication with Key A
    PICC_CMD_MF_AUTH_KEY_B    = 0x61,        // Perform authentication with Key B
    PICC_CMD_MF_READ        = 0x30,        // Reads one 16 byte block from the authenticated sector of the PICC. Also used for MIFARE Ultralight.
    PICC_CMD_MF_WRITE        = 0xA0,        // Writes one 16 byte block to the authenticated sector of the PICC. Called "COMPATIBILITY WRITE" for MIFARE Ultralight.
    PICC_CMD_MF_DECREMENT    = 0xC0,        // Decrements the contents of a block and stores the result in the internal data register.
    PICC_CMD_MF_INCREMENT    = 0xC1,        // Increments the contents of a block and stores the result in the internal data register.
    PICC_CMD_MF_RESTORE        = 0xC2,        // Reads the contents of a block into the internal data register.
    PICC_CMD_MF_TRANSFER    = 0xB0,        // Writes the contents of the internal data register to a block.
    // The commands used for MIFARE Ultralight (from http://www.nxp.com/documents/data_sheet/MF0ICU1.pdf, Section 8.6)
    // The PICC_CMD_MF_READ and PICC_CMD_MF_WRITE can also be used for MIFARE Ultralight.
    PICC_CMD_UL_WRITE        = 0xA2        // Writes one 4 byte page to the PICC.
} PICC_Command_t;



/** Gonna give this struct register thing a go, might 
 *  be easier to manager device settings? 
 **/

typedef struct mfrc_registers {

    /** 0x00 **/
    uint8_t reserved_0;

    /** 0x01 **/
    union {
        uint8_t regval;
        struct {
            uint8_t command     :   4;
            uint8_t pwr_down    :   1;
            uint8_t rcv_off     :   1;
            uint8_t rsvd_0      :   2;
        } regbits;
    } command_reg;
    
    /** 0x02 **/
    union {
        uint8_t regval;
        struct {
            uint8_t tmr_irq_en  :   1;
            uint8_t err_irq_en  :   1;
            uint8_t flo_irq_en  :   1;
            uint8_t fhi_irq_en  :   1;
            uint8_t idle_irq_en :   1;
            uint8_t rx_irq_en   :   1;
            uint8_t tx_irq_en   :   1;
            uint8_t irq_invert  :   1;
        } regbits;
    } command_en_reg;
    
    /** 0x03 **/
    union {
        uint8_t regval;
        struct {
            uint8_t rsvd_0      :   2;
            uint8_t crc_irq_en  :   1;
            uint8_t rsvd_1      :   1;
            uint8_t mfin_irq_en :   1;
            uint8_t rsvd_2      :   2;
            uint8_t irq_pp_od   :   1;
        } regbits;
    } irq_passing_reg;

    /** 0x04 **/
    union {
        uint8_t regval;
        struct {
            uint8_t tmr_irq     :   1;
            uint8_t err_irq     :   1;
            uint8_t flo_irq     :   1;
            uint8_t fli_irq     :   1;
            uint8_t idle_irq    :   1;
            uint8_t rx_irq      :   1;
            uint8_t tx_irq      :   1;
            uint8_t irq_set     :   1;
        } regbits;
    } com_irq_reg;
    
    /** 0x05 **/
    union {
        uint8_t regByte;
        struct {
            uint8_t rsvd_0      :   2;
            uint8_t crc_irq     :   1;
            uint8_t rsvd_1      :   1;
            uint8_t mfin_irq    :   1;
            uint8_t rsvd_2      :   2;
            uint8_t irq_set_2   :   1;
        } regBits;
    } div_irq_reg;

    /** 0x06 **/
    union {
        uint8_t regByte;
        struct {
            uint8_t protocol_err    :   1;
            uint8_t parity_err      :   1;
            uint8_t crc_err         :   1;
            uint8_t coll_err        :   1;
            uint8_t buffer_ovr_err  :   1;
            uint8_t rsvd_0          :   1;
            uint8_t tempr_err       :   1;
            uint8_t wrt_err         :   1;
        } regBits;
    } error_reg;

    /** 0x07 **/
    union {
        uint8_t regByte;
        struct {
            uint8_t fifo_lo_alert   :   1;
            uint8_t fifo_hi_alert   :   1;
            uint8_t rsvd_0          :   1;
            uint8_t tmr_running     :   1;
            uint8_t irq_alert       :   1;
            uint8_t crc_ready       :   1;
            uint8_t crc_ok          :   1;
            uint8_t rsvd_1          :   1;
        } regBits;
    } status_1_reg;

    /** 0x08 **/
    union {
        uint8_t regByte;
        struct {
            uint8_t modem_state     :   2;
            uint8_t mifare_crypt1_on :  2;
            uint8_t rsvd_0          :   2;
            uint8_t i2c_force_hs    :   1;
            uint8_t temp_sens_clr   :   1;
        } regBits;
    } status_2_reg;

    /** 0x09 **/
    union {
        uint8_t regByte;
        struct {
            uint8_t fifo_data : 8;
        } regBits;
    } fifo_data_reg;

    /** 0x0A **/
    union {
        uint8_t regByte;
        struct {
            uint8_t fifo_level      :   7;
            uint8_t flush_buffer    :   1;
        } regBits;
    } fifo_lvl_reg;

    /** 0x0B **/ 
    union {
        uint8_t regByte;
        struct {
            uint8_t fifo_watermark  :   6;
            uint8_t rsvd_0          :   2;
        } regBits;
    } fifo_watermark_reg;

    /** 0x0C **/
    union {
        uint8_t regByte;
        struct {
            uint8_t rx_last_bits    :   3;
            uint8_t rsvd_0          :   3;
            uint8_t tmr_start_now   :   1;
            uint8_t tmr_stop_now    :   1;
        } regBits;
    } control_reg;

    /** 0x0D **/
    union {
        uint8_t regByte;
        struct {
            uint8_t tx_last_bits    :   3;
            uint8_t rsvd_0          :   1;
            uint8_t rx_align        :   3;
            uint8_t start_send      :   1;
        } regBits;
    } bitframe_reg;

    /** 0x0E **/
    union {
        uint8_t regByte;
        struct {
            uint8_t collision_pos       :   5;
            uint8_t coll_pos_invalid    :   1;
            uint8_t rsvd_0              :   1;
            uint8_t clr_rx_on_collision :   1;
        } regBits;
    } collision_reg;

    /** 0x0F **/
    uint8_t reserved_1;

    /** 0x10 **/
    uint8_t reserved_2;

    /** 0x11 **/
    union {
        uint8_t regByte;
        struct {
            uint8_t crc_preset         :   2;
            uint8_t rsvd_0             :   1;
            uint8_t pol_mf_in          :   1;
            uint8_t rsvd_1             :   1;
            uint8_t tx_wait_rf         :   1;
            uint8_t rsvd_2             :   1;
            uint8_t crc_msb_first      :   1;
        } regBits;
    } mode_register;

    /** 0x12 **/
    union {
        uint8_t regByte;
        struct {
            uint8_t rsvd_0             :   3;
            uint8_t invert_modulation  :   1;
            uint8_t tx_speed           :   3;
            uint8_t tx_crc_en          :   1;
        } regBits;
    } tx_mode_reg;

    /** 0x13 **/
    union {
        uint8_t regByte;
        struct {
            uint8_t rsvd_0              :   2;
            uint8_t rx_multiple         :   1;
            uint8_t rx_no_err           :   1;
            uint8_t rx_speed            :   3;
            uint8_t rx_crc_en           :   1;
        } regBits;
    } rx_mode_reg;

    /** 0x14 **/
    union {
        uint8_t regByte;
        struct {
            uint8_t Tx1RFEn            :    1;
            uint8_t Tx2RFEn            :    1;
            uint8_t rsvd_0             :    1;
            uint8_t Tx2CW              :    1;
            uint8_t InvTx1RFOff        :    1;
            uint8_t InvTx2RFOff        :    1;
            uint8_t InvTx1RFOn         :    1;
            uint8_t InvTx2RFOn         :    1;
        } regBits;
    } tx_control_reg;

    /** 0x15 **/
    union {
        uint8_t regByte;
        struct {
            uint8_t rsvd_0             :    6;
            uint8_t Force100ASK        :    1;
            uint8_t rsvd_1             :    1;
        } regBits;
    } tx_ask_reg;

    /** 0x16 **/
    union {
        uint8_t regByte;
        struct {
            uint8_t MFOutSel           :    4;
            uint8_t DriverSel          :    2;
            uint8_t rsvd_0             :    2;
        } regBits;
    } tx_sel_reg;
    
    /** 0x17 **/
    union {
        uint8_t regByte;
        struct {
            uint8_t RxWait           :    6;
            uint8_t UartSel          :    2;
        } regBits;
    } rx_sel_reg;

    /** 0x18 **/
    union {
        uint8_t regByte;
        struct {
            uint8_t CollLevel          :    3;
            uint8_t rsvd_0             :    1;
            uint8_t MinLevel           :    4;
        } regBits;
    } rx_thresh_reg;
    
    /** 0x19 **/
    union {
        uint8_t regByte;
        struct {
            uint8_t AddIQ           :    2;
            uint8_t FixIQ           :    1;
            uint8_t TPrescalEven    :    1;
            uint8_t TauRcv          :    2;
            uint8_t TauSync         :    2;
        } regBits;
    } demod_reg;

    uint8_t reserved_3;
    uint8_t reserved_4;

    /** 0x1C **/
    union {
        uint8_t regByte;
        struct {
            uint8_t TxWait           :    2;
            uint8_t rsvd_0           :    6;
        } regBits;
    } mf_tx_reg;

    /** 0x1D **/
    union {
        uint8_t regByte;
        struct {
            uint8_t rsvd_0          :    4;
            uint8_t ParityDisable   :    1;
            uint8_t rsvd_1          :    3;
        } regBits;
    } mf_rx_reg;

    uint8_t reserved_5;

    /** 0x1E **/
    union {
        uint8_t regByte;
        struct {
            uint8_t BR_T1           :    5;
            uint8_t BR_T0           :    3;
        } regBits;
    } serial_speed_reg;

    uint8_t reserved_6;

    /** 0x21 **/
    union {
        uint8_t regByte;
        struct {
            uint8_t crc_res_msb       :    8;
        } regBits;
    } crc_res_msb_reg;

    /** 0x22 **/
    union {
        uint8_t regByte;
        struct {
            uint8_t crc_res_lsb        :    8;
        } regBits;
    } crc_res_lsb_reg;

    /** 0x23 **/
    struct {
        union {
            uint8_t regbyte;
            struct {
                uint8_t modwidth        :    8;
            } regbits;
        } value;
        uint8_t address;
        uint8_t rstval;
    } modwidth_reg; 

    /** 0x24 **/
    struct {
        union {
            uint8_t regbyte;
            struct {
                uint8_t rsvd0        :    8;
            } regbits;
        } value;
        uint8_t address;
        uint8_t rstval;
    } reserved6; 

    /** 0x25 **/
    struct {
        union {
            uint8_t regbyte;
            struct {
                uint8_t rsvd0        :    4;
                uint8_t rx_gain        :    3;
                uint8_t rsvd1        :    1;
            } regbits;
        } value;
        uint8_t address;
        uint8_t rstval;
    } rfccfg_reg; 

    /** 0x26 **/
    struct {
        union {
            uint8_t regbyte;
            struct {
                uint8_t mod_gsn        :    4;
                uint8_t cw_gsn        :    4;
            } regbits;
        } value;
        uint8_t address;
        uint8_t rstval;
    } GsN_reg; 

    /** 0x27 **/
    struct {
        union {
            uint8_t regbyte;
            struct {
                uint8_t cwg_sp        :    6;
                uint8_t rsvd0        :    2;
            } regbits;
        } value;
        uint8_t address;
        uint8_t rstval;
    } cwg_sp_reg; 

    /** 0x28 **/
    struct {
        union {
            uint8_t regbyte;
            struct {
                uint8_t mod_gsp        :    6;
                uint8_t rsvd0        :    2;
            } regbits;
        } value;
        uint8_t address;
        uint8_t rstval;
    } mod_gsp_reg; 

    /** 0x29 **/
    struct {
        union {
            uint8_t regbyte;
            struct {
                uint8_t tprescaler_hi        :    4;
                uint8_t tmr_autorestart        :    1;
                uint8_t tmr_gated            :    2;
                uint8_t tmr_auto            :    1;
            } regbits;
        } value;
        uint8_t address;
        uint8_t rstval;
    } tmode_reg; 

    /** 0x2A **/
    struct {
        union {
            uint8_t regbyte;
            struct {
                uint8_t tprescaler_low        :    8;

            } regbits;
        } value;
        uint8_t address;
        uint8_t rstval;
    } tprescaler_low; 

    /** 0x2B **/
    struct {
        union {
            uint8_t regbyte;
            struct {
                uint8_t tmr_reload_msb        :    8;

            } regbits;
        } value;
        uint8_t address;
        uint8_t rstval;
    } tmr_reload_reg; 

    /** 0x2C **/
    struct {
        union {
            uint8_t regbyte;
            struct {
                uint8_t tmr_reload_low        :    8;

            } regbits;
        } value;
        uint8_t address;
        uint8_t rstval;
    } tmr_reload_lsb_reg; 

    /** 0x2D **/
    struct {
        union {
            uint8_t regbyte;
            struct {
                uint8_t tmr_ctr_msb        :    8;

            } regbits;
        } value;
        uint8_t address;
        uint8_t rstval;
    } tmr_ctr_reg; 

    /** 0x2E **/
    struct {
        union {
            uint8_t regbyte;
            struct {
                uint8_t tmr_ctr_lsb        :    8;

            } regbits;
        } value;
        uint8_t address;
        uint8_t rstval;
    } tmr_ctr_lsb_reg; 

    /** 0x2F **/
    struct {
        union {
            uint8_t regbyte;
            struct {
                uint8_t rsvd0        :    8;

            } regbits;
        } value;
        uint8_t address;
        uint8_t rstval;
    } reserved7; 

    /** 0x30 **/
    struct {
        union {
            uint8_t regbyte;
            struct {
                uint8_t tstbus_sel        :    3;
                uint8_t rsvd0            :    5;
            } regbits;
        } value;
        uint8_t address;
        uint8_t rstval;
    } test_sel1_reg; 

    /** 0x31 **/
    struct {
        union {
            uint8_t regbyte;
            struct {
                uint8_t tstbus_sel        :    5;
                uint8_t prbs15            :    1;
                uint8_t prbs9            :    1;
                uint8_t tstbus_flip        :    1;
            } regbits;
        } value;
        uint8_t address;
        uint8_t rstval;
    } tst_sel2_reg; 

    /** 0x32 **/
    struct {
        union {
            uint8_t regbyte;
            struct {
                uint8_t rsvd0            :    1;
                uint8_t testpin_en        :    6;
                uint8_t rs232line_en    :    1;
            } regbits;
        } value;
        uint8_t address;
        uint8_t rstval;
    } testpin_reg; 

    /** 0x33 **/
    struct {
        union {
            uint8_t regbyte;
            struct {
                uint8_t rsvd0                :    1;
                uint8_t testpin_value        :    6;
                uint8_t use_io                :    1;
            } regbits;
        } value;
        uint8_t address;
        uint8_t rstval;
    } testpin_value_reg; 

    /** 0x34 **/
    struct {
        union {
            uint8_t regbyte;
            struct {
                uint8_t testbus        :    8;
            } regbits;
        } value;
        uint8_t address;
        uint8_t rstval;
    } testbus_reg; 

   
    struct {
        union {
            uint8_t regbyte;
            struct {
                uint8_t selftest    :    4;
                uint8_t rft            :    2;
                uint8_t amp_rcv        :    1;
                uint8_t rsvd0        :    1;
            } regbits;
        } value;
        uint8_t address;
        uint8_t rstval;
    } autotest_reg; 

    
    struct {
        union {
            uint8_t regbyte;
            struct {
                uint8_t version        :    8;

            } regbits;
        } value;
        uint8_t address;
        uint8_t rstval;
    } version_reg; 

    
    struct {
        union {
            uint8_t regbyte;
            struct {
                uint8_t analogsel_aux2        :    4;
                uint8_t analogsel_aux1        :    4;

            } regbits;
        } value;
        uint8_t address;
        uint8_t rstval;
    } analogtest_reg; 

    
    struct {
        union {
            uint8_t regbyte;
            struct {
                uint8_t testdac1    :    6;
                uint8_t rsvd0        :    2;

            } regbits;
        } value;
        uint8_t address;
        uint8_t rstval;
    } testdac1_reg; 

    
    struct {
        union {
            uint8_t regbyte;
            struct {
                uint8_t testdac2    :    6;
                uint8_t rsvd0        :    2;
            } regbits;
        } value;
        uint8_t address;
        uint8_t rstval;
    } testdac2_reg; 

    
    struct {
        union {
            uint8_t regbyte;
            struct {
                uint8_t adc_q        :    4;
                uint8_t adc_i        :    4;
            } regbits;
        } value;
        uint8_t address;
        uint8_t rstval;
    } testadc_reg; 


} MFRC_Registers_t;



typedef struct MFRC522_Driver
{
    /* data */
    uint8_t comms_bus;              /**< the comms bus used **/
    mfrc_comms_mode_t comms_mode;   /**< the communication type **/
    gpio_num_t rst_pin;             /**< the optional reset pin **/
    gpio_num_t irq_pin;             /**< optional irq pin **/
    void *comms_handle;             /**< handle for spi/i2c driver **/
    MFRC_Registers_t map;           /**< register value map **/
    bool intr_en;                   /**< if driver is currently using interrupts **/
    bool rst_en;                    /**< if driver uses reset pin **/
    bool irq_en;                    /**< if driver uses irq pin **/
    bool power_en;                  /**< device power is enabled **/
    bool rx_available;            /**< data is available to be read **/
    bool tx_done;
    TaskHandle_t task;              /**< rtos task handle **/
    TaskHandle_t isr_task;
    mfrc_cmd_t current_cmd;         /**< current command executing **/
    uint8_t raw_data[MFRC_FIFO_SIZE_BYTES+1];           /**< the raw data to rad/write **/

    uint8_t comms_intr_mask;
    uint8_t dev_intr_mask;

    bool tx_in_progress;
    bool rx_in_progress;
    bool trx_in_progress;

} MFRC522_Driver_t;


typedef MFRC522_Driver_t * MFRC_DEV;

/******** Function Declarations *********/

MFRC_DEV mfrc_init(mfrc_init_t *init);

void mfrc_deinit(MFRC_DEV dev);

esp_err_t transceive_with_target(MFRC_DEV dev);

esp_err_t mfrc_set_command(MFRC_DEV dev, mfrc_cmd_t *cmd);

esp_err_t mfrc_set_comm_interrupt_mask(MFRC_DEV dev, uint8_t *intr);

esp_err_t mfrc_get_comm_interrupt_mask(MFRC_DEV dev, uint8_t *intr);

esp_err_t mfrc_set_powerdown(MFRC_DEV dev);

esp_err_t mfrc_unset_powerdown(MFRC_DEV dev);

esp_err_t mfrc_enable_receiver(MFRC_DEV dev);

esp_err_t mfrc_disable_receiver(MFRC_DEV dev);

/** TODO: Remove **/
// esp_err_t mfrc_enable_interrupt_pin(MFRC_DEV dev);

// esp_err_t mfrc_disable_interrupt_pin(MFRC_DEV dev);

esp_err_t mfrc_set_ask(MFRC_DEV dev, bool *val);

esp_err_t mfrc_get_ask(MFRC_DEV dev, bool *val);

esp_err_t mfrc_set_crc_base(MFRC_DEV dev, mfrc_crc_preset_t *set);

esp_err_t mfrc_get_crc_base(MFRC_DEV dev, mfrc_crc_preset_t *set);

esp_err_t mfrc_set_antenna_en(MFRC_DEV dev);

esp_err_t mfrc_clear_collisions(MFRC_DEV dev);

esp_err_t mfrc_get_opendrain_en(MFRC_DEV dev, bool * en);

esp_err_t mfrc_set_opendrain_en(MFRC_DEV dev, bool * en);

esp_err_t mfrc_get_rx_uart_sel(MFRC_DEV dev, mfrc_uart_sel_t *sel);

esp_err_t mfrc_set_rx_uart_sel(MFRC_DEV dev, mfrc_uart_sel_t *sel);

esp_err_t mfrc_get_rx_wait(MFRC_DEV dev, uint8_t *wait);

esp_err_t mfrc_set_rx_wait(MFRC_DEV dev, uint8_t *wait);

esp_err_t mfrc_get_rx_min_level(MFRC_DEV dev, uint8_t *min);

esp_err_t mfrc_set_rx_min_level(MFRC_DEV dev, uint8_t *min);

esp_err_t mfrc_get_coll_level(MFRC_DEV dev, uint8_t *level);

esp_err_t mfrc_set_coll_level(MFRC_DEV dev, uint8_t *level);

esp_err_t mfrc_get_add_iq_en(MFRC_DEV dev, bool *en);

esp_err_t mfrc_set_add_iq_en(MFRC_DEV dev, bool *en);

esp_err_t mfrc_get_fix_iq_en(MFRC_DEV dev, bool *en);

esp_err_t mfrc_set_fix_iq_en(MFRC_DEV dev, bool *en);

esp_err_t mfrc_get_parity_disable(MFRC_DEV dev, bool *en);

esp_err_t mfrc_set_parity_disable(MFRC_DEV dev, bool *en);

esp_err_t mfrc_set_rx_align(MFRC_DEV dev, uint8_t *align);

esp_err_t mfrc_get_rx_align(MFRC_DEV dev, uint8_t *align);

esp_err_t mfrc_set_tx_lastbits(MFRC_DEV dev, uint8_t *bits);

esp_err_t mfrc_get_tx_lastbits(MFRC_DEV dev, uint8_t *bits);

esp_err_t mfrc_get_tmr_prescale_even(MFRC_DEV dev, bool *en);

esp_err_t mfrc_set_tmr_prescale_even(MFRC_DEV dev, bool *en);

esp_err_t mfrc_get_tmr_auto_en(MFRC_DEV dev, bool *en);

esp_err_t mfrc_set_tmr_auto_en(MFRC_DEV dev, bool *en);

esp_err_t mfrc_get_tmr_prescaler(MFRC_DEV dev, uint16_t *ps);

esp_err_t mfrc_set_tmr_prescaler(MFRC_DEV dev, uint16_t *ps);

esp_err_t mfrc_get_fifo_waterlevel(MFRC_DEV dev, uint8_t *wl);

esp_err_t mfrc_set_fifo_waterlevel(MFRC_DEV dev, uint8_t *wl);

esp_err_t mfrc_get_modwidth(MFRC_DEV dev, uint8_t *var);

esp_err_t mfrc_set_modwidth(MFRC_DEV dev, uint8_t *var);

esp_err_t mfrc_get_rx_gain(MFRC_DEV dev, mfrc_rx_gain_t *gain);

esp_err_t mfrc_set_rx_gain(MFRC_DEV dev, mfrc_rx_gain_t *gain);

esp_err_t mfrc_set_mod_gsn(MFRC_DEV dev, uint8_t *var);

esp_err_t mfrc_get_mod_gsn(MFRC_DEV dev, uint8_t *var);

esp_err_t mfrc_set_cw_gsn(MFRC_DEV dev, uint8_t *var);

esp_err_t mfrc_get_cw_gsn(MFRC_DEV dev, uint8_t *var);

esp_err_t mfrc_set_cwg_sp(MFRC_DEV dev, uint8_t *var);

esp_err_t mfrc_get_cwg_sp(MFRC_DEV dev, uint8_t *var);

esp_err_t mfrc_set_mod_gsp(MFRC_DEV dev, uint8_t *var);

esp_err_t mfrc_get_mod_gsp(MFRC_DEV dev, uint8_t *var);

esp_err_t mfrc_set_tmr_gated(MFRC_DEV dev, uint8_t *var);

esp_err_t mfrc_get_tmr_gated(MFRC_DEV dev, uint8_t *var);

esp_err_t mfrc_set_tmr_reload(MFRC_DEV dev, uint16_t *var);

esp_err_t mfrc_get_tmr_reload(MFRC_DEV dev, uint16_t *var);

esp_err_t mfrc_get_tmr_ctr(MFRC_DEV dev, uint16_t *var);

esp_err_t mfrc_set_tstbus_sel(MFRC_DEV dev, uint8_t *var);

esp_err_t mfrc_get_tstbus_sel(MFRC_DEV dev, uint8_t *var);

esp_err_t mfrc_set_prbs15(MFRC_DEV dev, uint8_t *var);

esp_err_t mfrc_get_prbs15(MFRC_DEV dev, uint8_t *var);

esp_err_t mfrc_set_prbs9(MFRC_DEV dev, uint8_t *var);

esp_err_t mfrc_get_prbs9(MFRC_DEV dev, uint8_t *var);

esp_err_t mfrc_set_tstbus_flip(MFRC_DEV dev, uint8_t *var);

esp_err_t mfrc_get_tstbus_flip(MFRC_DEV dev, uint8_t *var);

esp_err_t mfrc_set_testpin_en(MFRC_DEV dev, uint8_t *var);

esp_err_t mfrc_get_testpin_en(MFRC_DEV dev, uint8_t *var);

esp_err_t mfrc_set_rs232line_en(MFRC_DEV dev, uint8_t *var);

esp_err_t mfrc_get_rs232line_en(MFRC_DEV dev, uint8_t *var);

esp_err_t mfrc_set_testpin_value(MFRC_DEV dev, uint8_t *var);

esp_err_t mfrc_get_testpin_value(MFRC_DEV dev, uint8_t *var);

esp_err_t mfrc_set_use_io(MFRC_DEV dev, uint8_t *var);

esp_err_t mfrc_get_use_io(MFRC_DEV dev, uint8_t *var);

esp_err_t mfrc_set_testbus(MFRC_DEV dev, uint8_t *var);

esp_err_t mfrc_get_testbus(MFRC_DEV dev, uint8_t *var);


#ifdef DEBUG_MODE

esp_err_t print_register(MFRC_DEV dev, uint8_t address);

#endif 

#endif /* HEADER_H */
