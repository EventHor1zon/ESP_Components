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


#define MFRC_REGADDR_COMMAND            0x01
#define MFRC_REGADDR_COMM_INTR_EN       0x02
#define MFRC_REGADDR_DEV_INTR_EN        0x03
#define MFRC_REGADDR_COMM_IRQ           0x04
#define MFRC_REGADDR_DIV_IRQ            0x05
#define MFRC_REGADDR_ERRORS             0x06
#define MFRC_REGADDR_STATUS_1           0x07
#define MFRC_REGADDR_STATUS_2           0x08
#define MFRC_REGADDR_FIFO_DATA          0x09
#define MFRC_REGADDR_FIFO_LEVEL         0x0A
#define MFRC_REGADDR_FIFO_WATERLEVEL    0x0B
#define MFRC_REGADDR_CONTROL            0x0C
#define MFRC_REGADDR_BIT_FRAMING        0x0D
#define MFRC_REGADDR_COLLISION_DETECT   0x0E
#define MFRC_REGADDR_MODE               0x11
#define MFRC_REGADDR_TX_MODE            0x12
#define MFRC_REGADDR_RX_MODE            0x13
#define MFRC_REGADDR_TX_CONTROL         0x14
#define MFRC_REGADDR_TX_ASK             0x15
#define MFRC_REGADDR_TX_SELECT          0x16
#define MFRC_REGADDR_RX_SELECT          0x17
#define MFRC_REGADDR_RX_THRESH          0x18
#define MFRC_REGADDR_DEMODULATOR        0x19
#define MFRC_REGADDR_MIFARE_TX          0x1C
#define MFRC_REGADDR_MIFARE_RX          0x1D
#define MFRC_REGADDR_SERIAL_SPEED       0x1F

#define MFRC_REGADDR_CRC_RESULT_MSB     0x21
#define MFRC_REGADDR_CRC_RESULT_LSB     0x22
#define MFRC_REGADDR_MOD_WIDTH          0x24
#define MFRC_REGADDR_RFC_GAIN           0x26
#define MFRC_REGADDR_GS_N               0x27
#define MFRC_REGADDR_CWF_SP             0x28
#define MFRC_REGADDR_MODG_SP            0x29
#define MFRC_REGADDR_TMR_MODE           0x2A
#define MFRC_REGADDR_TMR_PRESCALE       0x2B
#define MFRC_REGADDR_TCNTR_RELOAD_MSB   0x2C
#define MFRC_REGADDR_TCNTR_RELOAD_LSB   0x2D
#define MFRC_REGADDR_TCNTR_MSB          0x2E
#define MFRC_REGADDR_TCNTR_LSB          0x2F

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

#define MFRC_SPI_ADDR_SHIFT             1
#define MFRC_SPI_READ_BIT              (1 << 7)
#define MFRC_FIFO_SIZE_BITS             8 * 64
#define MFRC_FIFO_SIZE_BYTES            64

#define MFRC_TIMER_INPUT_FREQ_HZ        13560000
#define MIFARE_BLOCK_LEN_BYTES          16

/********** Types **********************/

typedef enum {
    BLK_ACC_COND_NEVER,
    BLK_ACC_COND_A,
    BLK_ACC_COND_B,
    BLK_ACC_COND_A_OR_B,
} blk_access_cond_t;


typedef enum {
    MFRC_PWRMODE_ON,
    MFRC_PWRMODE_STARTING,
    MFRC_PWRMODE_POWER_DOWN,
} mfrc_pwrmode_t;

typedef enum {
    MFRC_RECVR_PWR_ON,
    MFRC_RECVR_PWR_ANLG_OFF,
} mfrc_rcvr_pwr_t;

typedef enum {
    MFRC_TX_PWR_ON,
    MFRC_TX_PWR_OFF,
} mfrc_tx_pwr_t;


typedef enum {
    MFRC_CRC_PRESET_0,
    MFRC_CRC_PRESET_6363,
    MFRC_CRC_PRESET_A671,
    MFRC_CRC_PRESET_FFFF,
    MFRC_CRC_PRESET_INVALID,
} mfrc_crc_preset_t;


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


typedef struct sector_trailer
{
    uint8_t keysec_a[6];
    uint8_t access_cond[4];
    uint8_t keysec_b[6];
} sector_trailer_t;


typedef struct block_contents {
    uint8_t block_0[MIFARE_BLOCK_LEN_BYTES];
    uint8_t block_1[MIFARE_BLOCK_LEN_BYTES];
    uint8_t block_2[MIFARE_BLOCK_LEN_BYTES];
    sector_trailer_t trailer;
} block_content_t;

typedef struct block_description {
    blk_access_cond_t access_conditions[4];
    block_content_t data;
} block_descr_t;

typedef struct MFRC_Init
{
    /* data */
    uint8_t comms_bus;
    mfrc_comms_mode_t comms_mode;
    gpio_num_t rst_pin;
    gpio_num_t irq_pin;

    gpio_num_t mosi;
    gpio_num_t miso;
    gpio_num_t sel;
    gpio_num_t clk;

    gpio_num_t sda;
    gpio_num_t scl;


} mfrc_init_t;


typedef enum PICC_Command {
    // The commands used by the PCD to manage communication with several PICCs (ISO 14443-3, Type A, section 6.4)
    PICC_CMD_REQA			= 0x26,		// REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
    PICC_CMD_WUPA			= 0x52,		// Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
    PICC_CMD_CT				= 0x88,		// Cascade Tag. Not really a command, but used during anti collision.
    PICC_CMD_SEL_CL1		= 0x93,		// Anti collision/Select, Cascade Level 1
    PICC_CMD_SEL_CL2		= 0x95,		// Anti collision/Select, Cascade Level 2
    PICC_CMD_SEL_CL3		= 0x97,		// Anti collision/Select, Cascade Level 3
    PICC_CMD_HLTA			= 0x50,		// HaLT command, Type A. Instructs an ACTIVE PICC to go to state HALT.
    PICC_CMD_RATS           = 0xE0,     // Request command for Answer To Reset.
    // The commands used for MIFARE Classic (from http://www.mouser.com/ds/2/302/MF1S503x-89574.pdf, Section 9)
    // Use PCD_MFAuthent to authenticate access to a sector, then use these commands to read/write/modify the blocks on the sector.
    // The read/write commands can also be used for MIFARE Ultralight.
    PICC_CMD_MF_AUTH_KEY_A	= 0x60,		// Perform authentication with Key A
    PICC_CMD_MF_AUTH_KEY_B	= 0x61,		// Perform authentication with Key B
    PICC_CMD_MF_READ		= 0x30,		// Reads one 16 byte block from the authenticated sector of the PICC. Also used for MIFARE Ultralight.
    PICC_CMD_MF_WRITE		= 0xA0,		// Writes one 16 byte block to the authenticated sector of the PICC. Called "COMPATIBILITY WRITE" for MIFARE Ultralight.
    PICC_CMD_MF_DECREMENT	= 0xC0,		// Decrements the contents of a block and stores the result in the internal data register.
    PICC_CMD_MF_INCREMENT	= 0xC1,		// Increments the contents of a block and stores the result in the internal data register.
    PICC_CMD_MF_RESTORE		= 0xC2,		// Reads the contents of a block into the internal data register.
    PICC_CMD_MF_TRANSFER	= 0xB0,		// Writes the contents of the internal data register to a block.
    // The commands used for MIFARE Ultralight (from http://www.nxp.com/documents/data_sheet/MF0ICU1.pdf, Section 8.6)
    // The PICC_CMD_MF_READ and PICC_CMD_MF_WRITE can also be used for MIFARE Ultralight.
    PICC_CMD_UL_WRITE		= 0xA2		// Writes one 4 byte page to the PICC.
} picc_cmd_t;


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
        } bits;
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
        } bits;
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
        } bits;
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
        } bits;
    } com_irq_reg;
    
    /** 0x05 **/
    union {
        uint8_t regval;
        struct {
            uint8_t rsvd_0      :   2;
            uint8_t crc_irq     :   1;
            uint8_t rsvd_1      :   1;
            uint8_t mfin_irq    :   1;
            uint8_t rsvd_2      :   2;
            uint8_t irq_set_2   :   1;
        } bits;
    } div_irq_reg;

    /** 0x06 **/
    union {
        uint8_t regval;
        struct {
            uint8_t protocol_err    :   1;
            uint8_t parity_err      :   1;
            uint8_t crc_err         :   1;
            uint8_t coll_err        :   1;
            uint8_t buffer_ovr_err  :   1;
            uint8_t rsvd_0          :   1;
            uint8_t tempr_err       :   1;
            uint8_t wrt_err         :   1;
        } bits;
    } error_reg;

    /** 0x07 **/
    union {
        uint8_t regval;
        struct {
            uint8_t fifo_lo_alert   :   1;
            uint8_t fifo_hi_alert   :   1;
            uint8_t rsvd_0          :   1;
            uint8_t tmr_running     :   1;
            uint8_t irq_alert       :   1;
            uint8_t crc_ready       :   1;
            uint8_t crc_ok          :   1;
            uint8_t rsvd_1          :   1;
        } bits;
    } status_1_reg;

    /** 0x08 **/
    union {
        uint8_t regval;
        struct {
            uint8_t modem_state     :   2;
            uint8_t mifare_crypt1_on :  2;
            uint8_t rsvd_0          :   2;
            uint8_t i2c_force_hs    :   1;
            uint8_t temp_sens_clr   :   1;
        } bits;
    } status_2_reg;

    /** 0x09 **/
    union {
        uint8_t regval;
        struct {
            uint8_t fifo_data : 8;
        } bits;
    } fifo_data_reg;

    /** 0x0A **/
    union {
        uint8_t regval;
        struct {
            uint8_t fifo_level      :   7;
            uint8_t flush_buffer    :   1;
        } bits;
    } fifo_lvl_reg;

    /** 0x0B **/ 
    union {
        uint8_t regval;
        struct {
            uint8_t fifo_watermark  :   6;
            uint8_t rsvd_0          :   2;
        } bits;
    } fifo_watermark_reg;

    /** 0x0C **/
    union {
        uint8_t regval;
        struct {
            uint8_t rx_last_bits    :   3;
            uint8_t rsvd_0          :   3;
            uint8_t tmr_start_now   :   1;
            uint8_t tmr_stop_now    :   1;
        } bits;
    } control_reg;

    /** 0x0D **/
    union {
        uint8_t regval;
        struct {
            uint8_t tx_last_bits    :   3;
            uint8_t rsvd_0          :   1;
            uint8_t rx_align        :   3;
            uint8_t start_send      :   1;
        } bits;
    } bitframe_reg;

    /** 0x0E **/
    union {
        uint8_t regval;
        struct {
            uint8_t collision_pos       :   5;
            uint8_t coll_pos_invalid    :   1;
            uint8_t rsvd_0              :   1;
            uint8_t clr_rx_on_collision :   1;
        } bits;
    } collision_reg;

    /** 0x0F **/
    uint8_t reserved_1;

    /** 0x10 **/
    uint8_t reserved_2;

    /** 0x11 **/
    union {
        uint8_t regval;
        struct {
            uint8_t crc_preset         :   2;
            uint8_t rsvd_0             :   1;
            uint8_t pol_mf_in          :   1;
            uint8_t rsvd_1             :   1;
            uint8_t tx_wait_rf         :   1;
            uint8_t rsvd_2             :   1;
            uint8_t crc_msb_first      :   1;
        } bits;
    } mode_reg;

    /** 0x12 **/
    union {
        uint8_t regval;
        struct {
            uint8_t rsvd_0             :   3;
            uint8_t invert_modulation  :   1;
            uint8_t tx_speed           :   3;
            uint8_t tx_crc_en          :   1;
        } bits;
    } tx_mode_reg;

    /** 0x13 **/
    union {
        uint8_t regval;
        struct {
            uint8_t rsvd_0              :   2;
            uint8_t rx_multiple         :   1;
            uint8_t rx_no_err           :   1;
            uint8_t rx_speed            :   3;
            uint8_t rx_crc_en           :   1;
        } bits;
    } rx_mode_reg;

    /** 0x14 **/
    union {
        uint8_t regval;
        struct {
            uint8_t tx1_rf_en  :  1;
            uint8_t tx2_rf_en  :  1;
            uint8_t rsvd_0     :  1;
            uint8_t tx2_carrier:  1;
            uint8_t tx1_inv_rf_off : 1;
            uint8_t tx2_inv_rf_off : 1;
            uint8_t tx1_inv_rf_on  : 1;
            uint8_t tx2_inv_rf_on  : 1;
        } bits;
    } txcontrol_reg;

    /** 0x15 **/
    union {
        uint8_t regval;
        struct {
            uint8_t rsvd_0 : 6;
            uint8_t force_100_ask : 1;
            uint8_t rsvd_1 : 1;
        } bits;
    } tx_ask_reg;

    /** 0x16 **/
    union {
        uint8_t regval;
        struct {
            uint8_t mf_out_sel  :  4;
            uint8_t driver_sel  :  2;
            uint8_t rsvd_0      :  2;
        } bits;
    } tx_select_reg;

    /** 0x17 **/
    union {
        uint8_t regval;
        struct {
            uint8_t rx_wait     :   6;
            uint8_t uart_sel    :   2;
        } bits;
    } rx_select_reg;

    /** 0x18 **/
    union {
        uint8_t regval;
        struct {
            uint8_t collision_lvl  :  3;
            uint8_t rsvd_0         :  1;
            uint8_t min_lvl        :  4;
        } bits;
    } rx_threshold_reg;

    /** 0x19 **/

    union {
        uint8_t regval;
        struct {
            uint8_t tau_sync    :  2;
            uint8_t tau_recv    :  2;
            uint8_t tmr_prescale_even : 1;
            uint8_t fix_iq      :  1;
            uint8_t add_iq      :  2;
        } bits;
    } demod_reg;

    /** 0x1A **/
    uint8_t reserved_3;

    /** 0x1B **/
    uint8_t reserved_4;

    /** 0x1C **/
    union {
        uint8_t regval;
        struct {
            uint8_t tx_wait  :  1;
            uint8_t rsvd_0   :  7;
        } bits;
    } mifare_tx_reg;

    /** 0x1D **/
    union {
        uint8_t regval;
        struct {
            uint8_t rsvd_0 : 4;
            uint8_t parity_disable : 1;
            uint8_t rsvd_1 : 3;
        } bits;
    } mifare_rx_reg;

    /** 0x1E **/
    uint8_t reserved_5;

    /** 0x1F **/
    union {
        uint8_t regval;
        struct {
            uint8_t br_t1 : 5;
            uint8_t br_t0 : 3;
        } bits;
    } serial_speed_reg;

    /** 0x20 **/
    uint8_t reserved_7;

    /** 0x21 **/ 
    uint8_t crc_result_msb;

    /** 0x22 **/
    uint8_t crc_result_lsb;

    /** 0x23 **/
    uint8_t reserved_8;

    /** 0x24 **/
    uint8_t mod_width;

    /** 0x25 **/
    uint8_t reserved_9;

    /** 0x26 **/
    union {
        uint8_t regval;
        struct {
            uint8_t rsvd_0  :  4;
            uint8_t rx_gain :  3;
            uint8_t rsvd_1  :  1;
        } bits;
    } rf_cfg_reg;

    /** 0x27 **/
    union {
        uint8_t regval;
        struct {
            uint8_t mod_gsn  :  4;
            uint8_t cw_gsn   :  4;
        } bits;
    } gsn_reg;

    /** 0x28 **/
    union {
        uint8_t regval;
        struct {
            uint8_t cw_gsp  :  6;
            uint8_t rsvd_0  :  2;
        } bits;
    } cw_gsp_reg;

    /** 0x29 **/
    union {
        uint8_t regval;
        struct {
            uint8_t mod_gsp  : 6;
            uint8_t rsvd  :  2;
        } bits;
    } mod_gs_reg;

    /** 0x2A **/
    union {
        uint8_t regval;
        struct {
            uint8_t tmr_prescale_msb : 4;
            uint8_t tmr_auto_restart : 1;
            uint8_t tmr_gated :  2;
            uint8_t tmr_auto  :  1;
        } bits;
    } tmr_mode_reg;

    /** 0x2B **/
    uint8_t tmr_prescaler_lsb;

    /** 0x2C **/
    uint8_t tmr_reload_msb;

    /** 0x2D **/
    uint8_t tmr_reload_lsb;

    /** 0x2E **/
    uint8_t tmr_cnt_msb;

    /** 0x2F **/
    uint8_t tmr_cnt_lsb;

    /** 0x30 **/
    uint8_t reserved_10;

    /** 0x31 **/
    union {
        uint8_t regval;
        struct {
            uint8_t test_bus_bit_sel : 3;
            uint8_t rsvd_0           : 5;
        } bits;
    } test_sel_1_reg;

    /** 0x32 **/
    union {
        uint8_t regval;
        struct {
            uint8_t test_bus_sel  : 5;
            uint8_t prb_s15       : 1;
            uint8_t prb_s9        : 1;
            uint8_t test_bus_flip : 1;
        } bits;
    } test_sel_2_reg;

    /** 0x33 **/
    union {
        uint8_t regval;
        struct {
            uint8_t rsvd_0 : 1;
            uint8_t test_pin_en : 6;
            uint8_t rs232_line_end : 1;
        } bits;
    } testpin_en_reg;

    /** 0x34 **/
    union {
        uint8_t regval;
        struct {
            uint8_t rsvd_0 : 1;
            uint8_t test_pin_val : 6;
            uint8_t use_io : 1;
        } bits;
    } testpin_val_reg;

    /** 0x35 **/
    uint8_t testbus;

    /** 0x36 **/
    union {
        uint8_t regval;
        struct {
            uint8_t self_test : 4;
            uint8_t rft  :  2;
            uint8_t amp_rcv : 1;
            uint8_t rsvd_0 : 1;
        } bits;
    } auto_test_reg;

    /** 0x37 **/
    union {
        uint8_t regval;
        struct {
            uint8_t version : 4;
            uint8_t chip_type : 4;
        } bits;
    } version_reg;

    /** 0x38 **/
    union {
        uint8_t regval;
        struct {
            uint8_t analog_sel_aux_2 : 4;
            uint8_t analog_sel_aux_1 : 4;
        } bits;
    } analog_test_reg;

    /** 0x39 **/
    union {
        uint8_t regval;
        struct {
            uint8_t test_dac_1 : 6;
            uint8_t rsvd_0 : 2;
        } bits;
    } test_dac1_reg;

    /** 0x3A **/
    union {
        uint8_t regval;
        struct {
            uint8_t test_dac_2 : 6;
            uint8_t rsvd_0 : 2;
        } bits;
    } test_dac2_reg;

    /** 0x3B **/
    union {
        uint8_t regval;
        struct {
            uint8_t adc_q : 4;
            uint8_t adc_i : 4;
        } bits;
    } adc_test_reg;

    /** 0x3C **/
    uint8_t reserved_11;

    /** 0x3D **/
    uint8_t reserved_12;

    /** 0x3F **/
    uint8_t reserved_13;

} mfrc_register_t;



typedef struct MFRC522_Driver
{
    /* data */
    uint8_t comms_bus;
    mfrc_comms_mode_t comms_mode;
    gpio_num_t rst_pin;
    gpio_num_t irq_pin;

    gpio_num_t data_pin;
    gpio_num_t clock_pin;
    gpio_num_t select_pin;
    gpio_num_t data_in_pin;

    bool rst_en;
    bool irq_en;

    mfrc_register_t registers;

    TaskHandle_t task;
    spi_device_handle_t spi_handle;

    uint8_t raw_data[64];


} MFRC522_Driver_t;


typedef MFRC522_Driver_t * MFRC_DEV;

/******** Function Declarations *********/

MFRC_DEV mfrc_init(mfrc_init_t *init);

void mfrc_deinit(MFRC_DEV dev);



// esp_err_t mfrc_set_interrupt_mask(MFRC_DEV dev, uint8_t *intr);

// esp_err_t mfrc_get_interrupt_mask(MFRC_DEV dev, uint8_t *intr);

// esp_err_t mfrc_enable_interrupt_pin(MFRC_DEV dev);

// esp_err_t mfrc_disable_interrupt_pin(MFRC_DEV dev);

// esp_err_t mfrc_soft_power_down(MFRC_DEV dev);






#endif /* HEADER_H */
