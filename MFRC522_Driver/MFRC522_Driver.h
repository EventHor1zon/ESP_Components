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
#define MRFC_REGADDR_CWF_SP             0x28
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

#define MFRC_TIMER_INPUT_FREQ_HZ        13560000

/********** Types **********************/


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


typedef struct MFRC_Init
{
    /* data */
    uint8_t comms_bus;
    mfrc_comms_mode_t comms_mode;
    gpio_num_t rst_pin;
    gpio_num_t irq_pin;


} mfrc_init_t;






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


}





















typedef struct MFRC522_Driver
{
    /* data */
    uint8_t comms_bus;
    mfrc_comms_mode_t comms_mode;
    gpio_num_t rst_pin;
    gpio_num_t irq_pin;

    bool rst_en;
    bool irq_en;

    TaskHandle_t task;

    uint8_t raw_data[64];


} MFRC522_Driver_t;


typedef MFRC522_Driver_t * MFRC_DEV;

/******** Function Declarations *********/

MFRC_DEV mfrc_init(mfrc_init_t *init);

void mfrc_deinit(MFRC_DEV dev);



esp_err_t mfrc_set_interrupt_mask(MFRC_DEV dev, uint8_t *intr);

esp_err_t mfrc_get_interrupt_mask(MFRC_DEV dev, uint8_t *intr);

esp_err_t mfrc_enable_interrupt_pin(MFRC_DEV dev);

esp_err_t mfrc_disable_interrupt_pin(MFRC_DEV dev);

esp_err_t mfrc_soft_power_down(MFRC_DEV dev);

esp_err_t mfrc_





#endif /* HEADER_H */
