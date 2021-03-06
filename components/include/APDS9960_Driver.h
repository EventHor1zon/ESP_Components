/****************************************
* \file     APDS9960_Driver.h
* \brief    header file 
* \date     March 2021
* \author   RJAM
****************************************/

#ifndef APDS9960_DRIVER_H
#define APDS9960_DRIVER_H

/********* Includes ********************/

#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef CONFIG_USE_PERIPH_MANAGER

#include "CommandAPI.h"
#define apds_periph_len    20

const peripheral_t apds_periph_mapping[apds_periph_len];

#endif


/********* Definitions *****************/

#define APDS_I2C_ADDRESS                0x39

#define APDS_REGADDR_RAM_START          0x00
#define APDS_REGADDR_RAM_END            0x79
#define APDS_REGADDR_ENABLE             0x80
#define APDS_REGADDR_ADCTIME            0x81
#define APDS_REGADDR_WAITTIME           0x83
#define APDS_REGADDR_ALS_THR_LOW_LSB    0x84
#define APDS_REGADDR_ALS_THR_LOW_MSB    0x85
#define APDS_REGADDR_ALS_THR_HIGH_LSB   0x86
#define APDS_REGADDR_ALS_THR_HIGH_MSB   0x87
#define APDS_REGADDR_PRX_THR_LOW        0x89
#define APDS_REGADDR_PRX_THR_HIGH       0x8B
#define APDS_REGADDR_ISR_PERSIST_FLTR   0x8C
#define APDS_REGADDR_CONFIG_1           0x8D
#define APDS_REGADDR_PRX_PULSE_LEN      0x8E
#define APDS_REGADDR_GAIN_CTRL          0x8F
#define APDS_REGADDR_CONFIG_2           0x90
#define APDS_REGADDR_ID                 0x92
#define APDS_REGADDR_STATUS             0x93
#define APDS_REGADDR_CLRCHAN_DATA_LSB   0x94
#define APDS_REGADDR_CLRCHAN_DATA_MSB   0x95
#define APDS_REGADDR_REDCHAN_DATA_LSB   0x96
#define APDS_REGADDR_REDCHAN_DATA_MSB   0x97
#define APDS_REGADDR_GRNCHAN_DATA_LSB   0x98
#define APDS_REGADDR_GRNCHAN_DATA_MSB   0x99
#define APDS_REGADDR_BLUCHAN_DATA_LSB   0x9A
#define APDS_REGADDR_BLUCHAN_DATA_MSB   0x9B
#define APDS_REGADDR_PROX_DATA          0x9C
#define APDS_REGADDR_PROX_OFFSET_UR     0x9D
#define APDS_REGADDR_PROX_OFFSET_LD     0x9E
#define APDS_REGADDR_CONFIG_3           0x9F
#define APDS_REGADDR_GST_ENTR_THR       0xA0
#define APDS_REGADDR_GST_EXT_THR        0xA1
#define APDS_REGADDR_GST_CONFIG_1       0xA2
#define APDS_REGADDR_GST_CONFIG_2       0xA3
#define APDS_REGADDR_GST_OFFSET_U       0xA4
#define APDS_REGADDR_GST_OFFSET_D       0xA5
#define APDS_REGADDR_GST_OFFSET_L       0xA7
#define APDS_REGADDR_GST_OFFSET_R       0xA9
#define APDS_REGADDR_GST_PULSE_LEN      0xA6
#define APDS_REGADDR_GST_CONFIG_3       0xAA
#define APDS_REGADDR_GST_CONFIG_4       0xAB
#define APDS_REGADDR_GST_FIFO_LVL       0xAE
#define APDS_REGADDR_GST_STATUS         0xAF
#define APDS_REGADDR_FORCE_ISR          0xE4
#define APDS_REGADDR_PRX_ISR_CLR        0xE5
#define APDS_REGADDR_ALS_ISR_CLR        0xE6
#define APDS_REGADDR_NONGST_ISR_CLR     0xE7
#define APDS_REGADDR_GST_FIFO_U         0xFC
#define APDS_REGADDR_GST_FIFO_D         0xFD
#define APDS_REGADDR_GST_FIFO_L         0xFE
#define APDS_REGADDR_GST_FIFO_R         0xFF

/** ENABLE REG BITS **/
#define APDS_REGBIT_PWR_ON      (1 << 0)
#define APDS_REGBIT_ALS_EN      (1 << 1)
#define APDS_REGBIT_PRX_EN      (1 << 2)
#define APDS_REGBIT_WAIT_EN     (1 << 3)
#define APDS_REGBIT_ALS_INT_EN  (1 << 4)
#define APDS_REGBIT_PRX_INT_EN  (1 << 5)
#define APDS_REGBIT_GST_EN      (1 << 6)

/** persistence bits **/

#define APDS_PROX_INT_PERSIST_MASK  0xf0
#define APDS_ALS_INT_PERSIST_MASK   0x0f

/** CONFIG 1 BITS **/
#define APDS_CONFIG_1_MASK          0b01100000;
#define APDS_REGBIT_WLON_EN         (1 << 1)

/** CONTROL REG 1 **/
#define APDS_REG_OFFSET_LDRIVE      6
#define APDS_REG_OFFSET_PGAIN       2

/** CONFIG 2 **/
#define APDS_REGBIT_PRX_SAT_INT_EN   (1 << 7)
#define APDS_REGBIT_CLRPD_SAT_INT_EN (1 << 6)
#define APDS_REG_OFFSET_LED_BOOST       4
#define APDS_REG_RSVD_MASK           0b0001

/** ID **/
#define APDS_ID_VALUE               0xAB

/** STATUS **/
#define APDS_REGBIT_CP_SAT          (1 << 7)
#define APDS_REGBIT_PRX_GST_SAT     (1 << 6)
#define APDS_REGBIT_PRX_INT         (1 << 5)
#define APDS_REGBIT_ALS_INT         (1 << 4)
#define APDS_REGBIT_GST_INT         (1 << 2)
#define APDS_REGBIT_PRX_VALID       (1 << 1)
#define APDS_REGBIT_ALS_VALID       (1 << 0)

/** CONFIG 3 **/
#define APDS_REGBIT_SLP_POST_INT    (1 << 4)
#define APDS_REGBIT_PMASK_U_DIS    (1 << 3)
#define APDS_REGBIT_PMASK_D_DIS    (1 << 3)
#define APDS_REGBIT_PMASK_L_DIS    (1 << 3)
#define APDS_REGBIT_PMASK_R_DIS    (1 << 3)


/** GST CONFIG 1 **/
#define APDS_REGOFFSET_GST_FIFO_OVR     6
#define APDS_REGOFFSET_GST_EXIT_MASK    2

#define APDS_DISABLE_R_DTCR_MASK    0b0001             
#define APDS_DISABLE_L_DTCR_MASK    0b0010
#define APDS_DISABLE_D_DTCR_MASK    0b0100
#define APDS_DISABLE_U_DTCR_MASK    0b1000


/** GST CONFIG 2 **/
#define APDS_REGOFFSET_GST_GAIN         5
#define APDS_REGOFFSET_GST_LED_STR      3

/** GST CFG 4 **/
#define APDS_REGBIT_GST_FIFO_CLR    (1 << 2)
#define APDS_REGBIT_GST_INT_EN      (1 << 1)
#define APDS_REGBIT_GST_MODE        (1 << 0)

/** GST STAT **/
#define APDS_REGBIT_GST_FIFO_OVR    (1 << 1)
#define APDS_REGBIT_GST_DATA_AVAIL  (1 << 0)


/********** Types **********************/


typedef enum {
    APDS_GST_PLSLEN_4US,
    APDS_GST_PLSLEN_8US,
    APDS_GST_PLSLEN_16US,
    APDS_GST_PLSLEN_32US,
    APDS_GST_PLSLEN_MAX,
} gst_pulselen_t;

typedef enum {
    APDS_GST_DIR_UD_LR,
    APDS_GST_DIR_UD,
    APDS_GST_DIR_LR,
    APDS_GST_DIR_LR_UD,
    APDS_GST_DIR_MAX,
} gst_direction_t;

typedef enum {
    APDS_GST_GAIN_1,
    APDS_GST_GAIN_2,
    APDS_GST_GAIN_4,
    APDS_GST_GAIN_8,
    APDS_GST_GAIN_MAX,
}gst_gain_t ; 

typedef enum {
    APDS_PRX_GAIN_1,
    APDS_PRX_GAIN_4,
    APDS_PRX_GAIN_16,
    APDS_PRX_GAIN_64,
    APDS_PRX_GAIN_MAX,
}als_gain_t ; 

typedef enum {
    APDS_GST_LEDDRIVE_100MA,
    APDS_GST_LEDDRIVE_50MA,
    APDS_GST_LEDDRIVE_25MA,
    APDS_GST_LEDDRIVE_12_5MA,
    APDS_GST_LEDDRIVE_MAX,
}gst_led_drive_t ; 

typedef enum {
    APDS_GST_WAIT_T_0MS,
    APDS_GST_WAIT_T_2_8MS,
    APDS_GST_WAIT_T_5_6MS,
    APDS_GST_WAIT_T_8_4MS,
    APDS_GST_WAIT_T_14MS,
    APDS_GST_WAIT_T_22_4MS,
    APDS_GST_WAIT_T_30_8MS,
    APDS_GST_WAIT_T_39_2MS,
    APDS_GST_WAIT_T_MAX,
}gst_wait_t ; 


typedef enum {
    APDS_GST_END_PERSIST_1,
    APDS_GST_END_PERSIST_2,
    APDS_GST_END_PERSIST_4,
    APDS_GST_END_PERSIST_7,
    APDS_GST_END_PERSIST_MAX,
} gst_end_persist_t;

typedef enum {
    APDS_PRX_LEDTIME_4US,
    APDS_PRX_LEDTIME_8US,
    APDS_PRX_LEDTIME_16US,
    APDS_PRX_LEDTIME_32US,
    APDS_PRX_LEDTIME_MAX,
} prx_ledtime_t;


typedef enum {
    APDS_LEDBOOST_100PC,
    APDS_LEDBOOST_150PC,
    APDS_LEDBOOST_200PC,
    APDS_LEDBOOST_300PC,
    APDS_LEDBOOST_MAX,
} apds_ledboost_t;

typedef struct APDS_Init
{
    /* data */

    uint8_t i2c_bus;
    gpio_num_t intr_pin;

} apds_init_t;


typedef struct APDS9960_ALS_Settings
{
    /* data */
    bool asl_en;
    bool wait_long_en;
    bool clr_diode_satr_en;
    bool asl_intr_en;
    uint8_t wait_time;
    uint8_t als_gain;
    uint8_t als_persist;
    uint8_t adc_intg_time;

    uint16_t als_thresh_l;
    uint16_t als_thresh_h;

} als_settings_t;


typedef struct APDS9960_PRX_Settings
{
    /* data */
    bool prx_en;
    bool prox_intr_en;
    bool prox_satr_int_en;
    bool prox_gain_comp_en;

    uint8_t prox_thresh_l;
    uint8_t prox_thresh_h;
    uint8_t prox_gain;
    uint8_t prox_led_en_mask;
    uint8_t led_pulse_n;
    uint8_t prox_perist_cycles;
    prx_ledtime_t ledtime;

} prx_settings_t;

typedef struct APDS_GeneralSettings
{
    /* data */
    bool sleep_after_intr;
    bool pwr_on;

} gen_settings_t;


typedef struct APDS9960_GST_Settings
{
    /* data */
    bool gst_en;
    bool low_pwr_clk;
    bool gst_int_en;

    uint8_t gst_thresh_entr;
    uint8_t gst_thresh_exit;
    uint8_t gst_gain_ctrl;
    uint8_t gst_exit_persist;
    uint8_t gst_exit_mask;
    uint8_t gst_led_drive_str;
    uint8_t gst_wait_time;
    uint8_t gst_satr;
    uint8_t gst_pulse_cnt;
    uint8_t gst_pulse_len;
    uint8_t gst_d_select;

} gst_settings_t;


typedef struct APDS9960_Driver
{
    /* data */

    uint8_t bus;
    uint8_t addr;

    gpio_num_t intr_pin;

    gst_settings_t gst_settings;
    prx_settings_t prx_settings;
    als_settings_t als_settings;
    gen_settings_t gen_settings;
    TaskHandle_t t_handle;

} adps_handle_t;



typedef adps_handle_t * APDS_DEV;


/******** Function Definitions *********/



APDS_DEV apds_init(apds_init_t *init);


/** Set status values **/

esp_err_t apds_get_pwr_on_status(APDS_DEV dev, uint8_t *on);

esp_err_t apds_set_pwr_on_status(APDS_DEV dev, uint8_t *on);

esp_err_t apds_get_proximity_status(APDS_DEV dev, uint8_t *on);

esp_err_t apds_set_proximity_status(APDS_DEV dev, uint8_t *on);

esp_err_t apds_get_als_status(APDS_DEV dev, uint8_t *on);

esp_err_t apds_set_als_status(APDS_DEV dev, uint8_t *on);

esp_err_t apds_get_gesture_status(APDS_DEV dev, uint8_t *on);

esp_err_t apds_set_gesture_status(APDS_DEV dev, uint8_t *on);


/** General settings **/

esp_err_t apds_get_wait_time(APDS_DEV dev, uint8_t *wait);

esp_err_t apds_set_wait_time(APDS_DEV dev, uint8_t *wait);

esp_err_t apds_get_longwait_en(APDS_DEV dev, uint8_t *thr);

esp_err_t apds_set_longwait_en(APDS_DEV dev, uint8_t *thr);

esp_err_t apds_set_led_drive_strength(APDS_DEV dev, gst_led_drive_t *drive);

esp_err_t apds_get_led_drive_strength(APDS_DEV dev, gst_led_drive_t *drive);


/** TODO: Interrupt settings **/

esp_err_t apds_get_sleep_after_intr(APDS_DEV dev, uint8_t *en);

esp_err_t apds_set_sleep_after_intr(APDS_DEV dev, uint8_t *en);


/** ALS **/

esp_err_t apds_get_adc_time(APDS_DEV dev, uint8_t *adct);

esp_err_t apds_set_adc_time(APDS_DEV dev, uint8_t *adct);

esp_err_t apds_get_alsintr_low_thr(APDS_DEV dev, uint16_t *thr);

esp_err_t apds_set_alsintr_low_thr(APDS_DEV dev, uint16_t *thr);

esp_err_t apds_get_alsintr_hi_thr(APDS_DEV dev, uint16_t *thr);

esp_err_t apds_set_alsintr_hi_thr(APDS_DEV dev, uint16_t *thr);

esp_err_t apds_get_als_intr_persistence(APDS_DEV dev, uint8_t *cnt);

esp_err_t apds_set_als_intr_persistence(APDS_DEV dev, uint8_t *cnt);

esp_err_t apds_get_als_gain(APDS_DEV dev, als_gain_t *g);

esp_err_t apds_set_als_gain(APDS_DEV dev, als_gain_t *g);


/** Proximity settings **/

esp_err_t apds_get_prxintr_low_thr(APDS_DEV dev, uint8_t *thr);

esp_err_t apds_set_prxintr_low_thr(APDS_DEV dev, uint8_t *thr);

esp_err_t apds_get_prxintr_high_thr(APDS_DEV dev, uint8_t *thr);

esp_err_t apds_set_prxintr_high_thr(APDS_DEV dev, uint8_t *thr);

esp_err_t apds_set_prx_intr_persistence(APDS_DEV dev, uint8_t *cnt);

esp_err_t apds_get_prx_ledpulse_t(APDS_DEV dev, prx_ledtime_t *t);

esp_err_t apds_set_prx_ledpulse_t(APDS_DEV dev, prx_ledtime_t *t);

esp_err_t apds_get_prx_pulses(APDS_DEV dev, uint8_t *cnt);

esp_err_t apds_set_prx_pulses(APDS_DEV dev, uint8_t *cnt);

esp_err_t apds_get_prx_gain(APDS_DEV dev, gst_gain_t *g);

esp_err_t apds_set_prx_gain(APDS_DEV dev, gst_gain_t *g);


/** Gesture settings **/

esp_err_t apds_get_gst_proximity_ent_thr(APDS_DEV dev, uint8_t *d);

esp_err_t apds_set_gst_proximity_ent_thr(APDS_DEV dev, uint8_t *d);

esp_err_t apds_get_gst_proximity_ext_thr(APDS_DEV dev, uint8_t *d);

esp_err_t apds_set_gst_proximity_ext_thr(APDS_DEV dev, uint8_t *d);


/** Get results **/

esp_err_t apds_get_red_data(APDS_DEV dev, uint16_t *d);

esp_err_t apds_get_blue_data(APDS_DEV dev, uint16_t *d);

esp_err_t apds_get_green_data(APDS_DEV dev, uint16_t *d);

esp_err_t apds_get_clear_data(APDS_DEV dev, uint16_t *d);

esp_err_t apds_get_proximity_data(APDS_DEV dev, uint8_t *d);









#endif /* APDS9960_DRIVER_H */
