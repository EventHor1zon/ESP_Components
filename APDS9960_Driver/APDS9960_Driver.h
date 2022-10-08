/****************************************
* file     APDS9960_Driver.h
* brief    header file 
* date     March 2021
* author   RJAM
****************************************/

#ifndef APDS9960_DRIVER_H
#define APDS9960_DRIVER_H

/********* Includes ********************/

#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#ifdef CONFIG_USE_PERIPH_MANAGER
#include "CommandAPI.h"
#define apds_param_len    20
const parameter_t apds_parameter_map[apds_param_len];
const peripheral_t apds_periph_template;
#endif





/** @defgroup APDS9960_RegisterDefines
 *  @brief Definitions of the IC registers and offsets
 * @{ 
 */

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

#define APDS_FIFO_NUM_ROWS              32
#define APDS_FIFO_BYTES_PER_ROW         4   
#define APDS_FIFO_LEN_BYTES             (APDS_FIFO_NUM_ROWS * APDS_FIFO_BYTES_PER_ROW)

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
#define APDS_CONFIG_1_MASK          0b01100000
#define APDS_REGBITS_WLON_EN        (APDS_CONFIG_1_MASK | (1 << 1))

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
#define APDS_REGBIT_SLP_POST_INT   (1 << 4)
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

/** GST CONFIG 3 **/
#define APDS_DIR_BOTH_PARS_ACTIVE       0
#define APDS_DIR_UPDOWN_ONLY            1
#define APDS_DIR_LEFTRIGHT_ONLY         2

/** GST CFG 4 **/
#define APDS_REGBIT_GST_FIFO_CLR    (1 << 2)
#define APDS_REGBIT_GST_INT_EN      (1 << 1)
#define APDS_REGBIT_GST_MODE        (1 << 0)

/** GST STAT **/
#define APDS_REGBIT_GST_FIFO_OVR    (1 << 1)
#define APDS_REGBIT_GST_DATA_VALID  (1 << 0)

/* @} APDS9960_RegisterDefines */

/** @defgroup APDS9960_TypeDefines
 *  @brief    Definitions of the types used in APDS9960 driver
 *  @{
 */

/** @enum apds_direction_t
 *  @brief APDS gesture detected type 
 */
typedef enum {
    APDS_DIRECTION_UP,
    APDS_DIRECTION_DOWN,
    APDS_DIRECTION_LEFT,
    APDS_DIRECTION_RIGHT,
    APDS_DIRECTION_UNK,
    APDS_DIRECTION_INVALID,
} apds_direction_t;

/** @enum apds_swipe_t
 *  @brief APDS swipe detected type 
 */
typedef enum {
    APDS_SWIPE_UP_TO_DOWN,
    APDS_SWIPE_DOWN_TO_UP,
    APDS_SWIPE_LEFT_TO_RIGHT,
    APDS_SWIPE_RIGHT_TO_LEFT,
    APDS_SWIPE_UNCERTAIN,
} apds_swipe_t;

/** @enum gst_fifo_thresh_t
 *  @brief Gst FIFO threshold packet number
 */
typedef enum {
    APDS_FIFO_THRESH_1,
    APDS_FIFO_THRESH_4,
    APDS_FIFO_THRESH_8,
    APDS_FIFO_THRESH_16,
    APDS_FIFO_THRESH_MAX
} gst_fifo_thresh_t;

/** @enum gst_pulselen_t
 *  @brief Gst led pulse length
 */
typedef enum {
    APDS_GST_PLSLEN_4US,
    APDS_GST_PLSLEN_8US,
    APDS_GST_PLSLEN_16US,
    APDS_GST_PLSLEN_32US,
    APDS_GST_PLSLEN_MAX,
} gst_pulselen_t;

/** @enum gst_direction_t
 *  @brief DEPRECATED, remove
 */
typedef enum {
    APDS_GST_DIR_UD_LR,
    APDS_GST_DIR_UD,
    APDS_GST_DIR_LR,
    APDS_GST_DIR_LR_UD,
    APDS_GST_DIR_MAX,
} gst_direction_t;

/** @enum gst_gain_t
 *  @brief Gst gain setting
 */
typedef enum {
    APDS_GST_GAIN_1,
    APDS_GST_GAIN_2,
    APDS_GST_GAIN_4,
    APDS_GST_GAIN_8,
    APDS_GST_GAIN_MAX,
} gst_gain_t ; 

/** @enum als_gain_t
 *  @brief Proximity gain setting
 */
typedef enum {
    APDS_PRX_GAIN_1,
    APDS_PRX_GAIN_4,
    APDS_PRX_GAIN_16,
    APDS_PRX_GAIN_64,
    APDS_PRX_GAIN_MAX,
} als_gain_t ; 

/** @enum apds_led_drive_t
 *  @brief led drive strength setting
 */
typedef enum {
    APDS_LEDDRIVE_100MA,
    APDS_LEDDRIVE_50MA,
    APDS_LEDDRIVE_25MA,
    APDS_LEDDRIVE_12_5MA,
    APDS_LEDDRIVE_MAX,
} apds_led_drive_t ; 

/** @enum gst_wait_t
 *  @brief gesture wait duration (time between measures)
 */
typedef enum {
    APDS_GST_WAIT_T_0MS,
    APDS_GST_WAIT_T_2_8MS,
    APDS_GST_WAIT_T_5_6MS,
    APDS_GST_WAIT_T_8_4MS,
    APDS_GST_WAIT_T_14MS,
    APDS_GST_WAIT_T_22_4MS,
    APDS_GST_WAIT_T_30_8MS,
    APDS_GST_WAIT_T_39_2MS,
    APDS_GST_WAIT_T_MAX,    /** do not use this value - check purposes only **/
} gst_wait_t ; 

/** @enum gst_end_persist_t
 *  @brief gesture end persistence 
 *          (number of measures outside of threshold 
 *          before GST eengine exits)
 */
typedef enum {
    APDS_GST_END_PERSIST_1,
    APDS_GST_END_PERSIST_2,
    APDS_GST_END_PERSIST_4,
    APDS_GST_END_PERSIST_7,
    APDS_GST_END_PERSIST_MAX,
} gst_end_persist_t;

/** @enum prx_ledtime_t
 *  @brief led on-time per cycle
 */
typedef enum {
    APDS_PRX_LEDTIME_4US,
    APDS_PRX_LEDTIME_8US,
    APDS_PRX_LEDTIME_16US,
    APDS_PRX_LEDTIME_32US,
    APDS_PRX_LEDTIME_MAX,
} prx_ledtime_t;

/** @enum apds_ledboost_t
 *  @brief led power boost setting
 */
typedef enum {
    APDS_LEDBOOST_100PC,
    APDS_LEDBOOST_150PC,
    APDS_LEDBOOST_200PC,
    APDS_LEDBOOST_300PC,
    APDS_LEDBOOST_MAX,
} apds_ledboost_t;

/** @struct apds_init_t
 *  @brief The initialiser info for the APDS driver
 */
typedef struct APDS_Init
{
    uint8_t i2c_bus;        /**!< i2c bus to use (should be initialised) **/
    gpio_num_t intr_pin;    /**!< interrupt pin to use (optional -1 for unused) **/

} apds_init_t;


/** @struct apds_data_t
 *  @brief The latest fifo data
 */
typedef struct APDS9960_Data
{
    /* data */
    uint8_t gst_fifo_u_data[32];
    uint8_t gst_fifo_d_data[32];
    uint8_t gst_fifo_l_data[32];
    uint8_t gst_fifo_r_data[32];
} apds_data_t;

/** @struct als_settings_t
 *  @brief The driver's ALS settings
 */
typedef struct APDS9960_ALS_Settings
{
    /* data */
    bool asl_en;               /**!< ALS enabled **/
    bool clr_diode_satr_en;     /**!< enable clear diode saturation ?? **/
    bool asl_intr_en;           /**!< enable ALS to generate interrupt **/
    uint8_t als_gain;           /**!< ALS measurement gain **/
    uint8_t als_persist;        /**!< ALS exit persistence **/
    uint8_t adc_intg_time;      /**!< the ALS ADC intergration time **/

    uint16_t als_thresh_l;      /**< the ALS threshold lower limit **/
    uint16_t als_thresh_h;      /**< the ALS threshold upper limit **/

} als_settings_t;

/** @struct prx_settings_t
 *  @brief The driver's Proximity settings
 */
typedef struct APDS9960_PRX_Settings
{
    /* data */
    bool prx_en;            /**< Proximity engine enabled **/
    bool prox_intr_en;      /**< proximity interrupt enabled **/
    bool prox_satr_int_en;  /**< interrupt if proximity saturated **/
    bool prox_gain_comp_en; /**< DEPRECATED **/

    uint8_t prox_thresh_l;  /**< proximity threshold lower limit **/
    uint8_t prox_thresh_h;  /**< proximity threshold upper limit **/
    uint8_t prox_gain;          /**< proximity measurement gain **/
    uint8_t prox_led_en_mask;   /**< proximity led enabled mask **/
    uint8_t led_pulse_n;        /**< proximity led pulses per measure **/
    uint8_t prox_perist_cycles; /**< proximity exit persistence **/
    prx_ledtime_t ledtime;      /**< proximity led on-time **/

} prx_settings_t;

/** @struct gen_settings_t
 *  @brief The driver's general operation settings
 */
typedef struct APDS_GeneralSettings
{
    bool sleep_after_intr;      /** sleep after interrupt enabled **/
    bool pwr_on;                /** power on! **/
    uint32_t wait_time_ms;      /** time to idle per cycle in ms(see datasheet for state machine) **/
    uint8_t wait_time;          /**  value set to wait register **/
    bool wait_long_en;          /**  longer waits enabled **/
    bool wait_en;               /** wait enabled **/
} gen_settings_t;

/** @struct gst_settings_t
 *  @brief The driver's gesture engine settings
 */
typedef struct APDS9960_GST_Settings
{
    bool gst_en;        
    bool gmode;
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
    uint8_t gst_dir_select;

    uint8_t gst_fifo_thresh;
    bool gst_ovr;
    bool gst_fifo_rdy;

    uint8_t fifo_pkts_read;
    apds_swipe_t last_swipe;

} gst_settings_t;


/** @struct apds_handle_t
 *  @brief The driver handle structure
 */
typedef struct APDS9960_Driver
{
    uint8_t bus;                    /* I2C bus */
    uint8_t addr;                   /* I2C address */

    bool intr_en;                   /* interrupts enabled */
    gpio_num_t intr_pin;            /* gpio pin for interrupts */

    gst_settings_t gst_settings;    /* gesture settings struct **/
    prx_settings_t prx_settings;    /* proximity settings struct **/
    als_settings_t als_settings;    /* Colour engine settings struct **/
    gen_settings_t gen_settings;    /* general settings struct **/

    apds_data_t data;               /* latest data */

    TaskHandle_t t_handle;          /* the driver task handle */

} adps_handle_t;



typedef adps_handle_t * APDS_DEV;

/** @} APDS9960_TypeDefines **/





/** @defgroup APDS9960_FunctionDefines 
 *  @brief The APDS9960 driver functions
 *  @{
 */

/** 
 * @defgroup APDS9960_SetStatusFunctions
 * @brief Used to set the device status for 
 *       gesture, proximity and ALS engines
 * @{
 ***/

/**
 * \brief - intialises the device
 * \param init - pointer to apds_init struct
 * 
 * \return Handle or Null on fail
 **/
APDS_DEV apds_init(apds_init_t *init);

/**
 * \brief - Get power-on status
 * \param dev - apds handle
 * \param on - pointer to value
 * \return ESP_OK or error
 **/
esp_err_t apds_get_pwr_on_status(APDS_DEV dev, uint8_t *on);

/**
 * \brief - Set power-on status
 * \param dev - apds handle
 * \param on - pointer to value (0 - 1)
 * \return ESP_OK or error
 **/
esp_err_t apds_set_pwr_on_status(APDS_DEV dev, uint8_t *on);

/**
 * \brief - Get proximity sensor status
 * \param dev - apds handle
 * \param on - pointer to value
 * \return ESP_OK or error
 **/
esp_err_t apds_get_proximity_status(APDS_DEV dev, uint8_t *on);

/**
 * \brief - Set proximity sensor on/off status
 * \param dev - apds handle
 * \param on - pointer to value (0-1)
 * \return ESP_OK or error
 **/
esp_err_t apds_set_proximity_status(APDS_DEV dev, uint8_t *on);

/**
 * \brief - Get ALS (colour) sensor status
 * \param dev - apds handle
 * \param on - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_get_als_status(APDS_DEV dev, uint8_t *on);

/**
 * \brief - Set ALS sensor on/off status
 * \param dev - apds handle
 * \param on - pointer to value (0-1)
 * \return ESP_OK or error
 **/
esp_err_t apds_set_als_status(APDS_DEV dev, uint8_t *on);

/**
 * \brief - Get gesture sensor status
 * \param dev - apds handle
 * \param on - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_get_gesture_status(APDS_DEV dev, uint8_t *on);

/**
 * \brief - Set gesture sensor on/off status
 * \param dev - apds handle
 * \param on - pointer to value (0-1)
 * \return ESP_OK or error
 **/
esp_err_t apds_set_gesture_status(APDS_DEV dev, uint8_t *on);

/** @} APDS9960_SetStatusFunctions */

/** General settings
 *  @defgroup APDS9960_GeneralFunctions
 *  @brief Functions to change the general settings of the 
 *          device
 * @{
 ***/


/**
 * \brief - Set wait state enabled (wait once between each loop)
 * \param dev - apds handle
 * \param val - pointer to value 
 * \return ESP_OK or error
 **/
esp_err_t apds_set_wait_enable(APDS_DEV dev, uint8_t *val);


/**
 * \brief - Get wait enabled
 * \param dev - apds handle
 * \param val - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_get_wait_enable(APDS_DEV dev, uint8_t *val);


/**
 * \brief - Get wait time (time to wait in idle state each loop)
 * \param dev - apds handle
 * \param wait - pointer to value storage (see gst_wait_t)
 * \return ESP_OK or error
 **/
esp_err_t apds_get_wait_time(APDS_DEV dev, uint8_t *wait);

/**
 * \brief - Get wait time (time to wait in idle state each loop)
 * \param dev - apds handle
 * \param wait - pointer to value storage (see gst_wait_t)
 * \return ESP_OK or error
 **/
esp_err_t apds_set_wait_time(APDS_DEV dev, uint8_t *wait);

/**
 * \brief - Get long wait enabled
 * \param dev - apds handle
 * \param thr - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_get_longwait_en(APDS_DEV dev, uint8_t *thr);

/**
 * \brief - Set long wait enabled - increases the wait time if enabled
 * \param dev - apds handle
 * \param thr - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_set_longwait_en(APDS_DEV dev, uint8_t *thr);

/**
 * \brief - set led drive strength - one of gst_led_drive_t
 * \param dev - apds handle
 * \param drive - pointer to value
 * \return ESP_OK or error
 **/
esp_err_t apds_set_led_drive(APDS_DEV dev, apds_led_drive_t *drive);


/**
 * \brief - get current proximity led drive strength
 * \param dev - apds handle
 * \param drive - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_get_led_drive(APDS_DEV dev, apds_led_drive_t *drive);

/** @} APDS9960_GeneralFunctions **/

/** Interrupt settings 
 *  @defgroup APDS9960_InterruptFunctions
 *  @brief Functions for configuring the APDS9960 driver interupt
 *         
 *  @{
 ***/

/**
 * \brief - get sleep after interrupt setting
 * \param dev - apds handle
 * \param en - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_get_sleep_after_intr(APDS_DEV dev, uint8_t *en);


/**
 * \brief - set sleep after interrupt setting
 * \param dev - apds handle
 * \param en - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_set_sleep_after_intr(APDS_DEV dev, uint8_t *en);


/**
 * \brief - set gesture interupt enabled
 * \param dev - apds handle
 * \param en - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_set_gst_intr(APDS_DEV dev, uint8_t *en);


/**
 * \brief - get gesture interupt enabled
 * \param dev - apds handle
 * \param en - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_get_gst_intr(APDS_DEV dev, uint8_t *en);


/**
 * \brief - set proximity interupt enabled
 * \param dev - apds handle
 * \param en - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_set_prox_intr(APDS_DEV dev, uint8_t *en);


/**
 * \brief - set proximity threshold interupt enabled
 * \param dev - apds handle
 * \param en - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_get_prox_intr(APDS_DEV dev, uint8_t *en);


/**
 * \brief - set als/colour threshold interupt enabled
 * \param dev - apds handle
 * \param en - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_set_als_intr(APDS_DEV dev, uint8_t *en);


/**
 * \brief - set als/colour threshold interupt enabled state
 * \param dev - apds handle
 * \param en - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_get_als_intr(APDS_DEV dev, uint8_t *en);

/** @} APDS9960_InterruptFunctions **/

/** ALS
 *  @defgroup APDS9960_ALSFunctions
 *  @brief    functions to control the APDS9960 ALS colour-detect  
 *              engine
 *  @{ 
 ***/

/**
 * \brief - get ADC conversion time setting
 * \param dev - apds handle
 * \param adct - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_get_adc_time(APDS_DEV dev, uint8_t *adct);

/**
 * \brief - set ADC conversion time
 *          set to 0xFF at power-on. Saturation value = 1-25 * CYCLES (max 65535)
 *          register value 0 - 256 cycles ~ 712ms 
 *                          = 256 - VALUE / 2.78 ms
 *                         255 - 1 cycle ~ 2.78ms 
 * \param dev - apds handle
 * \param adct - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_set_adc_time(APDS_DEV dev, uint8_t *adct);

/**
 * \brief - get ALS low interrupt threshold
 * \param dev - apds handle
 * \param thr - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_get_alsintr_low_thr(APDS_DEV dev, uint16_t *thr);

/**
 * \brief - set ALS low interrupt threshold
 * \param dev - apds handle
 * \param thr - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_set_alsintr_low_thr(APDS_DEV dev, uint16_t *thr);

/**
 * \brief - get ALS high interrupt threshold
 * \param dev - apds handle
 * \param thr - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_get_alsintr_hi_thr(APDS_DEV dev, uint16_t *thr);

/**
 * \brief - set ALS high interrupt threshold
 * \param dev - apds handle
 * \param thr - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_set_alsintr_hi_thr(APDS_DEV dev, uint16_t *thr);

/**
 * \brief - get ALS interrupt persistence - a divisor for number of
 *          out of thresh measures before interrupt is triggered
 * \param dev - apds handle
 * \param thr - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_get_als_intr_persistence(APDS_DEV dev, uint8_t *cnt);

/**
 * \brief - Set ALS interrupt persistence - a divisor for number of
 *          out of thresh measures before interrupt is triggered. Valid 
 *          range is 0 to 15
 * \param dev - apds handle
 * \param thr - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_set_als_intr_persistence(APDS_DEV dev, uint8_t *cnt);

/**
 * \brief - get ALS Gain value
 * \param dev - apds handle
 * \param g - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_get_als_gain(APDS_DEV dev, als_gain_t *g);

/**
 * \brief - Set ALS Gain value - one of apds_gain_t
 * \param dev - apds handle
 * \param g - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_set_als_gain(APDS_DEV dev, als_gain_t *g);

/** @} APDS9960_ALSFunctions **/


/** Proximity settings 
 *  @defgroup APDS9960_ProximityFunctions 
 *  @brief Functions to control and configure the device's 
 *          proximity engine
 *  @{
 ***/

/**
 * \brief - get proximity low interrupt threshold
 * \param dev - apds handle
 * \param thr - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_get_prxintr_low_thr(APDS_DEV dev, uint8_t *thr);

/**
 * \brief - set proximity low interrupt threshold
 * \param dev - apds handle
 * \param thr - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_set_prxintr_low_thr(APDS_DEV dev, uint8_t *thr);

/**
 * \brief - get proximity high interrupt threshold
 * \param dev - apds handle
 * \param thr - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_get_prxintr_high_thr(APDS_DEV dev, uint8_t *thr);

/**
 * \brief - set proximity high interrupt threshold
 * \param dev - apds handle
 * \param thr - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_set_prxintr_high_thr(APDS_DEV dev, uint8_t *thr);

/**
 * \brief - get proximity interrupt persistence value
 * \param dev - apds handle
 * \param cnt - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_get_prx_intr_persistence(APDS_DEV dev, uint8_t *cnt);

/**
 * \brief - set proximity interrupt persistence value
 * \param dev - apds handle
 * \param cnt - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_set_prx_intr_persistence(APDS_DEV dev, uint8_t *cnt);

/**
 * \brief - get proximity led pulse duration
 * \param dev - apds handle
 * \param t - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_get_prx_ledpulse_t(APDS_DEV dev, prx_ledtime_t *t);

/**
 * \brief - set proximity led pulse duration (one of prx_ledtime_t)
 * \param dev - apds handle
 * \param t - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_set_prx_ledpulse_t(APDS_DEV dev, prx_ledtime_t *t);

/**
 * \brief - get proximity number of led pulses
 * \param dev - apds handle
 * \param cnt - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_get_prx_pulses(APDS_DEV dev, uint8_t *cnt);

/**
 * \brief - set proximity number of led pulses (0-63)
 * \param dev - apds handle
 * \param cnt - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_set_prx_pulses(APDS_DEV dev, uint8_t *cnt);

/**
 * \brief - get proximity sensor gain
 * \param dev - apds handle
 * \param g - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_get_prx_gain(APDS_DEV dev, gst_gain_t *g);

/**
 * \brief - set proximity sensor gain (one of gst_gain_t - yes, i know...)
 * \param dev - apds handle
 * \param g - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_set_prx_gain(APDS_DEV dev, gst_gain_t *g);

/** @} APDS9960_ProximityFunctions **/

/** Gesture settings 
 *  @defgroup APDS9960_GestureFunctions 
 *  @brief      Functions to control the device's gesture sense 
 *              engine
 *  @{
 ***/

/**
 * \brief - get gesture proximity entry threshold value
 * \param dev - apds handle
 * \param d - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_get_gst_proximity_ent_thr(APDS_DEV dev, uint8_t *d);

/**
 * \brief - set gesture proximity entry threshold value
 * \param dev - apds handle
 * \param d - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_set_gst_proximity_ent_thr(APDS_DEV dev, uint8_t *d);

/**
 * \brief - get gesture proximity exit threshold value
 * \param dev - apds handle
 * \param d - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_get_gst_proximity_ext_thr(APDS_DEV dev, uint8_t *d);

/**
 * \brief - set gesture proximity exit threshold value
 * \param dev - apds handle
 * \param d - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_set_gst_proximity_ext_thr(APDS_DEV dev, uint8_t *d);

/**
 * \brief - get gesture direction mode 
 *          APDS_DIR_BOTH_PARS_ACTIVE       0
 *          APDS_DIR_UPDOWN_ONLY            1
 *          APDS_DIR_LEFTRIGHT_ONLY         2
 * \param dev - apds handle
 * \param mode - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_set_gst_direction_mode(APDS_DEV dev, uint8_t *mode);

/**
 * \brief - set gesture direction mode 
 *          APDS_DIR_BOTH_PARS_ACTIVE       0
 *          APDS_DIR_UPDOWN_ONLY            1
 *          APDS_DIR_LEFTRIGHT_ONLY         2
 * \param dev - apds handle
 * \param mode - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_get_gst_direction_mode(APDS_DEV dev, uint8_t *mode);

/**
 * \brief - get gesture exit persistence
 * \param dev - apds handle
 * \param val - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_get_gst_ext_persist(APDS_DEV dev, uint8_t *val);

/**
 * \brief - set gesture exit persistence
 * \param dev - apds handle
 * \param val - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_set_gst_ext_persist(APDS_DEV dev, uint8_t *val);

/**
 * \brief - get gesture pulse length
 * \param dev - apds handle
 * \param val - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_get_gst_pulse_len(APDS_DEV dev, uint8_t *cnt);

/**
 * \brief - set gesture pulse length
 * \param dev - apds handle
 * \param val - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_set_gst_pulse_len(APDS_DEV dev, uint8_t *cnt);

/**
 * \brief - get gesture gmode setting
 * \param dev - apds handle
 * \param val - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_set_gst_gmode(APDS_DEV dev, uint8_t *val);

/**
 * \brief - set gesture gmode setting (remain in gesture mode indefinitely)
 * \param dev - apds handle
 * \param val - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_get_gst_gmode(APDS_DEV dev, uint8_t *val);

/**
 * \brief - get gesture engine gain
 * \param dev - apds handle
 * \param gain - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_set_gst_gain(APDS_DEV dev, uint8_t *gain);

/**
 * \brief - set gesture engine gain
 * \param dev - apds handle
 * \param gain - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_get_gst_gain(APDS_DEV dev, uint8_t *gain);

/**
 * \brief - get gesture led drive strength 
 * \param dev - apds handle
 * \param drive - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_set_gst_led_drive(APDS_DEV dev, apds_led_drive_t *drive);

/**
 * \brief - set gesture led drive strength (one of @apds_led_drive_t)
 * \param dev - apds handle
 * \param drive - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_get_gst_led_drive(APDS_DEV dev, apds_led_drive_t *drive);

/**
 * \brief - get gesture wait time (time between gst measurements)
 * \param dev - apds handle
 * \param wait - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_set_gst_wait(APDS_DEV dev, uint8_t *wait);

/**
 * \brief - set gesture wait time (time between gst measurements)
 * \param dev - apds handle
 * \param wait - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_get_gst_wait(APDS_DEV dev, uint8_t *wait);

/** @} APDS9960_GestureFunctions **/


/** Get results
 *  @defgroup APDS9960_DataFunctions
 *  @brief    Functions to read the latest data from the device
 *  @{
 *  **/

/**
 * \brief - Get the latest red data from the ALS engine
 * \param dev - apds handle
 * \param d - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_get_red_data(APDS_DEV dev, uint16_t *d);

/**
 * \brief - Get the latest blue data from the ALS engine
 * \param dev - apds handle
 * \param d - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_get_blue_data(APDS_DEV dev, uint16_t *d);

/**
 * \brief - Get the latest green data from the ALS engine
 * \param dev - apds handle
 * \param d - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_get_green_data(APDS_DEV dev, uint16_t *d);

/**
 * \brief - Get the latest clear (brightness) data from the ALS engine
 * \param dev - apds handle
 * \param d - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_get_clear_data(APDS_DEV dev, uint16_t *d);

/**
 * \brief - Get the latest proximity data from the PRX engine
 * \param dev - apds handle
 * \param d - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_get_proximity_data(APDS_DEV dev, uint8_t *d);

/** @} APDS9960_DataFunctions **/

/** @defgroup APDS9960_FifoFunctions
 *  @brief Functions to configure and control the device's built-in
 *          fifo buffer
 * @{
 */

/**
 * \brief - Read all available fifo data - stored in the device struct
 * \param dev - apds handle
 * \param d - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_read_fifo_full(APDS_DEV dev);

/**
 * \brief - Get num available fifo data packets
 * \param dev - apds handle
 * \param level - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_get_gst_fifo_lvl(APDS_DEV dev, uint8_t *level);

/**
 * \brief - Read errr... can't remember what the plan was here. TODO: remind 
 * \param dev - apds handle
 * \param index - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_read_fifo_data_set(APDS_DEV dev, uint8_t *index);

/**
 * \brief - Get fifo valid status
 * \param dev - apds handle
 * \param valid - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_get_fifo_valid(APDS_DEV dev, bool *valid);

/**
 * \brief - Get fifo overflow status
 * \param dev - apds handle
 * \param valid - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_get_fifo_overflow(APDS_DEV dev, bool *ov);

/**
 * \brief - Get fifo threshold value 
 * \param dev - apds handle
 * \param valid - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_get_fifo_thresh(APDS_DEV dev, uint8_t *thr);

/**
 * \brief - Set fifo threshold value (one of gst_fifo_thresh_t)
 * \param dev - apds handle
 * \param valid - pointer to value storage
 * \return ESP_OK or error
 **/
esp_err_t apds_set_fifo_thresh(APDS_DEV dev, uint8_t *thr);

/** @} APDS9960_FifoFunctions **/

/** @} APDS9960_FunctionDefines **/

#endif /* APDS9960_DRIVER_H */
