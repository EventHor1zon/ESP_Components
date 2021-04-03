/****************************************
* \file     DS2321_Driver.h
* \brief    Header file
* \date     March 2021
* \author   RJAM
****************************************/

#ifndef DS2321_DRIVER_H
#define DS2321_DRIVER_H

/********* Includes ********************/

#include "esp_err.h"
#include "esp_log.h"
#include "esp_types.h"

#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/********* Definitions *****************/

#define DS2321_I2C_DEVICE_ADDR  0x68

#define DS2321_REGADDR_SECONDS  0
#define DS2321_REGADDR_MINUTES  1
#define DS2321_REGADDR_HOURS    2
#define DS2321_REGADDR_DAY      3
#define DS2321_REGADDR_DATE     4
#define DS2321_REGADDR_MONTH    5
#define DS2321_REGADDR_YEAR     6
#define DS2321_REGADDR_ALARM_A_SECS  7
#define DS2321_REGADDR_ALARM_A_MINS  8
#define DS2321_REGADDR_ALARM_A_HOUR  9
#define DS2321_REGADDR_ALARM_A_DATE  0x0A
#define DS2321_REGADDR_ALARM_B_MINS  0x0B
#define DS2321_REGADDR_ALARM_B_HOUR  0x0C
#define DS2321_REGADDR_ALARM_B_DATE  0x0D
#define DS2321_REGADDR_CONTROL       0x0E
#define DS2321_REGADDR_STATUS        0x0F
#define DS2321_REGADDR_AGE_OFFSET    0x10
#define DS2321_REGADDR_TEMP_MSB      0x11
#define DS2321_REGADDR_TEMP_LSB      0x12

/********** Types **********************/



typedef enum {
    DS2321_OPMODE_ONDEMAND,
    DS2321_OPMODE_SAMPLE_1S,
    DS2321_OPMODE_SAMPLE_5S,
} ds_opmode_t;

typedef struct DS2321_Time
{
    /* data */
    uint8_t seconds;
    uint8_t minutes;
    union {
        uint8_t hours_12;
        uint8_t hours_24;
    } hours;
    bool am;
    uint8_t day;
    uint8_t date;
    uint8_t month;
    uint16_t year;
    char day_str[16];
    char month_str[16];
} ds2321_timedata_t;


typedef struct DS2321_Settings
{
    /* data */
    bool twentyfour_hour;    /**< 0 - am/pm 1 - 24hr **/
} ds2321_settings_t;


typedef struct DS2321_Init
{
    /* data */
    uint8_t  i2c_bus;

} ds2321_init_t;


typedef struct DS2321_Alarm 
{
    uint8_t num;
    uint8_t secs;
    uint8_t mins;
    uint8_t hours;
    uint8_t date;
} ds_alarm_t;


typedef struct DS2321_Driver
{
    /* data */
    uint8_t i2c_bus;
    TaskHandle_t t_handle;

    ds2321_timedata_t time;
    ds2321_settings_t settings;

    ds_opmode_t opmode;
} ds2321_handle_t;


typedef ds2321_handle_t * DS2321_DEV;


/******** Function Definitions *********/

DS2321_DEV ds2321_init(ds2321_init_t *ini);

esp_err_t ds2321_dump_time(DS2321_DEV dev);

esp_err_t ds2321_set_opmode(DS2321_DEV dev, uint8_t *opmode);

esp_err_t ds2321_get_twentyfour(DS2321_DEV Dev, bool *state);

esp_err_t ds2321_set_twentyfour(DS2321_DEV Dev, bool *state);

esp_err_t ds2321_update_time(DS2321_DEV dev);

esp_err_t ds2321_set_seconds(DS2321_DEV dev, uint8_t *secs);

esp_err_t ds2321_set_minutes(DS2321_DEV dev, uint8_t *mins);

esp_err_t ds2321_set_hours(DS2321_DEV dev, uint8_t *hours);

esp_err_t ds2321_set_day(DS2321_DEV dev, uint8_t *day);

esp_err_t ds2321_set_date(DS2321_DEV dev, uint8_t *date);

esp_err_t ds2321_set_month(DS2321_DEV dev, uint8_t *m);

esp_err_t ds2321_set_year(DS2321_DEV dev, uint8_t *y);








#endif /* DS2321_DRIVER_H */
