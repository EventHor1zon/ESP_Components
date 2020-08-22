/****************************************
* \file     LSM_Driver.h
* \brief    Header file for the LSM Driver
* \date     Aug 2020
* \author   RJAM
****************************************/
/*   the dev plan - do the i2c mode first 
*                 - cover the major actions
*                 - expose basic functionality
*                 - timer based read, interrupt based 
*                 - basic fifo modes 
*/

#ifndef LSM_DRIVER_H
#define LSM_DRIVER_H

/********* Includes ********************/

#include <stdint.h>

/********* Definitions *****************/

/** i2c stuff **/
#define LSM_I2C_ADDR 0b1101010
#define LSM_I2C_SA_HIGH (1)
#define LSM_I2C_SA_LOW (0)

#define LSM_FUNC_CFG_REG 0x01
#define LSM_SNES_SYNC_REG 0x04
#define LSM_FIFO_CTRL1_REG 0x06
#define LSM_FIFO_CTRL2_REG 0x07
#define LSM_FIFO_CTRL3_REG 0x08
#define LSM_FIFO_CTRL4_REG 0x09
#define LSM_FIFO_CTRL5_REG 0x0A

#define LSM_ORIENT_CFG_REG 0x0B
#define LSM_INT1_CTRL_REG 0x0D
#define LSM_INT2_CTRL_REG 0x0E
#define LSM_WHOAMI_REG 0x0F
#define LSM_CTRL1_XL_REG 0x10
#define LSM_CTRL2_G_REG 0x11
#define LSM_CTRL3_C_REG 0x12
#define LSM_CTRL4_C_REG 0x13
#define LSM_CTRL5_C_REG 0x14
#define LSM_CTRL6_C_REG 0x15
#define LSM_CTRL7_G_REG 0x16
#define LSM_CTRL8_XL_REG 0x17
#define LSM_CTRL9_XL_REG 0x18
#define LSM_CTRL10_C_REG 0x19
#define LSM_MASTER_CFG_REG 0x1A
#define LSM_WAKEUP_SRC_REG 0x1B
#define LSM_TAP_SRC_REG 0x1C
#define LSM_D6D_SRC_REG 0x1D
#define LSM_STATUS_REG 0x1E
#define LSM_TEMP_LSB_REG 0x20
#define LSM_TEMP_MSB_REG 0x21
#define LSM_GYROX_LSB_REG 0x22
#define LSM_GYROX_MSB_REG 0x23
#define LSM_GYROY_LSB_REG 0x24
#define LSM_GYROY_MSB_REG 0x25
#define LSM_GYROZ_LSB_REG 0x26
#define LSM_GYROZ_MSB_REG 0x27
#define LSM_ACCELX_LSB_REG 0x28
#define LSM_ACCELX_MSB_REG 0x29
#define LSM_ACCELY_LSB_REG 0x2A
#define LSM_ACCELY_MSB_REG 0x2B
#define LSM_ACCELZ_LSB_REG 0x2C
#define LSM_ACCELZ_MSB_REG 0x2D
/** SOME SENSORHUB REGISTERS HERE */
#define LSM_FIFO_STATUS1_REG 0x3A
#define LSM_FIFO_STATUS2_REG 0x3B
#define LSM_FIFO_STATUS3_REG 0x3C
#define LSM_FIFO_STATUS4_REG 0x3D
#define LSM_FIFO_DATA_LSB_REG 0x3E
#define LSM_FIFO_DATA_MSB_REG 0x3F
#define LSM_TIMESTAMP_0_REG 0x40
#define LSM_TIMESTAMP_1_REG 0x41
#define LSM_TIMESTAMP_2_REG 0x43
#define LSM_STEPTIME_LSB_REG 0x49
#define LSM_STEPTIME_MSB_REG 0x4A
#define LSM_STEPCOUNT_LSB_REG 0x4B
#define LSM_STEPCOUNT_MSB_REG 0x4C
/** more sensorhub registers here */
#define LSM_FUNC_SRC_REG 0x53
#define LSM_TAP_CFG_REG 0x58
#define LSM_TAP_THS_6D_REG 0x59
#define LSM_INT_DUR2_REG 0x5A
#define LSM_WKEUP_THS_REG 0x5B
#define LSM_WAKEUP_DUR_REG 0x5C
#define LSM_FREEFALL_REG 0x5D
#define LSM_MD1_CFG_REG 0x5E
#define LSM_MD2_CFG_REG 0x5F

/********** Types **********************/

/** Device operating modes **/
typedef enum LSM_OperatingMode
{
    LSM_OPMODE_ACCEL_ONLY,
    LSM_OPMODE_GYRO_ONLY,
    LSM_OPMODE_GYRO_ACCEL
} LSM_OperatingMode_t;

typedef enum LSM_GyroPowerMode
{
    LSM_GYROPWR_OFF,
    LSM_GYROPWR_LOWPWR,
    LSM_GYROPWR_NORMAL,
    LSM_GYROPWR_HPMODE
} LSM_GyroPwrMode_t;

typedef enum LSM_AccelPowerMode
{
    LSM_ACCELPWR_OFF,
    LSM_ACCELPWR_LOWPWR,
    LSM_ACCELPWR_NORMAL,
    LSM_ACCELPWR_HPMODE
} LSM_GyroPwrMode_t;

typedef enum LSM_FIFOMode
{
    LSM_FIFO_MODE_BYPASS,         /** < Fifo is not used */
    LSM_FIFO_MODE_FIFO,           /** < Fifo stores until full */
    LSM_FIFO_MODE_CONTINUOUS,     /** < continuously updates fifo dumping older data */
    LSM_FIFO_MODE_CONT_TO_FIFO,   /** < both fifo & cont, changes depending on event trigger */
    LSM_FIFO_MODE_BYPASS_TO_FIFO, /** < same as above but with bypass */
} LSM_FIFOMode_t;

/******** Function Definitions *********/

#endif /* LSM_DRIVER_H */
