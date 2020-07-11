/****************************************
* \file     BME280_Driver.h
* \brief    Header file for the BME280 bosch temperature sensor
*           Driver.
* \date     July 2020 
* \author   RJAM
****************************************/

#ifndef BME280_DRIVER_H
#define BME280_DRIVER_H

/********* Includes ********************/

/********* Definitions *****************/

#define BMP_I2C_ADDRESS_SDLOW 0x76
#define BMP_I2C_ADDRESS_SDHIGH 0x77

#define BMP_TRANSACTION_READ_BIT 0x01

#define BMP_REG_ADDR_DIGT1_LSB 0x88
#define BMP_REG_ADDR_DIGT1_MSB 0x89
#define BMP_REG_ADDR_DIGT2_LSB 0x8A
#define BMP_REG_ADDR_DIGT2_MSB 0x8B
#define BMP_REG_ADDR_DIGT3_LSB 0x8C
#define BMP_REG_ADDR_DIGT3_MSB 0x8D
#define BMP_REG_ADDR_DIGP1_LSB 0x8E
#define BMP_REG_ADDR_DIGP1_MSB 0x8F
#define BMP_REG_ADDR_DIGP2_LSB 0x90
#define BMP_REG_ADDR_DIGP2_MSB 0x91
#define BMP_REG_ADDR_DIGP3_LSB 0x92
#define BMP_REG_ADDR_DIGP3_MSB 0x93
#define BMP_REG_ADDR_DIGP4_LSB 0x94
#define BMP_REG_ADDR_DIGP4_MSB 0x95
#define BMP_REG_ADDR_DIGP5_LSB 0x96
#define BMP_REG_ADDR_DIGP5_MSB 0x97
#define BMP_REG_ADDR_DIGP6_LSB 0x98
#define BMP_REG_ADDR_DIGP6_MSB 0x99
#define BMP_REG_ADDR_DIGP7_LSB 0x9A
#define BMP_REG_ADDR_DIGP7_MSB 0x9B
#define BMP_REG_ADDR_DIGP8_LSB 0x9C
#define BMP_REG_ADDR_DIGP8_MSB 0x9D
#define BMP_REG_ADDR_DIGP9_LSB 0x9E
#define BMP_REG_ADDR_DIGP9_MSB 0x9F
#define BMP_REG_ADDR_DIGH1 0xA1
#define BMP_REG_ADDR_DIGH2_LSB 0xE1
#define BMP_REG_ADDR_DIGH2_MSB 0xE2
#define BMP_REG_ADDR_DIGH3 0xE3
#define BMP_REG_ADDR_DIGH4_LSB 0xE4
#define BMP_REG_ADDR_DIGH4_MSB 0xE5
#define BMP_REG_ADDR_DIGH5_LSB 0xE5
#define BMP_REG_ADDR_DIGH5_MSB 0xE6
#define BMP_REG_ADDR_DIGH6 0xE7
/**
 *  E4:   -  digH4[11:4]
 *  E5:   -  digH4[3:0]
 * 

/********** Types **********************/

/******** Function Definitions *********/

#endif /* BME280_DRIVER_H */
