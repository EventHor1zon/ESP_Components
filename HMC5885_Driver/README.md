# HMC5885 Driver


### Version

Beta - needs a bit of testing and refining

## Brief

HMC5883 Device is a magnetometer, it collects data in 3 axis and can be used with a data ready interrupt pin. A fairly simpled device - this is for the QST version of the device, not the Honeywell version.

## Interface Type

I2C

## How To Use

If the user wants to use the interrupt pin it should be set in the init struct. If using the interrupt pin, the task waits for a data ready signal before sampling the device. Otherwise, the task polls the device once per second. The device can be set into standby or continuous mode. The user can also set the oversampling, scale and data rate. After sampling, the user can retrieve the latest raw or callibrated (units=gauss) data using the getter functions. 

## Datasheet

https://www.best-microcontroller-projects.com/support-files/qmc5883l-datasheet-1.0.pdf

## Credits