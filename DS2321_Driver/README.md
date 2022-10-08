# DS2321 Driver


### Version

Alpha, needs tested and adjusted
Need to do alarms

## Brief

The DS2321 IC is a real time clock IC designed to keep track of time, often with a battery backup as it can operate for long times with little current. The device can also set alarms and has a couple of interrupt pins to use with these alarms, as well as a steady 1Hz square wave output. 

## Interface Type

I2C

## How To Use

Currently there are three driver modes, on-demand, where the user calls updateTime and then gets the time details from the getters or the device handle data.

The task jsut waits for the appropriate length of time so not a great solution. 


## Datasheet

https://datasheets.maximintegrated.com/en/ds/DS3231.pdf

## Credits