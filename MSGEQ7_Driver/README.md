# MSGEQ7 Driver


### Version

alpha - largely working but needs operational code not test code

## Brief

This device is a 7-channel audio spectrum analyser. The device works with a reset pin which needs to be periodically called to refresh the adc, a channel hop pin and an analog data out pin. The frequency bands are listed in the datasheet.


## Interface Type

ADC/GPIO

## How To Use

The driver is created with the three required pins - the data input pin should be an analog capable pin. The driver task periodically checks all channels for data and resets every N samples. The user can invoke the sample action themselves, and retrieve the latest data using the channel getters.

## Datasheet

https://www.sparkfun.com/datasheets/Components/General/MSGEQ7.pdf

## Credits