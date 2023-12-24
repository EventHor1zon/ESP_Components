# LSM6DS3 Driver


### Version

Beta

## Brief

LSM6DS3 is an accelerometer/gyroscope with lots of built-in functionality. There's a fifo and step detection and variable packet sizes and timestamping and fall detection and changeable axis and individually enabled axis and selectable output rates and scale ranges and interrupt masking and orientation settings and tap/double tap detection and so on and so forth.


## Interface Type

Only I2C currently supported

## How To Use

Driver sets up the handle and resets the device before setting it to the mode specified in the init structure. If the interrupt pin is used then the driver task waits for an interrupt before checking the interrupt cause and automatically taking actions to address it. Went far too far with this as it kick started the CBuffer driver and packet logging in order to try to save the fifo data into a buffer which was accessible to the uart or stream driver as a way to read data from the device and out to a computer as quickly as possible.

So yeah this driver needs some streamlining... But it works quite well!

## Datasheet


## Credits