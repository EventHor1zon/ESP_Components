# APDS9960 Driver

### Version
0.2

## Brief

A driver for the APDS9960 RGB Colour sensor/Proximity/Gesture sensor. A neat little IC I found on AliExpress.

This device can active any/all of three functions - RGBW sensing, Proximity sensing and Gesture detection. The device operates using i2c, so the user should make sure the i2c bus is configured for use before the driver. The driver also uses an optional interrupt pin, so the user should configure the gpio interrupt service first. 


Currently the task is driven entirely on interrupts but doesn't do much.
Also the gesture detection stuff I wrote is hella janky. The device returns positional data for start and end detection on four directional sensors so yeah, some work to do there...

## Interface Type

I2C Interface

## How To Use

This device can be used in a number of ways. Any or all of the three sesonrs can be active at any time. The device's interrupt pin enables automatic collection of fifo data and setting of saturated bits, etc. However, the user is free to implement a polling approach. 

## TODOs 

- Add events
- Add example sketches
- Finish the task stuff
- Multi device task


## Datasheet

https://docs.broadcom.com/doc/AV02-4191EN

## Credits

