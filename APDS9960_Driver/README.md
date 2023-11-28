# APDS9960 Driver

### Version
0.2

## Brief

A driver for theAPDS9960 RGB Colour sensor/Proximity/Gesture sensor. A neat little IC I found on AliExpress.

This device can active any/all of three functions - RGBW sensing, Proximity sensing and Gesture detection. The device operates using i2c, so the user should make sure the i2c bus is configured for use before the driver. The driver also uses an optional interrupt pin, so the user should configure the gpio interrupt service first. 


Currently the task is driven entirely on interrupts but doesn't do much.
Also the gesture detection stuff I wrote is hella janky. The device returns positional data for start and end detection on four directional sensors so yeah, some work to do there...

## Interface Type

I2C Interface

## How To Use


## Datasheet

https://ww1.microchip.com/downloads/en/DeviceDoc/21419D.pdf

## Credits

