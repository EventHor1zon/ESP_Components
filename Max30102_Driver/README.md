# Max30102 Driver


### Version
0.0 - barely started. Both IC's I got were broken.
0.1 - basic functionality looks good.
0.2 - fifo measurement, still a little janky

## Brief

Driver for the Max30102 Heartbeat/SpO2 sensor. Not too complicated a device.

## Interface Type

I2C

## How To Use

Create an initialisation structure, supplying gpio pin for interrupt and a running i2c bus. Call the init function to start the driver. Once the driver has started, a good easy-star is to reset the device, then put the device into the desired mode, set the almost-full value to something sensible (4-5 seems a good shout without dropping too many packets) and then enable the almost full interrupt. The task is interrupt based and reads fifo on an almost-full interrupt.

## TODOs

- Events
- Smarter handle naming
- Support multiple devices
- Support cbuffer
- Simplify handle
- Data Processing

## Datasheet

https://www.analog.com/media/en/technical-documentation/data-sheets/max30102.pdf

https://www.analog.com/media/en/technical-documentation/user-guides/max3010x-ev-kits-recommended-configurations-and-operating-profiles.pdf

## Credits

