# APA102 Driver


### Version

Beta - works but needs some improvement

## Brief

This driver is intended to control strands of APA102 addressable leds. 
Initialise the driver by calling `APA102_init` with an empty `LedStrand_t` pointer
and a populated `apa102_init_t` pointer. The interface allows user to get and set the 
animation mode, brightness, colour of the LED strip. The init function assigns memory on 
the heap for the led strand information and the task controls the refresh of led animations.

All colours used in the LED drivers are stored as a 32-bit XRGB with Blue being the 
least significant byte. Any RGB ordering for LED strips is handled by functions in LedEffects.h

The driver is configured to allow up to 8 led strips to be controlled.


## Interface Type

SPI

## How To Use


## Datasheet


## Credits