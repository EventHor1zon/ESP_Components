# A4988 Driver

### Version
Beta - all features working but no support for multi device

## Brief

A driver for the A4988 Stepper motor driver IC. Motor steps can be queued to be executed every period. The step size and direction can be set by the user.

Initialisation requires pins for the operation of the device.
The required pins are 

- step pin
- reset pin
- direction pin

Optional pins are:

- ms1/2/3 pins (step size programming)
- sleep pin (must be provided to enable sleep mode)
- enable pin (must be provided to toggle enabled mode)

The step queue can also be cleared and there is a programmable millisecond precision timer which the user can use to set the wait time between steps.


## Interface Type

I2C Interface

## How To Use

Initialise the driver using the init function. 

### TODOs

- refactor for multiple devices
- replace task notify with queue
- use hardware timer for step pulse low
- keep pin state in struct

Re. hardware timers - the earlier silicon revisions of the ESP32 only have 4 hardware timers available for the user. This unfortunately limits the number of driver instances available. It might be better to give the user the option of using software timers for pin deassert - these might not be able to offer the same short pulse as using the hardware timer but you could have more instances. The other option is to ditch the hardware timer idea and look at some kind of pulse implementation instead, maybe leveraging another ESP32 component. Will first test the hardware timers and see if they can deal with the short duration pulse first though. 

- Add turn by degrees and optional Steps-per-turn in the init struct 

## Datasheet

https://www.allegromicro.com/~/media/Files/Datasheets/A4988-Datasheet.ashx


## Credits

