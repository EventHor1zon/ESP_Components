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
- use hardware timer for step pulse low, keep state in struct


## Datasheet

https://www.allegromicro.com/~/media/Files/Datasheets/A4988-Datasheet.ashx


## Credits

