# A4988 Driver

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

## Datasheet

https://www.allegromicro.com/~/media/Files/Datasheets/A4988-Datasheet.ashx