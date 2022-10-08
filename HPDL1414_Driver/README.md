# HPDL1414 Driver


### Version

Beta - tested but not working quite right

## Brief

This fun little device is a bubble led encoder display. It contains a fixed character map and 4 led display sections. Each led is selected with the two address gpios and the character to be output is selected from the character map by 7 selection gpios. Sounds fun but is a pain to use in a breadboard and i'm not building a whole pcb for it.

## Interface Type

GPIO

## How To Use

Select a led with current_led setter and select a character for it. 

## Datasheet

https://www.farnell.com/datasheets/76528.pdf

## Credits