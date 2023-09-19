# Futaba VFD Display Driver


### Version

0.0 - yeah this doesn't work at all yet....
0.1 - it works!
1.0 - Alpha release. Could use some tidying, task does nothing but looks good! 

## Brief

Driver for the Futaba VFD 8/16 segment display. 

## Interface Type

SPI 3-wire

## How To Use

The driver is created using the `vfd_init` function, to which the user supplies a pointer to an empty `vfd_handle_t` structure and a populated `vfd_init_t` structure. The init structure expects an initialised SPI bus. The driver also requires a chip select pin and a reset pin. The CS pin can be set to -1 if not required.

The driver handle contains an internal buffer for storing contents to write to the display. These have associated getter and setter methods. The user can also use the `vfd_copy_string` method to copy up to VFD_SEGMENTS characters from a string into the buffer.

Once the buffer is set, the user can call `vfd_write_all_segments` or `vfd_write_segment` to update the display. To clear one/all segments, the user can call `vfd_clear_segment` or `vfd_clear_all_segments`. 

### Custom Characters

The device contains pages of CGram which can be used to store a segment of data. The function `vfd_load_custom_segment` takes a pointer to an array of five bytes. As the display segments are 5 * 7 pixels, the most significant bit is unused. The cgram is loaded column by column. The cgram page can be selected using `vfd_set_current_cgram_page`. To store a segment, set the cgram page, then write the 5 bytes with `vfd_load_custom_segment`. For instance, to create a character map which outlines a segment, the following code should work:

```
    /** create [] shape in the frame **/
    uint8_t zero = 0;
    uint8_t frame[5] = {0b1111111,0b1000001,0b1000001,0b1000001,0b1111111};
    /** select a cgram page **/
    vfd_set_current_cgram_page(device_handle, &zero);
    /** load the segment bytes **/
    vfd_load_custom_segment(device_handle, &frame[0]);
    /** display the custom cgram content in segment 0 **/
    vfd_display_custom_segment(device_handle, &zero);

```

With several (18 I think...) cgram pages to play with, the user can create animations pretty easily. See the vfd_animation_example.c file for an example.

### Changing the number of segments

The number of segments can be changed using the `#define VFD_SEGMENTS` line in the header file. This value can only be 8 or 16. Do not use other values.

## Datasheet

Good luck lol

## Credits

With thanks to https://github.com/positronicsengineer/rpi-spi-vfd and https://github.com/sfxfs/ESP32-VFD-8DM-Arduino/blob/oop/src/VFD.cpp

Whose libraries essentially replaced the data sheet on this one.