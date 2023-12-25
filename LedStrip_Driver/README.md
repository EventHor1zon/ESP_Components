# LEDStrip Driver


### Version
0.1

## Brief

Combined led strip interface with generic-ish write/animations

## Interface Type

RMT/SPI

## How To Use

This driver works slightly differently to the other drivers. The driver task is initialised with `ledstrip_init` and then each individual strip is added with `ledstrip_add_strip`. The add_strip method requires a populated `ledstrip_init_t`. The led_type member should be one of the types stored in the `led_type_e` enum. The only supported led types are WS2812b and APA102 currently. The channel member should be the channel of the interface used to drive the LEDs. The WS2812b uses the RMT driver (which has 8 available channels) and SPI driven LEDs like the APA102 require a running SPI bus.

There is only one running task in this driver which handles the animation and frame update timers for each separate strip. The maximum number of strips supported is 8 at present, but can be adjusted by changing the `#define LEDSTRIP_CONFIG_MAX_STRIPS` define.

### LedEffects

The LedStrip driver is used to store the translation functions and led type data, making the interface simple for the user. Effects on the LEDs are handled using the LedEffects module which is built along side the driver. Currently, the LedEffects driver only has a couple of animations. `ledstrip_set_mode` lets the user select an animation mode. Modes available:

0. Leds Off
1. Single Colour
2. Nightrider effect
3. Slow fade effect

These modes can be adjusted using the colour, brightness and speed attributes of the ledstrip driver. 

### Colour Order and Pixel types

Internally, all colours are treated as a 32-bit integer, with hex pairs representing `uint32_t colour 0xXXRRGGBB`, where `XX` is unused, and R, G, B values are as expected. Assumes 8-bit colour values. The actual order of the colours is recorded in the `ledtype_t` struct, which contains all the neccessary info for controlling an addressable led. Hopefully.

### Supported Led controller types

Currently supports:
- APA102
- WS2812b


## TODOs

- Implement adjustable animation timer
- More effects
- Fix APA102 bug re. rendering (split start frame from pixel frames?)
- Support more addressable led types

## Datasheet


## Credits