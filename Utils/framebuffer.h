/****************************************
* \file
* \brief
* \date
* \author
****************************************/

#ifndef FRAMEBUFFER_H
#define FRAMEBUFFER_H

/********* Includes ********************/

#include "esp_types.h"

/********* Definitions *****************/

#define FRAMEBUFFER_MAX_SUPPORTED_WIDTH     256
#define FRAMEBUFFER_MAX_SUPPORTED_HEIGHT    256

/********** Types **********************/

typedef void (pixelWrite)(void *addr, uint32_t value);

typedef enum {
    PIXEL_TYPE_BIT,
    PIXEL_TYPE_8BIT,
    PIXEL_TYPE_16BIT,
    PIXEL_TYPE_24BIT,
    PIXEL_TYPE_32BIT,
    PIXEL_TYPE_INVALID
} fb_pixel_t;

typedef struct {
    uint16_t x;
    uint16_t y;
} coord_t;


typedef struct 
{
    uint16_t frame_width;
    uint16_t frame_height;
    fb_pixel_t pixel_size;
} framebuff_init_t;


typedef struct {
    uint16_t frame_width;   /** in bytes */
    uint16_t frame_height;  /** in bytes */
    fb_pixel_t pixel_size;  /** in bytes */
    void *frame_start;
    uint32_t frame_len;
    pixelWrite *px_write;

} framebuff_handle_t;


typedef framebuff_handle_t * FB_h;

/******** Function Definitions *********/

#endif /* FRAMEBUFFER_H */
