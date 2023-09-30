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
    uint16_t frame_width;   /** in pixels */
    uint16_t frame_height;  /** in pixels */
    fb_pixel_t pixel_size;
    void *frame_start;
    uint32_t frame_len;
} framebuff_handle_t;


typedef framebuff_handle_t * FB_h;


/******** Function Definitions *********/


esp_err_t framebuffer_init(FB_h fb, framebuff_init_t *init);

esp_err_t framebuffer_draw_horizontal_xsteps(FB_h fb, coord_t *start, coord_t *end);

esp_err_t framebuffer_draw_vertical_ysteps(FB_h fb, coord_t *start, coord_t *end);

esp_err_t framebuffer_draw_line_xsteps(FB_h fb, coord_t *start, coord_t *end);

void framebuff_draw_circle_xsteps(FB_h fb, coord_t *centre, uint8_t radius);

#endif /* FRAMEBUFFER_H */
