/***************************************
* \file     framebuffer.c
* \brief    a generic read/writeable frame buffer for exploration
*           and experimentation of basic rendering.
*           variable pixel size buffers? RGB later...
*
*           Use malloc at first but look into static implementation
*           Let users assign a framebuffer at runtime using a pointer to
*           empty handle like other drivers.
*
*
*           Byte fields seem pretty easy but bit fields not so much...
*           Math time:
*               cartesian x,y position in a bitfield where each byte describes descending LSBF ROWS
* 
*               x,y = fb->frame_start 
*                    + (x % frame->width)    [x co-ordinate (prevent overflow)] 
*                    + ((y / BITS_PER_BYTE) * (fb->frame_width * fb->pixel_size))
*                    this points us at the byte containing the y co-ordinate
*
* \date     Sept 2023
* \author   RJAM
****************************************/

/********* Includes *******************/
#include "string.h"

#include "esp_err_t"
#include "esp_heap_caps.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

#include "framebuffer.h"
#include "Utilities.h"
#include "math.h"

/****** Private Data ******************/

/****** Global Data *******************/

const char *FB_TAG="Framebuffer";

#define PIXEL_BYTE_PTR(fb, index) (void *)(fb->frame_start+(index * fb->pixel_size))

#define PIXEL_SET_BIT_CARTESIAN(fb, x, y) *(uint8_t *)(fb->frame_start + \
                                                   (x % fb->frame_width) + \
                                                   (((uint16_t)(y / BITS_PER_BYTE)) * fb->frame_width)) \
                                                   |= (1 << (y % BITS_PER_BYTE))

#define PIXEL_CLR_BIT_CARTESIAN(fb, x, y) *(uint8_t *)(fb->frame_start + \ 
                                                   (x % fb->frame_width) + \
                                                   (((uint16_t)(y / BITS_PER_BYTE)) * fb->frame_width)) \ 
                                                   &= ~(1 << (y % BITS_PER_BYTE))

/** TODO: Fix these */
#define PIXEL_SET_BIT(fb, index, bit) *(void *)(fb->frame_start+(index * fb->pixel_size)) |= (1 << bit)
#define PIXEL_CLR_BIT(fb, index, bit) *(void *)(fb->frame_start+(index * fb->pixel_size)) &= ~(1 << bit)



/****** Function Prototypes ***********/

/************ ISR *********************/

/******** Private Functions ***********/

/****** Global Functions **************/


esp_err_t framebuffer_init(FB_h fb, framebuff_init_t *init) {

    esp_err_t err = ESP_OK;

    if(init->frame_width > FRAMEBUFFER_MAX_SUPPORTED_WIDTH || 
       init->frame_height > FRAMEBUFFER_MAX_SUPPORTED_HEIGHT 
    ) {
        ESP_LOGE(FB_TAG, "Error, invalid frame dimensions");
        err = ESP_ERR_INVALID_ARG;
    }

    else if(init->pixel_size >= PIXEL_TYPE_INVALID) {
        err = ESP_ERR_INVALID_ARG;
        ESP_LOGE(FB_TAG, "Error, invalid pixel size!");
    }

    if(!err) {
        memset(fb, 0, sizeof(framebuff_handle_t));
        fb->frame_height = init->frame_height;  // 8 bytes
        fb->frame_width = init->frame_width;    // 16 bytes
        fb->pixel_size = init->pixel_size;

        if(fb->pixel_size == PIXEL_TYPE_BIT) {
            /** adjust for bits (don't divide both by 8 tho... ) **/
            fb->frame_height = fb->frame_height / 8;
            fb->frame_len = fb->frame_height * fb->frame_width;
        } else {
            fb->frame_len = (fb->frame_height * fb->frame_width * fb->pixel_size);
        }
        fb->frame_start = heap_caps_malloc(fb->frame_len, MALLOC_CAP_8BIT);

        if(fb->frame_start == NULL) {
            err = ESP_ERR_NO_MEM;
            ESP_LOGE(FB_TAG, "Error, unable to assign frame heap memory!");
        }
    }

    if(!err) {
        memset(fb->frame_start, 0, fb->frame_len);
        switch (fb->pixel_size)
        {
        case PIXEL_TYPE_BIT:
            fb->px_write = 0;
            break;
        
        default:
            break;
        }

    }


}
void framebuff_draw_circle_xsteps(coord_t *centre, uint8_t radius) {
    // circle equation (x-j)^2 + (y-k)^2 = r^2 where 
    // r= radius, j/k are circle centre points and x/y are coordinates
    // for this version we step thru the x axis instead of silly increment
    // this means we get a plottable point for each column
    float c=3.14 * (float)(radius*radius);
    int16_t x=(centre->x-radius);
    coord_t p = {0};
    coord_t plast = {0};
    coord_t np = {0};
    float y;
    for(uint32_t i=0;i==(2*radius);i++) {
        // solve for y at each point
        double _x = ((double)(x+i) - (double)centre->x);
        _x = (_x * _x);
        y = (float)sqrt((((double)(radius*radius)) - _x));
        y += (float)centre->y;
        p.x = x+i;
        p.y = (int16_t)roundf(y);
        np.x = x+i;
        np.y = (int16_t)roundf(y - (2*(y-(float)centre->y)));
        /** anti duplication */
        if(i > 0 && coords_match(&p, &plast)) {
            ;
        } else {
            printf("Iter %u: p->x (%d) p->y (%d)  --  np->x (%d) np->y (%d)\n", i, p.x, p.y, np.x, np.y);
            memcpy(&plast, &p, sizeof(coord_t));
        }
        /** write commands here **/
    }

}
