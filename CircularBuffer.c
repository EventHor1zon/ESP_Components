/***************************************
* \file     CircularBuffer.c
* \brief    A simple library for creating & using circular buffers
*           made mainly to entertain myself during this FUCKING LOCKDOWN
*           Includes functionality to alert tasks when buffer nearing full
*           to facilitate streaming data.
*           v1.0  - basic complete
*           v2.0  - add new functionality: packets
*                   Allow user to specify a pattern which will be saved. The pattern 
*                   should consist of a string of python-style struct descriptors
*                   The packet descriptor should be adhered to in order to save data correctly.
*                   The packet also adds a sequence of separator bytes between packets so that the
*                   Data can be more easily unpacked externally.
*                   This will allow high-rate streaming of data inside a CBuffer to a python script
*                   And also nicely formatted dumping of CBuffer data to console, SD card or file.
*       
* \date     Feb 2021
* \author   RJAM
****************************************/

/********* Includes *******************/

/**
 *  HELLO I.T, HAVE YOU TRIED TURNING IT OFF AND ON AGAIN?
 * 
 **/    


#include <string.h>
#include <stdlib.h>

#include "esp_err.h"
#include "esp_types.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "device_config.h"

#include "driver/i2c.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "driver/uart.h"


#include "CircularBuffer.h"
#include "Utilities.h"
#include "genericCommsDriver.h"
const char *CBUFF_TAG = "CBUFF";

#include <time.h>
#include <sys/time.h>
#include "CommandAPI.h"

#ifdef CONFIG_USE_EVENTS
#include "esp_event.h"
#endif

/****** Function Prototypes ***********/


static uint16_t get_packet_size_from_pattern(char *pattern);

static bool buffer_is_full(CBuff handle);

static bool buffer_is_empty(CBuff handle);

static bool buffer_will_overrun(CBuff handle, uint32_t incomming_sz);

static uint32_t buffer_free_bytes(CBuff handle);

static uint32_t buffer_bytes_until_end(CBuff handle, bool read);

static bool buffer_will_overwrite(CBuff handle, uint32_t incomming_sz);

static void cbuffer_write_ll(CBuff handle, void *data, uint32_t length);

/************ ISR *********************/

/****** Private Data ******************/

/****** Private Functions *************/

static uint16_t get_packet_size_from_pattern(char *pattern) {

    uint16_t size = 0;
    char pattern_buffer[CBUFFER_MAX_PACKET_VALUES] = {0};
    ESP_LOGI("CBUFF", "Getting data packet length");

    for(uint8_t i=0; i < strlen(pattern); i++) {
        switch(pattern[i]) {
            case CBUFF_VARTYPE_BOOL:
            case CBUFF_VARTYPE_UINT8:
            case CBUFF_VARTYPE_INT8:
                size ++;
                break;
            case CBUFF_VARTYPE_UINT16:
            case CBUFF_VARTYPE_INT16:
                size += 2;
                break;
            case CBUFF_VARTYPE_UINT32:
            case CBUFF_VARTYPE_INT32:
            case CBUFF_VARTYPE_FLOAT:
                size += 4;
                break;
            case CBUFF_VARTYPE_DOUBLE:
                size += 8;
                break;
            default:
                return 0;   /** an invalid character found **/
        }
    }
    return size;
}


/** this write will overwrite unread data **/
static bool buffer_will_overwrite(CBuff handle, uint32_t incomming_sz) {
    uint32_t freebytes = buffer_free_bytes(handle);
    return incomming_sz > freebytes;
}

/** returns true if buffer is full **/
static bool buffer_is_full(CBuff handle) {
    return (handle->write_ptr - handle->read_ptr == 1);
}

/** returns true if buffer is empty **/
static bool buffer_is_empty(CBuff handle) {
    return (handle->read_ptr == handle->write_ptr);
}

/** buffer write will run past the buffer end - pointers need to be moved **/
static bool buffer_will_overrun(CBuff handle, uint32_t incomming_sz) {
    return ((handle->write_ptr + incomming_sz) > handle->buffer_end);
}

/** bytes available between write and read pointer **/
static uint32_t buffer_free_bytes(CBuff handle) {

    uint32_t free = (handle->write_ptr > handle->read_ptr) ?  // if write pointer is ahead of read...
                    (handle->buffer_end - handle->write_ptr) + ((handle->read_ptr - 1) - handle->buffer_start) : // go round the buffer
                    ((handle->read_ptr -1) - handle->write_ptr);    // else go up to read-1
    return free;
}

/** bytes available between read and write pointers **/
uint32_t buffer_unread_bytes(CBuff handle) {
    uint32_t avail = (handle->read_ptr > handle->write_ptr ? 
                      ((handle->buffer_end - handle->read_ptr) + ((handle->write_ptr)-handle->buffer_start)) :
                      (handle->write_ptr - handle->read_ptr));
    return avail;
}

/** bytes from pointer to end of buffer **/
static uint32_t buffer_bytes_until_end(CBuff handle, bool read) {
    uint32_t bytes = 0;
    if(read) {
        bytes = handle->buffer_end - handle->read_ptr;
    } 
    else {
        bytes = handle->buffer_end - handle->write_ptr;
    }
    return bytes;
}

/** write data of length - will follow circular write & increment the write handle **/
static void cbuffer_write_ll(CBuff handle, void *data, uint32_t length) {

    if(buffer_will_overrun(handle, length)) {
        uint32_t first_write = buffer_bytes_until_end(handle, 0);
        memcpy(handle->write_ptr, data, first_write);
        handle->write_ptr = handle->buffer_start;
        memcpy(handle->write_ptr, (data + first_write), (length - first_write));
        assert(length > first_write);
        handle->write_ptr += (length - first_write);
    }
    else {
        memcpy(handle->write_ptr, data, length);
        handle->write_ptr += length;
    }
    // update the number of data butes available
    handle->data_len = buffer_unread_bytes(handle);
    return;
}

/** read data of length **/
static void cbuffer_read_ll(CBuff handle, void *data, uint32_t len) {
    uint32_t bytes_until_end = buffer_bytes_until_end(handle, 1);
    if(len > bytes_until_end) {
        memcpy(data, handle->read_ptr, bytes_until_end); 
        handle->read_ptr = handle->buffer_start;
        memcpy((data + bytes_until_end), handle->read_ptr, (len - bytes_until_end));
        handle->read_ptr += (len - bytes_until_end);
    }
    else {
        memcpy(data, handle->read_ptr, len);
        handle->read_ptr += len;
    }
    // update the number of data bytes available 
    handle->data_len = buffer_unread_bytes(handle);
}


#ifdef CONFIG_USE_EVENTS

/** Emit events for user handling
 *  to prevent numerous event raising, clear the event once raised
 *  User will have to re-implement the event when reading the buffer
 *  is finished
 **/
static esp_err_t emit_nearfull_event(CBuff handle) {

    esp_err_t err = ESP_OK;
    err = esp_event_post_to(handle->event_settings.loop, PM_EVENT_BASE, CBUFF_EVENTCODE_NEARFULL, handle, sizeof(CBuff), pdMS_TO_TICKS(CONFIG_EVENTPOST_WAIT_MS));
    printf("Posting nearful event %u\n\n", err);
    handle->event_settings.event_mask &= ~(CBUFF_EVENT_NEARFULL);
    return err;
}


static esp_err_t emit_full_event(CBuff handle) {

    esp_err_t err = ESP_OK;
    err = esp_event_post_to(handle->event_settings.loop, PM_EVENT_BASE, CBUFF_EVENTCODE_FULL, handle, sizeof(CBuff), pdMS_TO_TICKS(CONFIG_EVENTPOST_WAIT_MS));
    handle->event_settings.event_mask &= ~(CBUFF_EVENT_FULL);
    return err;
}


static esp_err_t emit_overwrite_event(CBuff handle) {

    esp_err_t err = ESP_OK;
    err = esp_event_post_to(handle->event_settings.loop, PM_EVENT_BASE, CBUFF_EVENTCODE_OVERWRITE, handle, sizeof(CBuff), pdMS_TO_TICKS(CONFIG_EVENTPOST_WAIT_MS));
    handle->event_settings.event_mask &= ~(CBUFF_EVENT_OVERWRITE);
    return err;
}


static esp_err_t emit_empty_event(CBuff handle) {

    esp_err_t err = esp_event_post_to(handle->event_settings.loop, PM_EVENT_BASE, CBUFF_EVENTCODE_EMPTY, handle, sizeof(CBuff), pdMS_TO_TICKS(CONFIG_EVENTPOST_WAIT_MS));
    handle->event_settings.event_mask &= ~(CBUFF_EVENT_EMPTY);
    return err;
}

#endif




/****** Global Data *******************/

/****** Global Functions *************/

CBuff cbuffer_create(CBuffer_init_t *init) {
    
    SemaphoreHandle_t sem = NULL;
    CBuff handle = NULL;
    void *buffer = NULL;
    esp_err_t err = ESP_OK;
    uint8_t malloc_caps = (init->malloc_caps) ? (init->malloc_caps) : MALLOC_CAP_DEFAULT;

    if (init->size <= CBUFFER_MAX_BUFFER_SIZE) {
        handle = (CBuff )heap_caps_calloc(1, sizeof(CBuffer_Handle_t), MALLOC_CAP_DEFAULT);
        buffer = heap_caps_malloc(init->size, MALLOC_CAP_DEFAULT);

        if(handle == NULL || buffer == NULL) {
            err = ESP_ERR_NO_MEM;
        }
        else {
            memset(buffer, 0, init->size);
        }
    }
    else {
        err = ESP_ERR_INVALID_ARG;
    }

    if(err == ESP_OK) {
        SemaphoreHandle_t sem = xSemaphoreCreateMutex();
        if(sem == NULL) {
            ESP_LOGE(CBUFF_TAG, "Error creating semaphore");
            err = ESP_ERR_NO_MEM;
        }
        else{
            handle->buffer_start = buffer;
            handle->buffer_end = (buffer + init->size); 
            handle->write_ptr = 0;
            handle->read_ptr = 0;
            handle->buffer_len = init->size;
            handle->data_len = 0;
            handle->allow_overwrite = init->allow_ovr;
            handle->is_claimed = false;
            handle->sem = sem;
            ESP_LOGI("CBUFF", "Created new CBuffer Size: %u at 0x%08x", handle->buffer_len, (uint32_t)buffer);
        }
    }
    
    if(err != ESP_OK) {
        if(buffer != NULL) {
            heap_caps_free(buffer);
        }
        if (handle != NULL) {
            heap_caps_free(handle);
        }
        ESP_LOGE(CBUFF_TAG, "Error creating CBuffer!");
    }

    return handle;
}


#ifdef CONFIG_USE_EVENTS

esp_err_t cbuffer_config_events(CBuff handle, bool use_events, esp_event_loop_handle_t loop, uint8_t event_flags, uint32_t nearfull_val) {

    esp_err_t err = ESP_OK;

    if(use_events) {
        if(loop == NULL) {
            err = ESP_ERR_INVALID_ARG;
        }
        else if((event_flags & CBUFF_EVENT_NEARFULL) && nearfull_val > handle->buffer_len) {
            err = ESP_ERR_INVALID_ARG;
        }       
        else {
            handle->use_events = true;
            handle->event_settings.loop = loop;
            handle->event_settings.event_mask = event_flags;
            handle->event_settings.nearfull_val = (event_flags & CBUFF_EVENT_NEARFULL) ? nearfull_val : 0;
        }
    }
    else {

        memset(&handle->event_settings, 0, sizeof(cbuffer_events_t));
        handle->use_events = false;
    }

    return err;
}


esp_err_t cbuffer_set_event_mask(CBuff handle, uint8_t event_mask) {
    handle->event_settings.event_mask = event_mask;

    if(event_mask) {
        handle->use_events = true;
    }
    else {
        handle->use_events = false;
    } 
    return ESP_OK;
}

#endif


esp_err_t cbuffer_config_packet(CBuff handle, bool use_sep, uint8_t *sep_bytes, uint8_t sep_length, bool use_timestamp, timestamp_precision_t ts_precision) {

    esp_err_t err = ESP_OK;

    if(use_sep && sep_length > CBUFF_MAX_SEP_BYTES) {
        err = ESP_ERR_INVALID_ARG;
    }

    else if(use_timestamp && ts_precision > 1) {
        err = ESP_ERR_INVALID_ARG;
        ESP_LOGE(CBUFF_TAG, "Error invalid ts precision");
    }

    if(!err) {
        if(use_sep) {
            handle->pkt_settings.use_sep_bytes = true;
            handle->pkt_settings.sep_length = sep_length;
            handle->pkt_settings.packet_sz_bytes += sep_length;
            memset(handle->pkt_settings.sep_bytes, 0, (sizeof(uint8_t ) * CBUFF_MAX_SEP_BYTES));
            memcpy(handle->pkt_settings.sep_bytes, sep_bytes, sep_length);
            printf("Loaded %u seperation bytes ", sep_length);
            for(uint8_t i=0; i < sep_length; i++) {
                printf("%02x ", handle->pkt_settings.sep_bytes[i]);
            }
            printf("\n");
        }

        if(use_timestamp) {
            uint8_t ts_size = ts_precision > 0 ? 8 : 4;
            handle->pkt_settings.use_timestamp = true;
            handle->pkt_settings.use_timestamp = use_timestamp;
            handle->pkt_settings.ts_precision = use_timestamp ? ts_precision : 0;
            handle->pkt_settings.timestamp_length = ts_size;
            handle->pkt_settings.packet_sz_bytes += ts_size;
            printf("Set time-stamp precision=%u, length=%u\n", handle->pkt_settings.ts_precision, handle->pkt_settings.timestamp_length);
        }

        err = cbuffer_reset_buffer(handle);
    }
    


    return err;
}


esp_err_t cbuffer_get_full_packet_length(CBuff handle, uint16_t *len) {

    esp_err_t err = ESP_OK;
    uint16_t length = 0;

    if(!handle->use_packets) {
        err = ESP_ERR_INVALID_STATE;
    }
    else {
        if(handle->pkt_settings.use_timestamp) {
            if(handle->pkt_settings.ts_precision == CBUFF_TS_PRECISION_MICROSECOND) {
                length += 8;
            }
            else if(handle->pkt_settings.ts_precision == CBUFF_TS_PRECISION_SECOND) {
                length += 4;
            }
        }
        /** copy the pattern from the buffer **/
        length += handle->pkt_settings.packet_data_sz_bytes;

        if(handle->pkt_settings.use_sep_bytes) {
            length += handle->pkt_settings.sep_length;
        }
    }

    *len = length;

    return err;
}


esp_err_t cbuffer_get_pattern(CBuff handle, char *pattern) {

    esp_err_t err = ESP_OK;

    if(!handle->use_packets) {
        err = ESP_ERR_INVALID_STATE;
    }
    
    if(!err) {
        char buffer[64];
        memset(buffer, 0, (sizeof(uint8_t) * 64));
        uint8_t index = 0;

        if(handle->pkt_settings.use_timestamp) {
            if(handle->pkt_settings.ts_precision == CBUFF_TS_PRECISION_MICROSECOND) {
                memset(&buffer[index], 0x71, sizeof(char)); /** 0x71 - 'q' **/
                index ++;
            }
            else if(handle->pkt_settings.ts_precision == CBUFF_TS_PRECISION_SECOND) {
                memset(&buffer[index], 0x6C, sizeof(char)); /** 0x6c = 'l' **/
                index ++;
            }
        }
        /** copy the pattern from the buffer **/
        memcpy(&buffer[index], handle->pkt_settings.pattern, sizeof(uint8_t) * handle->pkt_settings.packet_elements);
        index += handle->pkt_settings.packet_elements;

        if(handle->pkt_settings.use_sep_bytes) {
            memcpy(&buffer[index], handle->pkt_settings.sep_bytes, sizeof(uint8_t) * handle->pkt_settings.sep_length);
            index += handle->pkt_settings.sep_length;
        }
    }
    
    return err;
}


esp_err_t cbuffer_load_packet(CBuff handle, uint8_t values_per_packet, char *pattern, char **names_array, uint8_t *ids) {
    esp_err_t err = ESP_OK;
    uint16_t name_len = 0;
    uint16_t pkt_data_len = 0;
    ESP_LOGI(CBUFF_TAG, "Loading new pattern : %s", pattern);

    if(values_per_packet > CBUFFER_MAX_PACKET_VALUES) {
        err = ESP_ERR_INVALID_ARG;
    }
    else if (strlen(pattern) > values_per_packet) {
        err = ESP_ERR_INVALID_ARG;
    }
    else {
        for(uint8_t counter = 0; counter < values_per_packet; counter++) {
            if(names_array[counter] == NULL || strlen(names_array[counter]) > CBUFFER_MAX_NAME_LENGTH) {
                err = ESP_ERR_INVALID_ARG;
            }
            else if (ids[counter] == 0) {
                err = ESP_ERR_INVALID_ARG;
            }
        }
    }

    pkt_data_len = get_packet_size_from_pattern(pattern); 
    if( pkt_data_len == 0) {
        err = ESP_ERR_INVALID_ARG;
    }

    if(!err) {
        handle->pkt_settings.packet_data_sz_bytes = pkt_data_len;
        cbuffer_reset_buffer(handle);
    }

    if(!err) {
        /** clear the packet memory **/
        memset(&handle->packets[0], 0, (sizeof(cbuffer_pmember_t) * CBUFFER_MAX_PACKET_VALUES));
        memset(handle->pkt_settings.pattern, 0, (sizeof(char) * CBUFFER_MAX_PACKET_VALUES+1));
        /** copy the pattern into handle **/
        memcpy(handle->pkt_settings.pattern, pattern, sizeof(char) * values_per_packet);
        /** copy the rest of the pattern details **/
        ESP_LOGI(CBUFF_TAG, "Cleaned the buffers");
        for(uint8_t counter = 0; counter < values_per_packet; counter++) {
            name_len = strlen(names_array[counter]);
            handle->packets[counter].id = ids[counter];
            memcpy(&handle->packets[counter].name[0], names_array[counter], name_len); 
            handle->packets[counter].name_length = name_len;
            handle->packets[counter].p = *(pattern + counter);
            
            printf("Added to packet: %s %c %i\n", handle->packets[counter].name, handle->packets[counter].p, handle->packets[counter].id);
        }
        handle->pkt_settings.packet_sz_bytes  = pkt_data_len;
    }

    if(!err) {
        handle->use_packets = true;
        handle->pkt_settings.packet_elements  = values_per_packet;
        handle->pkt_settings.packet_index  = 0;
        ESP_LOGI(CBUFF_TAG, "Succesfully loaded new packet!");
        uint16_t l = 0;
        cbuffer_get_full_packet_length(handle, &l);
    }



    return err;
}


esp_err_t cbuffer_clear_packet(CBuff handle) {
    memset(handle->packets, 0, (sizeof(cbuffer_pmember_t) * CBUFFER_MAX_PACKET_VALUES));
    return ESP_OK;
}


esp_err_t cbuffer_get_available_packets(CBuff handle, uint32_t *avail) {
    esp_err_t err = ESP_OK;

    if(!handle->use_packets) {
        err = ESP_ERR_INVALID_STATE;
        *avail = 0;
    }
    else {
        *avail = handle->pkts_available;
    }
    return err;
}


esp_err_t cbuffer_write(CBuff handle, void *data, uint32_t wrt_len) {

    esp_err_t ret = ESP_OK;
    bool ovr = false;

    if(wrt_len > handle->buffer_len) {
        ret = ESP_ERR_INVALID_ARG;
    }
    else if(data == NULL || handle == NULL) {
        ret = ESP_ERR_NO_MEM;
    }
    else if(xSemaphoreTake(handle->sem, pdMS_TO_TICKS(CBUFFER_SEM_WAIT_MS)) != pdTRUE) {
        ESP_LOGE(CBUFF_TAG, "Error unlocking cbuffer semaphore!");
        ret = ESP_ERR_TIMEOUT;
    }

    if(!ret) {
        ovr = buffer_will_overwrite(handle, wrt_len);
#ifdef CONFIG_USE_EVENTS
        /** emit the overwrite event **/
        if(handle->use_events && \
            handle->event_settings.event_mask & CBUFF_EVENT_OVERWRITE && \
            handle->allow_overwrite) {
            emit_overwrite_event(handle);
        }
#endif

        if (ovr && !handle->allow_overwrite) {
            /** fill remaining space **/
            uint32_t bytes_left = buffer_free_bytes(handle);
            cbuffer_write_ll(handle, data, bytes_left);

#ifdef CONFIG_USE_EVENTS
            /** emit the 'full' event **/
            if(handle->use_events && \
               handle->event_settings.event_mask & CBUFF_EVENT_FULL) {
                emit_full_event(handle);
            }
#endif
        }
        else {
            /** write the packet, don't care about overwriting old data **/
            cbuffer_write_ll(handle, data, wrt_len);

#ifdef CONFIG_USE_EVENTS
            if(handle->use_events) {
                uint32_t unread_bytes = buffer_unread_bytes(handle);
                if((handle->event_settings.event_mask & CBUFF_EVENT_NEARFULL) && \
                   (unread_bytes >= handle->event_settings.nearfull_val)) {
                       emit_nearfull_event(handle);
                }
                if(unread_bytes == handle->buffer_len && \
                   (handle->event_settings.event_mask & CBUFF_EVENT_FULL)) {
                        emit_full_event(handle);
                }
            }
#endif
        }

        xSemaphoreGive(handle->sem);
    }

    return ret;
}


esp_err_t cbuffer_write_packet(CBuff handle, void *data, uint32_t length) {
    /* function to write an entire packet at once - do it faster this time! */
    esp_err_t err = ESP_OK;
    uint8_t sep[CBUFF_MAX_SEP_BYTES] = {0};
    uint32_t free = buffer_free_bytes(handle);
    bool overwrite = false;

    if (length != handle->pkt_settings.packet_data_sz_bytes) {
        ESP_LOGE(CBUFF_TAG, "Error, invalid length! Expected %u got %u", handle->pkt_settings.packet_data_sz_bytes, length);
        err = ESP_ERR_INVALID_ARG;
    }
    else if(length > free) {
        /** no space for another packet **/
        if(!handle->allow_overwrite) {
            err = ESP_ERR_NO_MEM;
#ifdef CONFIG_USE_EVENTS
        /** emit full event **/
            if(handle->use_events && \
            handle->event_settings.event_mask & CBUFF_EVENT_FULL
            ) {
                emit_full_event(handle);
            }
#endif
        }
#ifdef CONFIG_USE_EVENTS
        else if(handle->use_events && \
                handle->event_settings.event_mask & CBUFF_EVENT_OVERWRITE) {
                    emit_overwrite_event(handle);
        }
#endif
        if(handle->allow_overwrite) {
            overwrite = true;
        }
    }

    /** lock buffer for writing **/
    if(!err && xSemaphoreTake(handle->sem, pdMS_TO_TICKS(CBUFFER_SEM_WAIT_MS)) != pdTRUE) {
        err = ESP_ERR_TIMEOUT;
    }
    else {
        /** write timestamp **/
        if(handle->pkt_settings.use_timestamp) {
            if(handle->pkt_settings.ts_precision == CBUFF_TS_PRECISION_SECOND) {
                time_t sse = time(NULL);
                cbuffer_write_ll(handle, &sse, sizeof(time_t));
            }
            else {
                struct timeval tv_now;
                gettimeofday(&tv_now, NULL);
                int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
                cbuffer_write_ll(handle, &time_us, sizeof(int64_t));
            }
        }
        /** write data **/
        cbuffer_write_ll(handle, data, length);

        /** write seperation bytes **/
        if(handle->pkt_settings.use_sep_bytes) {
            cbuffer_write_ll(handle, handle->pkt_settings.sep_bytes, handle->pkt_settings.sep_length);
        }
        /** only increment the packet count if we're not overwriting old packets **/
        if(!overwrite) {
            handle->pkts_available++;
        }
        xSemaphoreGive(handle->sem);

#ifdef CONFIG_USE_EVENTS
        uint32_t unread_bytes = buffer_unread_bytes(handle);
        if( handle->use_events && \
            (handle->event_settings.event_mask & CBUFF_EVENT_NEARFULL) && \
            (unread_bytes >= handle->event_settings.nearfull_val)) {
                emit_nearfull_event(handle);
        }
#endif

    }

    // uint32_t free_bytes = buffer_free_bytes(handle);
    // ESP_LOGI(CBUFF_TAG, "Wrote %u byte packet to cbuffer, %u free bytes left", length, free_bytes);

    return err;
}


esp_err_t cbuffer_read(CBuff handle, void *buffer, uint32_t length){

    esp_err_t ret = ESP_OK;
    uint32_t unread = buffer_unread_bytes(handle);
    if(length > unread) {
        ret = ESP_ERR_INVALID_ARG;
    }
    else if(buffer == NULL) {
        ret = ESP_ERR_NO_MEM;
    } 
    else if(xSemaphoreTake(handle->sem, pdMS_TO_TICKS(CBUFFER_SEM_WAIT_MS)) != pdTRUE) {
        ESP_LOGE(CBUFF_TAG, "Error unlocking cbuffer semaphore!");
        ret = ESP_ERR_TIMEOUT;      
    }
    
    if(!ret) {
        cbuffer_read_ll(handle, buffer, length);
        xSemaphoreGive(handle->sem);
    }

#ifdef CONFIG_USE_EVENTS

    if(!ret && 
        handle->use_events &&
        (handle->pkts_available < 1 || handle->data_len < 1 ) &&
        handle->event_settings.event_mask & CBUFF_EVENT_EMPTY) {
            emit_empty_event(handle);
       }

#endif

    return ret;
}


esp_err_t cbuffer_read_packet(CBuff handle, void *data) {

    /* function to write an entire packet at once - do it faster this time! */
    esp_err_t err = ESP_OK;
    uint8_t sep[CBUFF_MAX_SEP_BYTES] = {0};

    /** lock buffer for writing **/
    if(!handle->use_packets) {
        ESP_LOGE(CBUFF_TAG, "Error, packets not configured for this cbuffer!");
        err = ESP_ERR_INVALID_STATE;
    }

    if(!err && handle->pkts_available < 1) {
        ESP_LOGI(CBUFF_TAG, "No packets available");
        err = ESP_ERR_NOT_FOUND;
    }

    if(!err && xSemaphoreTake(handle->sem, pdMS_TO_TICKS(CBUFFER_SEM_WAIT_MS)) != pdTRUE) {
        err = ESP_ERR_TIMEOUT;
    }
    
    if(!err) {

        uint16_t pkt_len = handle->pkt_settings.packet_sz_bytes;

        cbuffer_read_ll(handle, data, pkt_len);
        handle->pkts_available--;
        xSemaphoreGive(handle->sem);
    }

    return err;

}


esp_err_t cbuffer_clear(CBuff handle) {
    if(handle != NULL) {
        memset(handle->buffer_start, 0, handle->buffer_len);
    }
    return ESP_OK;
}


esp_err_t cbuffer_reset_pointers(CBuff handle) {
    if(handle != NULL) {
        handle->read_ptr = handle->buffer_start;
        handle->write_ptr = handle->buffer_start;
    }
    return ESP_OK;
}


esp_err_t cbuffer_reset_buffer(CBuff handle) {
    if(handle != NULL) {
        cbuffer_clear(handle);
        cbuffer_reset_pointers(handle);
    }
    return ESP_OK;
}


/** CBUFFER UTILS 
 **/


/** 
 *  After some thought, going to create a task which can be created to do async
 *  data-dumping. Could also do async data reading too. Whilst this is some cool stuff to do
 *  with cbuffers, it's not really core, so keeping this functionality to the extended 'cbuff utils'
 *  
 * 
 * 
 *  Keep task simple -  
 *      - condition - taskComplete (if complete, delete/stop task)
 *      - current action
 * 
 **/

static void cbuff_dispatch_task(void *args) {

    esp_err_t err = ESP_OK;
    CBuff handle = (CBuffer_Handle_t *)args;
    cbuffer_task_settings_t *task = (cbuffer_task_settings_t *)&handle->tx_task;
    uint32_t io_sz = 0;
    void *ptr = NULL;
    bool pkt_dispatch = false;
    bool done = false;

    if(task->current_task == CBUFF_TASK_PACKET_DISPATCH_CONTINUOUS) {
        pkt_dispatch = true;
        io_sz = handle->pkt_settings.packet_sz_bytes;
    }
    else {
        io_sz = task->chunk_sz;
    }

    while(!done) {
        
        /** if data available, dispatch via io **/
        if((pkt_dispatch && handle->pkts_available > 0) ||
            (!pkt_dispatch && buffer_unread_bytes(handle) >= task->chunk_sz) 
        ) {

            switch(task->io_type) {
                case CBUFF_IO_TYPE_CAN:
                    /** TODO: Can send from cbuffer **/
                    break;
                case CBUFF_IO_TYPE_I2C:
                    /** TODO: i2c send from cbuffer **/
                    break;  
                case CBUFF_IO_TYPE_SPI:
                    /** TODO: implement spi send from cbuffer **/                    
                    break;
                case CBUFF_IO_TYPE_UART:
                    /** place a packet into the uart queue **/

                    ptr = handle->read_ptr;
                    err = cbuff_dump_uart(handle, task->io_bus, io_sz);
                    if(!err && pkt_dispatch) {
                        handle->pkts_available--;
                    }
                    else if(err && pkt_dispatch) {
                        /** if we didn't send the whole packet, reset the read ptr to pre-read
                         *  so things don't get out of sync.
                         * **/
                        handle->read_ptr = ptr;
                    }
                    else if(err && !pkt_dispatch) {
                        /** do the same here, why not **/
                        handle->read_ptr = ptr;
                    }
                    break;
                default:
                    ESP_LOGE(CBUFF_TAG, "Error, invalid IO Type");
                    break;

            }

            vTaskDelay(pdMS_TO_TICKS(CBUFFER_CONFIG_DISPATCH_YIELD_MS));
        }
        /** there are no bytes left to send and task not continous **/
        else if(!task->continuous) {
            task->is_complete = true;
            done = true;
        }
        /** task is continous and no data available - wait new data **/
        else {
            vTaskDelay(pdMS_TO_TICKS(CBUFFER_CONFIG_WAIT_PKT_MS));
        }
    }
    /** here be dragons **/

#ifdef CONFIG_USE_EVENTS

    

#endif /** CONFIG_USE_EVENTS **/

    task->active = false;
    vTaskDelete(NULL);
    
    /** hopefully task is deleted by now - just in case **/
    while(1) {
        vTaskDelay(1);
    }
}




/** 
 *  TODO:
 *  Async receive task to periodically read data into the cbuffer 
 *  from an io peripheral
 *  
 *  Receive task could be free-running or stop on reaching a 
 *  pre-determined condition?
 *   
 ***/
static void cbuff_receive_task(void *args) {

    while(1) {
        vTaskDelay(100);
    }
}


esp_err_t cbuffer_start_task(CBuff handle, cbuffer_data_io_t io_type, uint8_t io_bus, uint8_t addr, cbuffer_task_t type, uint16_t chunk_sz) {
    /** 
     *  Start a cbuffer task to i/o data to/from a specific peripheral 
     *      - could look into using dma mem for this possibly? 
     *      - start a single or continuous task
     *      - single tasks should end their task when done
     *      - continous tasks can be ended by using this function and setting task type to NO_TASK
     **/
    esp_err_t err = ESP_OK;

    if((type == CBUFF_TASK_PACKET_DISPATCH_AVAILABLE ||
        type == CBUFF_TASK_PACKET_DISPATCH_CONTINUOUS) &&
        !handle->use_packets) {
        ESP_LOGE(CBUFF_TAG, "Error packets not configured for packet task type");
        err = ESP_ERR_INVALID_STATE;
    }

    else if((type == CBUFF_TASK_PACKET_DISPATCH_AVAILABLE ||
             type == CBUFF_TASK_PACKET_DISPATCH_CONTINUOUS ||
             type == CBUFF_TASK_DATA_DISPATCH_AVAILABLE || 
             type == CBUFF_TASK_PACKET_DISPATCH_AVAILABLE) &&
        handle->tx_task.active) {
        ESP_LOGE(CBUFF_TAG, "Error TX task is currently active");
        err = ESP_ERR_INVALID_STATE;
    }

    else if((type == CBUFF_TASK_DATA_READ ||
             type == CBUFF_TASK_PACKET_READ) &&
             handle->rx_task.active ) {
        ESP_LOGE(CBUFF_TAG, "Error RX task is currently active");
        err = ESP_ERR_INVALID_STATE;
    }

    else if((type == CBUFF_TASK_DATA_DISPATCH_AVAILABLE ||
         type == CBUFF_TASK_DATA_DISPATCH_CONTINUOUS) &&
        handle->use_packets) {
        ESP_LOGE(CBUFF_TAG, "Error packets configured but data task requested");
        err = ESP_ERR_INVALID_STATE;
    }
    
    if(type >= CBUFF_TASK_MAX) {
        ESP_LOGE(CBUFF_TAG, "Error invalid task type");
        err = ESP_ERR_INVALID_ARG;
    }

    if(handle->tx_task.active && type != CBUFF_TASK_NONE) {
        ESP_LOGE(CBUFF_TAG, "Error: task already running");
        err = ESP_ERR_INVALID_STATE;
    }

    if(!err) {
        if(type < CBUFF_TASK_DATA_READ ) {
            /** tx type **/
            handle->tx_task.addr = addr;
            handle->tx_task.io_bus = io_bus;
            handle->tx_task.io_type = io_type;
            handle->tx_task.is_complete = false;
            handle->tx_task.current_task = type;
            handle->tx_task.chunk_sz = chunk_sz;
        }
        else if (type >= CBUFF_TASK_DATA_READ) {
            handle->rx_task.addr = addr;
            handle->rx_task.io_bus = io_bus;
            handle->rx_task.io_type = io_type;
            handle->rx_task.is_complete = false;
            handle->rx_task.current_task = type;
            handle->rx_task.chunk_sz = chunk_sz;          
        }

        switch (type)
        {
        case CBUFF_TASK_NONE:
            /** if running a task, stop the task **/
            if(handle->tx_task.active) {
                ESP_LOGI(CBUFF_TAG, "Shutting down running task");
                vTaskSuspend(handle->tx_task.task_handle);
                vTaskDelete(handle->tx_task.task_handle);
                handle->tx_task.active = false;
            }
            break;

        case CBUFF_TASK_DATA_DISPATCH_AVAILABLE:
        case CBUFF_TASK_PACKET_DISPATCH_AVAILABLE:
            handle->tx_task.continuous = false;
            if(xTaskCreate(cbuff_dispatch_task, "cbuff_dispatch_task", 5012, handle, 5, &handle->tx_task.task_handle) != pdTRUE) {
                err = ESP_ERR_NO_MEM;
            }
            else {
                handle->tx_task.active = true;
            }
            break;
        case CBUFF_TASK_DATA_DISPATCH_CONTINUOUS:
        case CBUFF_TASK_PACKET_DISPATCH_CONTINUOUS:
            handle->tx_task.continuous = true;
            if(xTaskCreate(cbuff_dispatch_task, "cbuff_dispatch_task", 5012, handle, 5, &handle->tx_task.task_handle) != pdTRUE) {
                err = ESP_ERR_NO_MEM;
            } else {
                handle->tx_task.active = true;
            }
            break;
        case CBUFF_TASK_DATA_READ:
        case CBUFF_TASK_PACKET_READ:
            handle->rx_task.active = true;
            if(xTaskCreate(cbuff_receive_task, "cbuff_receive_task", 5012, handle, 5, &handle->rx_task.task_handle) != pdTRUE) {
                err = ESP_ERR_NO_MEM;
            }
            break;
        default:
            break;
        }
    }

    if(!err) {
        ESP_LOGI(CBUFF_TAG, "Started new task succesfully!");
    }

    return err;
}


esp_err_t cbuffer_read_i2c_pattern_block(CBuff handle, gcd_transaction_t *trx) {

    esp_err_t err = ESP_OK;
    uint32_t first_write = 0;
    uint32_t bytes_written = 0;
    uint32_t bytes_read = 0;
    uint8_t n_chunks = 0;
    uint16_t last_chunk_sz = 0;
    uint32_t read_len = 0;
    uint16_t pkts_in_block = 0;
    uint16_t pkts_per_chunk = 0;

    uint8_t middle_buffer[CBUFFER_CHUNK_READ_SIZE] = {0};

    if(!handle->use_packets) {
        err = ESP_ERR_INVALID_STATE;
        ESP_LOGE(CBUFF_TAG, "Error, packets are not configured for this CBuffer!");
    }
    if (!err && trx->len % handle->pkt_settings.packet_sz_bytes > 0) {
        ESP_LOGE(CBUFF_TAG, "Error, length is not a multiple of packet size, truncating read length...");
        trx->len = trx->len - (trx->len % handle->pkt_settings.packet_sz_bytes);
    }

    if(!err) {
        /** make our chunk read size multiple of whole packets  **/
        pkts_in_block = trx->len / handle->pkt_settings.packet_sz_bytes;

        if(trx->len > CBUFFER_CHUNK_READ_SIZE) {
            pkts_per_chunk = CBUFFER_CHUNK_READ_SIZE / handle->pkt_settings.packet_sz_bytes;
            n_chunks = pkts_in_block / pkts_per_chunk;
            read_len = pkts_per_chunk * handle->pkt_settings.packet_sz_bytes;
        }
        else {
            pkts_per_chunk = pkts_in_block;
            read_len = trx->len;
            n_chunks = 1;
        }

        /** while chunks to read, read i2c to middle buffer, then use the write packet function
         * to save packets as usual 
         */
        while(n_chunks) {
            memset(middle_buffer, 0, sizeof(uint8_t ) * CBUFFER_CHUNK_READ_SIZE);
            err = gcd_i2c_read_address(trx->bus, trx->dev, trx->reg, read_len, middle_buffer);
            if(err) {
                ESP_LOGE(CBUFF_TAG, "Error during buffer read - exiting");
                break;
            }
            else {
                cbuffer_write_packet(handle, middle_buffer, read_len);
                n_chunks--;
            }
        }
    }

    return err;
}


esp_err_t cbuffer_read_i2c_block(CBuff handle, uint8_t bus, uint8_t device_addr, uint8_t reg_addr, uint32_t len) {

    esp_err_t err = ESP_OK;
    uint32_t first_write = 0;
    uint32_t written = 0;

    if(len > handle->buffer_len) {
        ESP_LOGE(CBUFF_TAG, "Error: length greater than buffer size!");
        err = ESP_ERR_INVALID_ARG;
    }
    else if(len > buffer_free_bytes(handle) && !handle->allow_overwrite) {
        ESP_LOGE(CBUFF_TAG, "Error: Insufficient space without overwriting");
        err = ESP_ERR_INVALID_STATE;
    }
    else {
        err = gcd_i2c_bus_claim(bus);
    }

    if(!err) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);

        i2c_master_write_byte(cmd, device_addr << 1 | 0, 1);
        i2c_master_write_byte(cmd, reg_addr, 1);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, device_addr << 1 | 1, 1);

        /** read first bytes **/
        if(buffer_will_overrun(handle, len)) {
            first_write = buffer_bytes_until_end(handle, false);
            i2c_master_read(cmd, handle->write_ptr, first_write, 0);
            handle->write_ptr = handle->buffer_start;
            written += first_write;
        }
        
        /** read rest of bytes **/
        i2c_master_read(cmd, handle->write_ptr, len - written - 1, 0);
        handle->write_ptr += len - written - 1;

        /** read last byte with stop bit **/
        i2c_master_read_byte(cmd, handle->write_ptr, 1);
        handle->write_ptr += 1;

        i2c_master_stop(cmd);
        err = i2c_master_cmd_begin(bus, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);

        if (err == ESP_ERR_TIMEOUT)
        {
            ESP_LOGW(CBUFF_TAG, "I2C Timeout error");
        } 
        if (err != ESP_OK)
        {
            ESP_LOGW(CBUFF_TAG, "I2C error during block read {%u}", err);
        }

        gcd_i2c_bus_unclaim(bus);
    }

    return err;
}


esp_err_t cbuffer_dump_packet_uart(CBuff handle, uint8_t bus) {
    
    /** read a packet **/
    esp_err_t err = ESP_OK;

    /** send packet over uart **/
    err = gcd_uart_transmit_blocking(bus, handle->read_ptr, handle->pkt_settings.packet_sz_bytes);

    /** clear buffer **/
    handle->pkts_available--;

    return err;
}


esp_err_t cbuff_dump_uart(CBuff handle, uint8_t bus, uint32_t len) {

    esp_err_t err = ESP_OK;
    uint32_t sent = 0;
    uint32_t bytes_until_end = buffer_bytes_until_end(handle, true);

    if(len > handle->buffer_len) {
        ESP_LOGE(CBUFF_TAG, "Error: Dump length is greater than buffer length");
        err = ESP_ERR_INVALID_ARG;
    }

    if(!err && len > bytes_until_end) {
        uint32_t first_write = bytes_until_end; 
        sent = uart_tx_chars(handle->tx_task.io_bus, (const char *)handle->read_ptr, first_write);
        if(sent < first_write) {
            /** there wasn't enough space in uart fifo */
            ESP_LOGI(CBUFF_TAG, "UART fifo full");
            handle->read_ptr += sent;
            err = ESP_ERR_NO_MEM;
        }
        else {
            handle->read_ptr = handle->buffer_start;
            uint32_t second_write = len - first_write;
            sent = uart_tx_chars(handle->tx_task.io_bus, (const char *)handle->read_ptr, second_write);
            if(sent < second_write) {
                ESP_LOGI(CBUFF_TAG, "UART fifo full");          
                err = ESP_ERR_NO_MEM;
            }
            handle->read_ptr += sent;
        }
    }
    else {
        sent = uart_tx_chars(handle->tx_task.io_bus, (const char *)handle->read_ptr, len);
        if(sent < len) {
            ESP_LOGI(CBUFF_TAG, "UART fifo full");
        }
        handle->read_ptr += sent;
    }

    return err;
}


esp_err_t cbuff_dump_packets_uart(CBuff handle, uint8_t uart_bus) {

    esp_err_t err = ESP_OK;
    uint8_t mid_buffer[CBUFFER_MAX_PACKET_SIZE_BYTES] = {0};
    uint16_t avail_packets = 0;
    
    if(!handle->use_packets) {
        err = ESP_ERR_INVALID_STATE;
        ESP_LOGE("CBUFF_UTILS", "Error - packets not configured for this cbuffer!");
    }

    if(handle->pkts_available < 1) {
        err = ESP_ERR_INVALID_STATE;
        ESP_LOGE("CBUFF_UTILS", "Error - no packets available");        
    }

    if(!err) {
        uint16_t tx_size = handle->pkt_settings.packet_sz_bytes;
        ESP_LOGI(CBUFF_TAG, "Dumping %u packets of bytes %u on uart bus %u", handle->pkts_available, tx_size, uart_bus);
        avail_packets = handle->pkts_available;

        for(uint16_t i=0; i < avail_packets; i++) {
            /** read a packet **/
            err = cbuffer_read_packet(handle, mid_buffer);

            if(err) {
                ESP_LOGE(CBUFF_TAG, "Error reading packet %u - break [%u]", i, err);
                break;
            }
            /** send packet over uart **/
            err = gcd_uart_transmit_blocking(uart_bus, mid_buffer, tx_size);
            
            /** clear buffer **/
            memset(mid_buffer, 0, CBUFFER_MAX_PACKET_SIZE_BYTES);
            if(err) {
                ESP_LOGE(CBUFF_TAG, "Error dumping packet %u - break [%u]", i, err);
                break;
            }

            handle->pkts_available--;
        }
    }

    return err;
}