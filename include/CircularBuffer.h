/****************************************
* \file     CircularBuffer.h
* \brief    Circular Buffer header file
* \date
* \author
****************************************/

#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#define CBUFFER_SEMTAKE_TIMEOUT 200  /**< in ms, doesn't need to be big **/

/********* Includes ********************/

#include "esp_types.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/********* Definitions *****************/

#define CBUFFER_MAX_PACKET_VALUES   6
#define CBUFFER_MAX_NAME_LENGTH     24
#define CBUFFER_MAX_BUFFER_SIZE     8192
#define CBUFF_MAX_SEP_BYTES         8

#define CBUFFER_SEM_WAIT_MS         100

#ifdef CONFIG_USE_EVENTS
#include "esp_event_base.h"

#define CONFIG_EVENTPOST_WAIT_MS    200
#define CBUFF_EVENT_BASE            0x70
#define CBUFF_EVENT_FULL            (1 << 1)
#define CBUFF_EVENT_NEARFULL        (1 << 2)
#define CBUFF_EVENT_OVERWRITE       (1 << 3)
#define CBUFF_EVENTCODE_FULL        (CBUFF_EVENT_BASE << 16 | CBUFF_EVENT_FULL)
#define CBUFF_EVENTCODE_NEARFULL    (CBUFF_EVENT_BASE << 16 | CBUFF_EVENT_NEARFULL)
#define CBUFF_EVENTCODE_OVERWRITE   (CBUFF_EVENT_BASE << 16 | CBUFF_EVENT_OVERWRITE)

/**< stuct to define cbuffer event settings **/
typedef struct cbuffer_event_settings
{
    /* data */
    esp_event_loop_handle_t loop;   /**< the event loop to post to **/
    uint8_t event_mask;             /**< maask of events defined above **/
    uint32_t nearfull_val;          /**< nearly full value to alert on **/
} cbuffer_events_t;

#endif /** CONFIG_USE_EVENTS **/


/** cbuffer_pattern_t
 *  ascii codes for the python struct format characters
 *  (reduced set)
 **/
typedef enum {
    CBUFF_VARTYPE_BOOL      = 0x3F,     /**< '?' **/
    CBUFF_VARTYPE_UINT8     = 0x42,     /**< 'B' **/
    CBUFF_VARTYPE_UINT16    = 0x48,     /**< 'H' **/
    CBUFF_VARTYPE_UINT32    = 0x4C,     /**< 'L' **/
    CBUFF_VARTYPE_INT8      = 0x62,     /**< 'i' **/
    CBUFF_VARTYPE_DOUBLE    = 0x64,     /**< 'd' **/
    CBUFF_VARTYPE_FLOAT     = 0x66,     /**< 'f' **/
    CBUFF_VARTYPE_INT16     = 0x68,     /**< 'h' **/
    CBUFF_VARTYPE_INT32     = 0x6C,     /**< 'l' **/
    CBUFF_VARTYPE_PADBYTE   = 0x78,     /**< 'x' **/
} cbuffer_pattern_t;

/**< Timestamp precision enumeration **/
typedef enum {
    CBUFF_TS_PRECISION_SECOND = 1,      /**< 1s precision **/
    CBUFF_TS_PRECISION_MICROSECOND = 2, /**< 1uS precison **/
} timestamp_precision_t;


/**< member of packet structure **/
typedef struct CBuffer_packet_member
{
    /* data */
    uint8_t id;                         /**< id of member **/
    char name[CBUFFER_MAX_NAME_LENGTH+1]; /**< name of member **/
    uint8_t name_length;                /**< name length (bytes) **/
    cbuffer_pattern_t p;                /**< pattern character **/
} cbuffer_pmember_t;


/**< cbuffer packet settings **/
typedef struct cbuffer_packet_settings
{
    /* data */
    char pattern[CBUFFER_MAX_PACKET_VALUES+1];    /**< data pattern characters **/
    uint16_t packet_sz_bytes;   /**< size of packet data in bytes **/
    uint8_t packet_index;       /**< current pattern index **/
    uint8_t packet_elements;    /**< pattern length **/
    bool use_sep_bytes;         /**< add seperation marker bytes @ end of packet **/ 
    uint8_t sep_bytes[CBUFF_MAX_SEP_BYTES]; /**< seperation bytes **/
    uint8_t sep_length;         /**< number of seperator bytes to add **/
    bool use_timestamp;         /**< add a timestamp to the start of the packet **/
    timestamp_precision_t ts_precision; /**< 0 - second precision (32bit) 1 - microsecond precision (64bit) **/
    uint8_t timestamp_length;   /**< length of timestamp bytes **/
} cbuffer_pkt_settings_t;



/** CBuffer init data - keep this simple **/
typedef struct CBuffer_init {
    uint32_t size;      /**< buffer size, in bytes **/
    bool allow_ovr;
    uint8_t malloc_caps;
} CBuffer_init_t;


/** the cbuffer handle data structure **/
typedef struct CBuffer_Handle {
    void *buffer_start; /**< buffer start - start of buffer addr **/
    void *buffer_end;   /**< buffer end - end of buffer addr **/

    void *read_ptr;   /**< start of data in buffer **/
    void *write_ptr;     /**< end of data in buffer **/ 

    uint32_t buffer_len;    /**< total length of buffer **/
    uint16_t data_len;      /**< total length of data **/

    bool allow_overwrite;   /**< allow overwrite of unread data **/
    bool is_claimed;        /**< sem is held **/

    /** In order to save structured data implement packet system **/
    bool use_packets;       /**< use a packet_style packing **/
    cbuffer_pmember_t packets[CBUFFER_MAX_PACKET_VALUES]; /**< pattern to load **/
    cbuffer_pkt_settings_t pkt_settings;    /**< settings for pattern builder **/

#ifdef CONFIG_USE_EVENTS
    /** cbuffer events allow for storage/dumping on full/nearfull **/
    bool use_events;            /**< raise events **/
    cbuffer_events_t event_settings;    /**< event settings **/
#endif
    SemaphoreHandle_t sem;  /**< semaphore handle  **/
} CBuffer_Handle_t;


typedef CBuffer_Handle_t * CBuff;


/********** Types **********************/

/******** Function Definitions *********/

uint32_t buffer_unread_bytes(CBuff handle);

/**
 * \brief: Allocate a circular buffer of Size
 * \param size - size of buffer requested
 * \param  allow_overrun - allow overwriting unread data 
 * \return ptr to buffer or NULL on error
 **/
CBuff cbuffer_create(CBuffer_init_t *init);

/**
 * \brief: Configure the packet storage system 
 *          (Does not enable, packet storage is enabled with load packet)
 * \param  use_sep - add seperator bytes between packets
 * \param  sep_bytes - a pointer to an array of seperator bytes 
 * \param  sep_length - length of seperator bytes (max = 8) 
 * \param  use_timestamp - add a timestamp to the packet 
 * \return ptr to buffer or NULL on error
 **/
esp_err_t cbuffer_config_packet(CBuff handle, bool use_sep, uint8_t *sep_bytes, uint8_t sep_length, bool use_timestamp, timestamp_precision_t ts_precision);

/**
 *  \brief Gets the length of the full packet including timestamp and
 *          seperation bytes
 *  \param handle handle to the cbuffer
 *  \param len pointer to storage for length
 *  \return ESP_OK or error
 **/
esp_err_t cbuffer_get_full_packet_length(CBuff handle, uint16_t *len);


esp_err_t cbuffer_set_event_mask(CBuff handle, uint8_t event_mask);


/**
 * \brief gets the full packet pattern in string form,
 *          compatible with python struct (minus endian-ness)
 *  \param handle handle
 *  \param pattern a pointer to string storage
 *  \return ESP_OK or error
 **/
esp_err_t cbuffer_get_pattern(CBuff handle, char *pattern);

/**
 * \brief: Load a new packet structure. Resets the CBuffer and 
 *          begins recording packets in a structured manner.
 * \param handle cbuffer handle
 * \param values_per_packet - number of measurements making up this packet
 * \param pattern - a string of ascii values found in cbuffer_pattern_t 
 *                  must have same len as values_per_packet
 * \param names_array - an array of names for the packet values
 *                      must have array length = values_per_packet
 * \param IDs       - an array of uint8_t ids: must be same length as values_per_packet
 *                      and all values must be greater than zero
 * \return ESP_OK or error
**/
esp_err_t cbuffer_load_packet(CBuff handle, uint8_t values_per_packet, char *pattern, char **names_array, uint8_t *ids);


/**
 * \brief Deletes the packet configuration and deactivates the pattern storage
 * \param handle cbuffer handle
 * \return ESP_OK
 **/
esp_err_t cbuffer_clear_packet(CBuff handle);

/**
 * \brief: free a cbuffer
 * \param handle - pointer to cbuffer handle
 * \return ESP_OK or error
**/
esp_err_t cbuffer_destroy(CBuff handle); 


/** 
 * \brief: write data to the buffer
 * \param handle - ptr to the cbuffer handle
 * \param data - ptr to the data to write
 * \param length - length of data (in bytes) to write
 * \return ESP_OK or error
 **/
esp_err_t cbuffer_write(CBuff handle, void *data, uint32_t length);


/**
 * \brief: Write a formatted packet to the CBuffer
 * \param handle - cbuffer handle
 * \param data - pointer to the data
 * \param length - length of the data = is this needed?
 **/
esp_err_t cbuffer_write_packet(CBuff handle, void *data, uint32_t length);

/** 
 * \brief: read data from the buffer
 * \param handle - ptr to the cbuffer handle
 * \param buffer - ptr to read data into
 * \param length - length of data (in bytes) to read
 * \return ESP_OK or error
 **/
esp_err_t cbuffer_read(CBuff handle, void *buffer, uint32_t length);


/**
 * \brief: sets buffer contents to 0
 * \param handle see above
 * \return ESP_OK  
**/
esp_err_t cbuffer_clear(CBuff handle);


/***
 * \brief resets the r/w pointer locations to start 
 * \param handle see above
 * \return ESP_OK
**/
esp_err_t cbuffer_reset_pointers(CBuff handle);


/**
 *  \brief does the above 2 functions together 
 *  \param handle see above
 *  \return ESP_OK 
 **/
esp_err_t cbuffer_reset_buffer(CBuff handle);

#ifdef CONFIG_USE_EVENTS
/**
 * \brief configure the event settings
 * \param handle see above
 * \param use_events enable events for cbuffers
 * \param loop  the event loop to post to
 * \param event_flags an OR'd byte of CBuffer event flags (see above)
 * \param nearfull_val - the size (bytes) at which nearly full event triggers
 * \return ESP_OK or error
 **/
esp_err_t cbuffer_config_events(CBuff handle, bool use_events, esp_event_loop_handle_t loop, uint8_t event_flags, uint32_t nearfull_val);

#endif /** CONFIG_USE_EVENTS **/

#endif /* CIRCULAR_BUFFER_H */
