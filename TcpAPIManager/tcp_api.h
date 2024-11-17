/**
 *    @file    tcp_api.h
 *
 *    @brief    header file for tcp_api
 *
 *
 *
 *    @author    RJAM
 *    @created   ven. 26 avril 2024 00:28:06 CEST
 */

#ifndef TCP_API_H
#define TCP_API_H

/** Includes **/
#include "esp_err.h"

/** Defines **/

#define TCP_API_MAX_REQUEST_DATA_LEN 512
#define TCP_API_HEADER_LEN           6
#define TCP_API_CHECKSUM_LEN         1
#define TCP_API_MAX_MESSAGE_LEN                                                                    \
    (TCP_API_MAX_REQUEST_DATA_LEN + TCP_API_HEADER_LEN + TCP_API_CHECKSUM_LEN)
#define TCP_API_PORT_CHECK_PERIOD_MS 50
/** Typedefs **/

typedef struct {
    uint16_t port;

} tcp_api_init_t;

typedef struct {
    uint16_t port;
    uint8_t status;
    uint8_t max_clients;
    uint32_t trx_ctr;
} tcp_apt_t;

typedef struct tcp_api_request {
    uint8_t cmd;
    uint8_t periph_id;
    uint8_t param_id;
    uint8_t data_type;
    uint8_t data_len;
} tcp_api_request_t;

typedef tcp_api_request_t *TCP_API_h;

/** Function Declarations **/

esp_err_t tcp_api_init(tcp_api_init_t *init);

#endif /* TCP_API_H */
/** END **/
