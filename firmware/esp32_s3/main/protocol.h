/**
 * @file protocol.h
 * @brief Frame encoding, decoding, CRC-16 computation, and byte-stream parser.
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "protocol_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Compute CRC-16-CCITT (polynomial 0x1021, init 0xFFFF).
 */
uint16_t protocol_crc16_ccitt(const uint8_t *data, size_t len);

/**
 * @brief Encode a binary frame into out_buf.
 * @return Number of bytes written, or 0 on buffer overflow.
 */
size_t protocol_encode_frame(
    uint8_t msg_type,
    uint16_t seq_id,
    const uint8_t *payload,
    uint16_t payload_len,
    uint8_t *out_buf,
    size_t out_max_len
);

/**
 * @brief Validate CRC and decode an entire frame.
 * @return 0 on success, negative error code on failure.
 */
int protocol_decode_frame(
    const uint8_t *frame_data,
    size_t frame_len,
    frame_header_t *out_header,
    const uint8_t **out_payload
);

/**
 * @brief Streaming frame parser state machine for UART streams.
 */
typedef struct {
    uint8_t buffer[FRAME_HEADER_SIZE + MAX_PAYLOAD_SIZE + CRC_SIZE];
    size_t  bytes_collected;
    bool    in_frame;
} stream_parser_t;

void stream_parser_init(stream_parser_t *parser);

/**
 * @brief Feed a single byte into the streaming parser.
 * @return 1 if a complete frame was parsed, 0 if waiting, negative on error.
 */
int stream_parser_feed_byte(
    stream_parser_t *parser,
    uint8_t byte,
    frame_header_t *out_header,
    uint8_t *out_payload_buf,
    size_t max_payload_len
);

#ifdef __cplusplus
}
#endif

#endif // PROTOCOL_H
