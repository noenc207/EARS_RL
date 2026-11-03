/**
 * @file protocol.c
 * @brief Implementation of EARS-RL binary protocol framing and CRC verification.
 */

#include "protocol.h"
#include <string.h>

uint16_t protocol_crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000) {
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF;
            } else {
                crc = (crc << 1) & 0xFFFF;
            }
        }
    }
    return crc;
}

size_t protocol_encode_frame(
    uint8_t msg_type,
    uint16_t seq_id,
    const uint8_t *payload,
    uint16_t payload_len,
    uint8_t *out_buf,
    size_t out_max_len
) {
    if (payload_len > MAX_PAYLOAD_SIZE) {
        return 0;
    }
    size_t total_len = FRAME_HEADER_SIZE + payload_len + CRC_SIZE;
    if (total_len > out_max_len) {
        return 0;
    }

    frame_header_t *hdr = (frame_header_t *)out_buf;
    hdr->magic[0] = PROTOCOL_MAGIC_0;
    hdr->magic[1] = PROTOCOL_MAGIC_1;
    hdr->version = PROTOCOL_VERSION;
    hdr->msg_type = msg_type;
    hdr->sequence_id = seq_id;
    hdr->payload_len = payload_len;

    if (payload != NULL && payload_len > 0) {
        memcpy(out_buf + FRAME_HEADER_SIZE, payload, payload_len);
    }

    size_t body_len = FRAME_HEADER_SIZE + payload_len;
    uint16_t crc = protocol_crc16_ccitt(out_buf, body_len);

    out_buf[body_len] = (uint8_t)(crc & 0xFF);
    out_buf[body_len + 1] = (uint8_t)((crc >> 8) & 0xFF);

    return total_len;
}

int protocol_decode_frame(
    const uint8_t *frame_data,
    size_t frame_len,
    frame_header_t *out_header,
    const uint8_t **out_payload
) {
    if (frame_len < FRAME_HEADER_SIZE + CRC_SIZE) {
        return -1; // Incomplete
    }
    if (frame_data[0] != PROTOCOL_MAGIC_0 || frame_data[1] != PROTOCOL_MAGIC_1) {
        return -2; // Bad magic
    }
    const frame_header_t *hdr = (const frame_header_t *)frame_data;
    if (hdr->version != PROTOCOL_VERSION) {
        return -3; // Version mismatch
    }
    if (hdr->payload_len > MAX_PAYLOAD_SIZE) {
        return -4; // Payload too large
    }
    size_t expected_total = FRAME_HEADER_SIZE + hdr->payload_len + CRC_SIZE;
    if (frame_len < expected_total) {
        return -5; // Truncated
    }

    size_t body_len = FRAME_HEADER_SIZE + hdr->payload_len;
    uint16_t calc_crc = protocol_crc16_ccitt(frame_data, body_len);
    uint16_t recv_crc = (uint16_t)frame_data[body_len] | ((uint16_t)frame_data[body_len + 1] << 8);

    if (calc_crc != recv_crc) {
        return -6; // CRC mismatch
    }

    if (out_header != NULL) {
        *out_header = *hdr;
    }
    if (out_payload != NULL) {
        *out_payload = (hdr->payload_len > 0) ? (frame_data + FRAME_HEADER_SIZE) : NULL;
    }
    return 0;
}

void stream_parser_init(stream_parser_t *parser) {
    if (parser != NULL) {
        parser->bytes_collected = 0;
        parser->in_frame = false;
    }
}

int stream_parser_feed_byte(
    stream_parser_t *parser,
    uint8_t byte,
    frame_header_t *out_header,
    uint8_t *out_payload_buf,
    size_t max_payload_len
) {
    if (parser == NULL) return -1;

    // Search for magic bytes
    if (!parser->in_frame) {
        if (parser->bytes_collected == 0) {
            if (byte == PROTOCOL_MAGIC_0) {
                parser->buffer[0] = byte;
                parser->bytes_collected = 1;
            }
        } else if (parser->bytes_collected == 1) {
            if (byte == PROTOCOL_MAGIC_1) {
                parser->buffer[1] = byte;
                parser->bytes_collected = 2;
                parser->in_frame = true;
            } else if (byte == PROTOCOL_MAGIC_0) {
                // Stays at 1 byte
                parser->bytes_collected = 1;
            } else {
                parser->bytes_collected = 0;
            }
        }
        return 0;
    }

    // Accumulating inside frame
    parser->buffer[parser->bytes_collected++] = byte;

    if (parser->bytes_collected < FRAME_HEADER_SIZE) {
        return 0; // Still collecting header
    }

    const frame_header_t *hdr = (const frame_header_t *)parser->buffer;
    if (hdr->version != PROTOCOL_VERSION || hdr->payload_len > MAX_PAYLOAD_SIZE) {
        // Corrupt header detected: reset and search next magic
        parser->in_frame = false;
        parser->bytes_collected = 0;
        return -2;
    }

    size_t total_expected = FRAME_HEADER_SIZE + hdr->payload_len + CRC_SIZE;
    if (parser->bytes_collected < total_expected) {
        return 0; // Still collecting payload / CRC
    }

    // Full frame collected: validate CRC
    size_t body_len = FRAME_HEADER_SIZE + hdr->payload_len;
    uint16_t calc_crc = protocol_crc16_ccitt(parser->buffer, body_len);
    uint16_t recv_crc = (uint16_t)parser->buffer[body_len] | ((uint16_t)parser->buffer[body_len + 1] << 8);

    parser->in_frame = false;
    parser->bytes_collected = 0;

    if (calc_crc != recv_crc) {
        return -6; // CRC error
    }

    if (out_header != NULL) {
        *out_header = *hdr;
    }
    if (out_payload_buf != NULL && hdr->payload_len > 0) {
        size_t copy_len = (hdr->payload_len < max_payload_len) ? hdr->payload_len : max_payload_len;
        memcpy(out_payload_buf, parser->buffer + FRAME_HEADER_SIZE, copy_len);
    }

    return 1; // Complete valid frame!
}
