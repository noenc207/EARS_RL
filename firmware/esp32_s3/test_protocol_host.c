/**
 * @file test_protocol_host.c
 * @brief Host-side unit tests for ESP32-S3 binary protocol implementation.
 *
 * Compiles with standard host C compiler (gcc/clang) without requiring ESP-IDF.
 * Verifies byte-level parity with the Python reference implementation.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "protocol.h"
#include "protocol_defs.h"

// Helper to convert hex string to byte array
static size_t hex_to_bytes(const char *hex, uint8_t *out, size_t max_out) {
    size_t hex_len = strlen(hex);
    assert(hex_len % 2 == 0);
    size_t byte_len = hex_len / 2;
    assert(byte_len <= max_out);

    for (size_t i = 0; i < byte_len; i++) {
        unsigned int byte_val;
        sscanf(hex + (2 * i), "%02x", &byte_val);
        out[i] = (uint8_t)byte_val;
    }
    return byte_len;
}

// Helper to compare bytes and print diff on failure
static void assert_bytes_equal(const uint8_t *actual, const uint8_t *expected, size_t len, const char *msg) {
    if (memcmp(actual, expected, len) != 0) {
        fprintf(stderr, "FAIL: %s\nExpected: ", msg);
        for (size_t i = 0; i < len; i++) fprintf(stderr, "%02x ", expected[i]);
        fprintf(stderr, "\nActual:   ");
        for (size_t i = 0; i < len; i++) fprintf(stderr, "%02x ", actual[i]);
        fprintf(stderr, "\n");
        exit(1);
    }
}

static void test_crc16_computation(void) {
    printf("[TEST] CRC-16-CCITT computation... ");
    // Test known string "123456789" -> standard CCITT false (0xFFFF init) is 0x29B1
    const uint8_t data[] = "123456789";
    uint16_t crc = protocol_crc16_ccitt(data, 9);
    assert(crc == 0x29B1);
    printf("PASSED (0x29B1)\n");
}

static void test_encode_vectors(void) {
    printf("[TEST] Frame Encoding against Canonical Test Vectors... ");

    uint8_t out_buf[128];
    uint8_t expected[128];
    size_t exp_len;
    size_t enc_len;

    // Vector 1: PING_SEQ_1
    // Payload: uint64_t 1,000,000,000 ns (Little-Endian: 0x00ca9a3b00000000)
    uint64_t ts_ns = 1000000000ULL;
    enc_len = protocol_encode_frame(
        MSG_PING, 1, (const uint8_t *)&ts_ns, sizeof(ts_ns), out_buf, sizeof(out_buf)
    );
    exp_len = hex_to_bytes("aa5501010100080000ca9a3b000000009e7e", expected, sizeof(expected));
    assert(enc_len == exp_len);
    assert_bytes_equal(out_buf, expected, enc_len, "PING_SEQ_1 encoding mismatch");

    // Vector 2: SET_MODE_ARM_SEQ_2
    uint8_t mode_val = 1;
    enc_len = protocol_encode_frame(
        MSG_SET_MODE, 2, &mode_val, 1, out_buf, sizeof(out_buf)
    );
    exp_len = hex_to_bytes("aa55010302000100016d79", expected, sizeof(expected));
    assert(enc_len == exp_len);
    assert_bytes_equal(out_buf, expected, enc_len, "SET_MODE_ARM_SEQ_2 encoding mismatch");

    // Vector 3: ESTOP_SEQ_10
    enc_len = protocol_encode_frame(
        MSG_ESTOP, 10, NULL, 0, out_buf, sizeof(out_buf)
    );
    exp_len = hex_to_bytes("aa5501080a000000bd9a", expected, sizeof(expected));
    assert(enc_len == exp_len);
    assert_bytes_equal(out_buf, expected, enc_len, "ESTOP_SEQ_10 encoding mismatch");

    // Vector 4: RESET_FAULT_SEQ_11
    uint8_t reset_val = 0;
    enc_len = protocol_encode_frame(
        MSG_RESET_FAULT, 11, &reset_val, 1, out_buf, sizeof(out_buf)
    );
    exp_len = hex_to_bytes("aa5501090b000100003247", expected, sizeof(expected));
    assert(enc_len == exp_len);
    assert_bytes_equal(out_buf, expected, enc_len, "RESET_FAULT_SEQ_11 encoding mismatch");

    printf("PASSED (All 4 vectors matched)\n");
}

static void test_decode_vectors(void) {
    printf("[TEST] Frame Decoding against Canonical Test Vectors... ");

    uint8_t frame_bytes[64];
    size_t frame_len;
    frame_header_t hdr;
    const uint8_t *payload = NULL;

    // Decode PING
    frame_len = hex_to_bytes("aa5501010100080000ca9a3b000000009e7e", frame_bytes, sizeof(frame_bytes));
    int res = protocol_decode_frame(frame_bytes, frame_len, &hdr, &payload);
    assert(res == 0);
    assert(hdr.magic[0] == 0xAA && hdr.magic[1] == 0x55);
    assert(hdr.version == 1);
    assert(hdr.msg_type == MSG_PING);
    assert(hdr.sequence_id == 1);
    assert(hdr.payload_len == 8);
    assert(payload != NULL);
    uint64_t ts_out;
    memcpy(&ts_out, payload, 8);
    assert(ts_out == 1000000000ULL);

    // Decode ESTOP (zero payload)
    frame_len = hex_to_bytes("aa5501080a000000bd9a", frame_bytes, sizeof(frame_bytes));
    res = protocol_decode_frame(frame_bytes, frame_len, &hdr, &payload);
    assert(res == 0);
    assert(hdr.msg_type == MSG_ESTOP);
    assert(hdr.sequence_id == 10);
    assert(hdr.payload_len == 0);
    assert(payload == NULL);

    printf("PASSED\n");
}

static void test_crc_corruption_detection(void) {
    printf("[TEST] CRC Corruption Detection... ");

    uint8_t frame_bytes[64];
    size_t frame_len = hex_to_bytes("aa55010302000100016d79", frame_bytes, sizeof(frame_bytes));

    frame_header_t hdr;
    const uint8_t *payload = NULL;

    // Valid check
    assert(protocol_decode_frame(frame_bytes, frame_len, &hdr, &payload) == 0);

    // Corrupt 1 bit in payload
    frame_bytes[8] ^= 0x01;
    int res = protocol_decode_frame(frame_bytes, frame_len, &hdr, &payload);
    assert(res == -6); // CRC mismatch

    // Corrupt 1 bit in header sequence ID
    frame_bytes[8] ^= 0x01; // restore payload
    frame_bytes[4] ^= 0x01; // flip sequence id
    res = protocol_decode_frame(frame_bytes, frame_len, &hdr, &payload);
    assert(res == -6); // CRC mismatch

    printf("PASSED (Bit-flip errors rejected with code -6)\n");
}

static void test_stream_parser(void) {
    printf("[TEST] Stream Parser Fragment Handling & Noise Rejection... ");

    stream_parser_t parser;
    stream_parser_init(&parser);

    // Garbage prefix followed by valid SET_MODE frame
    const char *hex_stream = "ff00aa" "aa55010302000100016d79";
    uint8_t stream_bytes[64];
    size_t stream_len = hex_to_bytes(hex_stream, stream_bytes, sizeof(stream_bytes));

    frame_header_t hdr;
    uint8_t payload_buf[32];
    int frame_count = 0;

    for (size_t i = 0; i < stream_len; i++) {
        int r = stream_parser_feed_byte(&parser, stream_bytes[i], &hdr, payload_buf, sizeof(payload_buf));
        if (r == 1) {
            frame_count++;
            assert(hdr.msg_type == MSG_SET_MODE);
            assert(hdr.sequence_id == 2);
            assert(hdr.payload_len == 1);
            assert(payload_buf[0] == 1);
        }
    }
    assert(frame_count == 1);

    // Feed another valid frame in 2-byte chunks
    const char *ping_hex = "aa5501010100080000ca9a3b000000009e7e";
    size_t ping_len = hex_to_bytes(ping_hex, stream_bytes, sizeof(stream_bytes));
    frame_count = 0;

    for (size_t i = 0; i < ping_len; i++) {
        int r = stream_parser_feed_byte(&parser, stream_bytes[i], &hdr, payload_buf, sizeof(payload_buf));
        if (r == 1) {
            frame_count++;
            assert(hdr.msg_type == MSG_PING);
            assert(hdr.sequence_id == 1);
        }
    }
    assert(frame_count == 1);

    printf("PASSED\n");
}

int main(void) {
    printf("====================================================\n");
    printf(" EARS-RL ESP32-S3 Protocol Host Verification Suite   \n");
    printf("====================================================\n");

    test_crc16_computation();
    test_encode_vectors();
    test_decode_vectors();
    test_crc_corruption_detection();
    test_stream_parser();

    printf("====================================================\n");
    printf(" ALL ESP32-S3 PROTOCOL C TESTS PASSED (5/5)         \n");
    printf("====================================================\n");
    return 0;
}
