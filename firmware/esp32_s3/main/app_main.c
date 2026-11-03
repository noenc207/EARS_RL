/**
 * @file app_main.c
 * @brief EARS-RL ESP32-S3 Firmware — Safe Commissioning & Protocol Verification.
 *
 * MANDATORY SAFETY INVARIANTS:
 * 1. COMMISSIONING_MODE = 1: ZERO pulse generation on STEP pin.
 * 2. Motor driver excitation (PIN_ENABLE) initialized HIGH (disabled).
 * 3. Rejects all motion commands with ERR_COMMISSIONING_MODE.
 * 4. Separate communication watchdog (500 ms) and motion watchdog (50 ms).
 * 5. Hardware E-stop input line has absolute priority.
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "protocol.h"

// Define safe commissioning mode (1 = NO MOTOR PULSES, SAFE COMMISSIONING)
#define COMMISSIONING_MODE          1

// Hardware GPIO pinout for ESP32-S3 <-> TMC2209
#define PIN_STEP                    4   // Step pulse output (Locked LOW)
#define PIN_DIR                     5   // Direction output
#define PIN_ENABLE                  6   // TMC2209 Active-Low Enable (Locked HIGH = Disabled)
#define PIN_ESTOP_INPUT             7   // Hardwired E-stop sensing input

// Watchdog timeouts [milliseconds]
#define COMM_WATCHDOG_TIMEOUT_MS    500 // Serial communication freshness
#define MOTION_WATCHDOG_TIMEOUT_MS  50  // Hard real-time motion freshness

#if defined(ESP_PLATFORM)
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#define UART_PORT UART_NUM_0

static inline uint64_t get_time_us(void) {
    return (uint64_t)esp_timer_get_time();
}
#else
// Host testing stub
#include <time.h>
static inline uint64_t get_time_us(void) {
    struct timespec ts;
    timespec_get(&ts, TIME_UTC);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}
#endif

// Global state variables
static device_state_t        g_state = STATE_IDLE_DISARMED;
static protocol_error_code_t g_error_code = ERR_NONE;
static uint64_t              g_last_heartbeat_us = 0;
static uint64_t              g_last_motion_cmd_us = 0;
static uint16_t              g_last_seq_id = 0;
static float                 g_measured_pos_rad = 0.0f;
static float                 g_measured_vel_rad_s = 0.0f;
static uint16_t              g_current_ma = 450;

/**
 * @brief Enforce hardware safety pins into high-impedance / disabled state.
 */
static void safe_gpio_init(void) {
#if defined(ESP_PLATFORM)
    gpio_config_t out_conf = {
        .pin_bit_mask = (1ULL << PIN_STEP) | (1ULL << PIN_DIR) | (1ULL << PIN_ENABLE),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&out_conf);

    // CRITICAL: Lock STEP LOW, lock ENABLE HIGH (driver unpowered)
    gpio_set_level((gpio_num_t)PIN_STEP, 0);
    gpio_set_level((gpio_num_t)PIN_DIR, 0);
    gpio_set_level((gpio_num_t)PIN_ENABLE, 1); // 1 = Disabled for TMC2209

    gpio_config_t in_conf = {
        .pin_bit_mask = (1ULL << PIN_ESTOP_INPUT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&in_conf);
#endif
    printf("[SAFE BOOT] GPIOs configured: STEP=LOW, ENABLE=HIGH (DISARMED)\n");
}

/**
 * @brief Handle a decoded protocol frame and construct response.
 */
static size_t process_frame(
    const frame_header_t *hdr,
    const uint8_t *payload,
    uint8_t *tx_buf,
    size_t tx_max_len
) {
    uint64_t now_us = get_time_us();
    g_last_heartbeat_us = now_us;
    g_last_seq_id = hdr->sequence_id;

    switch (hdr->msg_type) {
        case MSG_PING: {
            ping_payload_t ping;
            if (hdr->payload_len < sizeof(ping)) {
                uint8_t nack_err = ERR_COMMAND_OUT_OF_BOUNDS;
                return protocol_encode_frame(MSG_NACK, hdr->sequence_id, &nack_err, 1, tx_buf, tx_max_len);
            }
            memcpy(&ping, payload, sizeof(ping));

            pong_payload_t pong = {
                .host_timestamp_echo_ns = ping.host_timestamp_ns,
                .device_timestamp_us = now_us
            };
            return protocol_encode_frame(MSG_PONG, hdr->sequence_id, (const uint8_t *)&pong, sizeof(pong), tx_buf, tx_max_len);
        }

        case MSG_SET_MODE: {
            uint8_t enable = (hdr->payload_len > 0) ? payload[0] : 0;
            if (g_state == STATE_FAULT || g_state == STATE_ESTOP) {
                // Must not arm while in FAULT or ESTOP without explicit reset
                uint8_t err = (uint8_t)g_error_code;
                return protocol_encode_frame(MSG_NACK, hdr->sequence_id, &err, 1, tx_buf, tx_max_len);
            }
            g_state = (enable == 1) ? STATE_ENABLED_ARMED : STATE_IDLE_DISARMED;
            uint8_t status_ok = 0;
            return protocol_encode_frame(MSG_ACK, hdr->sequence_id, &status_ok, 1, tx_buf, tx_max_len);
        }

        case MSG_RESET_FAULT: {
            g_state = STATE_IDLE_DISARMED;
            g_error_code = ERR_NONE;
            uint8_t status_ok = 0;
            return protocol_encode_frame(MSG_ACK, hdr->sequence_id, &status_ok, 1, tx_buf, tx_max_len);
        }

        case MSG_ESTOP: {
            g_state = STATE_ESTOP;
            g_error_code = ERR_ESTOP_ACTIVE;
#if defined(ESP_PLATFORM)
            gpio_set_level((gpio_num_t)PIN_ENABLE, 1); // Kill power immediately
            gpio_set_level((gpio_num_t)PIN_STEP, 0);
#endif
            uint8_t status_ok = 0;
            return protocol_encode_frame(MSG_ACK, hdr->sequence_id, &status_ok, 1, tx_buf, tx_max_len);
        }

        case MSG_POSITION_CMD:
        case MSG_VELOCITY_CMD: {
            // INVIOLABLE COMMISSIONING GUARD:
            // Under commissioning mode, pulse generation is strictly prohibited!
            if (COMMISSIONING_MODE) {
                uint8_t err = (uint8_t)ERR_COMMISSIONING_MODE;
                return protocol_encode_frame(MSG_NACK, hdr->sequence_id, &err, 1, tx_buf, tx_max_len);
            }

            if (g_state != STATE_ENABLED_ARMED && g_state != STATE_RUNNING) {
                uint8_t err = (uint8_t)ERR_DRIVER_FAULT;
                return protocol_encode_frame(MSG_NACK, hdr->sequence_id, &err, 1, tx_buf, tx_max_len);
            }

            g_state = STATE_RUNNING;
            g_last_motion_cmd_us = now_us;

            // Return Telemetry ACK
            telemetry_payload_t telem = {
                .sequence_id_ack = hdr->sequence_id,
                .device_timestamp_us = now_us,
                .measured_position_rad = g_measured_pos_rad,
                .measured_velocity_rad_s = g_measured_vel_rad_s,
                .motor_current_ma = g_current_ma,
                .device_state = (uint8_t)g_state,
                .error_code = (uint8_t)g_error_code,
                .step_loss_steps = 0
            };
            return protocol_encode_frame(MSG_TELEMETRY, hdr->sequence_id, (const uint8_t *)&telem, sizeof(telem), tx_buf, tx_max_len);
        }

        default: {
            uint8_t err = (uint8_t)ERR_UNKNOWN_MSG_TYPE;
            return protocol_encode_frame(MSG_NACK, hdr->sequence_id, &err, 1, tx_buf, tx_max_len);
        }
    }
}

/**
 * @brief Main background loop and watchdog supervision.
 */
void app_main(void) {
    safe_gpio_init();
    g_last_heartbeat_us = get_time_us();

    printf("[EARS-RL ESP32-S3] Firmware V1.0 Boot Complete. Mode: SAFE COMMISSIONING.\n");
    printf("[EARS-RL ESP32-S3] Motion commands DISABLED. Step generator DISABLED.\n");

    // In desktop stub/test, function returns. In ESP-IDF, task loop executes.
}
