/**
 * @file protocol_defs.h
 * @brief EARS-RL Single-Axis Binary Protocol Definitions (Host PC <-> ESP32-S3).
 *
 * Defines binary frame layout, packed message structs, error codes, and state enums.
 * Little-endian byte order enforced across all multi-byte fields.
 */

#ifndef PROTOCOL_DEFS_H
#define PROTOCOL_DEFS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PROTOCOL_MAGIC_0        0xAA
#define PROTOCOL_MAGIC_1        0x55
#define PROTOCOL_VERSION        1
#define FRAME_HEADER_SIZE       8
#define CRC_SIZE                2
#define MAX_PAYLOAD_SIZE        256

/**
 * @brief Protocol Message Type Identifiers.
 */
typedef enum {
    MSG_PING         = 0x01,
    MSG_PONG         = 0x02,
    MSG_SET_MODE     = 0x03,
    MSG_POSITION_CMD = 0x04,
    MSG_VELOCITY_CMD = 0x05,
    MSG_ESTOP        = 0x08,
    MSG_RESET_FAULT  = 0x09,
    MSG_TELEMETRY    = 0x10,
    MSG_ACK          = 0x20,
    MSG_NACK         = 0x21
} protocol_msg_type_t;

/**
 * @brief Actuator Controller Operating Lifecycle States.
 */
typedef enum {
    STATE_UNINITIALIZED = 0,
    STATE_IDLE_DISARMED = 1,
    STATE_ENABLED_ARMED = 2,
    STATE_RUNNING       = 3,
    STATE_FAULT         = 4,
    STATE_ESTOP         = 5
} device_state_t;

/**
 * @brief Standardized Protocol Error Codes.
 */
typedef enum {
    ERR_NONE                   = 0,
    ERR_CRC_MISMATCH           = 1,
    ERR_UNKNOWN_MSG_TYPE       = 2,
    ERR_WATCHDOG_TIMEOUT       = 3,
    ERR_COMMAND_OUT_OF_BOUNDS  = 4,
    ERR_DRIVER_FAULT           = 5,
    ERR_STEP_LOSS_DETECTED     = 6,
    ERR_ESTOP_ACTIVE           = 7,
    ERR_INVALID_SEQUENCE       = 8,
    ERR_COMMISSIONING_MODE     = 9
} protocol_error_code_t;

#pragma pack(push, 1)

/**
 * @brief Frame Header Structure (8 bytes).
 */
typedef struct {
    uint8_t  magic[2];       ///< 0xAA, 0x55
    uint8_t  version;        ///< 0x01
    uint8_t  msg_type;       ///< protocol_msg_type_t
    uint16_t sequence_id;    ///< Sequence counter (Little-Endian)
    uint16_t payload_len;    ///< Payload byte count (Little-Endian)
} frame_header_t;

/**
 * @brief Ping Payload (8 bytes).
 */
typedef struct {
    uint64_t host_timestamp_ns;  ///< Host PC monotonic timestamp [ns]
} ping_payload_t;

/**
 * @brief Pong Payload (16 bytes).
 */
typedef struct {
    uint64_t host_timestamp_echo_ns; ///< Echo of host timestamp [ns]
    uint64_t device_timestamp_us;    ///< Controller local microsecond timer [us]
} pong_payload_t;

/**
 * @brief Position Command Payload (12 bytes).
 */
typedef struct {
    float target_position_rad;     ///< Target angle [rad]
    float max_velocity_rad_s;      ///< Velocity limit [rad/s]
    float max_acceleration_rad_s2;  ///< Acceleration limit [rad/s^2]
} position_command_payload_t;

/**
 * @brief Telemetry Payload (24 bytes).
 */
typedef struct {
    uint16_t sequence_id_ack;        ///< Sequence ID acknowledged
    uint64_t device_timestamp_us;    ///< Microcontroller timestamp [us]
    float    measured_position_rad;  ///< Encoder shaft position [rad]
    float    measured_velocity_rad_s;///< Estimated joint velocity [rad/s]
    uint16_t motor_current_ma;       ///< Motor current [mA]
    uint8_t  device_state;           ///< device_state_t
    uint8_t  error_code;             ///< protocol_error_code_t
    int16_t  step_loss_steps;        ///< Verified step loss count
} telemetry_payload_t;

#pragma pack(pop)

#ifdef __cplusplus
}
#endif

#endif // PROTOCOL_DEFS_H
