"""Single-Axis Actuator Bench Binary Communication Protocol (Host PC <-> ESP32-S3).

Specifies frame framing, binary serialization, CRC-16 integrity verification,
clock domain synchronization, device state transitions, and telemetry payloads.
Operates strictly in simulation/mock mode during automated testing.
"""

from __future__ import annotations

import struct
from collections import deque
from dataclasses import dataclass
from enum import IntEnum
from typing import Any

from ears_rl.core.interfaces import CommandTransport

# Protocol Framing Constants
PROTOCOL_MAGIC: bytes = b"\xAA\x55"
PROTOCOL_VERSION: int = 1
FRAME_HEADER_SIZE: int = 8  # Magic (2B) + Ver (1B) + Type (1B) + Seq (2B) + Len (2B)
CRC_SIZE: int = 2
MAX_PAYLOAD_SIZE: int = 256  # Maximum permitted payload length in bytes


class ProtocolMessageType(IntEnum):
    """Message identifiers for Host <-> Controller communication."""
    PING = 0x01
    PONG = 0x02
    SET_MODE = 0x03
    POSITION_CMD = 0x04
    VELOCITY_CMD = 0x05
    ESTOP = 0x08
    RESET_FAULT = 0x09
    TELEMETRY = 0x10
    ACK = 0x20
    NACK = 0x21


class DeviceState(IntEnum):
    """Operating lifecycle states of actuator controller firmware."""
    UNINITIALIZED = 0
    IDLE_DISARMED = 1
    ENABLED_ARMED = 2
    RUNNING = 3
    FAULT = 4
    ESTOP_TRIGGERED = 5


class ProtocolErrorCode(IntEnum):
    """Standardized error codes returned by controller firmware."""
    ERR_NONE = 0
    ERR_CRC_MISMATCH = 1
    ERR_UNKNOWN_MSG_TYPE = 2
    ERR_WATCHDOG_TIMEOUT = 3
    ERR_COMMAND_OUT_OF_BOUNDS = 4
    ERR_DRIVER_FAULT = 5
    ERR_STEP_LOSS_DETECTED = 6
    ERR_ESTOP_ACTIVE = 7
    ERR_INVALID_SEQUENCE = 8
    ERR_COMMISSIONING_MODE = 9  # Motion rejected because safe commissioning mode is active


def compute_crc16_ccitt(data: bytes) -> int:
    """Compute CRC-16-CCITT (polynomial 0x1021, init 0xFFFF)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


@dataclass
class PositionCommandPayload:
    """Position command dispatched from Host PC to Controller."""
    target_position_rad: float
    max_velocity_rad_s: float = 6.28
    max_acceleration_rad_s2: float = 30.0

    def serialize(self) -> bytes:
        # 3 floats (32-bit single precision IEEE 754) = 12 bytes
        return struct.pack("<fff", self.target_position_rad, self.max_velocity_rad_s, self.max_acceleration_rad_s2)

    @classmethod
    def deserialize(cls, data: bytes) -> PositionCommandPayload:
        if len(data) < 12:
            raise ValueError(f"Payload too short for PositionCommand: {len(data)} bytes, expected 12")
        pos, vel, acc = struct.unpack("<fff", data[:12])
        return cls(target_position_rad=pos, max_velocity_rad_s=vel, max_acceleration_rad_s2=acc)


@dataclass
class TelemetryPayload:
    """Telemetry report streamed from Controller to Host PC."""
    sequence_id_ack: int
    device_timestamp_us: int
    measured_position_rad: float
    measured_velocity_rad_s: float
    motor_current_ma: int
    device_state: DeviceState
    error_code: ProtocolErrorCode
    step_loss_steps: int = 0

    def serialize(self) -> bytes:
        # H (2B seq), Q (8B dev_us), f (4B pos), f (4B vel), H (2B current_ma), B (1B state), B (1B error), h (2B slip) = 24 bytes
        return struct.pack(
            "<HQffHBBh",
            self.sequence_id_ack,
            self.device_timestamp_us,
            self.measured_position_rad,
            self.measured_velocity_rad_s,
            self.motor_current_ma,
            int(self.device_state),
            int(self.error_code),
            self.step_loss_steps,
        )

    @classmethod
    def deserialize(cls, data: bytes) -> TelemetryPayload:
        if len(data) < 24:
            raise ValueError(f"Payload too short for Telemetry: {len(data)} bytes, expected 24")
        seq, dev_us, pos, vel, curr, state, err, slip = struct.unpack("<HQffHBBh", data[:24])
        return cls(
            sequence_id_ack=seq,
            device_timestamp_us=dev_us,
            measured_position_rad=pos,
            measured_velocity_rad_s=vel,
            motor_current_ma=curr,
            device_state=DeviceState(state),
            error_code=ProtocolErrorCode(err),
            step_loss_steps=slip,
        )


@dataclass
class PingPayload:
    """Heartbeat ping dispatched from Host PC to Controller."""
    host_timestamp_ns: int

    def serialize(self) -> bytes:
        return struct.pack("<Q", self.host_timestamp_ns)

    @classmethod
    def deserialize(cls, data: bytes) -> PingPayload:
        if len(data) < 8:
            raise ValueError(f"Payload too short for Ping: {len(data)} bytes, expected 8")
        (ts,) = struct.unpack("<Q", data[:8])
        return cls(host_timestamp_ns=ts)


@dataclass
class PongPayload:
    """Heartbeat pong returned by Controller for RTT and clock sync."""
    host_timestamp_echo_ns: int
    device_timestamp_us: int

    def serialize(self) -> bytes:
        return struct.pack("<QQ", self.host_timestamp_echo_ns, self.device_timestamp_us)

    @classmethod
    def deserialize(cls, data: bytes) -> PongPayload:
        if len(data) < 16:
            raise ValueError(f"Payload too short for Pong: {len(data)} bytes, expected 16")
        host_ns, dev_us = struct.unpack("<QQ", data[:16])
        return cls(host_timestamp_echo_ns=host_ns, device_timestamp_us=dev_us)


class ProtocolCodec:
    """Encodes and decodes binary frames for the EARS-RL single-axis bench."""

    @staticmethod
    def encode_frame(msg_type: ProtocolMessageType, sequence_id: int, payload: bytes) -> bytes:
        """Construct frame with Header, Payload, and CRC-16-CCITT."""
        payload_len = len(payload)
        header = PROTOCOL_MAGIC + struct.pack(
            "<BBHH",
            PROTOCOL_VERSION,
            int(msg_type),
            sequence_id & 0xFFFF,
            payload_len,
        )
        body = header + payload
        crc = compute_crc16_ccitt(body)
        return body + struct.pack("<H", crc)

    @staticmethod
    def decode_frame(raw_data: bytes) -> tuple[ProtocolMessageType, int, bytes]:
        """Validate and decode a binary frame.

        Returns:
            (msg_type, sequence_id, payload_bytes)

        Raises:
            ValueError: If frame magic, CRC, length, or version is invalid.
        """
        if len(raw_data) < FRAME_HEADER_SIZE + CRC_SIZE:
            raise ValueError(f"Frame length {len(raw_data)} is less than minimum size")

        magic = raw_data[:2]
        if magic != PROTOCOL_MAGIC:
            raise ValueError(f"Invalid frame magic: {magic.hex()} (expected {PROTOCOL_MAGIC.hex()})")

        version, msg_type_val, seq_id, payload_len = struct.unpack("<BBHH", raw_data[2:8])
        if version != PROTOCOL_VERSION:
            raise ValueError(f"Protocol version mismatch: {version} (expected {PROTOCOL_VERSION})")

        if payload_len > MAX_PAYLOAD_SIZE:
            raise ValueError(f"Payload length {payload_len} exceeds maximum allowed {MAX_PAYLOAD_SIZE}")

        expected_total_len = FRAME_HEADER_SIZE + payload_len + CRC_SIZE
        if len(raw_data) < expected_total_len:
            raise ValueError(f"Frame incomplete: got {len(raw_data)} bytes, expected {expected_total_len}")

        body = raw_data[: FRAME_HEADER_SIZE + payload_len]
        received_crc = struct.unpack("<H", raw_data[FRAME_HEADER_SIZE + payload_len : expected_total_len])[0]
        calculated_crc = compute_crc16_ccitt(body)

        if received_crc != calculated_crc:
            raise ValueError(f"CRC mismatch: received 0x{received_crc:04X}, calculated 0x{calculated_crc:04X}")

        msg_type = ProtocolMessageType(msg_type_val)
        payload = raw_data[FRAME_HEADER_SIZE : FRAME_HEADER_SIZE + payload_len]
        return msg_type, seq_id, payload


class StreamFrameParser:
    """Accumulates incoming serial byte stream and extracts complete validated frames.

    Handles fragmentation, concatenated frames, noise bytes preceding magic,
    oversized payloads, and CRC validation.
    """

    def __init__(self, max_payload_len: int = MAX_PAYLOAD_SIZE) -> None:
        self._buffer = bytearray()
        self._max_payload_len = max_payload_len

    def feed(self, chunk: bytes) -> list[tuple[ProtocolMessageType, int, bytes]]:
        """Append incoming bytes and extract all complete, validated frames."""
        self._buffer.extend(chunk)
        frames: list[tuple[ProtocolMessageType, int, bytes]] = []

        while len(self._buffer) >= FRAME_HEADER_SIZE + CRC_SIZE:
            # 1. Search for frame magic (0xAA 0x55)
            idx = self._buffer.find(PROTOCOL_MAGIC)
            if idx == -1:
                # Keep last byte in case of split magic (e.g. 0xAA at buffer end)
                if len(self._buffer) > 0 and self._buffer[-1] == PROTOCOL_MAGIC[0]:
                    self._buffer = self._buffer[-1:]
                else:
                    self._buffer.clear()
                break

            if idx > 0:
                # Discard leading noise bytes before magic marker
                del self._buffer[:idx]

            if len(self._buffer) < FRAME_HEADER_SIZE:
                # Incomplete header, wait for next serial read
                break

            version, _type_val, _seq_id, payload_len = struct.unpack("<BBHH", self._buffer[2:8])
            if version != PROTOCOL_VERSION or payload_len > self._max_payload_len:
                # Corrupt header or oversized payload: discard first magic byte and rescan
                del self._buffer[:1]
                continue

            total_len = FRAME_HEADER_SIZE + payload_len + CRC_SIZE
            if len(self._buffer) < total_len:
                # Frame body not yet complete in buffer
                break

            frame_candidate = bytes(self._buffer[:total_len])
            del self._buffer[:total_len]

            try:
                msg_type, seq, payload = ProtocolCodec.decode_frame(frame_candidate)
                frames.append((msg_type, seq, payload))
            except ValueError:
                # Corrupt payload or CRC failure, continue scanning
                continue

        return frames


class MockBenchTransceiver(CommandTransport):
    """Software loopback mock of the ESP32-S3 firmware for offline testing.

    Simulates firmware state machine, dual watchdogs (communication vs motion),
    safe commissioning mode, CRC validation, and telemetry generation without hardware.
    Conforms to CommandTransport interface contract.
    """

    def __init__(
        self,
        start_us: int = 1_000_000,
        commissioning_mode: bool = True,
        comm_watchdog_timeout_us: int = 500_000,   # 500 ms serial connection watchdog
        motion_watchdog_timeout_us: int = 50_000,   # 50 ms motion trajectory watchdog
    ) -> None:
        self.state: DeviceState = DeviceState.IDLE_DISARMED
        self.device_time_us: int = start_us
        self.last_heartbeat_us: int = start_us
        self.last_motion_cmd_us: int = start_us
        self.commissioning_mode: bool = commissioning_mode
        self.comm_watchdog_timeout_us: int = comm_watchdog_timeout_us
        self.motion_watchdog_timeout_us: int = motion_watchdog_timeout_us
        self.target_position_rad: float = 0.0
        self.measured_position_rad: float = 0.0
        self.measured_velocity_rad_s: float = 0.0
        self.last_seq_id: int = 0
        self.error_code: ProtocolErrorCode = ProtocolErrorCode.ERR_NONE
        self._is_connected: bool = True
        self._response_queue: deque[tuple[Any, int, bytes]] = deque()

    @property
    def is_connected(self) -> bool:
        """Check whether mock transport is connected."""
        return self._is_connected

    def connect(self) -> None:
        """Connect mock transport."""
        self._is_connected = True

    def disconnect(self) -> None:
        """Disconnect mock transport and clear buffers."""
        self._is_connected = False
        self._response_queue.clear()

    def send_frame(
        self,
        msg_type: Any,
        sequence_id: int,
        payload: bytes = b"",
    ) -> None:
        """Encode, process loopback frame, and enqueue response."""
        frame_bytes = ProtocolCodec.encode_frame(msg_type, sequence_id, payload)
        resp_bytes = self.process_incoming_frame(frame_bytes)
        resp_type, resp_seq, resp_payload = ProtocolCodec.decode_frame(resp_bytes)
        self._response_queue.append((resp_type, resp_seq, resp_payload))

    def receive_frame(
        self, timeout_s: float | None = None
    ) -> tuple[Any, int, bytes]:
        """Pop the oldest queued response frame."""
        if not self._response_queue:
            raise TimeoutError("MockBenchTransceiver response queue is empty")
        return self._response_queue.popleft()

    def advance_time(self, delta_us: int) -> None:
        """Advance device microsecond clock and check both watchdogs."""
        self.device_time_us += delta_us

        # Communication watchdog: checks freshness of any serial packet
        if self.state in (DeviceState.ENABLED_ARMED, DeviceState.RUNNING):
            if self.device_time_us - self.last_heartbeat_us > self.comm_watchdog_timeout_us:
                self.state = DeviceState.FAULT
                self.error_code = ProtocolErrorCode.ERR_WATCHDOG_TIMEOUT

        # Motion watchdog: checks freshness of continuous motion commands
        if self.state == DeviceState.RUNNING:
            if self.device_time_us - self.last_motion_cmd_us > self.motion_watchdog_timeout_us:
                self.state = DeviceState.FAULT
                self.error_code = ProtocolErrorCode.ERR_WATCHDOG_TIMEOUT

    def process_incoming_frame(self, frame_bytes: bytes) -> bytes:
        """Process incoming frame and generate response frame."""
        try:
            msg_type, seq_id, payload = ProtocolCodec.decode_frame(frame_bytes)
        except ValueError:
            # Send NACK on CRC or parsing error
            nack_payload = struct.pack("<BB", int(ProtocolErrorCode.ERR_CRC_MISMATCH), 0)
            return ProtocolCodec.encode_frame(ProtocolMessageType.NACK, 0, nack_payload)

        # Heartbeat freshness
        self.last_heartbeat_us = self.device_time_us
        self.last_seq_id = seq_id

        if msg_type == ProtocolMessageType.PING:
            ping = PingPayload.deserialize(payload)
            pong = PongPayload(host_timestamp_echo_ns=ping.host_timestamp_ns, device_timestamp_us=self.device_time_us)
            return ProtocolCodec.encode_frame(ProtocolMessageType.PONG, seq_id, pong.serialize())

        elif msg_type == ProtocolMessageType.SET_MODE:
            # 1B enable flag (1 = arm, 0 = disarm)
            enable = payload[0] == 1 if payload else False
            if self.state == DeviceState.FAULT or self.state == DeviceState.ESTOP_TRIGGERED:
                # Cannot arm while in fault without explicit reset
                return ProtocolCodec.encode_frame(
                    ProtocolMessageType.NACK, seq_id, bytes([int(self.error_code)])
                )
            self.state = DeviceState.ENABLED_ARMED if enable else DeviceState.IDLE_DISARMED
            return ProtocolCodec.encode_frame(ProtocolMessageType.ACK, seq_id, b"\x00")

        elif msg_type == ProtocolMessageType.RESET_FAULT:
            self.state = DeviceState.IDLE_DISARMED
            self.error_code = ProtocolErrorCode.ERR_NONE
            return ProtocolCodec.encode_frame(ProtocolMessageType.ACK, seq_id, b"\x00")

        elif msg_type == ProtocolMessageType.ESTOP:
            self.state = DeviceState.ESTOP_TRIGGERED
            self.error_code = ProtocolErrorCode.ERR_ESTOP_ACTIVE
            return ProtocolCodec.encode_frame(ProtocolMessageType.ACK, seq_id, b"\x00")

        elif msg_type in (ProtocolMessageType.POSITION_CMD, ProtocolMessageType.VELOCITY_CMD):
            # SAFE COMMISSIONING INVARIANT:
            # Motion commands are unconditionally rejected in commissioning mode
            if self.commissioning_mode:
                return ProtocolCodec.encode_frame(
                    ProtocolMessageType.NACK, seq_id, bytes([int(ProtocolErrorCode.ERR_COMMISSIONING_MODE)])
                )

            if self.state != DeviceState.ENABLED_ARMED and self.state != DeviceState.RUNNING:
                return ProtocolCodec.encode_frame(
                    ProtocolMessageType.NACK, seq_id, bytes([int(ProtocolErrorCode.ERR_DRIVER_FAULT)])
                )
            cmd = PositionCommandPayload.deserialize(payload)
            self.target_position_rad = cmd.target_position_rad
            self.measured_position_rad = cmd.target_position_rad
            self.last_motion_cmd_us = self.device_time_us
            self.state = DeviceState.RUNNING

            # Return immediate telemetry ACK
            telem = TelemetryPayload(
                sequence_id_ack=seq_id,
                device_timestamp_us=self.device_time_us,
                measured_position_rad=self.measured_position_rad,
                measured_velocity_rad_s=self.measured_velocity_rad_s,
                motor_current_ma=450,
                device_state=self.state,
                error_code=self.error_code,
            )
            return ProtocolCodec.encode_frame(ProtocolMessageType.TELEMETRY, seq_id, telem.serialize())

        else:
            return ProtocolCodec.encode_frame(
                ProtocolMessageType.NACK, seq_id, bytes([int(ProtocolErrorCode.ERR_UNKNOWN_MSG_TYPE)])
            )


# Canonical test vectors shared between Python and C/C++ firmware
PROTOCOL_TEST_VECTORS: list[dict[str, Any]] = [
    {
        "name": "PING_SEQ_1",
        "msg_type": ProtocolMessageType.PING,
        "sequence_id": 1,
        "payload": PingPayload(host_timestamp_ns=1_000_000_000).serialize(),
        # 0xAA 0x55 (magic) + 0x01 (ver) + 0x01 (ping) + 0x0001 (seq) + 0x0008 (len) + 8B ts + 2B crc
        "expected_hex": ProtocolCodec.encode_frame(
            ProtocolMessageType.PING, 1, PingPayload(host_timestamp_ns=1_000_000_000).serialize()
        ).hex(),
    },
    {
        "name": "SET_MODE_ARM_SEQ_2",
        "msg_type": ProtocolMessageType.SET_MODE,
        "sequence_id": 2,
        "payload": b"\x01",
        "expected_hex": ProtocolCodec.encode_frame(ProtocolMessageType.SET_MODE, 2, b"\x01").hex(),
    },
    {
        "name": "ESTOP_SEQ_10",
        "msg_type": ProtocolMessageType.ESTOP,
        "sequence_id": 10,
        "payload": b"",
        "expected_hex": ProtocolCodec.encode_frame(ProtocolMessageType.ESTOP, 10, b"").hex(),
    },
    {
        "name": "RESET_FAULT_SEQ_11",
        "msg_type": ProtocolMessageType.RESET_FAULT,
        "sequence_id": 11,
        "payload": b"\x00",
        "expected_hex": ProtocolCodec.encode_frame(ProtocolMessageType.RESET_FAULT, 11, b"\x00").hex(),
    },
]
