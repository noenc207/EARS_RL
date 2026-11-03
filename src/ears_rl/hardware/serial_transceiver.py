"""Serial communication transceiver for ESP32-S3 actuator bench over USB UART.

Enforces binary framing, CRC-16 integrity, streaming packet reassembly,
and fail-safe timeout handling.
"""

from __future__ import annotations

import logging
import time
from typing import Any

from ears_rl.core.exceptions import ConnectionError, HardwareError
from ears_rl.core.interfaces import CommandTransport
from ears_rl.hardware.protocol import (
    PingPayload,
    PongPayload,
    PositionCommandPayload,
    ProtocolCodec,
    ProtocolErrorCode,
    ProtocolMessageType,
    StreamFrameParser,
)

logger = logging.getLogger(__name__)


class SerialBenchTransceiver(CommandTransport):
    """Serial communication client communicating with ESP32-S3 over USB UART.

    Implements CommandTransport interface contract.
    """

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        timeout_s: float = 2.0,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout_s = timeout_s
        self._serial: Any = None
        self._parser = StreamFrameParser()

    @property
    def is_connected(self) -> bool:
        """Check if serial connection is currently open."""
        return self._serial is not None and getattr(self._serial, "is_open", False)

    def connect(self) -> None:
        """Open serial port connection."""
        if self.is_connected:
            return

        try:
            import serial
        except ImportError as exc:
            raise HardwareError(
                "pyserial package is not installed. Install with 'pip install pyserial' "
                "or 'pip install -e .[hardware]' to enable physical serial communication."
            ) from exc

        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.1,  # Non-blocking / small chunk reads
            )
            # Flush existing junk or startup logs
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
            self._parser.reset()
            logger.info("Connected to ESP32-S3 on %s at %d baud", self.port, self.baudrate)
        except Exception as exc:
            raise ConnectionError(
                f"Failed to open serial port '{self.port}' at {self.baudrate} baud: {exc}"
            ) from exc

    def disconnect(self) -> None:
        """Close serial port safely."""
        if self._serial is not None:
            try:
                if getattr(self._serial, "is_open", False):
                    self._serial.close()
            except Exception as e:
                logger.warning("Error closing serial port: %e", e)
            finally:
                self._serial = None
                self._parser.reset()
                logger.info("Disconnected from %s", self.port)

    def __enter__(self) -> SerialBenchTransceiver:
        self.connect()
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        self.disconnect()

    def send_frame(
        self,
        msg_type: ProtocolMessageType,
        sequence_id: int,
        payload: bytes = b"",
    ) -> None:
        """Encode and transmit a binary frame over the serial link."""
        if not self.is_connected:
            raise ConnectionError(f"Cannot send frame: serial port '{self.port}' is not open.")

        frame = ProtocolCodec.encode_frame(msg_type, sequence_id, payload)
        self._serial.write(frame)
        self._serial.flush()

    def receive_frame(
        self,
        timeout_s: float | None = None,
    ) -> tuple[ProtocolMessageType, int, bytes]:
        """Read bytes and reassemble the next complete valid frame."""
        if not self.is_connected:
            raise ConnectionError(f"Cannot receive frame: serial port '{self.port}' is not open.")

        max_timeout = timeout_s if timeout_s is not None else self.timeout_s
        deadline = time.monotonic() + max_timeout

        while time.monotonic() < deadline:
            # Check if parser already has a parsed frame buffered
            frames = self._parser.feed(b"")
            if frames:
                return frames[0]

            # Read available bytes from UART
            waiting = getattr(self._serial, "in_waiting", 0)
            chunk_size = max(waiting, 1)
            raw = self._serial.read(chunk_size)
            if raw:
                frames = self._parser.feed(raw)
                if frames:
                    return frames[0]
            else:
                time.sleep(0.005)

        raise TimeoutError(f"Timed out waiting for response from ESP32-S3 after {max_timeout:.2f} s")


def run_commissioning_diagnostic(
    transceiver_or_mock: Any,
    is_mock: bool = False,
    timeout_s: float = 2.0,
) -> dict[str, Any]:
    """Execute complete safe commissioning sequence on transceiver (mock or physical).

    Sequence:
    1. Send PING with host timestamp T1.
    2. Receive PONG, compute RTT and clock offset.
    3. Send SET_MODE (arm = 1), verify ACK.
    4. Attempt sending POSITION_CMD to verify safe rejection in commissioning mode.
       Controller MUST return NACK with ERR_COMMISSIONING_MODE (9).
    5. Send SET_MODE (arm = 0) to disarm safely.
    6. Return structured diagnostic report.
    """
    report: dict[str, Any] = {
        "timestamp_iso": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "mode": "MOCK / SYNTHETIC" if is_mock else "PHYSICAL_SERIAL",
        "checks": {},
        "overall_status": "FAILED",
    }

    # Helper to send and receive frame from either mock or serial transceiver
    def _exchange(
        msg_type: ProtocolMessageType, seq: int, payload: bytes
    ) -> tuple[ProtocolMessageType, int, bytes]:
        if hasattr(transceiver_or_mock, "send_frame") and hasattr(transceiver_or_mock, "receive_frame"):
            transceiver_or_mock.send_frame(msg_type, seq, payload)
            return transceiver_or_mock.receive_frame(timeout_s=timeout_s)
        elif hasattr(transceiver_or_mock, "process_incoming_frame"):
            encoded_req = ProtocolCodec.encode_frame(msg_type, seq, payload)
            encoded_resp = transceiver_or_mock.process_incoming_frame(encoded_req)
            return ProtocolCodec.decode_frame(encoded_resp)
        else:
            raise TypeError(
                f"Object {transceiver_or_mock} does not implement CommandTransport or process_incoming_frame"
            )

    # 1. PING / PONG Handshake & Clock Sync
    t1_ns = time.monotonic_ns()
    ping_payload = PingPayload(host_timestamp_ns=t1_ns).serialize()
    resp_type, resp_seq, resp_payload = _exchange(ProtocolMessageType.PING, 1, ping_payload)
    t2_ns = time.monotonic_ns()

    if resp_type != ProtocolMessageType.PONG:
        report["checks"]["ping_pong"] = {
            "status": "FAIL",
            "reason": f"Expected PONG (0x02), received {resp_type.name} (0x{resp_type:02x})",
        }
        return report

    pong = PongPayload.deserialize(resp_payload)
    rtt_ns = t2_ns - pong.host_timestamp_echo_ns
    rtt_ms = rtt_ns / 1_000_000.0

    # Clock offset estimation
    estimated_host_midpoint_ns = pong.host_timestamp_echo_ns + (rtt_ns // 2)
    dev_time_ns = pong.device_timestamp_us * 1000
    clock_offset_ns = dev_time_ns - estimated_host_midpoint_ns
    clock_offset_ms = clock_offset_ns / 1_000_000.0

    report["checks"]["ping_pong"] = {
        "status": "PASS",
        "round_trip_time_ms": round(rtt_ms, 3),
        "device_timestamp_us": pong.device_timestamp_us,
        "estimated_clock_offset_ms": round(clock_offset_ms, 3),
    }

    # 2. Arm Transition (SET_MODE: 1)
    resp_type, resp_seq, resp_payload = _exchange(ProtocolMessageType.SET_MODE, 2, b"\x01")
    if resp_type != ProtocolMessageType.ACK:
        report["checks"]["arm_controller"] = {
            "status": "FAIL",
            "reason": f"Failed to arm controller: received {resp_type.name}",
        }
        return report
    report["checks"]["arm_controller"] = {"status": "PASS", "state": "ENABLED_ARMED"}

    # 3. SAFETY INVARIANT VERIFICATION: Motion Command Rejection in Commissioning Mode
    # Send Position Command while in commissioning mode
    pos_payload = PositionCommandPayload(
        target_position_rad=0.25,
        max_velocity_rad_s=0.5,
        max_acceleration_rad_s2=1.0,
    ).serialize()

    resp_type, resp_seq, resp_payload = _exchange(ProtocolMessageType.POSITION_CMD, 3, pos_payload)

    if resp_type != ProtocolMessageType.NACK:
        report["checks"]["motion_rejection_safety_invariant"] = {
            "status": "CRITICAL_FAIL",
            "reason": (
                "SAFETY VIOLATION: Commissioning mode did not reject motion command! "
                f"Received {resp_type.name} instead of NACK."
            ),
        }
        return report

    err_code = resp_payload[0] if resp_payload else -1
    if err_code != int(ProtocolErrorCode.ERR_COMMISSIONING_MODE):
        report["checks"]["motion_rejection_safety_invariant"] = {
            "status": "FAIL",
            "reason": (
                f"Expected error code ERR_COMMISSIONING_MODE ({int(ProtocolErrorCode.ERR_COMMISSIONING_MODE)}), "
                f"got code {err_code}."
            ),
        }
        return report

    report["checks"]["motion_rejection_safety_invariant"] = {
        "status": "PASS",
        "verified_error_code": "ERR_COMMISSIONING_MODE (9)",
        "message": "Motion command safely rejected; driver excitation confirmed disabled.",
    }

    # 4. Disarm Transition (SET_MODE: 0)
    resp_type, resp_seq, resp_payload = _exchange(ProtocolMessageType.SET_MODE, 4, b"\x00")
    if resp_type != ProtocolMessageType.ACK:
        report["checks"]["disarm_controller"] = {
            "status": "FAIL",
            "reason": f"Failed to disarm controller: received {resp_type.name}",
        }
        return report
    report["checks"]["disarm_controller"] = {"status": "PASS", "state": "IDLE_DISARMED"}

    report["overall_status"] = "PASSED"
    return report
