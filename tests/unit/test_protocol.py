"""Unit tests for single-axis bench binary communication protocol."""

import pytest

from ears_rl.hardware.protocol import (
    DeviceState,
    MockBenchTransceiver,
    PingPayload,
    PongPayload,
    PositionCommandPayload,
    ProtocolCodec,
    ProtocolErrorCode,
    ProtocolMessageType,
    StreamFrameParser,
    compute_crc16_ccitt,
)


def test_crc16_computation_and_verification():
    """Verify CRC-16-CCITT correctly detects bit errors."""
    data = b"Hello, EARS-RL Single-Axis Protocol!"
    crc = compute_crc16_ccitt(data)
    assert isinstance(crc, int)
    assert 0 <= crc <= 0xFFFF

    # Identical data yields identical CRC
    assert compute_crc16_ccitt(data) == crc

    # Single bit corruption changes CRC
    corrupted_data = b"Hello, EARS-RL Single-Axis Protocol?"
    assert compute_crc16_ccitt(corrupted_data) != crc


def test_protocol_encode_decode_roundtrip():
    """Verify frame encoding, serialization, CRC appending, and valid decoding."""
    pos_payload = PositionCommandPayload(
        target_position_rad=1.5708,
        max_velocity_rad_s=3.1415,
        max_acceleration_rad_s2=15.0,
    )
    payload_bytes = pos_payload.serialize()
    frame = ProtocolCodec.encode_frame(ProtocolMessageType.POSITION_CMD, sequence_id=42, payload=payload_bytes)

    # Decode valid frame
    msg_type, seq_id, decoded_payload = ProtocolCodec.decode_frame(frame)
    assert msg_type == ProtocolMessageType.POSITION_CMD
    assert seq_id == 42

    deserialized = PositionCommandPayload.deserialize(decoded_payload)
    assert pytest.approx(deserialized.target_position_rad, rel=1e-4) == 1.5708
    assert pytest.approx(deserialized.max_velocity_rad_s, rel=1e-4) == 3.1415


def test_protocol_rejects_corrupted_crc():
    """Verify frame decoder rejects frame when CRC is corrupted."""
    pos_payload = PositionCommandPayload(target_position_rad=0.5)
    frame = bytearray(ProtocolCodec.encode_frame(ProtocolMessageType.POSITION_CMD, 1, pos_payload.serialize()))

    # Corrupt last byte (CRC)
    frame[-1] ^= 0xFF

    with pytest.raises(ValueError, match="CRC mismatch"):
        ProtocolCodec.decode_frame(bytes(frame))


def test_protocol_rejects_invalid_magic():
    """Verify frame decoder rejects frame with incorrect header magic."""
    pos_payload = PositionCommandPayload(target_position_rad=0.5)
    frame = bytearray(ProtocolCodec.encode_frame(ProtocolMessageType.POSITION_CMD, 1, pos_payload.serialize()))

    # Corrupt magic byte
    frame[0] = 0x00

    with pytest.raises(ValueError, match="Invalid frame magic"):
        ProtocolCodec.decode_frame(bytes(frame))


def test_clock_synchronization_ping_pong_rtt():
    """Verify Host <-> ESP32 ping/pong round-trip enables RTT measurement and clock offset estimation."""
    transceiver = MockBenchTransceiver(start_us=5_000_000)

    host_send_ns = 1_000_000_000  # 1.0 s
    ping = PingPayload(host_timestamp_ns=host_send_ns)
    ping_frame = ProtocolCodec.encode_frame(ProtocolMessageType.PING, 1, ping.serialize())

    # Advance time to simulate transport delay (e.g. 2 ms = 2000 us)
    transceiver.advance_time(2000)
    pong_frame = transceiver.process_incoming_frame(ping_frame)

    # Host receives pong at 1.004 s (4 ms round-trip)
    host_recv_ns = 1_004_000_000

    msg_type, seq_id, pong_payload_bytes = ProtocolCodec.decode_frame(pong_frame)
    assert msg_type == ProtocolMessageType.PONG
    pong = PongPayload.deserialize(pong_payload_bytes)

    # 1. Measured RTT
    rtt_ns = host_recv_ns - pong.host_timestamp_echo_ns
    rtt_ms = rtt_ns / 1e6
    assert pytest.approx(rtt_ms, rel=1e-3) == 4.0

    # 2. Clock offset estimation
    estimated_host_midpoint_ns = pong.host_timestamp_echo_ns + (rtt_ns // 2)
    dev_time_ns = pong.device_timestamp_us * 1000
    clock_offset_ns = dev_time_ns - estimated_host_midpoint_ns
    assert isinstance(clock_offset_ns, int)


def test_watchdog_timeout_triggers_fault_and_rejects_motion():
    """Verify that watchdog timeout transitions controller to FAULT, blocking commands until reset."""
    transceiver = MockBenchTransceiver(start_us=1_000_000)

    # 1. Arm controller
    set_mode_frame = ProtocolCodec.encode_frame(ProtocolMessageType.SET_MODE, 1, b"\x01")
    ack_frame = transceiver.process_incoming_frame(set_mode_frame)
    msg_type, _, _ = ProtocolCodec.decode_frame(ack_frame)
    assert msg_type == ProtocolMessageType.ACK
    assert transceiver.state == DeviceState.ENABLED_ARMED

    # 2. Advance time past watchdog timeout (500 ms = 500,000 us) without heartbeat
    transceiver.advance_time(600_000)
    assert transceiver.state == DeviceState.FAULT
    assert transceiver.error_code == ProtocolErrorCode.ERR_WATCHDOG_TIMEOUT

    # 3. Position command during FAULT must be rejected with NACK
    pos_cmd = PositionCommandPayload(target_position_rad=1.0)
    cmd_frame = ProtocolCodec.encode_frame(ProtocolMessageType.POSITION_CMD, 2, pos_cmd.serialize())
    resp_frame = transceiver.process_incoming_frame(cmd_frame)
    msg_type, _, payload = ProtocolCodec.decode_frame(resp_frame)
    assert msg_type == ProtocolMessageType.NACK

    # 4. Attempting to arm directly without explicit fault reset must fail
    arm_again = transceiver.process_incoming_frame(set_mode_frame)
    msg_type, _, _ = ProtocolCodec.decode_frame(arm_again)
    assert msg_type == ProtocolMessageType.NACK
    assert transceiver.state == DeviceState.FAULT

    # 5. Explicit fault reset restores system to IDLE_DISARMED
    reset_frame = ProtocolCodec.encode_frame(ProtocolMessageType.RESET_FAULT, 3, b"\x00")
    reset_ack = transceiver.process_incoming_frame(reset_frame)
    msg_type, _, _ = ProtocolCodec.decode_frame(reset_ack)
    assert msg_type == ProtocolMessageType.ACK
    assert transceiver.state == DeviceState.IDLE_DISARMED
    assert transceiver.error_code == ProtocolErrorCode.ERR_NONE


def test_stream_frame_parser_fragmentation_and_noise():
    """Verify StreamFrameParser handles arbitrary byte fragmentation and junk prefix bytes."""
    ping = PingPayload(host_timestamp_ns=1_234_567_890)
    frame = ProtocolCodec.encode_frame(ProtocolMessageType.PING, sequence_id=99, payload=ping.serialize())

    parser = StreamFrameParser()

    # Prepend 3 bytes of garbage noise before the valid frame
    dirty_stream = b"\xFF\x00\xAA" + frame

    # Feed in 4 small fragments simulating UART chunks
    f1 = dirty_stream[:5]
    f2 = dirty_stream[5:11]
    f3 = dirty_stream[11:15]
    f4 = dirty_stream[15:]

    res1 = parser.feed(f1)
    assert len(res1) == 0  # Incomplete

    res2 = parser.feed(f2)
    assert len(res2) == 0  # Incomplete

    res3 = parser.feed(f3)
    assert len(res3) == 0  # Incomplete

    res4 = parser.feed(f4)
    assert len(res4) == 1  # Completed!

    msg_type, seq_id, payload = res4[0]
    assert msg_type == ProtocolMessageType.PING
    assert seq_id == 99
    decoded_ping = PingPayload.deserialize(payload)
    assert decoded_ping.host_timestamp_ns == 1_234_567_890


def test_stream_frame_parser_concatenated_frames():
    """Verify parser extracts multiple concatenated frames received in a single buffer read."""
    parser = StreamFrameParser()

    f1 = ProtocolCodec.encode_frame(ProtocolMessageType.SET_MODE, 1, b"\x01")
    f2 = ProtocolCodec.encode_frame(ProtocolMessageType.ESTOP, 2, b"")
    f3 = ProtocolCodec.encode_frame(ProtocolMessageType.RESET_FAULT, 3, b"\x00")

    extracted = parser.feed(f1 + f2 + f3)
    assert len(extracted) == 3
    assert extracted[0][0] == ProtocolMessageType.SET_MODE
    assert extracted[1][0] == ProtocolMessageType.ESTOP
    assert extracted[2][0] == ProtocolMessageType.RESET_FAULT


def test_canonical_protocol_test_vectors():
    """Verify bitwise agreement against canonical PROTOCOL_TEST_VECTORS for cross-language validation."""
    from ears_rl.hardware.protocol import PROTOCOL_TEST_VECTORS

    assert len(PROTOCOL_TEST_VECTORS) >= 4
    for vec in PROTOCOL_TEST_VECTORS:
        encoded = ProtocolCodec.encode_frame(vec["msg_type"], vec["sequence_id"], vec["payload"])
        assert encoded.hex() == vec["expected_hex"]

        # Decode back
        msg_type, seq, payload = ProtocolCodec.decode_frame(bytes.fromhex(vec["expected_hex"]))
        assert msg_type == vec["msg_type"]
        assert seq == vec["sequence_id"]
        assert payload == vec["payload"]


def test_commissioning_mode_strictly_rejects_motion():
    """SAFETY INVARIANT: In commissioning mode, any motion command is rejected with ERR_COMMISSIONING_MODE."""
    transceiver = MockBenchTransceiver(commissioning_mode=True)

    # Arm device in commissioning mode
    arm_frame = ProtocolCodec.encode_frame(ProtocolMessageType.SET_MODE, 1, b"\x01")
    ack_frame = transceiver.process_incoming_frame(arm_frame)
    msg_type, _, _ = ProtocolCodec.decode_frame(ack_frame)
    assert msg_type == ProtocolMessageType.ACK
    assert transceiver.state == DeviceState.ENABLED_ARMED

    # Attempt to send position motion command
    pos_cmd = PositionCommandPayload(target_position_rad=1.0)
    cmd_frame = ProtocolCodec.encode_frame(ProtocolMessageType.POSITION_CMD, 2, pos_cmd.serialize())
    resp = transceiver.process_incoming_frame(cmd_frame)

    msg_type, _, err_payload = ProtocolCodec.decode_frame(resp)
    assert msg_type == ProtocolMessageType.NACK
    assert err_payload[0] == int(ProtocolErrorCode.ERR_COMMISSIONING_MODE)
    # State must NOT transition to RUNNING!
    assert transceiver.state == DeviceState.ENABLED_ARMED
    assert transceiver.target_position_rad == 0.0


def test_dual_watchdog_motion_separation():
    """Verify motion watchdog (50 ms) triggers FAULT if motion setpoints stall, even if comm is active."""
    # Transceiver with commissioning_mode=False for testing motor loop watchdog
    transceiver = MockBenchTransceiver(commissioning_mode=False, motion_watchdog_timeout_us=50_000)

    # Arm and send motion command
    transceiver.process_incoming_frame(ProtocolCodec.encode_frame(ProtocolMessageType.SET_MODE, 1, b"\x01"))
    pos_cmd = PositionCommandPayload(target_position_rad=0.5)
    transceiver.process_incoming_frame(ProtocolCodec.encode_frame(ProtocolMessageType.POSITION_CMD, 2, pos_cmd.serialize()))
    assert transceiver.state == DeviceState.RUNNING

    # Send heartbeat PING at 40 ms (comm is alive)
    transceiver.advance_time(40_000)
    transceiver.process_incoming_frame(ProtocolCodec.encode_frame(ProtocolMessageType.PING, 3, PingPayload(1).serialize()))
    assert transceiver.state == DeviceState.RUNNING

    # But no motion command arrives for another 20 ms (total elapsed since motion cmd = 60 ms > 50 ms)
    transceiver.advance_time(20_000)
    assert transceiver.state == DeviceState.FAULT
    assert transceiver.error_code == ProtocolErrorCode.ERR_WATCHDOG_TIMEOUT

