"""Safe hardware commissioning workflow.

Executes read-only diagnostic handshake, validates round-trip timing,
verifies firmware commissioning rejection invariant, and generates structured reports.
Supports fully configurable commissioning gates and hardware acceptance criteria.
"""

from __future__ import annotations

import logging
from dataclasses import asdict, dataclass, field
from typing import Any

from ears_rl.core.interfaces import CommandTransport
from ears_rl.hardware.serial_transceiver import run_commissioning_diagnostic

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class CommissioningGateConfig:
    """Configurable acceptance thresholds for hardware commissioning gates.

    TERMINOLOGY & SAFETY DISTINCTIONS:
    - unpowered_vm_to_gnd_min_resistance_kohm: Unpowered DC resistance between VM socket pin and GND pin
      measured with bench supply DISCONNECTED and capacitors fully DISCHARGED. Purpose: verifies absence of
      dead short, solder bridges, or blown MOSFET bridge before applying power.
      CRITICAL: This is a DC rail-to-ground non-short test; it is NOT a galvanic isolation barrier measurement.
    - ground_continuity_max_ohm: Measured DC resistance between MCU digital GND and driver power GND pin.
      CRITICAL: DC continuity (R <= 0.2 Ohm) verifies static star GND connectivity but does NOT prove low
      impedance or absence of ground bounce at PWM switching frequencies (20-40 kHz).
    """

    # Gate B electrical thresholds
    unpowered_vm_to_gnd_min_resistance_kohm: float = 10.0
    isolation_resistance_min_kohm: float | None = None  # Deprecated backwards-compatible alias
    ground_continuity_max_ohm: float = 0.2
    vref_tolerance_v: float = 0.02
    max_unloaded_quiescent_current_a: float = 0.05
    initial_ocp_limit_a: float = 0.4
    bulk_capacitor_min_uf: float = 100.0

    # Gate C communication thresholds
    packet_loss_max_percent: float = 0.1
    watchdog_trip_max_ms: float = 500.0
    crc_error_max_percent: float = 0.01

    # Gate D telemetry & encoder thresholds
    encoder_cpr: int = 4096
    standstill_noise_max_rad: float = 0.005
    timestamp_jitter_max_ms: float = 2.0

    # Gate E motion thresholds
    motion_crawl_position_rad: float = 0.2
    motion_crawl_velocity_rad_s: float = 0.5
    thermal_derating_factor: float = 0.70

    def __post_init__(self) -> None:
        if self.isolation_resistance_min_kohm is not None:
            object.__setattr__(
                self,
                "unpowered_vm_to_gnd_min_resistance_kohm",
                self.isolation_resistance_min_kohm,
            )

    @property
    def vm_to_gnd_kohm(self) -> float:
        return self.unpowered_vm_to_gnd_min_resistance_kohm


@dataclass(frozen=True)
class CommissioningGate:
    """Specification of a progressive hardware commissioning gate."""

    gate_id: str
    title: str
    preconditions: list[str] = field(default_factory=list)
    measurement_method: str = ""
    acceptance_criteria: str = ""
    abort_criteria: str = ""
    evidence_format: str = ""
    requires_physical_hardware: bool = True
    status: str = "PENDING"  # "PASSED", "BLOCKED", "NOT_EXECUTED", "PENDING"
    status_reason: str = ""

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def get_default_commissioning_gates(
    config: CommissioningGateConfig | None = None,
    is_mock: bool = False,
) -> list[CommissioningGate]:
    """Generate the standardized 5-gate commissioning protocol with parameterized thresholds.

    SAFETY & INTEGRITY RULES:
    1. If is_mock is True, mock commissioning CANNOT sign off physical hardware gates.
       - Gate A and Gate B are marked BLOCKED.
       - Gate C is marked PASSED (pure protocol/firmware loopback).
       - Gate D and Gate E are marked NOT_EXECUTED.
    2. Gate B is strictly BLOCKED if transient TVS protection design is unverified.
    3. Gate E is strictly NOT_EXECUTED in software test environments (physical motion prohibited).
    """
    cfg = config or CommissioningGateConfig()

    gate_a_status = "BLOCKED" if is_mock else "PENDING"
    gate_a_reason = (
        "Component and schematic inspection requires physical hardware in hand; mock simulation cannot sign off."
        if is_mock else "Pending physical microscopic and schematic inspection."
    )

    gate_b_status = "BLOCKED" if is_mock else "BLOCKED"
    gate_b_reason = (
        "Physical power rails, star GND, bulk capacitor, and E-stop unverified on bench. "
        "TVS transient overvoltage protection is UNRESOLVED (HARDWARE POWER-UP BLOCKED)."
    )

    gate_c_status = "PASSED" if is_mock else "PENDING"
    gate_c_reason = (
        "Protocol V1 loopback, CRC-16, and commissioning motion rejection verified on MockBenchTransceiver."
        if is_mock else "Pending UART protocol handshake verification."
    )

    gate_d_status = "NOT_EXECUTED" if is_mock else "PENDING"
    gate_d_reason = (
        "Encoder sign and scale verification requires manual physical shaft rotation; not executed in mock."
        if is_mock else "Pending manual shaft rotation test."
    )

    gate_e_status = "NOT_EXECUTED"
    gate_e_reason = (
        "Physical single-axis motion commissioning strictly prohibited in mock/automated environments. "
        "Gates A through D must be formally signed off on physical hardware first."
    )

    return [
        CommissioningGate(
            gate_id="gate_a",
            title="Gate A: Component, Datasheet, Module & Schematic Inspection",
            preconditions=["Bench unpowered", "High-magnification inspection tool available"],
            measurement_method="Visual inspection of driver breakout markings, SMD sense resistor codes, pinout diagram",
            acceptance_criteria=(
                "TMC2209 IC genuine marking confirmed; SMD sense resistors verified (R110/R075/R220); "
                "stepper rated phase current and coil resistance recorded"
            ),
            abort_criteria="Unidentifiable driver IC, missing thermal heatsink, unreadable SMD sense markings, or pinout mismatch",
            evidence_format="High-resolution component photographs and signed schematic inspection record",
            requires_physical_hardware=True,
            status=gate_a_status,
            status_reason=gate_a_reason,
        ),
        CommissioningGate(
            gate_id="gate_b",
            title="Gate B: Power Rails, Common Ground & Protection (Motor UNCONNECTED)",
            preconditions=[
                "Motor phase leads physically disconnected from driver output pins",
                "Bench DC supply completely unpowered and disconnected",
                "All bulk decoupling capacitors fully discharged to 0.0 V before multimeter resistance measurement",
                "Bench supply current limit capped at initial OCP threshold (e.g. 0.4 A)",
            ],
            measurement_method=(
                "1. Pre-power DC bus check: Calibrated DMM 2-wire resistance between VM socket pin and driver GND with supply disconnected and capacitor discharged. "
                "2. Star ground continuity: Calibrated DMM 4-wire resistance between ESP32-S3 digital GND and driver power GND socket pin. "
                "3. Initial power-on metering: Bench supply current metering and oscilloscope monitoring during power-up. "
                "4. Transient protection review: Verification of transient clamping suitability; if TVS unverified, power-up is blocked."
            ),
            acceptance_criteria=(
                f"Unpowered DC bus non-short check: R(VM, GND) > {cfg.vm_to_gnd_kohm:.1f} kOhm (measured with supply DISCONNECTED and capacitors DISCHARGED; verifies absence of dead short/solder bridge; NOT a galvanic isolation barrier); "
                f"MCU GND to driver GND resistance <= {cfg.ground_continuity_max_ohm:.2f} Ohm (star ground verified; notice: DC continuity alone does NOT prove low impedance at PWM switching frequencies); "
                f"Bulk decoupling capacitor >= {cfg.bulk_capacitor_min_uf:.0f} uF verified across VM/GND pins; "
                "Transient protection: TVS evaluated and qualified (HARDWARE POWER-UP BLOCKED if unverified); "
                f"Unloaded quiescent current <= {cfg.max_unloaded_quiescent_current_a:.3f} A; "
                f"Analog VREF adjusted within +/- {cfg.vref_tolerance_v:.2f} V of target formula; "
                "Physical E-stop cuts VM immediately to 0.0 V"
            ),
            abort_criteria=(
                f"Unpowered VM-GND resistance <= {cfg.vm_to_gnd_kohm:.1f} kOhm (dead short hazard); "
                f"Quiescent current > {cfg.max_unloaded_quiescent_current_a:.3f} A; floating ground; "
                "failure of hardwired E-stop to cut VM; inability to adjust stable VREF; "
                "unverified TVS protection component (HARDWARE POWER-UP BLOCKED)"
            ),
            evidence_format="DMM resistance log, VREF calibration record, and E-stop cut-off oscilloscope capture",
            requires_physical_hardware=True,
            status=gate_b_status,
            status_reason=gate_b_reason,
        ),
        CommissioningGate(
            gate_id="gate_c",
            title="Gate C: Firmware & Serial Communication Protocol (Non-Stepping Mode)",
            preconditions=["ESP32-S3 flashed with Protocol V1", "Driver ENABLE held HIGH (disabled)"],
            measurement_method="Automated CRC-16 telemetry loopback, host heartbeat ping/pong handshake, watchdog timeout",
            acceptance_criteria=(
                f"Packet loss <= {cfg.packet_loss_max_percent:.2f}%; CRC errors <= {cfg.crc_error_max_percent:.3f}%; "
                f"Host heartbeat watchdog trips and asserts FAULT within <= {cfg.watchdog_trip_max_ms:.1f} ms; "
                "Motion commands strictly rejected in commissioning mode with ERR_COMMISSIONING_MODE"
            ),
            abort_criteria=(
                f"CRC failure rate > {cfg.crc_error_max_percent:.3f}%; watchdog failing to trip; "
                "spontaneous step pulses generated on oscilloscope"
            ),
            evidence_format="JSON diagnostic report with RTT, clock offset, and invariant validation results",
            requires_physical_hardware=False,
            status=gate_c_status,
            status_reason=gate_c_reason,
        ),
        CommissioningGate(
            gate_id="gate_d",
            title="Gate D: Telemetry, Timestamps & Encoder Validation (Manual Shaft Rotation)",
            preconditions=["Motor unpowered or driver disabled", "Telemetry streaming active"],
            measurement_method="Manual 360-degree shaft rotation with high-precision optical/magnetic encoder streaming",
            acceptance_criteria=(
                f"Sequence IDs increment strictly monotonic (+1); timestamps strictly monotonic; "
                f"Single manual clockwise turn produces exactly +{cfg.encoder_cpr} counts (+2*pi rad); "
                f"Stationary standstill peak-to-peak noise <= {cfg.standstill_noise_max_rad:.4f} rad"
            ),
            abort_criteria="Inverted rotation sign polarity, missing encoder pulses, non-monotonic timestamps, or sequence gaps",
            evidence_format="Raw telemetry stream log (.jsonl or .csv) and encoder sign verification record",
            requires_physical_hardware=True,
            status=gate_d_status,
            status_reason=gate_d_reason,
        ),
        CommissioningGate(
            gate_id="gate_e",
            title="Gate E: Protected Single-Axis Motion Commissioning (Lab Bench Pre-Flight)",
            preconditions=[
                "Gates A through D formally signed off",
                "Hard mechanical stops with polyurethane dampers installed",
                "Operator hand on physical hardwired E-stop button",
                f"Current limit set to {cfg.initial_ocp_limit_a:.1f} A on bench supply",
            ],
            measurement_method="Slow crawl trapezoidal motion profile commanded through ActionShield",
            acceptance_criteria=(
                f"Motor phase current derated to {cfg.thermal_derating_factor * 100:.0f}% of rated current; "
                f"Crawl command (+/- {cfg.motion_crawl_position_rad:.2f} rad at {cfg.motion_crawl_velocity_rad_s:.2f} rad/s) "
                "executes smoothly without acoustic resonance; encoder feedback confirms tracking agreement"
            ),
            abort_criteria="Position error exceeding 0.1 rad, thermal runaway (> 70 deg C casing), excessive audible grinding, or limit switch hit",
            evidence_format="Synchronous command vs encoder telemetry log and thermal temperature log",
            requires_physical_hardware=True,
            status=gate_e_status,
            status_reason=gate_e_reason,
        ),
    ]


class RunCommissioningWorkflow:
    """Orchestrates hardware commissioning diagnostic verification."""

    def __init__(
        self,
        transport: CommandTransport,
        timeout_s: float = 2.0,
        is_mock: bool | None = None,
        gate_config: CommissioningGateConfig | None = None,
    ) -> None:
        self.transport = transport
        self.timeout_s = timeout_s
        self.gate_config = gate_config or CommissioningGateConfig()
        if is_mock is None:
            from ears_rl.hardware.protocol import MockBenchTransceiver
            self.is_mock = isinstance(transport, MockBenchTransceiver)
        else:
            self.is_mock = is_mock

    def get_gates(self) -> list[CommissioningGate]:
        """Return the structured commissioning gates with active configured thresholds."""
        return get_default_commissioning_gates(self.gate_config, is_mock=self.is_mock)

    def execute(self) -> dict[str, Any]:
        """Execute commissioning diagnostic sequence.

        Returns:
            Structured diagnostic report dictionary.
        """
        logger.info("Executing commissioning workflow with transport %s (mock=%s)", self.transport, self.is_mock)
        was_connected = self.transport.is_connected
        if not was_connected:
            self.transport.connect()

        try:
            report = run_commissioning_diagnostic(
                self.transport,
                is_mock=self.is_mock,
                timeout_s=self.timeout_s,
            )
            report["commissioning_gate_config"] = asdict(self.gate_config)
            report["gates"] = [g.to_dict() for g in self.get_gates()]
            report["hardware_power_up_status"] = (
                "UNRESOLVED — HARDWARE POWER-UP BLOCKED"
                if self.is_mock else "REQUIRES_PHYSICAL_BENCH_SIGN_OFF"
            )
            return report
        finally:
            if not was_connected:
                self.transport.disconnect()
