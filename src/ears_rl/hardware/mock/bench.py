"""End-to-End Mock Actuator Bench Orchestrator for EARS-RL.

Executes reproducible synthetic data generation sessions without physical hardware.
Produces standardized BenchRecord datasets, DatasetMetadata, and manifests.
"""

from __future__ import annotations

import logging
from collections.abc import Callable
from pathlib import Path

import numpy as np

from ears_rl.config.loader import compute_config_hash
from ears_rl.config.schema import ExperimentConfig
from ears_rl.core.clock import MockClock, TimeProvider
from ears_rl.core.types import JointCommand
from ears_rl.data.logger import BufferedDataLogger
from ears_rl.data.schema import BenchRecord, DatasetMetadata
from ears_rl.hardware.mock.actuator import MockActuator

logger = logging.getLogger(__name__)


class MockActuatorBench:
    """Orchestrates an end-to-end simulation session for a single-axis actuator bench."""

    def __init__(
        self,
        config: ExperimentConfig,
        output_dir: Path | str,
        clock: TimeProvider | None = None,
        random_seed: int = 42,
        simulated_delay_ms: float | None = None,
        tau_s: float | None = None,
    ) -> None:
        self.config = config
        self.output_dir = Path(output_dir)
        self.random_seed = random_seed
        self._clock = clock if clock is not None else MockClock(start_ns=1_000_000_000)

        # Build actuator from configuration
        act_cfg = config.actuators[0] if config.actuators else None
        joint_id = act_cfg.joint_id if act_cfg else "mock_joint_0"

        act_tau = tau_s if tau_s is not None else 0.04  # 40 ms lag
        act_delay = simulated_delay_ms if simulated_delay_ms is not None else 10.0  # 10 ms delay

        self.actuator = MockActuator(
            joint_id=joint_id,
            clock=self._clock,
            tau_s=act_tau,
            latency_ms=act_delay,
            random_seed=random_seed,
        )

        self.logger = BufferedDataLogger(
            output_dir=self.output_dir,
            buffer_size=config.data.buffer_size,
            file_format=config.data.log_format,
        )

    def run_session(
        self,
        duration_s: float = 2.0,
        sample_rate_hz: float = 100.0,
        trajectory_type: str = "chirp",
        inject_fault_at_s: float | None = None,
    ) -> Path:
        """Run a full synthetic bench test session and write output files.

        Args:
            duration_s: Session duration in seconds.
            sample_rate_hz: Sampling / control frequency in Hz.
            trajectory_type: 'sine', 'step', 'chirp', or 'random'.
            inject_fault_at_s: Optional time to inject a step loss event.

        Returns:
            Path to the generated data file.
        """
        logger.info(
            "Starting mock bench session '%s' (duration=%.1fs, rate=%.1fHz, traj=%s)",
            self.config.experiment_id, duration_s, sample_rate_hz, trajectory_type
        )

        # 1. Start logging session
        metadata = {
            "experiment_id": self.config.experiment_id,
            "session_id": "session_001",
            "description": f"Synthetic bench session with {trajectory_type} trajectory",
            "data_source": "synthetic",  # MANDATORY SCIENTIFIC PROVENANCE
            "sample_rate_hz_expected": sample_rate_hz,
            "software_version": self.config.software_version,
            "hardware_config_hash": compute_config_hash(self.config),
            "random_seed": self.random_seed,
        }
        self.logger.start_session("session_001", metadata)

        # 2. Connect and enable mock actuator
        self.actuator.connect()
        self.actuator.enable_motion()

        dt_s = 1.0 / sample_rate_hz
        total_steps = int(duration_s * sample_rate_hz)
        trajectory_fn = self._build_trajectory(trajectory_type, duration_s)

        for step in range(total_steps):
            t_s = step * dt_s

            # Inject synthetic step loss if configured
            if inject_fault_at_s is not None and abs(t_s - inject_fault_at_s) < (dt_s / 2.0):
                logger.info("Injecting synthetic step loss at t=%.3fs", t_s)
                self.actuator.inject_step_loss(slip_rad=0.35)

            # Generate target position
            target_rad = float(trajectory_fn(t_s))

            # Dispatch command
            cmd_ts = self._clock.now()
            cmd = JointCommand(
                joint_id=self.actuator.joint_id,
                position_rad=target_rad,
                timestamp=cmd_ts,
                sequence_id=step + 1,
            )
            self.actuator.send_position_command(cmd)

            # Advance physical simulation
            self.actuator.step_simulation(dt_s)

            # Advance mock clock if using MockClock
            if isinstance(self._clock, MockClock):
                self._clock.tick_ms(dt_s * 1000.0)

            # Read sensor feedback
            enc_reading = self.actuator.encoder.read()
            measured_pos = enc_reading.position_rad
            measured_vel = self.actuator.get_velocity().radians_per_second

            # Construct BenchRecord with scientifically standardized timing metrics
            sample_period_ms = dt_s * 1000.0
            now_timestamp_ns = self._clock.now_ns()

            record = BenchRecord(
                experiment_id=self.config.experiment_id,
                session_id="session_001",
                sequence_id=step + 1,
                timestamp_ns=now_timestamp_ns,
                wall_time_utc=None,  # Optional: omitted in deterministic mock clock
                joint_id=self.actuator.joint_id,
                command_position_rad=target_rad,
                command_velocity_rad_s=None,  # Mode-dependent: position control mode
                command_acceleration_rad_s2=None,  # Mode-dependent: position control mode
                measured_motor_position_rad=measured_pos,
                measured_joint_position_rad=measured_pos,
                measured_velocity_rad_s=measured_vel,
                current_amp=self.actuator.get_current_amp(),
                voltage_v=24.0,  # Standard 24V stepper bus
                estimated_load_nm=0.05,
                command_sent_timestamp_ns=cmd_ts.ns,
                feedback_received_timestamp_ns=now_timestamp_ns,
                sample_period_ms=sample_period_ms,
                configured_simulated_delay_ms=self.actuator.latency_ms,
                command_to_feedback_delay_ms=None,  # Synchronous mock loop: physical packet RTT is unmeasured
                latency_ms=self.actuator.latency_ms,  # Legacy field: indicates configured simulated delay
                actuator_mode=self.actuator.get_state().value,
                event_label=self.actuator.get_state().value,
                event_source="synthetic_injection" if self.actuator.get_state().value != "normal" else "model_estimate",
                ground_truth_status=self.actuator.get_state().value,
                data_quality_flags=["valid", "synthetic"],
                data_source="synthetic",
                hardware_config_hash=compute_config_hash(self.config),
                software_version=self.config.software_version,
            )

            self.logger.log_record(record.model_dump())

        # 3. Clean up and close
        self.actuator.disconnect()
        self.logger.close()

        # 4. Write finalized metadata
        final_meta = DatasetMetadata(
            experiment_id=self.config.experiment_id,
            session_id="session_001",
            description=f"Synthetic mock bench run: {trajectory_type} profile",
            data_source="synthetic",
            sample_rate_hz_expected=sample_rate_hz,
            sample_rate_hz_measured=sample_rate_hz,
            total_records=total_steps,
            total_duration_s=duration_s,
            sensors_used=["mock_encoder"],
            hardware_config_hash=compute_config_hash(self.config),
            software_version=self.config.software_version,
            random_seed=self.random_seed,
        )
        with open(self.output_dir / "metadata.json", "w", encoding="utf-8") as f:
            f.write(final_meta.model_dump_json(indent=2))

        logger.info("Mock bench completed. Data saved to %s", self.logger.data_file_path)
        return self.logger.data_file_path or (self.output_dir / "data.csv")

    def _build_trajectory(self, traj_type: str, duration_s: float) -> Callable[[float], float]:
        """Construct trajectory generator function."""
        if traj_type == "sine":
            return lambda t: 1.0 * np.sin(2.0 * np.pi * 0.5 * t)
        elif traj_type == "step":
            return lambda t: 0.8 if (int(t * 2) % 2 == 1) else -0.8
        elif traj_type == "chirp":
            # Frequency ramps from 0.2 Hz to 2.0 Hz over duration
            f0, f1 = 0.2, 2.0
            return lambda t: 1.2 * np.sin(2.0 * np.pi * (f0 + (f1 - f0) * (t / duration_s) / 2.0) * t)
        else:
            # Multi-frequency sum
            return lambda t: 0.6 * np.sin(2.0 * np.pi * 0.5 * t) + 0.3 * np.sin(2.0 * np.pi * 1.5 * t)
