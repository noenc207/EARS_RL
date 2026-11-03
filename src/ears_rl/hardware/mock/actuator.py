"""Mock Stepper-Encoder Actuator for development, testing, and system identification."""

from __future__ import annotations

import logging
from collections import deque

import numpy as np

from ears_rl.core.clock import TimeProvider
from ears_rl.core.types import (
    ActuatorCapabilities,
    ActuatorState,
    ActuatorType,
    ConnectionState,
    HardwareMode,
    JointCommand,
    JointPosition,
    JointVelocity,
)
from ears_rl.hardware.base import BaseActuator
from ears_rl.hardware.mock.encoder import MockEncoder

logger = logging.getLogger(__name__)


class MockActuator(BaseActuator):
    """Simulated hybrid stepper-servo actuator.

    Simulates:
    1. First-order lag response to position commands: d_pos/dt = (cmd - pos) / tau.
    2. Configurable latency buffer (pure time delay).
    3. Velocity saturation (clamping to velocity_max_rad_s).
    4. Optional synthetic step-loss and stall injection for detector benchmark tests.
    5. Integrated MockEncoder for realistic feedback.
    """

    def __init__(
        self,
        joint_id: str = "mock_joint_0",
        capabilities: ActuatorCapabilities | None = None,
        clock: TimeProvider | None = None,
        tau_s: float = 0.05,  # 50 ms mechanical time constant
        latency_ms: float = 10.0,  # 10 ms communication delay
        random_seed: int = 42,
    ) -> None:
        if capabilities is None:
            capabilities = ActuatorCapabilities(
                actuator_type=ActuatorType.STEPPER,
                has_position_control=True,
                has_velocity_control=True,
                position_min_rad=-np.pi,
                position_max_rad=np.pi,
                velocity_max_rad_s=6.28,  # ~1 rev/s
                acceleration_max_rad_s2=30.0,
                limits_verified=True,  # Mock mode limits are verified in simulation
            )

        super().__init__(
            joint_id=joint_id,
            capabilities=capabilities,
            hardware_mode=HardwareMode.MOCK,
        )

        from ears_rl.core.clock import SystemClock
        self._clock = clock if clock is not None else SystemClock()
        self.tau_s = max(0.001, tau_s)
        self.latency_ms = max(0.0, latency_ms)

        # Dynamic state
        self._true_position_rad: float = 0.0
        self._true_velocity_rad_s: float = 0.0
        self._current_amp: float = 0.4  # baseline quiescent current [A]
        self._last_step_time_ns: int = self._clock.now_ns()

        # Command latency pipeline: stores (dispatch_time_ns, command)
        self._command_delay_queue: deque[tuple[int, JointCommand]] = deque()
        self._active_command: JointCommand = JointCommand(joint_id=joint_id, position_rad=0.0)

        # Fault injection controls
        self._injected_step_loss_rad: float = 0.0
        self._force_stall: bool = False

        # Integrated encoder
        self.encoder = MockEncoder(
            encoder_id=f"{joint_id}_enc",
            clock=self._clock,
            resolution_counts_per_rev=4096,
            random_seed=random_seed,
        )

    def connect(self) -> None:
        """Connect mock actuator."""
        self._connection_state = ConnectionState.CONNECTED
        self._last_step_time_ns = self._clock.now_ns()
        logger.info("Connected mock actuator '%s'", self.joint_id)

    def disconnect(self) -> None:
        """Disconnect mock actuator."""
        self.disable_motion()
        self._connection_state = ConnectionState.DISCONNECTED
        self._command_delay_queue.clear()
        logger.info("Disconnected mock actuator '%s'", self.joint_id)

    def get_position(self) -> JointPosition:
        return JointPosition(
            radians=self.encoder.read_position(),
            joint_id=self.joint_id,
            source="mock_encoder",
        )

    def get_velocity(self) -> JointVelocity:
        return JointVelocity(
            radians_per_second=self._true_velocity_rad_s,
            joint_id=self.joint_id,
        )

    def get_current_amp(self) -> float:
        return self._current_amp

    def send_position_command(self, command: JointCommand) -> bool:
        """Accept a target position command and enqueue it into the delay pipeline."""
        self._pre_command_check(command)
        now_ns = self._clock.now_ns()
        self._command_delay_queue.append((now_ns, command))
        return True

    def send_velocity_command(self, command: JointCommand) -> bool:
        """Accept a target velocity command and enqueue it into the delay pipeline."""
        self._pre_command_check(command)
        now_ns = self._clock.now_ns()
        self._command_delay_queue.append((now_ns, command))
        return True

    def step_simulation(self, dt_s: float | None = None) -> None:
        """Advance physical simulation of actuator by dt_s seconds."""
        now_ns = self._clock.now_ns()
        if dt_s is None:
            dt_s = max(0.0001, (now_ns - self._last_step_time_ns) / 1e9)
        self._last_step_time_ns = now_ns

        # 1. Process delayed commands that have matured past latency_ms
        delay_ns = int(self.latency_ms * 1_000_000)
        while self._command_delay_queue and (now_ns - self._command_delay_queue[0][0]) >= delay_ns:
            _, cmd = self._command_delay_queue.popleft()
            self._active_command = cmd

        if not self._is_motion_enabled or self._connection_state != ConnectionState.CONNECTED:
            self._true_velocity_rad_s = 0.0
            self.encoder.set_true_position(self._true_position_rad)
            return

        # 2. Check for stall condition
        if self._force_stall:
            self._state = ActuatorState.STALL
            self._true_velocity_rad_s = 0.0
            self._current_amp = 1.8  # Elevated stall current
            self.encoder.set_true_position(self._true_position_rad)
            return

        # 3. Simulate first-order dynamic response
        target_pos = self._active_command.position_rad
        if target_pos is not None:
            # Position error
            error = target_pos - self._true_position_rad
            desired_vel = error / self.tau_s

            # Clamp to maximum velocity limit
            max_vel = self._capabilities.velocity_max_rad_s or 6.28
            clamped_vel = float(np.clip(desired_vel, -max_vel, max_vel))
            self._true_velocity_rad_s = clamped_vel

            if abs(desired_vel) > max_vel:
                self._state = ActuatorState.SATURATED
            else:
                self._state = ActuatorState.NORMAL

            # Integrate position
            self._true_position_rad += self._true_velocity_rad_s * dt_s

            # Current consumption proportional to velocity + static load
            self._current_amp = 0.4 + 0.8 * (abs(self._true_velocity_rad_s) / max_vel)

        # 4. Apply step loss injection if triggered
        if abs(self._injected_step_loss_rad) > 1e-5:
            self._state = ActuatorState.STEP_LOSS
            self._true_position_rad += self._injected_step_loss_rad
            self._injected_step_loss_rad = 0.0

        # 5. Enforce hard physical stops if configured
        if self._capabilities.position_min_rad is not None:
            self._true_position_rad = max(self._capabilities.position_min_rad, self._true_position_rad)
        if self._capabilities.position_max_rad is not None:
            self._true_position_rad = min(self._capabilities.position_max_rad, self._true_position_rad)

        # 6. Update encoder sensor
        self.encoder.set_true_position(self._true_position_rad)

    def inject_step_loss(self, slip_rad: float) -> None:
        """Inject synthetic step loss (desynchronization) for testing detectors."""
        self._injected_step_loss_rad = slip_rad

    def set_stall(self, stall: bool) -> None:
        """Force motor into mechanical stall condition."""
        self._force_stall = stall
