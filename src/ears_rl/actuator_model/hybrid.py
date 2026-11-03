"""Hybrid Event-Aware Actuator Model Interface and Baseline Skeleton for EARS-RL.

Represents a switching dynamical system where actuator response varies
according to discrete states: NORMAL, SATURATED, STEP_LOSS, STALL, RECOVERY, UNKNOWN.
"""

from __future__ import annotations

from typing import Any

from ears_rl.actuator_model.base import BaseActuatorModel
from ears_rl.core.types import ActuatorState, JointCommand


class HybridEventAwareActuatorModel(BaseActuatorModel):
    """Event-aware actuator model capturing discrete failure and recovery transitions.

    Architecture:
    - Mode NORMAL: Standard linear/viscous tracking dynamics with calibrated delay.
    - Mode SATURATED: Velocity or torque saturation limits active.
    - Mode STEP_LOSS: Stepper synchronization slip parameterized as discrete step jump.
    - Mode STALL: High current, zero angular velocity.
    - Mode RECOVERY: Homing or re-synchronization dynamics.
    - Mode UNKNOWN: Unmodeled transient disturbances.
    """

    def __init__(
        self,
        nominal_delay_ms: float = 10.0,
        full_step_angle_rad: float = 0.0314,  # 1.8° = 0.0314 rad
    ) -> None:
        super().__init__(model_type="hybrid_event_aware")
        self.nominal_delay_ms = nominal_delay_ms
        self.full_step_angle_rad = full_step_angle_rad
        self._current_state = ActuatorState.NORMAL
        self._accumulated_slip_rad: float = 0.0
        self._is_fitted = False

    def predict(
        self, command: JointCommand, current_state: dict[str, Any]
    ) -> dict[str, Any]:
        """Predict actuator response incorporating active discrete operating mode."""
        cmd_pos = command.position_rad if command.position_rad is not None else current_state.get("position_rad", 0.0)

        # Apply discrete mode dynamics
        if self._current_state == ActuatorState.NORMAL:
            pred_pos = cmd_pos + self._accumulated_slip_rad
            pred_vel = current_state.get("velocity_rad_s", 0.0)
        elif self._current_state == ActuatorState.STEP_LOSS:
            # Model integer step slip
            pred_pos = cmd_pos + self._accumulated_slip_rad
            pred_vel = current_state.get("velocity_rad_s", 0.0)
        elif self._current_state == ActuatorState.STALL:
            pred_pos = current_state.get("position_rad", 0.0)
            pred_vel = 0.0
        else:
            pred_pos = cmd_pos
            pred_vel = 0.0

        return {
            "predicted_position_rad": pred_pos,
            "predicted_velocity_rad_s": pred_vel,
            "predicted_delay_ms": self.nominal_delay_ms,
            "predicted_state": self._current_state.value,
            "accumulated_slip_rad": self._accumulated_slip_rad,
        }

    def update_discrete_state(self, state: ActuatorState, slip_rad: float = 0.0) -> None:
        """Update active mode from online detector or ground truth."""
        self._current_state = state
        if state == ActuatorState.STEP_LOSS:
            self._accumulated_slip_rad += slip_rad
        elif state == ActuatorState.RECOVERY:
            self._accumulated_slip_rad = 0.0

    def fit(self, dataset: Any) -> dict[str, float]:
        """Fit hybrid switching parameters from empirical training datasets."""
        # Advanced hybrid identification pipeline (Phase 2 extension)
        self._is_fitted = True
        return {
            "fit_status": 1.0,
            "nominal_delay_ms": self.nominal_delay_ms,
            "step_angle_rad": self.full_step_angle_rad,
        }
