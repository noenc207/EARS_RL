"""Nominal actuator model representing ideal delay-free kinematic response."""

from __future__ import annotations

from typing import Any

from ears_rl.actuator_model.base import BaseActuatorModel
from ears_rl.core.types import JointCommand


class NominalActuatorModel(BaseActuatorModel):
    """Baseline Model A: Assumes actuator tracks target position immediately without delay.

    Output equals commanded input clamped to configured limits.
    Serves as zero-hypothesis baseline (B0/B1).
    """

    def __init__(self, velocity_max_rad_s: float = 6.28) -> None:
        super().__init__(model_type="nominal")
        self.velocity_max_rad_s = velocity_max_rad_s
        self._is_fitted = True  # Nominal model has no free parameters to fit

    def predict(
        self, command: JointCommand, current_state: dict[str, Any]
    ) -> dict[str, Any]:
        """Predict immediate tracking of command."""
        target_pos = command.position_rad if command.position_rad is not None else current_state.get("position_rad", 0.0)
        curr_pos = current_state.get("position_rad", 0.0)
        dt_s = current_state.get("dt_s", 0.01)

        desired_vel = (target_pos - curr_pos) / max(1e-4, dt_s)
        clamped_vel = max(-self.velocity_max_rad_s, min(self.velocity_max_rad_s, desired_vel))

        return {
            "predicted_position_rad": target_pos,
            "predicted_velocity_rad_s": clamped_vel,
            "predicted_delay_ms": 0.0,
            "predicted_state": "normal",
        }

    def fit(self, dataset: Any) -> dict[str, Any]:
        """Nominal model parameters are fixed; zero-hypothesis baseline predictor.

        Scientific Integrity: Nominal model contains zero free/learnable parameters.
        Therefore training fit_loss is strictly not applicable (None / null), NOT 0.0.
        Prediction error on evaluation data is computed separately via MAE/RMSE.
        """
        self._is_fitted = True
        return {
            "fit_loss": None,
            "status": "not_applicable",
            "message": "Nominal baseline model has no learnable parameters (zero-hypothesis baseline).",
            "trainable_parameters": 0,
        }
