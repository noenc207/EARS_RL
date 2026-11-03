"""First-Order Lag Actuator Model (Grey-Box without Dead Time).

Transfer function:
    G(s) = K / (tau * s + 1)

Units:
    K: static gain [dimensionless rad/rad]
    tau: mechanical time constant [seconds]
    u: commanded position [rad]
    y: measured actuator position [rad]
"""

from __future__ import annotations

import logging
from typing import Any

import numpy as np
import pandas as pd
from scipy.optimize import minimize

from ears_rl.actuator_model.base import BaseActuatorModel
from ears_rl.core.types import JointCommand

logger = logging.getLogger(__name__)


class FirstOrderActuatorModel(BaseActuatorModel):
    """First-order lag response model representing mechanical/electrical low-pass response.

    Discrete Zero-Order Hold (ZOH) update equation:
        alpha = exp(-dt / tau)
        y[k] = alpha * y[k-1] + K * (1 - alpha) * u[k]
    """

    def __init__(
        self,
        gain: float = 1.0,
        tau_s: float = 0.05,
    ) -> None:
        super().__init__(model_type="first_order")
        self.gain = float(gain)
        self.tau_s = max(0.0001, float(tau_s))
        self._parameters["gain"] = self.gain
        self._parameters["tau_s"] = self.tau_s
        self._last_position_rad: float = 0.0

    def reset(self) -> None:
        self._last_position_rad = 0.0

    def simulate(
        self,
        commands: np.ndarray,
        dt_s: float,
        initial_position_rad: float = 0.0,
        gain: float | None = None,
        tau_s: float | None = None,
    ) -> np.ndarray:
        """Simulate discrete first-order lag response for an entire command sequence."""
        k_val = self.gain if gain is None else gain
        tau_val = self.tau_s if tau_s is None else max(1e-4, tau_s)

        alpha = float(np.exp(-dt_s / tau_val))
        beta = float(k_val * (1.0 - alpha))

        n = len(commands)
        y = np.zeros(n, dtype=np.float64)
        curr = initial_position_rad

        for i in range(n):
            y[i] = curr
            curr = alpha * curr + beta * commands[i]

        return y

    def predict(
        self, command: JointCommand, current_state: dict[str, Any]
    ) -> dict[str, Any]:
        """Predict next position given current state and command."""
        target_pos = (
            command.position_rad
            if command.position_rad is not None
            else current_state.get("position_rad", 0.0)
        )
        curr_pos = current_state.get("position_rad", self._last_position_rad)
        dt_s = max(1e-4, current_state.get("dt_s", 0.01))

        alpha = float(np.exp(-dt_s / self.tau_s))
        beta = float(self.gain * (1.0 - alpha))
        pred_pos = alpha * curr_pos + beta * target_pos
        pred_vel = (pred_pos - curr_pos) / dt_s

        self._last_position_rad = pred_pos
        return {
            "predicted_position_rad": pred_pos,
            "predicted_velocity_rad_s": pred_vel,
            "predicted_state": "normal",
            "model_type": self._model_type,
        }

    def fit(self, dataset: Any) -> dict[str, Any]:
        """Identify K and tau from telemetry data via bounded non-linear least squares."""
        try:
            df = dataset if isinstance(dataset, pd.DataFrame) else pd.DataFrame(dataset)
            if df.empty:
                return {
                    "status": "failed",
                    "error": "Dataset is empty",
                    "is_identifiable": False,
                }

            # 1. Derive sample period dt_s
            dt_s = 0.01
            if "timestamp_ns" in df.columns and len(df["timestamp_ns"].dropna()) > 1:
                ts_vals = df["timestamp_ns"].dropna().astype(float).values
                diffs_s = np.diff(ts_vals) / 1e9
                valid_diffs = diffs_s[diffs_s > 0]
                if len(valid_diffs) > 0:
                    dt_s = float(np.median(valid_diffs))

            # 2. Extract commanded and measured trajectories
            if "command_position_rad" not in df.columns or "measured_motor_position_rad" not in df.columns:
                return {
                    "status": "failed",
                    "error": "Missing required command or measured position columns",
                    "is_identifiable": False,
                }

            cmd = df["command_position_rad"].dropna().astype(float).values
            meas = df["measured_motor_position_rad"].dropna().astype(float).values
            min_len = min(len(cmd), len(meas))
            if min_len < 10:
                return {
                    "status": "failed",
                    "error": f"Insufficient sample count ({min_len} < 10)",
                    "is_identifiable": False,
                }

            cmd = cmd[:min_len]
            meas = meas[:min_len]

            # Filter non-finite
            finite_mask = np.isfinite(cmd) & np.isfinite(meas)
            cmd = cmd[finite_mask]
            meas = meas[finite_mask]
            min_len = len(cmd)

            # 3. Excitation / identifiability check
            cmd_variance = float(np.var(cmd))
            if cmd_variance < 1e-6:
                return {
                    "status": "unidentifiable",
                    "error": f"Insufficient excitation: command variance ({cmd_variance:.2e} rad^2) is negligible.",
                    "is_identifiable": False,
                    "command_variance_rad2": cmd_variance,
                    "identified_gain": None,
                    "identified_time_constant_ms": None,
                }

            nominal_rmse = float(np.sqrt(np.mean((cmd - meas) ** 2)))
            y0 = float(meas[0])

            # 4. Optimization objective function
            def objective(params: list[float]) -> float:
                k_val, tau_val = params
                y_sim = self.simulate(cmd, dt_s, initial_position_rad=y0, gain=k_val, tau_s=tau_val)
                return float(np.sum((y_sim - meas) ** 2))

            # Bounded L-BFGS-B optimization
            # Gain bounded in [0.5, 2.0], tau in [0.001, 1.0] s (1 ms to 1000 ms)
            init_guess = [1.0, 0.05]
            bounds = [(0.5, 2.0), (0.001, 1.0)]
            res = minimize(objective, init_guess, method="L-BFGS-B", bounds=bounds)

            if not res.success and not np.isfinite(res.fun):
                return {
                    "status": "failed",
                    "error": f"Optimization failed to converge: {res.message}",
                    "is_identifiable": False,
                }

            opt_k, opt_tau = float(res.x[0]), float(res.x[1])
            y_best = self.simulate(cmd, dt_s, initial_position_rad=y0, gain=opt_k, tau_s=opt_tau)
            residual_rmse = float(np.sqrt(np.mean((y_best - meas) ** 2)))

            self.gain = opt_k
            self.tau_s = opt_tau
            self._parameters["gain"] = opt_k
            self._parameters["tau_s"] = opt_tau
            self._is_fitted = True

            return {
                "status": "fitted",
                "is_identifiable": True,
                "identified_gain": round(opt_k, 4),
                "identified_time_constant_s": round(opt_tau, 5),
                "identified_time_constant_ms": round(opt_tau * 1000.0, 2),
                "nominal_rmse_rad": round(nominal_rmse, 5),
                "residual_rmse_rad": round(residual_rmse, 5),
                "rmse_reduction_rad": round(nominal_rmse - residual_rmse, 5),
                "sample_period_ms": round(dt_s * 1000.0, 2),
                "estimator_method": "first_order_bounded_lbfgsb",
            }
        except Exception as e:
            logger.exception("First-order identification failed: %s", e)
            return {
                "status": "error",
                "error": str(e),
                "is_identifiable": False,
            }
