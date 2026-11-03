"""Scientific integrity verification: Delay identification vs first-order mechanical lag.

Ablation study proving:
1. When mechanical lag tau ~ 0, FixedDelayActuatorModel identifies pure transport delay tau_d exactly.
2. When mechanical lag tau > 0, a zero-order delay model absorbs dynamic phase lag into an
   effective lumped delay tau_effective ~= tau_d + tau_phase(omega).
"""

from __future__ import annotations

import numpy as np
import pandas as pd
import pytest

from ears_rl.actuator_model.fixed_delay import FixedDelayActuatorModel
from ears_rl.core.clock import MockClock
from ears_rl.core.types import JointCommand
from ears_rl.hardware.mock.actuator import MockActuator


def _generate_synthetic_chirp_data(
    tau_s: float,
    latency_ms: float,
    duration_s: float = 4.0,
    sample_rate_hz: float = 100.0,
) -> pd.DataFrame:
    """Generate deterministic simulated bench dataset under specified tau and latency."""
    clock = MockClock(start_ns=1_000_000_000)
    actuator = MockActuator(
        joint_id="test_actuator",
        clock=clock,
        tau_s=tau_s,
        latency_ms=latency_ms,
        random_seed=42,
    )
    actuator.connect()
    actuator.enable_motion()

    dt_s = 1.0 / sample_rate_hz
    steps = int(duration_s * sample_rate_hz)
    records = []

    for step in range(steps):
        t_s = step * dt_s
        # Chirp frequency sweep from 0.2 Hz to 2.0 Hz
        f_t = 0.2 + (2.0 - 0.2) * (t_s / duration_s)
        target_rad = 0.5 * np.sin(2.0 * np.pi * f_t * t_s)

        cmd = JointCommand(joint_id="test_actuator", position_rad=target_rad)
        actuator.send_position_command(cmd)

        clock.tick(int(dt_s * 1e9))
        actuator.step_simulation(dt_s=dt_s)

        meas_pos = actuator.get_position().radians
        records.append({
            "timestamp_ns": clock.now_ns(),
            "command_position_rad": target_rad,
            "measured_motor_position_rad": meas_pos,
        })

    return pd.DataFrame(records)


@pytest.mark.parametrize("pure_delay_ms", [10.0, 30.0, 70.0])
def test_pure_delay_identification_without_mechanical_lag(pure_delay_ms: float):
    """When tau -> 0 (negligible mechanical lag), FixedDelayActuatorModel must identify pure delay EXACTLY."""
    # Near-zero mechanical lag (1 ms)
    df = _generate_synthetic_chirp_data(tau_s=0.001, latency_ms=pure_delay_ms)

    model = FixedDelayActuatorModel()
    fit_res = model.fit(df)

    assert fit_res["status"] == "fitted"
    assert fit_res["identified_delay_ms"] == pure_delay_ms
    assert fit_res["residual_rmse_rad"] < fit_res["nominal_rmse_rad"]


def test_mechanical_lag_induces_effective_delay_in_pure_delay_model():
    """When tau = 40 ms and pure delay = 0 ms, the identified delay reflects dynamic phase lag."""
    df = _generate_synthetic_chirp_data(tau_s=0.040, latency_ms=0.0)

    model = FixedDelayActuatorModel()
    fit_res = model.fit(df)

    assert fit_res["status"] == "fitted"
    # Even with 0 transport delay, mechanical lag creates ~30-40 ms effective delay
    assert fit_res["identified_delay_ms"] >= 30.0
    # Residual RMSE after shift must be strictly lower than zero-shift nominal RMSE
    assert fit_res["residual_rmse_rad"] < fit_res["nominal_rmse_rad"]


def test_combined_delay_and_lag_monotonicity():
    """Identified delay of (tau=40ms, delay=10ms) must be strictly greater than or equal to (tau=40ms, delay=0ms)."""
    df_no_delay = _generate_synthetic_chirp_data(tau_s=0.040, latency_ms=0.0)
    df_with_delay = _generate_synthetic_chirp_data(tau_s=0.040, latency_ms=20.0)

    model1 = FixedDelayActuatorModel()
    res1 = model1.fit(df_no_delay)

    model2 = FixedDelayActuatorModel()
    res2 = model2.fit(df_with_delay)

    assert res2["identified_delay_ms"] > res1["identified_delay_ms"]
