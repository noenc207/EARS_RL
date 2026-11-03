"""Unit tests for Decoupled Continuous-Time Analytical and RK4 Actuator Validation."""

from __future__ import annotations

import math

import numpy as np
import pytest

from ears_rl.identification.independent_validation import (
    build_independent_multistep_dataset,
    fopdt_analytical_multistep,
    fopdt_rk4_continuous_simulation,
    run_comprehensive_independent_validation_matrix,
)


def test_analytical_multistep_exactness():
    """Verify analytical closed-form continuous FOPDT solution at known theoretical points."""
    dt_s = 0.001  # 1 ms resolution
    t = np.arange(0, 0.500, dt_s)
    k = 1.0
    tau_s = 0.050  # 50 ms
    theta_s = 0.020  # 20 ms
    u0 = 1.0

    y = fopdt_analytical_multistep(
        t_array_s=t,
        gain=k,
        tau_s=tau_s,
        dead_time_s=theta_s,
        step_levels=[u0],
        step_times_s=[0.0],
        initial_val=0.0,
    )

    # 1. Before dead time: strictly zero
    idx_15ms = int(round(0.015 / dt_s))
    assert y[idx_15ms] == 0.0

    # 2. At dead time: exactly zero
    idx_20ms = int(round(0.020 / dt_s))
    assert y[idx_20ms] == 0.0

    # 3. At 1 time constant past dead time (t = theta + tau = 70 ms):
    # Theoretical: u0 * (1 - e^(-1)) = 1.0 * (1 - 1/e) = 0.6321205588
    idx_70ms = int(round(0.070 / dt_s))
    expected_70ms = u0 * (1.0 - math.exp(-1.0))
    assert pytest.approx(y[idx_70ms], rel=1e-6) == expected_70ms

    # 4. Long-term steady state (t = 400 ms >> tau): reaches K * u0
    idx_400ms = int(round(0.400 / dt_s))
    assert pytest.approx(y[idx_400ms], rel=1e-3) == k * u0


def test_rk4_simulation_matches_analytical_solution():
    """Verify high-order continuous RK4 simulation matches analytical closed form within 0.001 rad."""
    dt_s = 0.005  # 5 ms sampling
    t = np.arange(0, 1.0, dt_s)
    k = 1.0
    tau_s = 0.040
    theta_s = 0.015
    u_level = 0.8

    # Analytical
    y_anal = fopdt_analytical_multistep(
        t_array_s=t,
        gain=k,
        tau_s=tau_s,
        dead_time_s=theta_s,
        step_levels=[u_level],
        step_times_s=[0.0],
        initial_val=0.0,
    )

    # RK4 Continuous
    y_rk4 = fopdt_rk4_continuous_simulation(
        t_array_s=t,
        gain=k,
        tau_s=tau_s,
        dead_time_s=theta_s,
        command_fn=lambda t_eval: u_level,
        initial_val=0.0,
        substeps_per_sample=50,
    )

    max_diff = float(np.max(np.abs(y_anal - y_rk4)))
    assert max_diff < 0.001, f"RK4 diverged from analytical solution: max_diff = {max_diff:.6f} rad"


def test_independent_multistep_dataset_generation():
    """Verify independent dataset builder returns standardized schema with provenance."""
    records, gt = build_independent_multistep_dataset(
        sample_period_s=0.010,
        duration_s=1.0,
        gain=1.0,
        tau_s=0.040,
        dead_time_s=0.020,
        noise_sigma_rad=0.002,
        random_seed=42,
        solver="analytical",
    )

    assert len(records) == 100
    assert records[0]["sequence_id"] == 1
    assert records[0]["data_source"] == "synthetic"
    assert records[0]["generator_solver"] == "independent_analytical"
    assert gt["solver"] == "analytical"
    assert gt["dead_time_s"] == 0.020


def test_independent_validation_matrix_discretization_uncertainty():
    """Verify independent validation matrix verifies zero systematic sample bias across full test grid."""
    results = run_comprehensive_independent_validation_matrix(random_seed=42)

    assert len(results) >= 15
    identifiable_count = 0
    unidentifiable_count = 0

    for r in results:
        if not r.is_identifiable:
            unidentifiable_count += 1
            assert r.identified_gain is None
            assert r.identified_dead_time_ms is None
            assert r.identified_tau_ms is None
            continue

        identifiable_count += 1
        assert r.identified_gain is not None
        assert pytest.approx(r.identified_gain, abs=0.05) == r.gt_gain
        # Time constant recovered accurately across solvers
        assert r.tau_error_ms <= 2.0, f"Tau error too large: {r.tau_error_ms} ms on {r.benchmark_type}"

        # Strictly proper ZOH eliminates the systematic 1-sample bias!
        # Assert dead-time error is within sub-sample tolerance (max 1.5 ms or dt/4),
        # completely ruling out the previous +1*dt (5-20 ms) systematic shift!
        dt = r.sample_period_ms
        assert r.dead_time_error_ms is not None
        assert (
            r.dead_time_error_ms <= max(1.5, dt / 4.0)
        ), f"Dead time error {r.dead_time_error_ms} ms on {r.benchmark_type} (th={r.gt_dead_time_ms}ms, dt={dt}ms) exceeds tolerance"

    assert identifiable_count >= 15
    assert unidentifiable_count >= 1
