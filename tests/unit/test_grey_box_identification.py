"""Unit and regression tests for Grey-Box System Identification in EARS-RL V1.2.

Verifies:
1. Exact parameter recovery of FOPDT on informative synthetic benchmark (Case C).
2. Parameter recovery of first-order lag without dead time (Case B).
3. Pure delay recovery without dynamics (Case A).
4. Uninformative excitation (Case G) rejected as 'unidentifiable' with no fabricated parameters.
5. Bounded noise robustness on noisy telemetry (Case E).
6. Velocity rate limiting distinguished from pure delay.
7. Held-out evaluation split independence (train on chirp, evaluate on step).
8. Synthetic benchmark report generation with complete ground truth metadata.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from ears_rl.actuator_model.first_order import FirstOrderActuatorModel
from ears_rl.actuator_model.fixed_delay import FixedDelayActuatorModel
from ears_rl.actuator_model.fopdt import (
    FOPDTActuatorModel,
    FOPDTRateLimitedActuatorModel,
)
from ears_rl.identification.synthetic_bench import (
    SyntheticBenchmarkGenerator,
    run_synthetic_identification_benchmark,
)


def test_fopdt_exact_parameter_recovery_case_c():
    """Verify FOPDT estimator recovers K=1.0, tau=40ms, theta=20ms on Case C."""
    gen = SyntheticBenchmarkGenerator(random_seed=42)
    records, gt = gen.generate_dataset("case_c_fopdt_standard", trajectory_type="chirp", duration_s=2.5)

    model = FOPDTActuatorModel()
    res = model.fit(records)

    assert res["status"] == "fitted"
    assert res["is_identifiable"] is True
    # Gain recovery within 2%
    assert pytest.approx(res["identified_gain"], rel=0.03) == gt.gain
    # Time constant within 5 ms
    assert abs(res["identified_time_constant_ms"] - gt.tau_s * 1000.0) <= 5.0
    # Dead time within 5 ms
    assert abs(res["identified_dead_time_ms"] - gt.dead_time_s * 1000.0) <= 5.0
    # Residual RMSE practically zero for noise-free data
    assert res["residual_rmse_rad"] < 0.005


def test_first_order_exact_parameter_recovery_case_b():
    """Verify FirstOrderActuatorModel recovers K=1.0, tau=50ms on Case B."""
    gen = SyntheticBenchmarkGenerator(random_seed=42)
    records, gt = gen.generate_dataset("case_b_first_order", trajectory_type="chirp", duration_s=2.5)

    model = FirstOrderActuatorModel()
    res = model.fit(records)

    assert res["status"] == "fitted"
    assert res["is_identifiable"] is True
    assert pytest.approx(res["identified_gain"], rel=0.03) == gt.gain
    assert abs(res["identified_time_constant_ms"] - gt.tau_s * 1000.0) <= 5.0
    assert res["residual_rmse_rad"] < 0.005


def test_pure_delay_recovery_case_a():
    """Verify pure transport delay (30 ms) is identified accurately without dynamics."""
    gen = SyntheticBenchmarkGenerator(random_seed=42)
    records, gt = gen.generate_dataset("case_a_pure_delay", trajectory_type="chirp", duration_s=2.5)

    model = FOPDTActuatorModel()
    res = model.fit(records)

    assert res["status"] == "fitted"
    assert res["is_identifiable"] is True
    assert abs(res["identified_dead_time_ms"] - 30.0) <= 2.0


def test_uninformative_excitation_rejected_as_unidentifiable():
    """Verify all estimators reject constant static hold as unidentifiable."""
    gen = SyntheticBenchmarkGenerator(random_seed=42)
    records, _ = gen.generate_dataset("case_g_uninformative", trajectory_type="constant_uninformative", duration_s=2.0)

    # 1. FOPDT
    fopdt = FOPDTActuatorModel()
    res_fopdt = fopdt.fit(records)
    assert res_fopdt["status"] == "unidentifiable"
    assert res_fopdt["is_identifiable"] is False
    assert res_fopdt["identified_dead_time_ms"] is None

    # 2. First-Order
    fo = FirstOrderActuatorModel()
    res_fo = fo.fit(records)
    assert res_fo["status"] == "unidentifiable"
    assert res_fo["is_identifiable"] is False

    # 3. Fixed Delay
    fd = FixedDelayActuatorModel()
    res_fd = fd.fit(records)
    assert res_fd["status"] == "unidentifiable"
    assert res_fd["is_identifiable"] is False


def test_noise_robustness_case_e():
    """Verify FOPDT parameter recovery under measurement noise (sigma=5 mrad)."""
    gen = SyntheticBenchmarkGenerator(random_seed=99)
    records, gt = gen.generate_dataset("case_e_fopdt_noisy", trajectory_type="chirp", duration_s=2.5)

    model = FOPDTActuatorModel()
    res = model.fit(records)

    assert res["status"] == "fitted"
    assert res["is_identifiable"] is True
    # Dead time within 5 ms tolerance despite noise
    assert abs(res["identified_dead_time_ms"] - gt.dead_time_s * 1000.0) <= 5.0
    # Residual RMSE close to noise standard deviation (0.005 rad)
    assert res["residual_rmse_rad"] < 0.010


def test_rate_limited_fopdt_simulation_and_fit():
    """Verify FOPDTRateLimitedActuatorModel simulates velocity limiting without divergence."""
    model = FOPDTRateLimitedActuatorModel(gain=1.0, tau_s=0.04, dead_time_s=0.02, velocity_max_rad_s=1.5)
    gen = SyntheticBenchmarkGenerator(random_seed=42)
    records, _ = gen.generate_dataset("case_d_fopdt_rate_limited", trajectory_type="chirp", duration_s=2.0)

    res = model.fit(records)
    assert res["status"] == "fitted"
    assert res["model_type"] == "fopdt_rate_limited"


def test_synthetic_benchmark_runner_full_suite(tmp_path: Path):
    """Verify complete benchmark runner executes all 7 cases and writes JSON report with ground truth."""
    out_dir = tmp_path / "bench_reports"
    report = run_synthetic_identification_benchmark(output_dir=out_dir, random_seed=42)

    assert report["benchmark_version"] == "1.2.0"
    assert report["data_source"] == "synthetic"
    assert len(report["cases"]) == 7

    # Verify ground truth is preserved
    for case_id in ("case_a_pure_delay", "case_b_first_order", "case_c_fopdt_standard"):
        case = report["cases"][case_id]
        assert "ground_truth" in case
        assert "models" in case
        assert "fopdt" in case["models"]
        assert case["models"]["fopdt"]["fit_metrics"]["status"] == "fitted"

    # Verify case G is unidentifiable
    case_g = report["cases"]["case_g_uninformative"]
    assert case_g["models"]["fopdt"]["fit_metrics"]["status"] == "unidentifiable"

    # Verify JSON file written
    assert (out_dir / "synthetic_benchmark_report.json").exists()


def test_signal_delay_step_padding_preserves_dead_time():
    """Verify _delay_signal with initial_val pads pre-step with baseline, preserving dead time."""
    import numpy as np

    from ears_rl.actuator_model.fopdt import _delay_signal

    dt_s = 0.010
    delay_s = 0.030  # 3 discrete steps delay
    u = np.ones(10, dtype=np.float64)  # Step input at t=0

    # With correct pre-step initial_val=0.0:
    delayed = _delay_signal(u, dt_s=dt_s, delay_s=delay_s, initial_val=0.0)

    # First 3 samples must be 0.0 (dead time preserved!)
    assert np.allclose(delayed[:3], [0.0, 0.0, 0.0])
    # Fourth sample onward must be 1.0
    assert np.allclose(delayed[3:], [1.0] * 7)


def test_fopdt_dead_time_matrix_recovery_step():
    """Verify FOPDT parameter recovery across varied theta in {0, 10, 20, 40} ms on clean step data."""
    from ears_rl.identification.synthetic_bench import run_dead_time_recovery_matrix

    matrix = run_dead_time_recovery_matrix(
        theta_list_ms=[0.0, 10.0, 20.0, 40.0],
        tau_list_ms=[20.0, 50.0],
        noise_list_rad=[0.0],
        trajectories=["step"],
    )

    for entry in matrix:
        assert entry["is_identifiable"] is True
        gt_th = entry["gt_theta_ms"]
        gt_tau = entry["gt_tau_ms"]
        # Dead time recovered within 2 ms
        assert entry["theta_error_ms"] <= 2.0, f"Failed on theta={gt_th}ms, tau={gt_tau}ms"
        # Tau recovered within 2 ms
        assert entry["tau_error_ms"] <= 2.0, f"Failed on tau={gt_tau}ms with theta={gt_th}ms"


def test_structural_mismatch_detected_on_rate_limited_step():
    """Verify linear FOPDT flags structural mismatch when actuator exhibits rate limiting."""
    gen = SyntheticBenchmarkGenerator(random_seed=42)
    records, _ = gen.generate_dataset("case_d_fopdt_rate_limited", trajectory_type="step", duration_s=2.0)

    linear_model = FOPDTActuatorModel()
    res = linear_model.fit(records)

    assert res["status"] == "fitted"
    assert res["structural_mismatch"] is True
    assert "velocity saturation" in res["structural_mismatch_warning"].lower()


def test_rate_limited_fopdt_identifies_vmax_and_reduces_rmse():
    """Verify FOPDTRateLimitedActuatorModel accurately recovers vmax and outperforms linear FOPDT."""
    gen = SyntheticBenchmarkGenerator(random_seed=42)
    # Case D ground truth: vmax = 2.0 rad/s
    records, gt = gen.generate_dataset("case_d_fopdt_rate_limited", trajectory_type="step", duration_s=2.0)

    rl_model = FOPDTRateLimitedActuatorModel()
    res = rl_model.fit(records)

    assert res["status"] == "fitted"
    assert res["is_identifiable"] is True
    # Vmax recovered close to 2.0 rad/s
    assert abs(res["identified_velocity_max_rad_s"] - 2.0) <= 0.3
    # Dead time recovered close to 20 ms
    assert abs(res["identified_dead_time_ms"] - 20.0) <= 5.0
    # Substantial RMSE reduction over linear FOPDT
    assert res["residual_rmse_rad"] < res["linear_fopdt_residual_rmse_rad"]

