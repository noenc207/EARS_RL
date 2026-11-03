"""Scientific Integrity Regression Tests for EARS-RL V1.1.

Verifies:
1. Sample period (e.g. 10 ms) is NOT reported as command-to-feedback latency.
2. Pure delay of 70 ms is clearly distinguished from 10 ms sample period.
3. Estimator reports actual non-zero residual error on model mismatch.
4. Nominal model fit_loss is None, not fabricated 0.0.
5. Datasets missing mandatory required fields are rejected by validator.
6. Datasets missing permitted optional fields are accepted and properly categorized.
7. Latency calculation returns None when paired timestamps are missing.
8. Latency calculation rejects subtracting unsynchronized clock domains.
9. Synthetic provenance is strictly preserved across metadata and reports.
10. Evaluator handles empty datasets, NaNs, and infinite values gracefully.
11. Seed determinism is strictly maintained for synthetic data generation.
12. Automated tests strictly maintain hardware motion disabled invariant.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from ears_rl.actuator_model.evaluation import (
    compute_regression_metrics,
    evaluate_dataset_tracking,
)
from ears_rl.actuator_model.fixed_delay import FixedDelayActuatorModel
from ears_rl.actuator_model.nominal import NominalActuatorModel
from ears_rl.config.loader import load_experiment_config
from ears_rl.core.clock import MockClock
from ears_rl.core.types import HardwareMode, JointCommand
from ears_rl.data.logger import BufferedDataLogger
from ears_rl.data.schema import BenchRecord, DatasetMetadata
from ears_rl.data.validation import validate_dataset
from ears_rl.hardware.mock.actuator import MockActuator
from ears_rl.hardware.mock.bench import MockActuatorBench
from ears_rl.identification.baseline import run_baseline_identification


# 1. Test sample period vs command-feedback latency distinction
def test_sample_period_not_reported_as_latency():
    """Verify sample period (10 ms at 100 Hz) is not conflated with command-to-feedback latency."""
    records = []
    # 5 samples at 100 Hz (dt = 10 ms), without paired command-feedback timestamps
    for i in range(5):
        records.append({
            "sequence_id": i + 1,
            "timestamp_ns": 1_000_000_000 + i * 10_000_000,  # 10 ms apart
            "command_position_rad": float(i) * 0.1,
            "measured_motor_position_rad": float(i) * 0.1,
            "command_to_feedback_delay_ms": None,  # Not measured!
            "data_source": "synthetic",
        })

    eval_res = evaluate_dataset_tracking(records)
    timing = eval_res["timing_metrics"]

    # Sample period must be derived from timestamp differences: ~10.0 ms
    assert timing["sample_period"] is not None
    assert pytest.approx(timing["sample_period"]["mean_ms"], rel=1e-3) == 10.0

    # Command-to-feedback delay must be None, NOT 10.0 ms!
    assert timing["command_to_feedback_delay"] is None


# 2. Test 70 ms pure delay distinguished from 10 ms sample period
def test_distinguish_70ms_pure_delay_from_10ms_sample_period():
    """Verify that a 70 ms pure delay at 100 Hz sampling (10 ms dt) is identified as 70 ms, not 10 ms."""
    dt_ms = 10.0
    true_delay_ms = 70.0  # Exactly 7 sample steps
    n_samples = 150

    t = np.arange(n_samples) * (dt_ms / 1000.0)
    cmd = np.sin(2.0 * np.pi * 1.0 * t)  # 1 Hz sine

    # Shift command by 7 steps (70 ms) to create pure delayed output
    delay_steps = int(round(true_delay_ms / dt_ms))
    meas = np.pad(cmd, (delay_steps, 0))[:n_samples]

    dataset = []
    for i in range(n_samples):
        dataset.append({
            "sequence_id": i + 1,
            "timestamp_ns": int(i * dt_ms * 1_000_000),
            "command_position_rad": float(cmd[i]),
            "measured_motor_position_rad": float(meas[i]),
            "sample_period_ms": dt_ms,
            "configured_simulated_delay_ms": true_delay_ms,
            "data_source": "synthetic",
        })

    model = FixedDelayActuatorModel()
    fit_metrics = model.fit(dataset)

    # Must identify the true delay (70 ms), NOT the sample period (10 ms)
    assert fit_metrics["sample_period_ms"] == 10.0
    assert fit_metrics["identified_delay_ms"] == 70.0
    assert fit_metrics["best_shift_steps"] == 7
    # Residual RMSE after shift must be practically zero for pure delay
    assert fit_metrics["residual_rmse_rad"] < 1e-4


# 3. Test estimator reports actual error when model is mismatched
def test_estimator_reports_actual_error_on_mismatch():
    """Verify that when fitting an actuator with dynamics using pure delay, residual RMSE is non-zero."""
    clock = MockClock(start_ns=1_000_000_000)
    # Actuator with heavy first-order lag (tau = 100 ms) + pure delay (20 ms)
    actuator = MockActuator(clock=clock, tau_s=0.10, latency_ms=20.0)
    actuator.connect()
    actuator.enable_motion()

    dataset = []
    dt_s = 0.01  # 100 Hz
    for step in range(100):
        t_s = step * dt_s
        target_rad = float(np.sin(2.0 * np.pi * 2.0 * t_s))  # Fast 2 Hz trajectory
        cmd = JointCommand(joint_id="j0", position_rad=target_rad, sequence_id=step + 1)
        actuator.send_position_command(cmd)
        actuator.step_simulation(dt_s)
        clock.tick_ms(dt_s * 1000.0)

        enc_pos = actuator.encoder.read().position_rad
        dataset.append({
            "sequence_id": step + 1,
            "timestamp_ns": clock.now_ns(),
            "command_position_rad": target_rad,
            "measured_motor_position_rad": enc_pos,
            "sample_period_ms": 10.0,
            "data_source": "synthetic",
        })

    model = FixedDelayActuatorModel()
    fit_res = model.fit(dataset)

    # Estimator must report actual non-zero residual RMSE due to dynamic lag
    assert fit_res["residual_rmse_rad"] is not None
    assert fit_res["residual_rmse_rad"] > 0.05
    assert fit_res["nominal_rmse_rad"] > fit_res["residual_rmse_rad"]


# 4. Test Nominal model fit_loss is None, not 0.0
def test_nominal_model_fit_loss_is_none():
    """Verify NominalActuatorModel does not claim fit_loss = 0.0 since it has no learnable parameters."""
    model = NominalActuatorModel()
    res = model.fit([{"command_position_rad": 1.0, "measured_motor_position_rad": 0.5}])

    assert res["fit_loss"] is None
    assert res["status"] == "not_applicable"
    assert res["trainable_parameters"] == 0


# 5. Test validator rejects missing required fields
def test_validator_rejects_missing_mandatory_field(tmp_path: Path):
    """Verify validator rejects dataset missing mandatory 'timestamp_ns' or 'sequence_id'."""
    csv_file = tmp_path / "missing_req.csv"
    with open(csv_file, "w", encoding="utf-8") as f:
        # sequence_id is missing!
        f.write("experiment_id,session_id,timestamp_ns,joint_id,data_source\n")
        f.write("exp,s1,1000000000,j0,synthetic\n")

    report = validate_dataset(csv_file)
    assert report.is_valid is False
    assert any("sequence_id" in err.lower() for err in report.errors)


# 6. Test validator accepts permitted optional missing fields
def test_validator_accepts_permitted_optional_missing_fields(tmp_path: Path):
    """Verify validator accepts dataset where optional mode-dependent fields are None."""
    log_dir = tmp_path / "optional_ok"
    logger = BufferedDataLogger(output_dir=log_dir, buffer_size=10, file_format="csv")
    logger.start_session("s1", {"experiment_id": "exp", "data_source": "synthetic"})

    for i in range(5):
        rec = BenchRecord(
            experiment_id="exp",
            session_id="s1",
            sequence_id=i + 1,
            timestamp_ns=(i + 1) * 10_000_000,
            joint_id="j0",
            command_position_rad=0.5,
            measured_motor_position_rad=0.49,
            wall_time_utc=None,  # Optional
            command_velocity_rad_s=None,  # Optional
            command_acceleration_rad_s2=None,  # Optional
            data_source="synthetic",
        )
        logger.append(rec.model_dump())
    logger.close()

    report = validate_dataset(log_dir / "data.csv")
    assert report.is_valid is True
    assert len(report.errors) == 0
    # Unrecorded optional fields must be categorized
    unrec = report.statistics.get("unrecorded_optional_fields", {})
    assert "wall_time_utc" in unrec
    assert "command_velocity_rad_s" in unrec


# 7. Test latency returns None when paired timestamps are missing
def test_latency_calculation_missing_timestamps():
    """Verify latency is null when command_sent or feedback_received timestamp is missing."""
    records = [
        {
            "sequence_id": 1,
            "timestamp_ns": 1_000_000_000,
            "command_sent_timestamp_ns": None,
            "feedback_received_timestamp_ns": 1_000_010_000,
            "data_source": "synthetic",
        }
    ]
    res = evaluate_dataset_tracking(records)
    assert res["timing_metrics"]["command_to_feedback_delay"] is None


def test_latency_calculation_rejects_incompatible_clock_domains(tmp_path: Path):
    """Verify latency calculation and validator reject incompatible clock domains / negative latency."""
    # 1. evaluate_dataset_tracking rejects negative latency
    records = [
        {
            "sequence_id": 1,
            "timestamp_ns": 1_000_000_000,
            "command_to_feedback_delay_ms": -10.0,  # Negative latency is physically impossible
            "data_source": "synthetic",
        }
    ]
    res = evaluate_dataset_tracking(records)
    assert res["timing_metrics"]["command_to_feedback_delay"] is None

    # 2. validate_dataset flags incompatible clock domains as error
    csv_file = tmp_path / "incompatible_clocks.csv"
    with open(csv_file, "w", encoding="utf-8") as f:
        f.write("experiment_id,session_id,sequence_id,timestamp_ns,joint_id,data_source,"
                "command_sent_timestamp_ns,feedback_received_timestamp_ns\n")
        f.write("exp,s1,1,1000000000,j0,synthetic,2000000000,1000000000\n")

    report = validate_dataset(csv_file)
    assert report.is_valid is False
    assert any("clock domain" in err.lower() or "time travel" in err.lower() for err in report.errors)


def test_end_to_end_delay_discrimination_in_identification(tmp_path: Path):
    """Verify full baseline identification pipeline distinguishes 70ms pure delay from 10ms sample period."""
    csv_file = tmp_path / "data.csv"
    dt_ms = 10.0
    true_delay_ms = 70.0
    n_samples = 150

    t = np.arange(n_samples) * (dt_ms / 1000.0)
    cmd = np.sin(2.0 * np.pi * 1.0 * t)
    delay_steps = int(round(true_delay_ms / dt_ms))
    meas = np.pad(cmd, (delay_steps, 0))[:n_samples]

    with open(csv_file, "w", encoding="utf-8") as f:
        f.write("experiment_id,session_id,sequence_id,timestamp_ns,joint_id,data_source,"
                "command_position_rad,measured_motor_position_rad,sample_period_ms,configured_simulated_delay_ms\n")
        for i in range(n_samples):
            f.write(
                f"exp,s1,{i + 1},{int(i * dt_ms * 1_000_000)},j0,synthetic,"
                f"{cmd[i]:.6f},{meas[i]:.6f},{dt_ms},{true_delay_ms}\n"
            )

    meta_file = tmp_path / "metadata.json"
    with open(meta_file, "w", encoding="utf-8") as f:
        json.dump({
            "experiment_id": "exp",
            "session_id": "s1",
            "data_source": "synthetic",
            "sample_rate_hz_expected": 100.0,
            "sample_rate_hz_measured": 100.0,
            "total_records": n_samples,
        }, f)

    out_reports = tmp_path / "reports"
    id_results = run_baseline_identification(csv_file, output_dir=out_reports)

    # 1. Distinguishes sample period vs pure delay
    audit = id_results["timing_and_latency_audit"]
    assert audit["sample_period_ms"] == 10.0
    assert audit["configured_simulated_delay_ms"] == 70.0
    assert audit["identified_pure_delay_ms"] == 70.0
    assert audit["delay_estimation_error_ms"] == 0.0

    # 2. Nominal model fit loss is strictly None (zero-hypothesis)
    assert id_results["nominal_model"]["fit_metrics"]["fit_loss"] is None
    assert id_results["nominal_model"]["fit_metrics"]["status"] == "not_applicable"

    # 3. Provenance is preserved
    assert id_results["data_source"] == "synthetic"



# 8. Test provenance preservation across identification reports
def test_provenance_preservation_in_identification(tmp_path: Path):
    """Verify identification report faithfully propagates 'synthetic' data origin."""
    cfg = load_experiment_config("configs/hardware/bench_mock.yaml")
    out_dir = tmp_path / "mock_provenance"
    bench = MockActuatorBench(config=cfg, output_dir=out_dir, random_seed=42)
    data_file = bench.run_session(duration_s=0.5, sample_rate_hz=50.0, trajectory_type="sine")

    id_results = run_baseline_identification(data_file, output_dir=tmp_path / "id_reports")
    assert id_results["data_source"] == "synthetic"
    report_file = tmp_path / "id_reports" / "identification_report.json"
    with open(report_file, "r", encoding="utf-8") as f:
        saved_data = json.load(f)
    assert saved_data["data_source"] == "synthetic"


# 9. Test provenance mismatch detection (synthetic data labeled real_hardware)
def test_provenance_mismatch_detected(tmp_path: Path):
    """Verify validator catches metadata claiming 'real_hardware' when data is synthetic."""
    csv_file = tmp_path / "data.csv"
    with open(csv_file, "w", encoding="utf-8") as f:
        f.write("experiment_id,session_id,sequence_id,timestamp_ns,joint_id,data_source,data_quality_flags\n")
        f.write("exp,s1,1,1000000000,j0,synthetic,synthetic\n")

    meta_file = tmp_path / "metadata.json"
    meta = DatasetMetadata(
        experiment_id="exp",
        session_id="s1",
        data_source="real_hardware",  # False claim!
        total_records=1,
    )
    with open(meta_file, "w", encoding="utf-8") as f:
        f.write(meta.model_dump_json())

    report = validate_dataset(csv_file, metadata_path=meta_file)
    assert report.is_valid is False
    assert any("integrity violation" in err.lower() or "provenance" in err.lower() for err in report.errors)


# 10. Test evaluation handles empty datasets and non-finite numbers safely
def test_evaluation_empty_and_non_finite():
    """Verify evaluate_dataset_tracking handles empty datasets and NaNs/Infs without crashing."""
    empty_res = evaluate_dataset_tracking([])
    assert empty_res["sample_count"] == 0
    assert empty_res["position_tracking"]["mae_rad"] is None

    # Dataset with non-finite values
    noisy_records = [
        {
            "command_position_rad": float("nan"),
            "measured_motor_position_rad": 1.0,
            "timestamp_ns": 1_000_000_000,
            "data_source": "synthetic",
        },
        {
            "command_position_rad": 1.0,
            "measured_motor_position_rad": float("inf"),
            "timestamp_ns": 1_010_000_000,
            "data_source": "synthetic",
        },
    ]
    noisy_eval = evaluate_dataset_tracking(noisy_records)
    assert noisy_eval["sample_count"] == 2

    reg_metrics = compute_regression_metrics(
        np.array([float("nan"), 1.0]), np.array([1.0, float("inf")])
    )
    assert np.isnan(reg_metrics["mae"])


# 11. Test seed determinism reproduces synthetic data
def test_seed_determinism(tmp_path: Path):
    """Verify identical random seeds generate bitwise identical synthetic bench records."""
    cfg = load_experiment_config("configs/hardware/bench_mock.yaml")

    clock1 = MockClock(start_ns=1_000_000_000)
    clock2 = MockClock(start_ns=1_000_000_000)

    bench1 = MockActuatorBench(config=cfg, output_dir=tmp_path / "run_seed_a", clock=clock1, random_seed=999)
    file1 = bench1.run_session(duration_s=0.3, sample_rate_hz=50.0, trajectory_type="chirp")

    bench2 = MockActuatorBench(config=cfg, output_dir=tmp_path / "run_seed_b", clock=clock2, random_seed=999)
    file2 = bench2.run_session(duration_s=0.3, sample_rate_hz=50.0, trajectory_type="chirp")

    assert file1.read_text(encoding="utf-8") == file2.read_text(encoding="utf-8")


# 12. Test safety invariant: hardware motion disabled by default in tests
def test_zero_real_hardware_motion_in_tests():
    """Verify that all default actuator instances operate with HardwareMode.MOCK and disabled motion."""
    actuator = MockActuator()
    assert actuator.hardware_mode == HardwareMode.MOCK
    assert actuator.is_motion_enabled is False
    assert actuator.connection_state.value == "disconnected"
