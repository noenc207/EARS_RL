"""Integration tests for application layer use cases and workflows."""

from __future__ import annotations

import json
from pathlib import Path

from ears_rl.application import (
    RunBenchmarkWorkflow,
    RunCommissioningWorkflow,
    RunEvaluationWorkflow,
    RunIdentificationWorkflow,
)
from ears_rl.config.loader import load_experiment_config
from ears_rl.hardware.protocol import MockBenchTransceiver


def test_run_benchmark_workflow_lifecycle_and_manifest(tmp_path: Path):
    """Verify RunBenchmarkWorkflow executes safely and produces data, metadata, and manifest."""
    cfg = load_experiment_config("configs/hardware/bench_mock.yaml")
    out_dir = tmp_path / "benchmark_run"

    workflow = RunBenchmarkWorkflow(config=cfg, output_dir=out_dir, random_seed=123)
    res = workflow.execute(duration_s=0.5, trajectory="sine", sample_rate_hz=50.0)

    assert Path(res["data_file"]).exists()
    assert Path(res["metadata_file"]).exists()
    assert Path(res["manifest_file"]).exists()
    assert res["total_records"] == 25

    with open(res["manifest_file"], "r", encoding="utf-8") as f:
        manifest = json.load(f)

    assert manifest["experiment_id"] == cfg.experiment_id
    assert manifest["data_source"] == "synthetic"
    assert manifest["total_records"] == 25
    assert manifest["duration_s"] == 0.5


def test_run_commissioning_workflow_mock():
    """Verify RunCommissioningWorkflow executes cleanly on MockBenchTransceiver."""
    transceiver = MockBenchTransceiver(commissioning_mode=True)
    workflow = RunCommissioningWorkflow(transport=transceiver, timeout_s=1.0)
    report = workflow.execute()

    assert report["overall_status"] == "PASSED"
    assert report["mode"] == "MOCK / SYNTHETIC"
    assert report["checks"]["ping_pong"]["status"] == "PASS"
    assert report["checks"]["motion_rejection_safety_invariant"]["status"] == "PASS"


def test_run_identification_workflow(tmp_path: Path):
    """Verify RunIdentificationWorkflow runs baseline SysID on benchmark data."""
    cfg = load_experiment_config("configs/hardware/bench_mock.yaml")
    out_dir = tmp_path / "id_bench"
    bench_wf = RunBenchmarkWorkflow(config=cfg, output_dir=out_dir, random_seed=42)
    bench_res = bench_wf.execute(duration_s=0.5, trajectory="chirp", sample_rate_hz=50.0)

    id_wf = RunIdentificationWorkflow(output_dir=tmp_path / "id_out")
    id_results = id_wf.execute(dataset_path=bench_res["data_file"])

    assert id_results["data_source"] == "synthetic"
    assert id_results["sample_count"] == 25
    assert "nominal_model" in id_results
    assert "fixed_delay_model" in id_results
    assert id_results["fixed_delay_model"]["fit_metrics"]["status"] == "fitted"
    assert id_results["fixed_delay_model"]["identified_delay_ms"] is not None


def test_run_evaluation_workflow(tmp_path: Path):
    """Verify RunEvaluationWorkflow parses dataset and computes standardized tracking metrics."""
    cfg = load_experiment_config("configs/hardware/bench_mock.yaml")
    out_dir = tmp_path / "eval_bench"
    bench_wf = RunBenchmarkWorkflow(config=cfg, output_dir=out_dir, random_seed=42)
    bench_res = bench_wf.execute(duration_s=0.5, trajectory="chirp", sample_rate_hz=50.0)

    eval_wf = RunEvaluationWorkflow()
    metrics = eval_wf.execute(dataset_path=bench_res["data_file"])

    assert metrics["data_source"] == "synthetic"
    assert metrics["sample_count"] == 25
    assert metrics["position_tracking"]["mae_rad"] is not None
    assert metrics["position_tracking"]["rmse_rad"] is not None
    assert metrics["timing_metrics"]["sample_period"]["mean_ms"] is not None
