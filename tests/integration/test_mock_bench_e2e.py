"""Integration test verifying end-to-end flow: Mock Bench -> Validation -> SysID."""

from pathlib import Path

from ears_rl.config.loader import load_experiment_config
from ears_rl.core.clock import MockClock
from ears_rl.data.validation import validate_dataset
from ears_rl.hardware.mock.bench import MockActuatorBench
from ears_rl.identification.baseline import run_baseline_identification


def test_mock_bench_end_to_end_flow(tmp_path: Path):
    """Verify full pipeline: MockActuatorBench -> CSV -> validate_dataset -> run_baseline_identification."""
    cfg_path = Path("configs/hardware/bench_mock.yaml")
    config = load_experiment_config(cfg_path)

    out_dir = tmp_path / "mock_session"
    clock = MockClock(start_ns=1_000_000_000)

    # 1. Run bench session (1.0 second, 100 Hz = 100 samples)
    bench = MockActuatorBench(config=config, output_dir=out_dir, clock=clock, random_seed=42)
    data_file = bench.run_session(duration_s=1.0, sample_rate_hz=100.0, trajectory_type="chirp")

    assert data_file.exists()
    assert (out_dir / "metadata.json").exists()

    # 2. Validate output dataset
    val_report = validate_dataset(data_file)
    assert val_report.is_valid is True
    assert val_report.statistics["record_count"] == 100
    assert val_report.statistics["metadata_data_source"] == "synthetic"

    # 3. Run baseline system identification
    id_results = run_baseline_identification(data_file, output_dir=tmp_path / "id_reports")
    assert id_results["data_source"] == "synthetic"
    assert id_results["sample_count"] == 100
    assert "fixed_delay_model" in id_results
    assert id_results["fixed_delay_model"]["identified_delay_ms"] >= 0.0
    assert (tmp_path / "id_reports" / "identification_report.json").exists()


def test_reproducibility_with_seed(tmp_path: Path):
    """Verify identical random seeds produce identical tracking data."""
    cfg = load_experiment_config("configs/hardware/bench_mock.yaml")

    clock1 = MockClock(start_ns=1_000_000_000)
    clock2 = MockClock(start_ns=1_000_000_000)

    bench1 = MockActuatorBench(config=cfg, output_dir=tmp_path / "run1", clock=clock1, random_seed=123)
    file1 = bench1.run_session(duration_s=0.5, sample_rate_hz=50.0, trajectory_type="sine")

    bench2 = MockActuatorBench(config=cfg, output_dir=tmp_path / "run2", clock=clock2, random_seed=123)
    file2 = bench2.run_session(duration_s=0.5, sample_rate_hz=50.0, trajectory_type="sine")

    content1 = file1.read_text(encoding="utf-8")
    content2 = file2.read_text(encoding="utf-8")
    assert content1 == content2
