"""Unit tests for dataset schema and validity enforcement."""

from pathlib import Path

from ears_rl.data.logger import BufferedDataLogger
from ears_rl.data.schema import BenchRecord
from ears_rl.data.validation import validate_dataset


def test_validator_accepts_clean_dataset(tmp_path: Path):
    """Verify validator accepts valid monotonic dataset."""
    log_dir = tmp_path / "valid_ds"
    logger = BufferedDataLogger(output_dir=log_dir, buffer_size=10, file_format="csv")
    logger.start_session("s1", {"experiment_id": "exp", "data_source": "synthetic"})

    for i in range(5):
        rec = BenchRecord(
            experiment_id="exp",
            session_id="s1",
            sequence_id=i + 1,
            timestamp_ns=(i + 1) * 10_000_000,
            joint_id="j0",
            data_source="synthetic",
        )
        logger.append(rec.model_dump())
    logger.close()

    report = validate_dataset(log_dir / "data.csv")
    assert report.is_valid is True
    assert len(report.errors) == 0


def test_validator_rejects_non_monotonic_timestamps(tmp_path: Path):
    """Verify validator catches backward-moving timestamps."""
    csv_file = tmp_path / "bad_time.csv"
    with open(csv_file, "w", encoding="utf-8") as f:
        f.write("experiment_id,session_id,sequence_id,timestamp_ns,joint_id,data_source\n")
        f.write("exp,s1,1,2000000000,j0,synthetic\n")
        f.write("exp,s1,2,1000000000,j0,synthetic\n")  # Timestamps decreased!

    report = validate_dataset(csv_file)
    assert report.is_valid is False
    assert any("monotonicity" in e.lower() for e in report.errors)


def test_validator_rejects_invalid_data_source(tmp_path: Path):
    """Verify validator catches unknown data provenance."""
    csv_file = tmp_path / "bad_source.csv"
    with open(csv_file, "w", encoding="utf-8") as f:
        f.write("experiment_id,session_id,sequence_id,timestamp_ns,joint_id,data_source\n")
        f.write("exp,s1,1,1000000000,j0,fake_data_from_nowhere\n")

    report = validate_dataset(csv_file)
    assert report.is_valid is False
    assert any("data_source" in e.lower() for e in report.errors)


def test_validator_rejects_empty_dataset(tmp_path: Path):
    """Verify validator rejects an empty dataset file with 0 records."""
    empty_csv = tmp_path / "empty.csv"
    with open(empty_csv, "w", encoding="utf-8") as f:
        f.write("experiment_id,session_id,sequence_id,timestamp_ns,joint_id,data_source\n")

    report = validate_dataset(empty_csv)
    assert report.is_valid is False
    assert any("empty" in e.lower() for e in report.errors)


def test_validator_rejects_nan_inf_data_values(tmp_path: Path):
    """Verify validator flags non-finite float readings."""
    bad_float_csv = tmp_path / "nan_val.csv"
    with open(bad_float_csv, "w", encoding="utf-8") as f:
        f.write("experiment_id,session_id,sequence_id,timestamp_ns,joint_id,data_source,measured_motor_position_rad\n")
        f.write("exp,s1,1,1000000000,j0,synthetic,NaN\n")

    report = validate_dataset(bad_float_csv)
    assert report.is_valid is False
    assert any("nan" in e.lower() or "invalid value" in e.lower() for e in report.errors)


def test_validator_field_hierarchy_behavior(tmp_path: Path):
    """Verify three-tier field hierarchy: required (error), expected (warning), optional (categorized)."""
    # 1. Dataset missing expected telemetry (command_position_rad, measured_motor_position_rad)
    # but having required fields and omitting optional mode-dependent fields
    csv_file = tmp_path / "expected_missing.csv"
    with open(csv_file, "w", encoding="utf-8") as f:
        f.write("experiment_id,session_id,sequence_id,timestamp_ns,joint_id,data_source\n")
        f.write("exp,s1,1,1000000000,j0,synthetic\n")

    report = validate_dataset(csv_file)
    assert report.is_valid is True  # Valid because required fields are present
    assert len(report.errors) == 0
    assert len(report.warnings) > 0  # Warns because expected telemetry is missing
    assert any("expected telemetry" in w.lower() for w in report.warnings)

    # 2. Dataset with required AND expected fields present, plus metadata, but optional mode-dependent missing
    session_dir = tmp_path / "valid_session"
    session_dir.mkdir()
    csv_complete_telemetry = session_dir / "data.csv"
    with open(csv_complete_telemetry, "w", encoding="utf-8") as f:
        f.write(
            "experiment_id,session_id,sequence_id,timestamp_ns,joint_id,data_source,"
            "command_position_rad,measured_motor_position_rad\n"
        )
        f.write("exp,s1,1,1000000000,j0,synthetic,0.5,0.48\n")

    meta_file = session_dir / "metadata.json"
    with open(meta_file, "w", encoding="utf-8") as f:
        f.write('{"experiment_id": "exp", "session_id": "s1", "data_source": "synthetic", "total_records": 1}')

    report2 = validate_dataset(csv_complete_telemetry, metadata_path=meta_file)
    assert report2.is_valid is True
    assert len(report2.errors) == 0
    assert len(report2.warnings) == 0  # 0 warnings because expected telemetry & metadata are present
    assert "wall_time_utc" in report2.statistics["unrecorded_optional_fields"]
    assert "command_velocity_rad_s" in report2.statistics["unrecorded_optional_fields"]


