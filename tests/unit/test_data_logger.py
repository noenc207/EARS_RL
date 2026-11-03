"""Unit tests for BufferedDataLogger lifecycle."""

from pathlib import Path

from ears_rl.data.logger import BufferedDataLogger
from ears_rl.data.schema import BenchRecord


def test_logger_lifecycle(tmp_path: Path):
    """Verify logger open, buffered append, auto flush, and close."""
    log_dir = tmp_path / "test_session"
    logger = BufferedDataLogger(output_dir=log_dir, buffer_size=5, file_format="csv")

    meta = {"experiment_id": "exp_01", "session_id": "s01", "data_source": "synthetic"}
    logger.start_session("s01", meta)

    # Append records
    for i in range(12):
        record = BenchRecord(
            experiment_id="exp_01",
            session_id="s01",
            sequence_id=i + 1,
            timestamp_ns=(i + 1) * 10_000_000,
            joint_id="j0",
            command_position_rad=0.1 * i,
            measured_motor_position_rad=0.09 * i,
            data_source="synthetic",
        )
        logger.append(record.model_dump())

    assert logger.get_record_count() == 12
    logger.close()

    data_file = log_dir / "data.csv"
    meta_file = log_dir / "metadata.json"
    assert data_file.exists()
    assert meta_file.exists()

    # Read back CSV lines (1 header + 12 rows = 13 lines)
    lines = data_file.read_text(encoding="utf-8").strip().split("\n")
    assert len(lines) == 13
