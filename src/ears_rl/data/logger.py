"""Buffered data logger implementing DataLoggerInterface for EARS-RL.

Supports streaming write to CSV and JSONL formats with resource safety,
periodic auto-flush, and deterministic session management.
"""

from __future__ import annotations

import csv
import json
import logging
from pathlib import Path
from typing import Any

from ears_rl.core.exceptions import DataError
from ears_rl.core.interfaces import DataLoggerInterface
from ears_rl.data.schema import BenchRecord

logger = logging.getLogger(__name__)


class BufferedDataLogger(DataLoggerInterface):
    """High-throughput buffered logger for bench telemetry and experiment logs."""

    def __init__(
        self,
        output_dir: Path | str,
        buffer_size: int = 100,
        file_format: str = "csv",
    ) -> None:
        self.output_dir = Path(output_dir)
        self.buffer_size = max(1, buffer_size)
        self.file_format = file_format.lower()
        if self.file_format not in ("csv", "jsonl"):
            raise ValueError(f"Unsupported format '{file_format}'. Must be 'csv' or 'jsonl'")

        self._session_id: str | None = None
        self._experiment_id: str | None = None
        self._buffer: list[dict[str, Any]] = []
        self._record_count: int = 0
        self._is_active: bool = False
        self._data_file: Path | None = None
        self._metadata_file: Path | None = None
        self._csv_writer: csv.DictWriter | None = None
        self._csv_file_handle: Any = None
        self._fieldnames: list[str] = list(BenchRecord.model_fields.keys())

    def start_session(self, session_id: str, metadata: dict[str, Any]) -> None:
        """Start a new logging session and create storage directories."""
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self._session_id = session_id
        self._experiment_id = metadata.get("experiment_id", "default_exp")
        self._record_count = 0
        self._buffer.clear()

        # Target file paths
        if self.file_format == "csv":
            self._data_file = self.output_dir / "data.csv"
            self._csv_file_handle = open(self._data_file, "w", newline="", encoding="utf-8")
            self._csv_writer = csv.DictWriter(self._csv_file_handle, fieldnames=self._fieldnames)
            self._csv_writer.writeheader()
        else:
            self._data_file = self.output_dir / "data.jsonl"
            # Truncate / create new
            open(self._data_file, "w", encoding="utf-8").close()

        self._metadata_file = self.output_dir / "metadata.json"
        meta_dict = dict(metadata)
        if "session_id" not in meta_dict:
            meta_dict["session_id"] = session_id
        if "experiment_id" not in meta_dict:
            meta_dict["experiment_id"] = self._experiment_id
        self._write_metadata(meta_dict)
        self._is_active = True
        logger.info("Started logging session '%s' -> %s", session_id, self._data_file)

    def append(self, record: dict[str, Any]) -> None:
        """Append a record to the buffer and flush if capacity reached."""
        if not self._is_active:
            raise DataError("Cannot append: Logging session is not active")

        self._buffer.append(record)
        self._record_count += 1

        if len(self._buffer) >= self.buffer_size:
            self.flush()

    def log_record(self, record: dict[str, Any]) -> None:
        """Alias for append()."""
        self.append(record)

    def flush(self) -> None:
        """Flush memory buffer to storage."""
        if not self._buffer or not self._is_active:
            return

        try:
            if self.file_format == "csv" and self._csv_writer is not None:
                for row in self._buffer:
                    # Sanitize list fields for CSV
                    clean_row = {}
                    for k in self._fieldnames:
                        val = row.get(k)
                        if isinstance(val, list):
                            clean_row[k] = ",".join(str(x) for x in val)
                        else:
                            clean_row[k] = val
                    self._csv_writer.writerow(clean_row)
                self._csv_file_handle.flush()
            elif self.file_format == "jsonl" and self._data_file is not None:
                with open(self._data_file, "a", encoding="utf-8") as f:
                    f.writelines(json.dumps(item) + "\n" for item in self._buffer)
        except Exception as e:
            raise DataError(f"Failed to flush data logger buffer: {e}") from e
        finally:
            self._buffer.clear()

    def close(self) -> None:
        """Flush remaining buffer and close file handles."""
        if self._is_active:
            self.flush()
            if self._csv_file_handle is not None:
                try:
                    self._csv_file_handle.close()
                except Exception:
                    pass
                self._csv_file_handle = None
                self._csv_writer = None
            self._is_active = False
            logger.info("Closed logging session '%s', total %d records", self._session_id, self._record_count)

    def end_session(self) -> None:
        """Alias for close()."""
        self.close()

    def get_record_count(self) -> int:
        return self._record_count

    @property
    def data_file_path(self) -> Path | None:
        return self._data_file

    def _write_metadata(self, metadata_dict: dict[str, Any]) -> None:
        """Serialize metadata JSON alongside data file."""
        if self._metadata_file is not None:
            with open(self._metadata_file, "w", encoding="utf-8") as f:
                json.dump(metadata_dict, f, indent=2)
