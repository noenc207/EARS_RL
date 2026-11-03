"""Dataset validation and quality assurance for EARS-RL.

Validates datasets against:
1. BenchRecord Pydantic schema.
2. Strict monotonic increasing timestamps.
3. Continuous sequence IDs without gaps or duplicates.
4. NaN, Infinity, and out-of-range physical values.
5. Declared data origin ('synthetic', 'simulated', 'real_hardware').
6. Metadata consistency with recorded telemetry.
"""

from __future__ import annotations

import csv
import json
import logging
import math
from pathlib import Path
from typing import Any

from pydantic import ValidationError

from ears_rl.data.schema import (
    EXPECTED_TELEMETRY_FIELDS,
    MODE_DEPENDENT_OPTIONAL_FIELDS,
    REQUIRED_FIELDS,
    BenchRecord,
    DatasetMetadata,
)

logger = logging.getLogger(__name__)


class ValidationReport:
    """Detailed summary of dataset validation results."""

    def __init__(self) -> None:
        self.errors: list[str] = []
        self.warnings: list[str] = []
        self.statistics: dict[str, Any] = {}

    @property
    def is_valid(self) -> bool:
        return len(self.errors) == 0

    def add_error(self, message: str) -> None:
        self.errors.append(message)
        logger.error("Dataset validation error: %s", message)

    def add_warning(self, message: str) -> None:
        self.warnings.append(message)
        logger.warning("Dataset validation warning: %s", message)

    def to_dict(self) -> dict[str, Any]:
        return {
            "is_valid": self.is_valid,
            "error_count": len(self.errors),
            "warning_count": len(self.warnings),
            "errors": self.errors,
            "warnings": self.warnings,
            "statistics": self.statistics,
        }

    def summary(self) -> str:
        lines = [
            f"Dataset Valid: {'YES' if self.is_valid else 'NO'}",
            f"Total Errors:   {len(self.errors)}",
            f"Total Warnings: {len(self.warnings)}",
        ]
        if self.errors:
            lines.append("\nErrors:")
            for err in self.errors[:10]:
                lines.append(f"  - {err}")
            if len(self.errors) > 10:
                lines.append(f"  ... and {len(self.errors) - 10} more errors")

        if self.warnings:
            lines.append("\nWarnings:")
            for w in self.warnings[:10]:
                lines.append(f"  - {w}")

        if self.statistics:
            lines.append("\nDataset Statistics:")
            for k, v in self.statistics.items():
                if k in ("unrecorded_optional_fields", "missing_values"):
                    continue
                lines.append(f"  {k}: {v}")

            unrec = self.statistics.get("unrecorded_optional_fields")
            if unrec:
                lines.append("\nPermitted Optional / Mode-Dependent Fields (Unrecorded):")
                total_rec = self.statistics.get("record_count", 0)
                for field, count in unrec.items():
                    reason = MODE_DEPENDENT_OPTIONAL_FIELDS.get(field, "Optional field")
                    lines.append(f"  - {field}: {count}/{total_rec} unrecorded [{reason}]")

        return "\n".join(lines)


def validate_dataset(
    data_path: Path | str,
    metadata_path: Path | str | None = None,
) -> ValidationReport:
    """Validate a dataset file and its metadata.

    Args:
        data_path: Path to .csv or .jsonl data file.
        metadata_path: Optional path to metadata.json file.

    Returns:
        ValidationReport instance containing validation outcomes.
    """
    report = ValidationReport()
    path = Path(data_path)

    if not path.exists():
        report.add_error(f"Data file does not exist: {path}")
        return report

    # 1. Parse records
    records: list[dict[str, Any]] = []
    suffix = path.suffix.lower()

    try:
        if suffix == ".csv":
            records = _parse_csv(path, report)
        elif suffix in (".jsonl", ".json"):
            records = _parse_jsonl(path, report)
        else:
            report.add_error(f"Unsupported data format '{suffix}'. Must be .csv or .jsonl")
            return report
    except Exception as e:
        report.add_error(f"Failed to read file {path}: {e}")
        return report

    if not records:
        report.add_error("Dataset file is empty (0 records); cannot validate an empty dataset")
        report.statistics["record_count"] = 0
        return report

    report.statistics["record_count"] = len(records)

    # 2. Check each record against BenchRecord schema & value validity
    timestamps_ns: list[int] = []
    sequence_ids: list[int] = []
    missing_value_counts: dict[str, int] = {}
    unrecorded_optional_counts: dict[str, int] = {}
    missing_required_counts: dict[str, int] = {}
    missing_expected_counts: dict[str, int] = {}
    record_data_sources: set[str] = set()
    nan_inf_counts: int = 0
    valid_record_count = 0

    for i, raw_record in enumerate(records):
        # 2a. Check mandatory required fields explicitly
        for req_field in REQUIRED_FIELDS:
            val = raw_record.get(req_field)
            if val is None or val == "":
                missing_required_counts[req_field] = missing_required_counts.get(req_field, 0) + 1
                if missing_required_counts[req_field] <= 5:
                    report.add_error(f"Record {i}: Missing required mandatory field '{req_field}'")

        # 2b. Check for NaN / Inf in float or string representations
        for k, v in raw_record.items():
            is_non_finite = False
            if isinstance(v, float) and (math.isnan(v) or math.isinf(v)):
                is_non_finite = True
            elif isinstance(v, str) and v.lower() in ("nan", "inf", "-inf", "+inf"):
                is_non_finite = True

            if is_non_finite:
                nan_inf_counts += 1
                if nan_inf_counts <= 5:
                    report.add_error(f"Record {i}: field '{k}' has invalid value {v} (NaN or Inf)")

        try:
            record = BenchRecord(**raw_record)
            valid_record_count += 1
            timestamps_ns.append(record.timestamp_ns)
            sequence_ids.append(record.sequence_id)
            record_data_sources.add(record.data_source)

            # Check data source validity
            if record.data_source not in ("synthetic", "simulated", "real_hardware"):
                report.add_error(
                    f"Record {i}: Invalid data_source '{record.data_source}'. "
                    f"Must be 'synthetic', 'simulated', or 'real_hardware'"
                )

            # Check provenance integrity: synthetic data must never masquerade as real_hardware
            if record.data_source == "real_hardware" and "synthetic" in record.data_quality_flags:
                report.add_error(
                    f"Record {i}: Provenance integrity violation: 'synthetic' flag found in data_quality_flags "
                    f"while data_source is declared as 'real_hardware'."
                )

            # Check latency and paired timestamps sanity
            if record.command_to_feedback_delay_ms is not None and record.command_to_feedback_delay_ms < 0:
                report.add_error(
                    f"Record {i}: Invalid negative latency command_to_feedback_delay_ms={record.command_to_feedback_delay_ms} ms"
                )

            if record.command_sent_timestamp_ns is not None and record.feedback_received_timestamp_ns is not None:
                if record.feedback_received_timestamp_ns < record.command_sent_timestamp_ns:
                    report.add_error(
                        f"Record {i}: Clock domain mismatch or time travel: feedback timestamp "
                        f"({record.feedback_received_timestamp_ns} ns) is earlier than command timestamp "
                        f"({record.command_sent_timestamp_ns} ns)"
                    )

            # Record missing value statistics with classification
            for field_name, value in record.model_dump().items():
                if value is None:
                    missing_value_counts[field_name] = missing_value_counts.get(field_name, 0) + 1
                    if field_name in MODE_DEPENDENT_OPTIONAL_FIELDS:
                        unrecorded_optional_counts[field_name] = (
                            unrecorded_optional_counts.get(field_name, 0) + 1
                        )
                    elif field_name in EXPECTED_TELEMETRY_FIELDS:
                        missing_expected_counts[field_name] = (
                            missing_expected_counts.get(field_name, 0) + 1
                        )

        except ValidationError as e:
            if len(report.errors) < 10:
                report.add_error(f"Record {i} schema violation: {e}")

    report.statistics["valid_records"] = valid_record_count
    report.statistics["missing_values"] = missing_value_counts
    report.statistics["unrecorded_optional_fields"] = unrecorded_optional_counts
    if missing_expected_counts:
        report.statistics["missing_expected_telemetry"] = missing_expected_counts
        for field, count in missing_expected_counts.items():
            report.add_warning(
                f"Expected telemetry field '{field}' is unpopulated in {count}/{len(records)} records."
            )

    # 3. Timestamp monotonicity checks
    if len(timestamps_ns) >= 2:
        non_monotonic = 0
        zero_delta = 0
        dt_ns_list = []

        for j in range(1, len(timestamps_ns)):
            dt = timestamps_ns[j] - timestamps_ns[j - 1]
            if dt < 0:
                non_monotonic += 1
            elif dt == 0:
                zero_delta += 1
            dt_ns_list.append(dt)

        if non_monotonic > 0:
            report.add_error(f"Timestamp monotonicity violated {non_monotonic} times (timestamps decreased)")
        if zero_delta > 0:
            report.add_warning(f"Identical timestamps detected across consecutive records ({zero_delta} occurrences)")

        if dt_ns_list:
            import numpy as np
            dt_arr = np.array(dt_ns_list, dtype=np.float64)
            mean_dt_s = float(np.mean(dt_arr) / 1e9)
            report.statistics["mean_dt_ms"] = float(np.mean(dt_arr) / 1e6)
            report.statistics["std_dt_ms"] = float(np.std(dt_arr) / 1e6)
            if mean_dt_s > 0:
                report.statistics["measured_sample_rate_hz"] = round(1.0 / mean_dt_s, 2)

    # 4. Sequence ID continuity checks
    if len(sequence_ids) >= 2:
        seq_gaps = 0
        for j in range(1, len(sequence_ids)):
            if sequence_ids[j] != sequence_ids[j - 1] + 1:
                seq_gaps += 1
        if seq_gaps > 0:
            report.add_warning(f"Sequence ID non-consecutive jumps/gaps detected ({seq_gaps} occurrences)")

    # 5. Metadata verification
    meta_path = Path(metadata_path) if metadata_path else path.parent / "metadata.json"
    if meta_path.exists():
        _validate_metadata_file(meta_path, len(records), record_data_sources, report)
    else:
        report.add_warning(f"Metadata file not found at {meta_path}")

    return report


def _parse_csv(path: Path, report: ValidationReport) -> list[dict[str, Any]]:
    """Parse CSV rows converting numeric strings to typed numbers."""
    records = []
    with open(path, "r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            cleaned: dict[str, Any] = {}
            for k, v in row.items():
                if k == "data_quality_flags":
                    if v is None or v == "" or v == "[]":
                        cleaned[k] = []
                    else:
                        cleaned[k] = [x.strip() for x in v.split(",") if x.strip()]
                elif v is None or v == "":
                    cleaned[k] = None
                elif v.lower() in ("nan", "inf", "-inf", "+inf"):
                    cleaned[k] = float(v)
                else:
                    try:
                        if "." in v:
                            cleaned[k] = float(v)
                        else:
                            cleaned[k] = int(v)
                    except (ValueError, TypeError):
                        cleaned[k] = v
            records.append(cleaned)
    return records


def _parse_jsonl(path: Path, report: ValidationReport) -> list[dict[str, Any]]:
    """Parse newline-delimited JSON records."""
    records = []
    with open(path, "r", encoding="utf-8") as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError as e:
                report.add_error(f"JSON syntax error on line {line_num}: {e}")
    return records


def _validate_metadata_file(
    meta_path: Path,
    record_count: int,
    record_data_sources: set[str],
    report: ValidationReport,
) -> None:
    """Validate metadata.json contents and provenance consistency with recorded telemetry."""
    try:
        with open(meta_path, "r", encoding="utf-8") as f:
            data = json.load(f)

        meta = DatasetMetadata(**data)

        if meta.data_source not in ("synthetic", "simulated", "real_hardware"):
            report.add_error(
                f"Metadata data_source '{meta.data_source}' is invalid. "
                f"Must be 'synthetic', 'simulated', or 'real_hardware'"
            )

        # Provenance integrity: prevent manual relabeling of synthetic data as real hardware
        if meta.data_source == "real_hardware" and "synthetic" in record_data_sources:
            report.add_error(
                "Scientific Integrity Violation: Metadata declares data_source='real_hardware', "
                "but underlying telemetry records are tagged 'synthetic'. "
                "Relabeling synthetic datasets as real hardware is strictly prohibited."
            )

        if meta.total_records > 0 and meta.total_records != record_count:
            report.add_warning(
                f"Metadata declared total_records={meta.total_records}, but data file has {record_count}"
            )

        report.statistics["metadata_data_source"] = meta.data_source
        report.statistics["metadata_experiment_id"] = meta.experiment_id
    except ValidationError as e:
        report.add_error(f"Metadata schema validation error: {e}")
    except Exception as e:
        report.add_error(f"Failed to parse metadata file {meta_path}: {e}")
