# ADR-004: Dataset Contracts, Provenance Preservation & Missing Field Taxonomy

## Status
**ACCEPTED** (2026-10-24)

## Context
Initial telemetry validation in V1.0 reported 200/200 "missing values" on healthy synthetic datasets, confusing mode-dependent optional fields (like dual-encoder output or current sensing) with true missing data. Furthermore, scientific integrity requires strict prevention against synthetic data masquerading as real hardware data.

## Decision
1. **Three-Tier Schema Classification**:
   - `REQUIRED_FIELDS`: Mandatory for all records (`experiment_id`, `session_id`, `sequence_id`, `timestamp_ns`, `joint_id`, `data_source`).
   - `EXPECTED_TELEMETRY_FIELDS`: Required for standard single-axis bench tracking (`command_position_rad`, `measured_motor_position_rad`).
   - `MODE_DEPENDENT_OPTIONAL_FIELDS`: Valid unrecorded sensors with explicit scientific justification (e.g. `wall_time_utc` in deterministic simulation, `current_amp` without current shunt).
2. **Provenance Integrity Guard**:
   - `data_source` must be one of `synthetic`, `simulated`, `real_hardware`.
   - Datasets with `synthetic` quality flags can never declare `data_source: real_hardware` in metadata; validators reject this as a provenance integrity violation.
3. **Empty Dataset Handling**:
   - Datasets with zero records are flagged as invalid (`is_valid: false`), preventing empty files from being used in system identification or model training.

## Consequences
- **Positive**: Clean dataset validation reports distinguishing true data loss from expected sensor absence.
- **Positive**: Cryptographic config hashing and manifest tracking ensure 100% experiment reproducibility.
