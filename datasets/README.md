# EARS-RL Datasets Directory

This directory stores raw and processed actuator bench and simulation recordings.

## Directory Structure

```
datasets/
├── raw/
│   └── {experiment_id}/
│       └── {session_id}/
│           ├── data.csv (or data.jsonl)
│           └── metadata.json
└── processed/
    └── {experiment_id}/
        └── {session_id}/
```

## Scientific Data Invariant

Every dataset record and metadata file must declare its scientific origin:
- `synthetic`: Computationally generated via mock classes (`MockActuator`).
- `simulated`: Physics engine rollout (Isaac Lab, PyBullet).
- `real_hardware`: Physical sensor telemetry from physical test bench.

Large dataset files (`*.csv`, `*.parquet`) are ignored by Git. Only `README.md` is tracked.
