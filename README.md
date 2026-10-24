# EARS-RL — Event-Aware Risk-Sensitive Reinforcement Learning

> **Event-Aware Risk-Sensitive Reinforcement Learning for Low-Cost Hybrid-Actuated Biped Robots under Step Loss and Control Latency**

[![Python 3.11+](https://img.shields.io/badge/python-3.11+-blue.svg)](https://www.python.org/downloads/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Safety: Default Disabled](https://img.shields.io/badge/Safety-Hardware%20Disabled-green.svg)](docs/safety.md)
[![Sim-to-Real: Isaac Lab](https://img.shields.io/badge/Sim--to--Real-NVIDIA%20Isaac%20Lab-orange.svg)](docs/architecture.md)

---

## 1. Scientific Overview & Objectives

Bipedal robots driven by low-cost hybrid stepper-servo drivetrains suffer from non-deterministic control latency, transmission compliance, and catastrophic **step loss (desynchronization)** under sudden ground impact shocks or load saturation.

**EARS-RL** formulates a scientific framework combining:
1. **Hybrid Actuator Response Modeling**: Characterizing discrete dynamical regimes (`NORMAL`, `SATURATED`, `STEP_LOSS`, `STALL`, `RECOVERY`, `UNKNOWN`).
2. **Empirical Actuator System Identification**: Calibrating delay distributions and torque-speed envelopes from physical test bench telemetry.
3. **Sim-to-Real Adaptation in NVIDIA Isaac Lab**: Integrating identified actuator digital twins into high-throughput GPU simulations.
4. **Risk-Sensitive Reinforcement Learning & Online Action Shield**: Bounding catastrophic joint failure through Value-at-Risk / CVaR optimization and real-time kinematic shielding.

---

## 2. Decoupled Architecture

The repository enforces strict separation across 13 core layers:

```
[ A. Config & Experiment Management ] (Pydantic v2 schemas, reproducible SHA-256 hashing)
                │
                ▼
[ B. Hardware Abstraction ] ◄──────────┐
  (ActuatorInterface, Encoder, IMU)    │
                │                       │  MOCK / SIMULATOR FIRST
                ▼                       │  hardware_enabled: false default
[ C. Sensor Acquisition & Time Sync ] ──┘
  (TimeProvider, Monotonic Nanoseconds, Sequence IDs)
                │
                ▼
[ D. Data Logging & Dataset Management ]
  (BenchRecord Schema, Buffered Logger, Session Manifests)
                │
                ▼
[ E. Actuator System Identification ]
  (Nominal, Fixed-Delay, Residual, Hybrid Response Models)
                │
                ▼
[ F. Event Detection & Labeling ]
  (Discrete states: NORMAL, SATURATED, STEP_LOSS, STALL, RECOVERY)
                │
                ▼
[ G. Actuator Digital Model ] ──► [ H. Simulation Integration ]
                                   (Isolated NVIDIA Isaac Lab Adapter)
                                                │
                                                ▼
                                   [ I. Reinforcement Learning ]
                                     (Baselines B0–B4, Method P)
                                                │
                                                ▼
[ J. Risk Estimation & Action Shield ] ◄────────┘
  (Kinematic clamping, watchdog guards, safe states)
                │
                ▼
[ K. Benchmark & Statistical Evaluation ]
  (Multi-seed protocols, out-of-distribution evaluations)
                │
                ▼
[ L. Deployment & Runtime Monitoring ] ──► [ M. Visualization & Reporting ]
  (RPi 5 Policy + ESP32-S3 Pulser)         (Tracking errors, latency distributions)
```

---

## 3. Installation

### Prerequisites
- Python 3.11+ (Python 3.11 or 3.12).
- Node.js v20+ / v24+ (for MCP developer tooling).

### Development Installation
```bash
git clone <repo-url>
cd EARS-RL
pip install -e ".[dev]"
```

Verify environment and tooling:
```bash
ears-rl doctor
```

---

## 4. Quickstart: End-to-End Mock Actuator Bench

Run an end-to-end simulated bench session, validate telemetry against the scientific data contract, and execute system identification baselines:

### Step 1: Validate Configuration
```bash
ears-rl config validate --config configs/hardware/bench_mock.yaml
```

### Step 2: Run Mock Bench Session
```bash
ears-rl bench mock --config configs/hardware/bench_mock.yaml --duration-s 2.0 --trajectory chirp
```

### Step 3: Validate Generated Telemetry
```bash
ears-rl data validate --path datasets/raw/bench_mock_001/session_001/data.csv
```

### Step 4: Run Baseline System Identification
```bash
ears-rl identify --input datasets/raw/bench_mock_001/session_001/data.csv --output-dir reports/identification
```

### Step 5: Evaluate Benchmark Tracking Performance
```bash
ears-rl evaluate --input datasets/raw/bench_mock_001/session_001/data.csv
```

---

## 5. Command-Line Interface (CLI) Reference

| Command | Description |
| :--- | :--- |
| `ears-rl doctor` | Diagnostic audit of Python, dependencies, GPU, and Isaac Lab |
| `ears-rl config validate -c <path>` | Validate experiment YAML against Pydantic schemas |
| `ears-rl bench mock -c <path> -d <sec>` | Run simulated single-axis bench session (CPU-only) |
| `ears-rl data validate -p <path>` | Audit dataset schema, timestamp monotonicity, and sequence continuity |
| `ears-rl identify -i <path> -o <path>` | Run Nominal & Fixed-Delay system identification |
| `ears-rl evaluate -i <path>` | Compute MAE, RMSE, and latency distribution statistics |
| `ears-rl sim check` | Inspect NVIDIA Isaac Lab integration status |
| `ears-rl hardware status` | Read-only inspection of hardware bus & safety interlocks |

---

## 6. Hardware Safety Principles

1. **Hardware Disabled by Default**: `hardware_enabled: false` is strictly enforced in all configurations.
2. **Zero Hardware in Tests**: Automated tests run exclusively using `MockActuator` and `MockClock`.
3. **Physical E-Stop Primacy**: Software interlocks are complementary. An independent hardwired E-stop switch must directly cut motor driver power.
4. **Action Shield Guard**: All motion commands are validated and clamped against mechanical limits.

---

## 7. Automated Testing Suite

Run the complete test suite:
```bash
pytest tests/ -v
```

Run toolchain and MCP verification:
```bash
python scripts/verify_toolchain.py
```

---

## 8. Documentation

- [docs/architecture.md](docs/architecture.md) — Detailed layer separation and component contracts
- [docs/installation.md](docs/installation.md) — Setup instructions including PyTorch, CUDA, and Isaac Lab
- [docs/data_schema.md](docs/data_schema.md) — Complete specification of the `BenchRecord` schema
- [docs/safety.md](docs/safety.md) — Hardware fail-safe protocols and watchdog behavior
- [docs/experiment_protocol.md](docs/experiment_protocol.md) — Multi-seed reproducibility guidelines
- [docs/roadmap.md](docs/roadmap.md) — Project phases and milestone checklist
- [docs/AI_TOOLING.md](docs/AI_TOOLING.md) — AI agent tooling, Context7, and Filesystem MCP configuration
