# EARS-RL Experiment & Reproducibility Protocol (V2 Foundation)

## 1. Reproducibility Checklist & Artifact Generation

Every experiment run must automatically record and persist:
- Global random seed (`numpy`, `random`, `torch`, and simulation).
- Canonical SHA-256 hash of the experiment configuration (`compute_config_hash()`).
- Scientific `DatasetManifest` (`manifest.json`) capturing record counts, duration, provenance, software version, and tags.
- Exact Python environment versions (verifiable via `ears-rl doctor`).
- Provenance label (`synthetic`, `simulated`, or `real_hardware`) with tamper-evident validation.
- Metadata contract (`metadata.json`) containing sensor specifications and physical units.

---

## 2. Benchmark Protocol & Metrics Discipline

- **Pilot Studies**: Single-seed, rapid smoke testing.
- **Scientific Benchmarks**:
  - $\ge 5$ independent random seeds.
  - Split time-series data by session or motion profile, avoiding random shuffling to prevent temporal autocorrelation leakage.
  - Report mean $\pm$ standard deviation and 95% confidence intervals.
  - Strictly differentiate sample period ($T_s$) from command-to-feedback latency ($\tau_{\text{RTT}}$) and simulated transport delay ($\tau_{\text{sim}}$).
  - Report baseline models honestly: zero-hypothesis nominal predictors have `fit_loss: None`, not `0.0`.
  - Distinguish pure transport delay from dynamic mechanical lag: zero-order delay models identify lumped apparent latency ($\tau_{\text{lumped}} \approx \tau_d + \tau$).
  - Never fabricate or extrapolate ground-truth metrics when independent verification is unavailable.

---

## 3. Data Preprocessing & Integrity Invariants

1. **No Silent Modification**: Telemetry records must never be smoothed, modified, or dropped without explicit logging and provenance disclosure.
2. **Missing Data Taxonomy**: Distinguish `Required`, `Expected`, and `Mode-Dependent Optional` fields. Permitted optional fields remain `null`, never filled with fake zeros.
3. **Empty Data Guard**: Datasets with zero records are rejected by validators as scientifically invalid.
4. **Non-Finite Value Guard**: Any presence of `NaN` or `Inf` in raw datasets triggers a validation error.
