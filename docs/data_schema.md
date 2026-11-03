# EARS-RL Data Contract & Schema Specifications (V1.1 Research-Ready)

## 1. BenchRecord Schema Fields & Classifications

Every row in raw bench recordings (`data.csv` or `data.jsonl`) strictly conforms to `BenchRecord` (`src/ears_rl/data/schema.py`).
Fields are classified into three contract tiers:
1. **Mandatory Required**: Must be non-null in every valid record.
2. **Expected Telemetry**: Expected for primary trajectory tracking studies.
3. **Mode-Dependent / Optional**: Permitted to be null when omitted by control mode or test setup.

| Field Name | Type | Unit | Tier | Description & Scientific Semantics | Nullable? |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `experiment_id` | `str` | - | Mandatory | Unique experiment identifier | No |
| `session_id` | `str` | - | Mandatory | Session identifier within experiment | No |
| `sequence_id` | `int` | - | Mandatory | Monotonically increasing sequence ID (gap-free) | No |
| `timestamp_ns` | `int` | ns | Mandatory | Monotonic acquisition timestamp | No |
| `joint_id` | `str` | - | Mandatory | Target joint / actuator ID | No |
| `data_source` | `str` | - | Mandatory | Provenance: `'synthetic'`, `'simulated'`, or `'real_hardware'` | No |
| `command_position_rad` | `float` | rad | Expected | Target position command | Yes |
| `measured_motor_position_rad` | `float` | rad | Expected | Motor shaft encoder position | Yes |
| `sample_period_ms` | `float` | ms | Derived | Consecutive sample interval: $(t_k - t_{k-1})/10^6$ | Yes |
| `command_to_feedback_delay_ms` | `float` | ms | Optional | Measured round-trip latency in identical clock domain | Yes |
| `configured_simulated_delay_ms` | `float` | ms | Optional | Pure transport delay in mock/sim (null on real hardware) | Yes |
| `latency_ms` | `float` | ms | Legacy | Backward-compatible alias for simulated or measured latency | Yes |
| `wall_time_utc` | `str` | ISO-8601 | Optional | Wall-clock reference (omitted in deterministic mock clock) | Yes |
| `command_velocity_rad_s` | `float` | rad/s | Optional | Feed-forward velocity (omitted in position-only control) | Yes |
| `command_acceleration_rad_s2` | `float` | rad/s² | Optional | Feed-forward accel (omitted in position-only control) | Yes |
| `measured_joint_position_rad` | `float` | rad | Optional | Joint output-side position (dual-encoder rigs only) | Yes |
| `measured_velocity_rad_s` | `float` | rad/s | Optional | Measured / estimated joint velocity | Yes |
| `current_amp` | `float` | A | Optional | Motor phase current | Yes |
| `voltage_v` | `float` | V | Optional | DC bus voltage | Yes |
| `estimated_load_nm` | `float` | Nm | Optional | External load torque estimate | Yes |
| `command_sent_timestamp_ns` | `int` | ns | Optional | Host dispatch timestamp | Yes |
| `feedback_received_timestamp_ns` | `int` | ns | Optional | Host feedback arrival timestamp | Yes |
| `actuator_mode` | `str` | - | State | 'normal', 'step_loss', 'stall', 'saturated' | No |
| `event_label` | `str` | - | Event | Online detector or model event tag | Yes |
| `event_source` | `str` | - | Event | 'detector', 'model_estimate', 'ground_truth', 'synthetic_injection' | Yes |
| `ground_truth_status` | `str` | - | Ground Truth | Independent calibrated truth status | Yes |
| `data_quality_flags` | `list[str]` | - | Quality | Quality annotations ('valid', 'synthetic', 'noisy') | No |
| `software_version` | `str` | - | Provenance | EARS-RL software version string | Yes |
| `hardware_config_hash` | `str` | - | Provenance | SHA-256 hash of configuration | Yes |

---

## 2. Scientific Timing & Latency Distinction (Inviolable Rules)

1. **Sample Period ($\Delta t$) $\neq$ Latency ($\tau$)**:
   - `sample_period_ms` is the time interval between consecutive feedback telemetry records ($10.0\text{ ms}$ at $100\text{ Hz}$).
   - `command_to_feedback_delay_ms` is the round-trip latency from host dispatch to feedback reception.
   - **Never label sample period as latency**.
2. **Clock Domain Discipline**:
   - Do NOT subtract timestamps originating from unsynchronized clock domains (e.g. host PC clock vs microcontroller clock without round-trip synchronization).
3. **Nullable Semantics**:
   - Unrecorded optional fields MUST remain `null` / `None`. Never replace missing measurements with `0.0` or synthetic interpolations without explicit labeling.
4. **Provenance Integrity**:
   - Datasets with `data_quality_flags: ["synthetic"]` must never have metadata modified to `data_source: "real_hardware"`. The validator strictly detects and blocks this violation.
