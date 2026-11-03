# EARS-RL Project Development Roadmap

## Phase 0 — Repository Bootstrap & Tooling [COMPLETED]
- [x] Package structure, typing, exception hierarchy, Pydantic schemas.
- [x] 8 domain-specific Agent Skills (`.agents/skills/`).
- [x] Model Context Protocol (MCP) servers: Context7 and Filesystem scoped to workspace.
- [x] CLI commands (`doctor`, `config validate`, `data validate`, `bench mock`, `identify`, `evaluate`, `sim check`, `hardware status`).
- [x] Automated toolchain verification suite.

## Phase 1 — Mock Actuator Bench [COMPLETED]
- [x] End-to-end `MockActuator` with first-order dynamics, delay, and saturation.
- [x] `MockEncoder` with quantization and noise.
- [x] `BufferedDataLogger` for CSV and JSONL telemetry streaming.
- [x] Dataset validator checking timestamp monotonicity and sequence continuity.
- [x] Baseline system identification (Nominal and Fixed Delay models).
- [x] Scientific metrics (MAE, RMSE, latency statistics, event classification metrics).

## Phase 1.1 — Scientific Integrity Audit & Hardware Readiness [COMPLETED]
- [x] Corrected `NominalActuatorModel.fit()` (reports `fit_loss: None`, zero trainable parameters).
- [x] Decoupled `sample_period_ms` ($T_s$) from `command_to_feedback_delay_ms` ($\tau_{\text{RTT}}$) and `configured_simulated_delay_ms`.
- [x] Dynamic sample period calculation in `FixedDelayActuatorModel.fit()` from telemetry timestamps.
- [x] Classified dataset missing fields into Mandatory, Expected, and Mode-Dependent Optional.
- [x] Provenance integrity validator: prevents relabeling synthetic datasets as real hardware.
- [x] Action Shield NaN/non-finite command guard and intervention logging.
- [x] Single-Axis Binary Communication Protocol V1 (`src/ears_rl/hardware/protocol.py`, CRC-16, Ping/Pong RTT sync, Watchdog, FSM).
- [x] Pre-Flight Hardware Readiness Checklist (`docs/hardware_readiness.md`).
- [x] Full scientific integrity test suite (96 passing tests, 0 failures, 0 warnings).

## Phase 1.2 — Grey-Box System Identification & Bench Readiness [COMPLETED]
- [x] Implemented First-Order grey-box model (`FirstOrderActuatorModel`) without dead time.
- [x] Implemented FOPDT grey-box model (`FOPDTActuatorModel`) parametrically separating pure dead time $\theta$ from time constant $\tau$.
- [x] Implemented Rate-Limited FOPDT model (`FOPDTRateLimitedActuatorModel`) with velocity saturation.
- [x] Resolved delay misidentification root cause: clearly distinguished apparent lumped delay ($\theta + \tau$) from pure dead time ($\theta$).
- [x] Built Synthetic Benchmark Generator (`SyntheticBenchmarkGenerator`) with ground truth across Cases A–G.
- [x] Enforced persistent excitation requirements: static/uninformative input (Case G) strictly flagged as `unidentifiable` without parameter fabrication.
- [x] Disjoint train/held-out evaluation splits (chirp training vs step evaluation).
- [x] Added `--benchmark` CLI flag to `ears-rl identify`.
- [x] Rigorous TMC2209 driver safety rules (component-specific $R_{\text{sense}}$ and motor rating validation).
- [x] 7 new unit tests in `tests/unit/test_grey_box_identification.py` (total 103 passing tests).

## Phase 2 — Safe Hardware Commissioning [COMPLETED]
- [x] Protocol formalization: framing (`0xAA 0x55`), CRC-16-CCITT, Little-Endian, bounded payloads ($\le 256$ B).
- [x] Sliding-window byte stream reassembly (`StreamFrameParser` in Python, `stream_parser_t` in C).
- [x] Dual watchdog architecture decoupling: communication heartbeat ($500\text{ ms}$) vs continuous motion trajectory watchdog ($50\text{ ms}$).
- [x] ESP32-S3 safe firmware skeleton (`firmware/esp32_s3/`): pinout locked (`PIN_ENABLE=6` HIGH, `PIN_STEP=4` LOW), motion rejected with `ERR_COMMISSIONING_MODE (9)`.
- [x] Canonical test vectors verified across Python (`tests/unit/test_protocol.py`) and C host test runner (`test_protocol_host.exe`, 5/5 passing).
- [x] Safe commissioning CLI command (`ears-rl hardware commission [--mock | --port <PORT>]`) generating structured diagnostic JSON reports.
- [x] Scientific integrity ablation study (`tests/unit/test_scientific_delay_ablation.py`) proving mathematical separation between pure transport latency $\tau_d$ and first-order mechanical lag $\tau$.
- [x] Quality gates: Ruff 0 errors, Pytest 81 passing tests (zero physical motion commanded in tests).

## Phase 1.3 — Independent Validation & Physical Bench Preparation [COMPLETED]
- [x] Corrected TMC2209 motor current and VREF formula per Datasheet Rev 1.09; inverted formula implemented and tested.
- [x] Restructured electrical documentation: common star GND topology, power rail separation (VIO vs VM), decoupling bulk capacitor ($\ge 100\,\mu\text{F}$) overvoltage protection.
- [x] Fixed `_delay_signal` padding bug (`initial_val` vs `u[0]`), eliminating initial step dead time erasure.
- [x] Decoupled generator from estimator via independent closed-form analytical step solver and high-order RK4 continuous solver (`src/ears_rl/identification/independent_validation.py`).
- [x] Quantified continuous-time vs discrete ZOH discretization phase shift and sampling resolution limit ($\pm \Delta t$).
- [x] Reimplemented `FOPDTRateLimitedActuatorModel.fit()` with multi-start bounded optimization and structural mismatch plateau detection.
- [x] Refactored `BasicActionShield` for strict non-finite command rejection (`shielded_command = None`, `is_rejected = True`, `SafetyState.FAULT`) without neutral command substitution.
- [x] Established 5-tier safety boundary semantics: `COMMAND_BLOCKED`, `STEP_PULSES_SUPPRESSED`, `DRIVER_DEASSERTED`, `POWER_DISCONNECTED`, `MECHANICAL_REST`.
- [x] Organized single-axis physical commissioning into 5 progressive gates (Gates A–E) with explicit abort conditions; Gate E strictly reserved for physical lab bench.
- [x] Comprehensive test suite expanded to 120 passing tests; Ruff 0 errors; full CLI workflow validated.

## Phase 1.3.1 — Dead-Time Bias Audit & Hardware Safety Gate [COMPLETED]
- [x] Dissected root cause of systematic 1-sample dead-time bias ($\hat{\theta} = \theta + \Delta t$): identified recurrence updating state before recording, assigning $y[k] = x(t_{k+1})$.
- [x] Enforced strictly proper discrete ZOH recurrence ($y[k] = x(t_k)$ before update) across `FOPDTActuatorModel`, `FOPDTRateLimitedActuatorModel`, `FirstOrderActuatorModel`, and `SyntheticBenchmarkGenerator`.
- [x] Verified exact continuous-time analytical equivalence to machine precision ($< 1 \times 10^{-16}\text{ rad}$) without empirical offsets or fudge factors.
- [x] Expanded decoupled continuous validation suite to 19 cases: verified grid-aligned delays ($\text{error} = 0.00\text{ ms}$), continuous fractional delays ($|\hat{\theta} - \theta| \le 0.35\text{ ms}$), multiple initial conditions ($y_0 = \pm 0.5\text{ rad}$), noise, rate limiting, and uninformative excitation rejection.
- [x] Audited TMC2209 power architecture: clarified $< 15\text{ mm}$ layout target vs datasheet specification, common star GND return routing, and back-EMF / regenerative overvoltage protection (dual-pole E-stop + TVS clamping).
- [x] Replaced hardcoded commissioning constants with configurable `CommissioningGateConfig` defining Preconditions, Measurement Method, Acceptance Criteria, Abort Criteria, Recorded Evidence, and `requires_physical_hardware` status for all 5 gates.
- [x] Verified all safety regression tests: strict non-finite command rejection without neutral substitution, fault-state command blocking, no replay on reconnect, and semantic separation between driver disable and power cut.
- [x] Quality gates passing: 126/126 pytest tests passing, Ruff clean (0 errors), full CLI workflows validated.

## Phase 1.3.2 — Power Protection Correction & Pre-Commissioning Audit [COMPLETED]
- [x] Uncovered the "30V/33V TVS Clamping Fallacy": separated $V_{\text{RWM}}$, $V_{\text{BR}}$, and $V_{\text{C}}$ ($I_{\text{PP}}$); proved `SMBJ30CA` ($48.4\text{ V}$) and `SMBJ33CA` ($53.3\text{ V}$) exceed TMC2209 absolute maximum ratings ($32.0\text{ V}$ inductive / $33.0\text{ V}$ bridge max).
- [x] Modeled inductive coil energy ($E_{\text{inductive}} = \frac{1}{2} L I^2 \times 2$) and bus capacitor voltage rise ($V_{\text{final}} = \sqrt{V_{\text{init}}^2 + 2E/C}$) under abrupt supply cutoff and regenerative braking.
- [x] Classified 24V passive TVS protection as `UNRESOLVED — HARDWARE POWER-UP BLOCKED` in code and documentation; prohibited fabricated part numbers.
- [x] Clarified electrical terminology in Commissioning Gate B: renamed `isolation_resistance_min_kohm` to `unpowered_vm_to_gnd_min_resistance_kohm` (pre-power DC non-short test, NOT galvanic isolation).
- [x] Explicitly distinguished DC ground continuity ($R \le 0.2\,\Omega$) from high-frequency return path impedance at PWM switching frequencies ($20\text{ kHz} - 40\text{ kHz}$).
- [x] Documented mechanical consequences of holding torque loss upon power cut: horizontal coasting vs vertical gravitational back-driving drop hazards.
- [x] Implemented `TVSSpec` dataclass, `evaluate_tvs_protection_envelope()`, energy/voltage rise calculation functions, and 5 comprehensive unit tests (total 131 passing tests).
- [x] Documented quantization uncertainty bound ($\pm \Delta t / 2$) and reaffirmed that synthetic validation is not proof of physical motor system identification.
- [x] Quality gates passing: 131/131 pytest tests passing, Ruff 0 errors, all CLI workflows functional.

## Phase 1.3.3 — TMC2209 Datasheet Compliance & Final Pre-Power Review [COMPLETED]
- [x] Corrected TMC2209 silicon voltage boundaries per official Datasheet Rev 1.09 (§19 Absolute Maximum Ratings: $V_{VS} \le 32.0\text{ V}$ inductive, $V_{\text{VMAX}} \le 33.0\text{ V}$ bridge max; §20.1 Operational Range: $5.5\text{ V} - 29.0\text{ V}$ DC, logic $3.00\text{ V} - 5.25\text{ V}$ DC).
- [x] Completely purged all unverified claims citing 36.0 V continuous or 40.0 V surge ratings from codebase, unit tests, and documentation.
- [x] Defined complete manufacturer parameters for TVS candidate catalog (`SMBJ15CA`, `SMBJ18CA`, `SMBJ30CA`, `SMBJ33CA`) including pulse waveform ($10/1000\,\mu\text{s}$) and positive temperature coefficient derating.
- [x] Proved `SMBJ30CA` ($V_{\text{C}} = 48.4\text{ V}$) and `SMBJ33CA` ($V_{\text{C}} = 53.3\text{ V}$) exceed official $32.0\text{ V}$ inductive and $33.0\text{ V}$ bridge maximum ratings by $15.4\text{ V} - 20.3\text{ V}$.
- [x] Re-evaluated motor magnetic energy models: mathematically distinguished quadrature sinusoidal microstepping ($E = \frac{1}{2} L I_{\text{peak}}^2 = 4.0\text{ mJ}$) from full-step 2-phase on ($E = L I_{\text{peak}}^2 = 8.0\text{ mJ}$).
- [x] Implemented rotor/load kinetic energy model ($E_{\text{kin}} = \frac{1}{2} (J_{\text{rotor}} + J_{\text{load}}) \omega^2$) and bulk capacitor voltage rise with electrolytic tolerance ($C_{\text{eff}} = C_{\text{nom}} \times (1 - \text{tol}/100)$).
- [x] Enforced mock commissioning gating integrity: mock executions strictly mark physical Gates A and B as `BLOCKED`, Gate D as `NOT_EXECUTED`, and Gate E as `NOT_EXECUTED` (preventing mock simulation from passing physical gates).
- [x] Decoupled dead-time estimation uncertainty taxonomy: separated sample period, grid search spacing, parameter recovery error, and Cramér-Rao statistical uncertainty.
- [x] Quality gates passing: 135/135 pytest tests passing, Ruff 0 errors, all CLI workflows functional.

## Phase 1.3.4 — Energy Accounting & Transient Protection Review [COMPLETED]
- [x] Corrected supply filtering capacitor requirements per TMC2209 Datasheet Rev 1.09, §21.3: $47\,\mu\text{F}$ electrolytic baseline recommendation, $100\,\mu\text{F}$ EARS-RL project engineering target, $100\text{ nF} - 1\,\mu\text{F}$ MLCC on $V_{\text{S}}$, and $2.2\,\mu\text{F} - 4.7\,\mu\text{F}$ MLCC on 5VOUT.
- [x] Reconciled reported 98 mJ benchmark against physical motor parameters: verified baseline NEMA 17 total energy is $36.125\text{ mJ}$ ($4.0\text{ mJ}$ magnetic + $32.125\text{ mJ}$ kinetic), spiking $80\,\mu\text{F}$ bus to $38.46\text{ V} > 33.0\text{ V}$. Identified that 98 mJ was a high-speed ($85.5\text{ rad/s}$ / $816.7\text{ RPM}$) test fixture value.
- [x] Implemented structured `compute_energy_accounting()` and `EnergyAccountingResult` covering 5 distinct physical cases (Case 1 stationary, Case 2 rotating, Case 3 continuous supply, Case 4 dissipative, Case 5 missing parameters).
- [x] Proved the active brake chopper resistor continuous wattage fallacy: $5\,\Omega, 25\text{ W}$ dump resistor absorbs $162.45\text{ W}$ peak instantaneous power ($6.5\times$ continuous rating). Disproved that continuous wattage proves pulse survival without manufacturer pulse energy rating ($E_{\text{pulse}} \ge 2.0\text{ J}$).
- [x] Classified active brake chopper as `CONCEPTUAL_DESIGN — UNVERIFIED` and identified loss-of-logic-power gate drive collapse hazard.
- [x] Proved freewheeling body diode bridge rectification risk during E-stop: deasserting `ENABLE` does not prevent back-EMF pumping if rotor is spinning under load inertia.
- [x] Updated TVS `SMBJ18CA` evaluation: strictly maintained hardware power-up blocked (`CONDITIONALLY_QUALIFIED_TRANSIENT_ONLY — HARDWARE POWER-UP BLOCKED` on $\le 15\text{ V}$, `NORMAL_CONDUCTION_RISK` on $24\text{ V}$).
- [x] Added 9 unit tests to `tests/unit/test_tmc2209_safety.py` (total 144 passing tests, 0 Ruff errors).

## Phase 1.3.5 — Energy Traceability & Final Release Gate [COMPLETED]
- [x] Eliminated TVS 0.6 J rectangular pulse fallacy: derived true double-exponential integral ($E = 0.8656\text{ J}$ for $600\text{ W}$ $10/1000\,\mu\text{s}$) and classified application kinetic energy capacity as strictly `UNDETERMINED`.
- [x] Parameterized and logged Case 3 continuous supply injection: $V=24\text{ V}, I=1\text{ A}, t_{\text{detect}}=1\text{ ms}, t_{\text{contactor}}=5\text{ ms} \implies E_{\text{supply}}=144\text{ mJ}, E_{\text{net}}=180.125\text{ mJ}, V=71.27\text{ V}$ on $80\,\mu\text{F}$. Missing parameters strictly return `CASE_5_INDETERMINATE_MISSING_PARAMETERS`.
- [x] Standardized capacitor voltage rise calculations into 3-tiered taxonomy (`IDEALIZED_UPPER_BOUND_SCENARIO`, `MODEL_BASED_ESTIMATE`, `MEASURED_PHYSICAL_VOLTAGE`) with 4 declared physical assumptions.
- [x] Evaluated multi-architecture protection (voltage derating to 12-15V, active brake chopper, high-side electronic disconnect with reverse diode, mechanical holding brake).
- [x] Reaffirmed strict pre-power safety invariant: `HARDWARE POWER-UP BLOCKED`.
- [x] Added 5 unit tests to `tests/unit/test_tmc2209_safety.py` (total 149 passing tests, 0 Ruff errors).

## Phase 1.3.5.1 — Energy Model Correction & Mock-Only Audit [COMPLETED]
- [x] Rectified TVS pulse energy mathematical model: normalized double-exponential pulse integral ($p(t) = A (e^{-t/\tau_1} - e^{-t/\tau_2}) \implies E = 0.8617\text{ J}$ for $P_{\text{PPM}} = 600\text{ W}$, $t_{\text{peak}} = 10.0\,\mu\text{s}$). Disproved $1/\ln 2$ as a double-exponential general relation.
- [x] Established strict TVS energy metric taxonomy: `DATASHEET_PEAK_PULSE_POWER`, `MATHEMATICAL_PULSE_ENERGY_ESTIMATE`, and `APPLICATION_PULSE_ENERGY_CAPACITY` (strictly `UNDETERMINED` for motor waveforms without $Z_{\theta JA}(t)$).
- [x] Reclassified 12–15V + SMBJ18CA as `EVALUATED_CANDIDATE — UNVERIFIED` (`HARDWARE POWER-UP BLOCKED`).
- [x] Formulated power supply energy as integral $E_{\text{supply}} = \int_0^{t_{\text{fault}}} V_{\text{source}}(t) I_{\text{source}}(t) dt$; noted $I_{\text{ps}} = 1.0\text{ A}$ is an assumed upper-bound current limit, not measured current.
- [x] Confirmed 71.27 V is an `IDEALIZED_UPPER_BOUND_SCENARIO` on $80\,\mu\text{F}$ at $24\text{ V}$ with zero losses.
- [x] Disproved $E_{\text{supply}} = 0$ claim for high-side electronic disconnect: non-zero latency ($t_{\text{fault}} > 0$) causes continuous injection, and downstream capacitor energy remains on driver rail.
- [x] Marked brake chopper resistor pulse rating requirement ($E_{\text{pulse}}$) as `UNDETERMINED` without target deceleration trajectory and duty cycle.
- [x] Reconciled full test suite: 153/153 passing across 18 test files (explained previous 108 subset breakdown). Ruff 0 errors.

## Phase 1.3.5.2 — Mathematical Consistency & Evidence Audit [COMPLETED]
- [x] Resolved double-exponential intermediate quantity normalization inconsistency: proved exact mutual consistency for $\tau_1 = 1426.18\,\mu\text{s}, \tau_2 = 1.4496\,\mu\text{s}, P_{\text{PPM}} = 600.0\text{ W}$ yielding $t_{\text{peak}} \approx 10.0000\,\mu\text{s}$, $\text{norm} \approx 0.9920$ ($0.99200344$), $A \approx 604.84\text{ W}$ ($604.8366\text{ W}$), and $E \approx 0.8617\text{ J}$ ($0.861729\text{ J}$). Eliminated contradictory draft values ($0.9990$, $600.6\text{ W}$).
- [x] Formalized pulse waveform model as instantaneous power dissipation $p(t)$; declared implicit assumption of approximately constant clamping voltage $V_{\text{C}}$ or lumped dynamic impedance. Reaffirmed that this integral is strictly an idealized mathematical property, not measured energy or datasheet-certified absorption capability.
- [x] Rectified TVS 10/1000 µs surge waveform provenance: verified standardization under Bellcore/Telcordia GR-1089-CORE, IEEE C62.35, and JEDEC JESD210; eliminated inaccurate citations of IEC 61000-4-5.
- [x] Formally classified Littelfuse SMBJ Fig. 1 peak pulse power ratings: $600\text{ W}$ at $1\text{ ms}$ (datasheet rated specification), $\sim 180\text{ W}$ at $10\text{ ms}$ (approximate visual graph reading), and $<60\text{ W}$ at $100\text{ ms}$ (approximate visual graph reading / asymptotic trend toward $5.0\text{ W}$ continuous rating).
- [x] Reclassified 10–100 ms motor back-EMF duration as a hypothetical unverified engineering scenario assumption, not a proven physical property of the bench.
- [x] Enforced parameter traceability for Case 3 and disconnect: explicitly tagged $I_{\text{ps}} = 1.0\text{ A}$, $t_{\text{detect}}$, and contactor/switch-off delays as `[ASSUMED — NOT MEASURED]`, maintained $71.27\text{ V}$ as `IDEALIZED_UPPER_BOUND_SCENARIO`, and added non-negative parameter bounds checks.
- [x] Removed unreachable dead duplicate return block in `src/ears_rl/hardware/safety/tmc2209.py`.
- [x] Added 2 new rigorous regression tests in `tests/unit/test_tmc2209_safety.py`. Total test count: 155/155 passing across 18 test files, Ruff 0 errors, Git worktree clean with zero tracked diffs.
- [x] Strictly maintained `hardware_enabled: false` and `HARDWARE POWER-UP BLOCKED`.

## Phase 1.3.5.3 — Datasheet Thermal Conditions & Source Pinning [COMPLETED]
- [x] Pinned exact Littelfuse SMBJ Series datasheet PDF artifact to repository (`docs/datasheets/Littelfuse_TVS_Diode_SMBJ_Datasheet.pdf`, SHA-256: `8c61bd4e70a9cd0506005cc70cd1ba5b0c8e76c8545f18cbfd19a78e16b08aa4`, Catalog Rev. 2005/04, pp. 267–270).
- [x] Decoupled thermal and power conditions: $600\text{ W}$ non-repetitive peak pulse power ($10/1000\,\mu\text{s}$, mounted on $5.0 \times 5.0\text{ mm}$ ($0.2 \times 0.2''$) copper pad to each terminal) vs $5.0\text{ W}$ steady-state dissipation strictly under infinite-heat-sink conditions at lead temperature $T_L = 50^\circ\text{C}$ ($R_{\theta JL} \approx 20^\circ\text{C/W}$), representing a **CONDITIONAL RATING — NOT DIRECTLY APPLICABLE TO A TYPICAL PCB WITHOUT THERMAL VALIDATION**. Proved that steady-state free-air dissipation on standard $5.0 \times 5.0\text{ mm}$ copper pad to each terminal is only an estimated $\approx 1.25\text{ W}$ based on typical $R_{\theta JA} \approx 100^\circ\text{C/W}$ (a thermal estimate dependent on mounting geometry and ambient conditions, not a universal guaranteed continuous-power rating); disproved that pulse derating converges to the 5 W infinite-heat-sink rating.
- [x] Classified Figure 1 horizontal span: $0.1\,\mu\text{s} \le t_d \le 10\text{ ms}$; reading at $10\text{ ms}$ is $\sim 180\text{ W}$ (approximate visual graph reading); reading at $100\text{ ms}$ is UNAVAILABLE / NOT COVERED (Fig. 1 terminates at 10 ms; no extrapolation).
- [x] Pinned surge standard editions: REA Bulletin 345-65 / PE-60 (Fig. 3), Telcordia GR-1089-CORE Issue 4/6 Section 4, IEEE C62.35-2010 Section 4.3; disproved IEC 61000-4-5.
- [x] Verified mathematical consistency for instantaneous power integral: $\tau_1 = 1426.18\,\mu\text{s}, \tau_2 = 1.4496\,\mu\text{s}, t_{\text{peak}} \approx 10.0000\,\mu\text{s}, \text{norm} \approx 0.99200344, A \approx 604.8366\text{ W}, E \approx 0.861729\text{ J}$. Maintained `APPLICATION_PULSE_ENERGY_CAPACITY = UNDETERMINED`.
- [x] Distinguished mechanical contactor drop-out latency ($t_{\text{contactor}} = 5.0\text{ ms}$) from solid-state switch turn-off latency ($t_{\text{switch\_off}} = 5 - 10\,\mu\text{s}$), each explicitly tagged `[ASSUMED — NOT MEASURED]`.
- [x] Maintained 71.27 V as `IDEALIZED_UPPER_BOUND_SCENARIO` on $80\,\mu\text{F}$ at $24\text{ V}$.
- [x] Added 2 new rigorous regression tests in `tests/unit/test_tmc2209_safety.py` (total test count: 157 passing tests across 18 test files, Ruff 0 errors).
- [x] Strictly maintained `hardware_enabled: false` and `HARDWARE POWER-UP BLOCKED`.

## Phase 2.1 — Physical Single-Axis Test Bench [READY FOR HARDWARE EXECUTION]
- [ ] Connect physical ESP32-S3 and TMC2209 driver to test bench rig.
- [ ] Physical TMC2209 driver characterization and hard stop calibration.
- [ ] Real-world tracking error vs. step loss measurement.

## Phase 3 — Actuator Response Modeling & System ID [PLANNED]
- [ ] Residual and hybrid event-aware model training from physical bench datasets.
- [ ] Latency distribution fitting.

## Phase 4 — Isaac Lab Integration [PLANNED]
- [ ] Actuator digital twin integration in Isaac Lab.
- [ ] Domain randomization across latency and transmission stiffness.

## Phase 5 — RL Policy Training & Action Shield [PLANNED]
- [ ] Baselines B0–B4 and Method P (EARS-RL).
- [ ] Risk-constrained policy evaluation.
