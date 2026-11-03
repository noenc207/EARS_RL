# EARS-RL Architecture & System Blueprint (V2 Foundation)

## 1. Research Objectives

EARS-RL investigates **Event-Aware Risk-Sensitive Reinforcement Learning for Low-Cost Hybrid-Actuated Biped Robots under Step Loss and Control Latency**.

Key technical pillars:
1. **Actuator Modeling**: Hybrid stepper-servo model accounting for transmission compliance, latency, and step slip.
2. **System Identification**: Calibrating response models from single-axis and multi-axis bench data.
3. **Sim-to-Real**: Domain randomization + identified actuator dynamics in NVIDIA Isaac Lab.
4. **Risk-Constrained Control**: Value-at-risk / conditional value-at-risk (CVaR) objectives with real-time Action Shielding.

---

## 2. Clean Architecture Layer Separation

The software is strictly structured according to Clean Architecture dependency rules:
**Dependencies point inward: Domain/Core $\leftarrow$ Application $\leftarrow$ Infrastructure Adapters.**

```
+-----------------------------------------------------------------------------+
|                          ADAPTERS / INFRASTRUCTURE                          |
|  - CLI (Typer / Rich)                                                       |
|  - Mock Actuator & Encoder (MockActuatorBench)                              |
|  - Binary Serial Transport (SerialBenchTransceiver, USB UART)                |
|  - File Storage (BufferedDataLogger, CSV / JSONL)                           |
|  - Simulator Adapter (NVIDIA Isaac Lab plugin)                              |
+-----------------------------------------------------------------------------+
                                       │ (uses)
                                       ▼
+-----------------------------------------------------------------------------+
|                           APPLICATION / USE CASES                           |
|  - RunBenchmarkWorkflow (orchestrates bench session & manifest)             |
|  - RunCommissioningWorkflow (diagnostic handshake & safety rejection check)  |
|  - RunIdentificationWorkflow (dataset validation & baseline SysID)          |
|  - RunEvaluationWorkflow (tracking accuracy & timing metrics)               |
|  - ExperimentSession (fail-safe lifecycle & resource management)             |
+-----------------------------------------------------------------------------+
                                       │ (uses)
                                       ▼
+-----------------------------------------------------------------------------+
|                                DOMAIN / CORE                                |
|  - Domain Types & Records: Timestamp, JointCommand, JointPosition, etc.     |
|  - Enums: HardwareMode, SafetyState, ConnectionState, ActuatorState         |
|  - Interfaces: ActuatorBackend, Clock, CommandTransport, TelemetrySource,   |
|               DatasetStore, ActuatorModel, ExperimentRunner, SafetyMonitor  |
|  - Exceptions: Typed hierarchy inheriting from EarsRLError                  |
|  - Units: Explicit SI physical quantities (_rad, _rad_s, _rad_s2, _ns, _ms) |
+-----------------------------------------------------------------------------+
```

---

## 3. Fundamental Architectural Invariants

1. **Inward Dependency Direction**:
   - `core` has ZERO dependencies on external hardware, CLI, filesystem drivers, or simulation engines.
   - Application workflows receive dependencies via constructor injection; business logic never instantiates raw hardware transports internally.
2. **Hardware Disabled by Default**:
   - `hardware_enabled: false` is the mandatory startup invariant across all schemas, configs, and instances.
   - Automated tests strictly operate in `HardwareMode.MOCK` or `HardwareMode.SIMULATION`. Real hardware motion is strictly prohibited in tests.
3. **Multi-Layer Safety Interlocks**:
   - Hardware cannot be armed with unverified limits (`require_limits_verified: true`).
   - Non-finite commands (NaN/Inf) are blocked at both `ActionShield` and `BaseActuator._pre_command_check()`.
   - Latched faults require explicit recovery (`reset_fault()`); incoming heartbeat packets alone cannot clear a fault.
   - Software interlocks are complementary and NEVER replace a physical hardwired E-stop switch.
4. **Scientific Integrity & Provenance**:
   - Data origin (`synthetic`, `simulated`, `real_hardware`) is permanently tracked in telemetry and manifests.
   - Relabeling synthetic data as real hardware is detected and rejected by data validators.
   - Timing semantics strictly decouple sample interval ($T_s$), simulated transport delay ($\tau_{\text{sim}}$), dynamic mechanical lag ($\tau$), and measured round-trip latency ($\tau_{\text{RTT}}$).
