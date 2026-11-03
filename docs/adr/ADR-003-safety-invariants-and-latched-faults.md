# ADR-003: Multi-Layer Safety Invariants, Non-Finite Interception & Latched Faults

## Status
**ACCEPTED** (2026-10-24)

## Context
In robotics research, software bugs, policy divergence, or communication loss can produce dangerous physical runaway motion. Software guardrails must be defense-in-depth across multiple layers rather than relying on a single check in the CLI.

Furthermore, an audit of the initial `SafetyMonitor` revealed that receiving a heartbeat packet automatically cleared `SafetyState.FAULT` to `SafetyState.SAFE`, violating the fundamental principle of latched safety faults.

## Decision
We enforce multi-layer safety invariants across the entire platform:
1. **Hardware Disabled by Default**: `hardware_enabled: false` across all schemas, configs, and instances.
2. **Double Non-Finite Guard**: NaN and Inf commands are intercepted at both `ActionShield` (which clamps to safe neutral within $[min, max]$) and `BaseActuator._pre_command_check()` (which raises `CommandLimitError`).
3. **Neutral Position Envelope Check**: Neutral position cannot be assumed to be 0.0 rad; it must be clamped to the actuator's physical envelope $\text{clip}(0.0, \text{min\_pos}, \text{max\_pos})$.
4. **Strictly Latched Faults**: In `SafetyMonitor`, watchdog expiration or hardware faults transition to `SafetyState.FAULT`. Feeding a watchdog refreshes timestamps but **NEVER** automatically unlatches a fault. A fault can only be cleared by an explicit `reset_fault()` call when preconditions are satisfied.
5. **Physical E-Stop Supremacy**: Software interlocks are complementary. The physical E-stop switch is a hardwired loop directly interrupting motor driver power.
6. **No Motion Replay on Reconnect**: When serial connection drops and reconnects, queues are purged and motion is locked disabled until an explicit arming sequence.

## Consequences
- **Positive**: Complete hardware protection during synthetic tests, commissioning, and bench experiments.
- **Positive**: Verified by 9 dedicated safety invariant tests in `tests/safety/test_safety_invariants.py`.
