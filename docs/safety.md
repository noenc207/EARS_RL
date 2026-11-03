# EARS-RL Robotics Hardware Safety & Fail-Safe Guide (V2 Foundation)

## 1. Twelve Mandatory Safety Invariants

1. **Hardware Disabled by Default**: All configuration schemas have `hardware_enabled: false`. Physical motion cannot be triggered accidentally.
2. **No Motion Without Explicit Enable**: Actuator initialization leaves motion locked disabled until `enable_motion()` is deliberately invoked after successful connection and checks.
3. **Configuration Conflict Rejection**: Invalid configurations (e.g. `hardware_enabled: true` with unverified limits or missing serial port) are rejected at configuration load time.
4. **Strict Non-Finite Rejection (No Neutral Substitution)**: Non-finite commands (NaN or Inf) are strictly rejected at `ActionShield` (`shielded_command = None`, `is_rejected = True`, `SafetyState.FAULT`). No motion command—including neutral position—is dispatched to the actuator. `BaseActuator._pre_command_check()` strictly blocks `None` or non-finite commands with `CommandLimitError`.
5. **Configurable Out-of-Bounds Policy**: Commands exceeding mechanical limits support configurable policies (`out_of_bounds_policy: "clamp"` vs `"reject"`). Under `"clamp"`, safe boundary clamping is applied; under `"reject"`, the command is strictly suppressed.
6. **Communication Watchdog ($500\text{ ms}$)**: Expiration of heartbeat transitions system to `SafetyState.FAULT` / `DeviceState.FAULT`, disabling motor bridge excitation.
7. **Motion Trajectory Watchdog ($50\text{ ms}$)**: In running state, setpoints must arrive at $\ge 20\text{ Hz}$. Stalling setpoints for $> 50\text{ ms}$ kills motion pulses to prevent inertial drift.
8. **Physical E-Stop Primacy & Electrical/Mechanical Realities**: Software interlocks are supplementary. An independent, hardwired physical emergency stop switch directly interrupts the 24V motor power bus. E-stop assertion cannot be overridden or cleared by software.
   - *Supply Filtering vs. Transient-Energy Sizing*: TMC2209 Datasheet Rev 1.09, §21.3 recommends an electrolytic capacitor of $47\,\mu\text{F}$ or larger alongside ceramic decoupling ($100\text{ nF} - 1\,\mu\text{F}$) placed close to $V_{\text{S}}$ and $2.2\,\mu\text{F} - 4.7\,\mu\text{F}$ on 5VOUT for normal supply filtering. This datasheet filtering recommendation is NOT proof that a particular capacitance is sufficient for the application's transient energy events. EARS-RL adopts $100\,\mu\text{F}$ as a project engineering choice (unverified for transient fault clearance); bulk capacitors alone do NOT guarantee overvoltage protection from mechanical kinetic regeneration.
   - *Inductive vs Kinetic Energy Accounting & Case 3 Traceability*: Under $E_L = \frac{1}{2} L I^2$ for $L = 4.0\text{ mH}$ and $I = 1.0\text{ A}$, one single phase stores $2.0\text{ mJ}$ ($25.02\text{ V}$ on $80\,\mu\text{F}$) and two identical energized phases store $4.0\text{ mJ}$ total ($26.00\text{ V}$ on $80\,\mu\text{F}$; also $4.0\text{ mJ}$ two-phase total in sinusoidal microstepping at $I_{\text{RMS}} = 1.0\text{ A}, I_{\text{peak}} = 1.414\text{ A}$, or $8.0\text{ mJ}$ in full-step at $1.414\text{ A}$). The figure `4.41 mJ per phase` is unsupported by project parameters and classified as `UNDETERMINED`. Mechanical kinetic energy ($E_{\text{kin}} = 32.125\text{ mJ}$ at $\omega = 50\text{ rad/s}$) combines with coil energy ($36.125\text{ mJ}$ two-phase or $34.125\text{ mJ}$ single-phase) to spike bus voltage to $38.46\text{ V}$ (two-phase) or $37.80\text{ V}$ (single-phase)—or $55.01\text{ V}$ under high-speed $98\text{ mJ}$ conditions—far exceeding TMC2209 silicon absolute maximum ratings ($32.0\text{ V}$ inductive, $33.0\text{ V}$ bridge). Furthermore, delayed disconnect under continuous supply injection ($E_{\text{supply}} = \int_0^{t_{\text{fault}}} V_{\text{source}}(t) I_{\text{source}}(t) dt = 24\text{ V} \times 1.0\text{ A}^* \times 6\text{ ms} = 144.0\text{ mJ}$, where $I_{\text{ps}} = 1.0\text{ A}$ is an assumed upper-bound current limit labeled `[ASSUMED — NOT MEASURED]`, NOT measured current, and timing delays $t_{\text{detect}} = 1.0\text{ ms}$, $t_{\text{contactor}} = 5.0\text{ ms}$ are labeled `[ASSUMED — NOT MEASURED]`) drives net energy to $180.125\text{ mJ}$ (two-phase) or $178.125\text{ mJ}$ (single-phase) and idealized upper-bound voltage to $71.27\text{ V}$ (two-phase) or $70.92\text{ V}$ (single-phase) on $80\,\mu\text{F}$ with zero losses (`IDEALIZED_UPPER_BOUND_SCENARIO`).
   - *Capacitor Voltage Calculation Taxonomy*: Voltage rise calculations ($V = \sqrt{V_0^2 + 2E/C}$) are strictly categorized into `IDEALIZED_UPPER_BOUND_SCENARIO` (theoretical lossless upper ceiling; requires declared assumptions), `MODEL_BASED_ESTIMATE` (ODE simulation with $I^2 R$ and friction losses), or `MEASURED_PHYSICAL_VOLTAGE` (oscilloscope capture). Theoretical upper bounds are never conflated with physical measurements.
   - *TVS Clamping Verification & Energy Taxonomy*: Standard 30V/33V TVS diodes (`SMBJ30CA` $V_{\text{C}} = 48.4\text{ V}$, `SMBJ33CA` $V_{\text{C}} = 53.3\text{ V}$) fail to clamp below $32.0\text{ V} / 33.0\text{ V}$. `SMBJ18CA` causes continuous avalanche conduction on a 24V rail. TVS energy and thermal ratings are categorized per the pinned manufacturer artifact (`docs/datasheets/Littelfuse_TVS_Diode_SMBJ_Datasheet.pdf`, SHA-256: `8c61bd4e70a9cd0506005cc70cd1ba5b0c8e76c8545f18cbfd19a78e16b08aa4`, Catalog Rev. 2005/04):
     1. `DATASHEET_PEAK_PULSE_POWER`: $600\text{ W}$ non-repetitive at $10/1000\,\mu\text{s}$ waveform mounted on $5.0 \times 5.0\text{ mm}$ ($0.2 \times 0.2''$) copper pad to each terminal ($T_A = 25^\circ\text{C}$).
     2. `DATASHEET_STEADY_STATE_POWER`: $5.0\text{ W}$ strictly under infinite-heat-sink conditions at lead temperature $T_L = 50^\circ\text{C}$ ($R_{\theta JL} \approx 20^\circ\text{C/W}$), representing a **CONDITIONAL RATING — NOT DIRECTLY APPLICABLE TO A TYPICAL PCB WITHOUT THERMAL VALIDATION**. Under standard $5.0 \times 5.0\text{ mm}$ ($0.2 \times 0.2''$) copper pad to each terminal in free air, typical junction-to-ambient thermal resistance is $R_{\theta JA} \approx 100^\circ\text{C/W}$, yielding an estimated steady-state dissipation limit of $\approx 1.25\text{ W}$ ($(150 - 25) / 100\text{ W}$). The $1.25\text{ W}$ figure is a thermal estimate dependent on mounting geometry and ambient conditions, not a universal guaranteed continuous-power rating. Non-repetitive pulse derating does NOT converge to the 5 W infinite-heat-sink rating.
     3. `DATASHEET_FIGURE_1_COVERAGE`: Figure 1 covers $0.1\,\mu\text{s} \le t_d \le 10\text{ ms}$ (at $10\text{ ms}$, $P_{\text{PPM}} \approx 180\text{ W}$ approximate visual graph reading); $100\text{ ms}$ is UNAVAILABLE / NOT COVERED (Figure 1 terminates at $10\text{ ms}$, no extrapolation permitted).
     4. `MATHEMATICAL_PULSE_ENERGY_ESTIMATE`: Normalized double-exponential instantaneous power waveform integral with $\tau_1 = 1426.18\,\mu\text{s}, \tau_2 = 1.4496\,\mu\text{s}, t_{\text{peak}} \approx 10.0000\,\mu\text{s}, \text{norm} \approx 0.99200344, A \approx 604.8366\text{ W}, E \approx 0.861729\text{ J}$. This is strictly an idealized mathematical waveform property, not a physical absorption rating. Provenance: REA Bulletin 345-65 / PE-60 (Fig. 3), Telcordia GR-1089-CORE Issue 4/6 Section 4, IEEE C62.35-2010 Section 4.3; IEC 61000-4-5 does not define $10/1000\,\mu\text{s}$.
     5. `APPLICATION_PULSE_ENERGY_CAPACITY`: Strictly `UNDETERMINED` for motor discharges without measured $Z_{\theta JA}(t)$ and current profiles; 10 ms is an unverified hypothetical deceleration duration scenario. On derated $12\text{ V} - 15\text{ V}$ rails, `SMBJ18CA` is classified strictly as `EVALUATED_CANDIDATE — UNVERIFIED` (`HARDWARE POWER-UP BLOCKED`).
   - *Multi-Architecture Protection & Chopper Evaluation*: Active brake chopper is not the sole option; four architectures are evaluated: (1) Supply Voltage Derating to $12\text{ V} - 15\text{ V DC}$ (`EVALUATED_CANDIDATE — UNVERIFIED`), (2) Active Brake Chopper ($28.5\text{ V}$ clamp, $5\,\Omega / 25\text{ W}$; required pulse rating $E_{\text{pulse}}$ is `UNDETERMINED` without target profile; status `CONCEPTUAL_DESIGN — UNVERIFIED`), (3) High-Side Electronic Disconnect with Reverse Blocking Diode (notice: does NOT eliminate supply energy if detection/switch-off latency is non-zero, $t_{\text{fault}} > 0$), and (4) Mechanical Holding Brake.
   - *Freewheeling Body Diode Bridge Rectification*: Pulling driver `ENABLE` HIGH turns off gate drive, but if the rotor continues rotating with load inertia, motor back-EMF ($V_{\text{BEMF}} = K_e \omega$) forward-biases MOSFET parasitic body diodes, acting as an uncontrolled bridge rectifier that pumps high voltage into the disconnected DC bus.
   - *Mechanical Holding Torque Hazard*: Cutting power drops holding torque to detent torque ($\approx 5\% - 10\%$). Horizontal axes coast to rest by friction; vertical axes accelerate in gravitational free fall (back-driving) unless fitted with a mechanical power-off holding brake.
9. **Strictly Latched Faults**: When a fault or watchdog expiration occurs, incoming heartbeat packets (`feed_watchdog()`) refresh timestamps but **NEVER** automatically unlatch the fault. A fault requires an explicit `reset_fault()` command under verified safe conditions.
10. **Zero Hardware in Automated Tests**: Test suites operate strictly in `HardwareMode.MOCK` and synthetic/simulated modes. Physical actuators are never commanded during CI or automated testing.
11. **Resource Initialization Safety**: Failure during resource initialization or communication setup leaves the system in a safe disarmed state.
12. **Exception Safety via `ExperimentSession`**: If an unhandled exception occurs during an experiment session, the `ExperimentSession` context manager automatically disables motion across all actuators and safely closes all loggers and transports.

---

## 2. Watchdog Heartbeat Protocol & Reconnect Semantics

- The software loop sends periodic heartbeats (`PING`) to the controller.
- If no heartbeat is received within `watchdog_timeout_ms` (default 500 ms):
  1. Low-level driver disables H-bridge motor excitation (high impedance).
  2. Controller firmware enters `DeviceState.FAULT`.
  3. Reconnection purges all pending motion command queues to prevent automatic motion replay.
  4. Clearing `FAULT` state requires an explicit `RESET_FAULT` command after confirming safe conditions.

---

## 3. Action Shield Non-Finite Guards & Clamping Policies

- **Strict Non-Finite Suppression**: Any command containing `NaN` or `Inf` is strictly rejected without sending any motion command to hardware (`shielded_command = None`, `is_rejected = True`). Transitions safety state to `SafetyState.FAULT` and logs a critical error. The actuator backend enforces `if command is None: raise CommandLimitError(...)`.
- **Kinematic Policies**: Commands exceeding position or velocity bounds support `"clamp"` (saturates to valid physical bounds) or `"reject"` (blocks command completely). Intervention reasons are logged for scientific ablation.

---

## 4. Multi-Tier Safety Boundary Semantics (V1.3)

Safety states operate across five distinct, non-conflated boundaries:

| Layer | State Name | Action Taken | Physical Actuator Reality |
|---|---|---|---|
| **Software Filter** | `COMMAND_BLOCKED` | `ActionShield` drops command (`shielded_command = None`) | No new software setpoints. Step generator and motor power may still be active. |
| **Firmware Pulse** | `STEP_PULSES_SUPPRESSED` | MCU timer stops step pulse generation | Step pulses cease. Motor coils remain energized with static holding torque. |
| **Driver Bridge** | `DRIVER_DEASSERTED` | TMC2209 `ENABLE` pin set HIGH | Driver bridge freewheels. Motor holding torque drops to zero (detent torque only). |
| **Electrical Power** | `POWER_DISCONNECTED` | Physical NC E-Stop switch opened | Motor bus $V_{\text{M}} = 0\text{ V}$. Driver completely unpowered. |
| **Kinematic Payload**| `MECHANICAL_REST` | Shaft angular velocity $\omega = 0$ | Verified ONLY by encoder feedback, NOT software flags or timer states. |

> [!WARNING]
> Transitioning to `SafetyState.FAULT` in software indicates that software guardrails have blocked new commands. It does NOT guarantee that the physical motor has lost holding torque, that the 24V supply is cut, or that payload momentum has ceased.

