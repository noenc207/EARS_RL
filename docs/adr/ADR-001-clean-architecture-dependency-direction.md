# ADR-001: Inward Dependency Direction & Clean Architecture Layers

## Status
**ACCEPTED** (2026-10-24)

## Context
During initial development (V1.0), application logic, CLI presentation, and mock hardware orchestration were frequently coupled in single scripts or modules (e.g. `cli.py` calling internal classes directly and mixing presentation with business rules). As we move towards physical single-axis bench testing and Isaac Lab simulation, this coupling risked circular imports, untestable hardware dependencies in unit tests, and leaky abstractions.

## Decision
We enforce a strict 3-tier Clean Architecture:
$$\text{Domain / Core} \leftarrow \text{Application / Use Cases} \leftarrow \text{Infrastructure Adapters / CLI}$$

1. **Domain / Core (`src/ears_rl/core/`)**:
   - Contains abstract interface contracts (`ActuatorBackend`, `CommandTransport`, `TelemetrySource`, `Clock`, `DatasetStore`, `SafetyController`), typed exceptions (`EarsRLError`), and immutable domain types (`JointCommand`, `Timestamp`, `JointPosition`).
   - Zero dependencies on CLI, PySerial, GUI, or external simulators.
2. **Application / Use Cases (`src/ears_rl/application/`)**:
   - Contains business workflow orchestrators (`RunBenchmarkWorkflow`, `RunCommissioningWorkflow`, `RunIdentificationWorkflow`, `RunEvaluationWorkflow`, `ExperimentSession`).
   - Receives dependencies via constructor injection (e.g. taking `CommandTransport` rather than importing physical serial ports).
3. **Adapters / Infrastructure (`src/ears_rl/hardware/`, `src/ears_rl/cli.py`, etc.)**:
   - Implements concrete interfaces (`MockBenchTransceiver`, `SerialBenchTransceiver`, `BufferedDataLogger`, Typer CLI).

## Considered Alternatives
1. *Flat Script Structure*: Simple for rapid prototyping, but violates research reproducibility and creates tight coupling with physical serial hardware in automated test suites.
2. *Microservices / Message Queues*: Overkill for a local robotics bench and policy training loop; adds external process dependencies.

## Consequences
- **Positive**: 100% of unit tests and application workflows execute offline in synthetic/mock mode without requiring physical serial ports or Isaac Sim.
- **Positive**: Clean swapping between mock transceiver and physical ESP32-S3 UART link.
- **Trade-off**: Requires dependency injection in application workflows.
