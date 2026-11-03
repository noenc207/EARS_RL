# ADR-002: Scientific Distinction: Pure Transport Delay vs Dynamic Mechanical Lag

## Status
**ACCEPTED** (2026-10-24)

## Context
In early identification tests, running `FixedDelayActuatorModel` on a chirp benchmark with a 10 ms configured transport delay identified an optimal shift of 70 ms. In initial reports, this caused confusion: was this a bug, a sample rate confusion, or a scientific phenomenon?

Analysis of the actuator dynamics revealed that the simulated motor obeys first-order lag with pure delay:
$$G(s) = \frac{e^{-\tau_d s}}{\tau s + 1}$$
with $\tau_d = 10\text{ ms}$ and $\tau = 40\text{ ms}$.
Under dynamic excitation, the low-pass transfer function introduces a phase lag:
$$\phi(\omega) = -\omega \tau_d - \arctan(\omega \tau) \approx -\omega (\tau_d + \tau)$$
The effective apparent delay observed by discrete cross-correlation is:
$$\tau_{\text{effective}} \approx \tau_d + \tau \approx 10\text{ ms} + 40\text{ ms} = 50\text{ to }70\text{ ms}$$

## Decision
1. We establish a mandatory standardized timing taxonomy:
   - $T_s$ (`sample_period_ms`): consecutive telemetry interval.
   - $\tau_{\text{sim}}$ (`configured_simulated_delay_ms`): transport delay parameterized in simulation.
   - $\tau_{\text{RTT}}$ (`command_to_feedback_delay_ms`): round-trip latency measured in the host clock domain.
   - $\hat{\tau}_d$ (`identified_delay_ms`): delay identified by discrete cross-correlation.
2. In publications and technical documentation, we explicitly declare that zero-order delay models estimate **Lumped Apparent Latency ($\tau_{\text{lumped}}$)**, which absorbs mechanical bandwidth lag $\tau$.
3. Controlled unit tests (`tests/unit/test_scientific_delay_ablation.py`) verify that when $\tau \to 0$, pure transport delay is identified with 100% precision.

## Consequences
- **Positive**: Eliminates misleading scientific claims in academic papers.
- **Positive**: Prevents conflating sample period with transport delay or mechanical inertia.
