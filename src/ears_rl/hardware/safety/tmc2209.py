"""TMC2209 Stepper Driver VREF and Motor Current Calculations.

Based on official Trinamic / Analog Devices TMC2209 Datasheet (Rev 1.09):
    Section 7: Sense Resistors & Analog Current Scaling (Standalone Mode)

Formulas:
    Full-Scale Peak Current:
        I_FS = [V_FS / (R_SENSE + R_BRD)] * [V_REF / 2.5 V]
    RMS Current (assuming sinusoidal microstepping):
        I_RMS = [0.325 V / (R_SENSE + 0.020 Ohm)] * [1 / sqrt(2)] * [V_REF / 2.5 V]

    Inverted for V_REF:
        V_REF = I_RMS * sqrt(2) * (R_SENSE + 0.020 Ohm) * (2.5 V / 0.325 V)

Applicability Conditions:
    1. Operating in Standalone mode with external sense resistors.
    2. Analog current scaling active via VREF potentiometer pin.
    3. Standard internal reference voltage V_FS = 0.325 V.
    4. Estimated PCB trace resistance R_BRD = 0.020 Ohm.
    5. DOES NOT apply to UART digital current scaling (GLOBALSCALER / IHOLD_IRUN).
    6. DOES NOT apply to internal sense resistor configuration (VSENSE=1 / internal).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

# Default constants from TMC2209 datasheet (Rev 1.09)
DEFAULT_V_FS_V: float = 0.325  # Internal reference full-scale voltage [V]
DEFAULT_R_BRD_OHM: float = 0.020  # Typical PCB layout trace resistance [Ohm]
DEFAULT_V_REF_MAX_V: float = 2.500  # Maximum linear analog scaling voltage [V]

# TMC2209 Official Voltage Envelope (Trinamic / Analog Devices Datasheet Rev 1.09)
# Section 20.1 Operational Range:
TMC2209_VS_MIN_OPERATIONAL_V: float = 5.5  # Min supply voltage when using internal +5V regulator [V]
TMC2209_VS_MIN_BRIDGED_V: float = 4.7  # Min supply voltage when 5VOUT bridged to VS [V]
TMC2209_VS_MAX_RECOMMENDED_V: float = 29.0  # Max recommended operating voltage [V]
TMC2209_VIO_MIN_OPERATIONAL_V: float = 3.00  # Min logic I/O voltage [V]
TMC2209_VIO_MAX_OPERATIONAL_V: float = 5.25  # Max logic I/O voltage [V]

# Section 19 Absolute Maximum Ratings:
TMC2209_VS_ABS_MAX_INDUCTIVE_V: float = 32.0  # Supply voltage operating with inductive load max [V]
TMC2209_VMAX_ABS_MAX_BRIDGE_V: float = 33.0  # Supply and bridge voltage max (accounts for kickbacks/overshoots) [V]
TMC2209_VIO_ABS_MAX_V: float = 5.5  # I/O supply voltage absolute maximum [V]

# Section 21.3 Supply Filtering & Capacitor Requirements (Datasheet Rev 1.09, §21.3):
# Trinamic explicitly specifies: "decoupled with an electrolytic capacitor (47 µF or larger is recommended alongside ceramic surface-mount capacitors)"
# CRITICAL DISTINCTION:
# 1. Datasheet Filtering Role: 47 µF or larger electrolytic + ceramic decoupling placed close to VS is a supply filtering
#    recommendation to suppress switching ripple and bus impedance during normal PWM chopper operation.
# 2. Transient-Energy Sizing Role: The 47 µF datasheet recommendation is NOT proof that a particular capacitance is sufficient
#    for the application's transient energy events (inductive kickback or kinetic regeneration). Application transient-energy
#    sizing remains a separate, unverified engineering requirement.
TMC2209_DATASHEET_MIN_BULK_CAP_UF: float = 47.0  # Datasheet §21.3 recommended supply filtering bulk electrolytic (47 uF or larger) [uF]
EARS_RL_PROJECT_TARGET_BULK_CAP_UF: float = 100.0  # EARS-RL project engineering target for ripple/energy margin (unverified for transient faults) [uF]
TMC2209_5VOUT_MIN_CERAMIC_CAP_UF: float = 2.2  # Datasheet §21.3 recommended ceramic on 5VOUT [uF]
TMC2209_5VOUT_MAX_CERAMIC_CAP_UF: float = 4.7  # Datasheet §21.3 recommended ceramic on 5VOUT [uF]
TMC2209_VS_CERAMIC_DECOUPLING_MIN_UF: float = 0.1  # Low-ESR high-frequency ceramic MLCC directly across VS/GND [uF]

# Phase-Energy Audit Provenance Status:
# The figure "4.41 mJ per phase" is unsupported by the project's baseline parameters (L = 4.0 mH, I = 1.0 A -> 2.0 mJ/phase;
# two phases at 1.0 A -> 4.0 mJ total). Mathematically, 4.41 mJ requires unconfigured inputs (e.g. L = 4.0 mH, I = 1.4849 A
# or L = 2.0 mH, I = 2.1 A) and is formally classified as UNDETERMINED.
UNSUPPORTED_4_41_MJ_PER_PHASE_STATUS: str = "UNDETERMINED"

# Deprecated aliases for backwards compatibility (use TMC2209_VS_* constants above)
TMC2209_VM_MIN_V: float = TMC2209_VS_MIN_BRIDGED_V
TMC2209_VM_MAX_RECOMMENDED_V: float = TMC2209_VS_MAX_RECOMMENDED_V
TMC2209_VM_ABSOLUTE_MAX_V: float = TMC2209_VS_ABS_MAX_INDUCTIVE_V  # Locked to official 32.0 V (not 36 V)
TMC2209_VM_SHORT_SURGE_MAX_V: float = TMC2209_VMAX_ABS_MAX_BRIDGE_V  # Locked to official 33.0 V (not 40 V)


@dataclass(frozen=True)
class TVSSpec:
    """Rigorous parametric definition of a Transient Voltage Suppressor (TVS) diode.

    CRITICAL ELECTRICAL DISTINCTIONS:
    - V_RWM (Reverse Stand-Off Voltage): Voltage below which TVS does not conduct (leakage only).
      Must be strictly greater than maximum normal DC supply voltage + ripple.
    - V_BR (Breakdown Voltage): Voltage at which avalanche begins at test current I_T (typically 1 mA).
      V_BR is typically 10-15% above V_RWM.
    - V_CLAMP (Maximum Clamping Voltage): Voltage across TVS when carrying peak pulse surge current I_PP.
      V_CLAMP is substantially higher than V_BR due to dynamic resistance (V_CLAMP ≈ 1.35 to 1.6 x V_RWM).

    WARNING: Never choose a TVS diode solely based on its nominal designation (e.g. '30V' or '33V').
    An SMBJ30CA has V_CLAMP = 48.4 V and SMBJ33CA has V_CLAMP = 53.3 V, both of which severely exceed
    the official TMC2209 32.0 V inductive / 33.0 V bridge absolute maximum ratings!
    """

    part_number: str
    manufacturer: str
    datasheet_reference: str
    polarity: str  # "bidirectional" or "unidirectional"
    v_rwm_v: float  # Reverse stand-off voltage [V]
    v_br_min_v: float  # Minimum breakdown voltage at I_T [V]
    v_br_max_v: float  # Maximum breakdown voltage at I_T [V]
    v_clamp_max_v: float  # Maximum clamping voltage at I_PP [V]
    i_pp_a: float  # Peak pulse surge current rating [A]
    pulse_waveform: str = "10/1000us"  # Standard surge pulse duration
    p_ppm_w: float = 600.0  # Peak pulse power dissipation rating [W] (e.g. 600W for SMBJ)
    temp_derating_coeff_pct_c: float = -0.8  # Power derating %/deg C above 25 deg C
    v_br_temp_coeff_pct_c: float = 0.10  # Breakdown voltage positive tempco %/deg C
    is_verified: bool = False

    def clamping_voltage_at_temp(self, temp_c: float = 25.0) -> float:
        """Estimate clamping voltage at operating junction temperature using positive temperature coefficient."""
        if temp_c <= 25.0:
            return self.v_clamp_max_v
        delta_t = temp_c - 25.0
        return self.v_clamp_max_v * (1.0 + (self.v_br_temp_coeff_pct_c / 100.0) * delta_t)

    def calculate_double_exponential_energy_estimate(
        self,
        tau1_s: float = 1426.18e-6,
        tau2_s: float = 1.4496e-6,
    ) -> float:
        """Compute MATHEMATICAL_PULSE_ENERGY_ESTIMATE using normalized double-exponential pulse integral.

        Analytical model:
            p(t) = A * (exp(-t / tau1) - exp(-t / tau2)),  with tau1 > tau2 > 0
            t_peak = (tau1 * tau2) / (tau1 - tau2) * ln(tau1 / tau2)
            norm = exp(-t_peak / tau1) - exp(-t_peak / tau2)
            A = P_PPM / norm
            E_integral = A * (tau1 - tau2)

        For standard 10/1000 us pulse parameters (t_peak ≈ 10.0 us, tau1 ≈ 1426.18 us, tau2 ≈ 1.45 us, P_PPM = 600 W):
            norm ≈ 0.9920, A ≈ 604.84 W, E_integral ≈ 0.8617 J.

        SCIENTIFIC CLASSIFICATION:
        This value is strictly a MATHEMATICAL_PULSE_ENERGY_ESTIMATE of an idealized test waveform.
        It is NOT a rated hardware energy absorption limit and does NOT prove safety for real motor back-EMF dumping!
        """
        return calculate_double_exponential_pulse_integral(self.p_ppm_w, tau1_s, tau2_s)

    def calculate_10_1000us_integral_energy_j(self) -> float:
        """Alias for calculate_double_exponential_energy_estimate on standard 10/1000 us waveform."""
        return self.calculate_double_exponential_energy_estimate()


@dataclass(frozen=True)
class DoubleExponentialPulseParameters:
    """Exact analytical parameters of a normalized double-exponential pulse waveform."""

    tau1_s: float
    tau2_s: float
    t_peak_s: float
    norm_factor: float
    amplitude_a_w: float
    integral_energy_j: float
    p_ppm_w: float
    waveform_model_type: str = "INSTANTANEOUS_POWER_MODEL"
    assumed_clamping_behavior: str = (
        "Assumes instantaneous power p(t) follows double-exponential profile. "
        "If interpreted as v_clamp(t)*i(t), assumes clamping voltage is constant or "
        "lumped into the composite power curve; does not account for dynamic resistance or thermal rise."
    )


def compute_double_exponential_pulse_parameters(
    p_ppm_w: float,
    tau1_s: float = 1426.18e-6,
    tau2_s: float = 1.4496e-6,
) -> DoubleExponentialPulseParameters:
    """Compute exact analytical parameters of a normalized double-exponential pulse waveform.

    Mathematical Definition:
        p(t) = A * (exp(-t / tau1) - exp(-t / tau2)),  tau1 > tau2 > 0

    Peak Time (dp/dt = 0):
        t_peak = (tau1 * tau2) / (tau1 - tau2) * ln(tau1 / tau2)

    Waveform Normalization:
        norm = exp(-t_peak / tau1) - exp(-t_peak / tau2)
        A = P_PPM / norm  (ensures peak instantaneous power equals P_PPM exactly at t_peak)

    Analytical Time Integral from 0 to infinity:
        integral_0^inf p(t) dt = A * (tau1 - tau2)

    Standard 10/1000 us Waveform Provenance & Documented Editions:
        - Datasheet Reference: Littelfuse SMBJ Series (Fig. 3) defines "10/1000µsec. Waveform as defined by R.E.A."
          (U.S. Rural Electrification Administration PE-60 / Bulletin 345-65).
        - Telecommunication Standard: Telcordia GR-1089-CORE, Issue 4 (June 2006) / Issue 6 (May 2011), Section 4
          defines first- and second-level lightning surge test impulses of 10/1000 us.
        - Semiconductor Surge Standard: IEEE C62.35-2010 (Standard Test Methods for Avalanche Junction Semiconductor
          Surge-Protective Device Components), Section 4.3 (Pulse Current Shapes).
        - Standard Distinction: NOT defined by IEC 61000-4-5 (which defines 1.2/50 us voltage, 8/20 us current,
          and 10/700 us telecom waves, not 10/1000 us).
        - Parameters: tau1 ≈ 1426.18 us, tau2 ≈ 1.4496 us.
        - Calculated values for P_PPM = 600.0 W:
            * t_peak ≈ 10.0000 us (10.000035 us)
            * norm ≈ 0.992003 (approx 0.9920)
            * A ≈ 604.8366 W (approx 604.84 W)
            * E_integral ≈ 0.861729 J (approx 0.8617 J)

    PHYSICAL MODEL & CLAMPING ASSUMPTIONS:
        - Waveform Model: This function models instantaneous power p(t) directly.
        - Clamping Assumption: If interpreted as electrical power p(t) = v_clamp(t) * i(t),
          it implicitly assumes v_clamp(t) is approximately constant (V_C) or that dynamic clamping
          impedance is lumped into the power waveform. Physical TVS dynamic resistance and thermal drift
          are not modeled by this scalar impulse integral.
        - Disclaimer: E_integral is strictly a theoretical mathematical property of the test waveform.
          It is NOT measured physical energy and NOT a datasheet-certified absorption capacity.
    """
    if p_ppm_w <= 0.0:
        raise ValueError(f"p_ppm_w must be positive, got {p_ppm_w}")
    if tau1_s <= tau2_s or tau2_s <= 0.0:
        raise ValueError(f"Invalid time constants: requires tau1 ({tau1_s}) > tau2 ({tau2_s}) > 0")

    t_peak = (tau1_s * tau2_s) / (tau1_s - tau2_s) * math.log(tau1_s / tau2_s)
    norm = math.exp(-t_peak / tau1_s) - math.exp(-t_peak / tau2_s)
    amp = p_ppm_w / norm
    energy = amp * (tau1_s - tau2_s)

    return DoubleExponentialPulseParameters(
        tau1_s=tau1_s,
        tau2_s=tau2_s,
        t_peak_s=t_peak,
        norm_factor=norm,
        amplitude_a_w=amp,
        integral_energy_j=energy,
        p_ppm_w=p_ppm_w,
    )


def calculate_double_exponential_pulse_integral(
    p_ppm_w: float,
    tau1_s: float = 1426.18e-6,
    tau2_s: float = 1.4496e-6,
) -> float:
    """Calculate the theoretical time integral of a normalized double-exponential pulse waveform.

    Forwards calculation to compute_double_exponential_pulse_parameters.
    """
    params = compute_double_exponential_pulse_parameters(p_ppm_w, tau1_s, tau2_s)
    return float(params.integral_energy_j)


TVS_ENERGY_METRIC_TAXONOMY: dict[str, str] = {
    "DATASHEET_PEAK_PULSE_POWER": (
        "Datasheet peak pulse power rating (e.g. 600 W for SMBJ series) evaluated at standard 10/1000 us impulse peak."
    ),
    "MATHEMATICAL_PULSE_ENERGY_ESTIMATE": (
        "Theoretical time integral of a normalized double-exponential mathematical waveform (p(t) = A*(exp(-t/tau1) - exp(-t/tau2))). "
        "This is a mathematical benchmark estimate of the test waveform, NOT a rated hardware absorption capacity."
    ),
    "APPLICATION_PULSE_ENERGY_CAPACITY": (
        "Actual pulse energy capacity under real-world motor back-EMF discharge. Strictly UNDETERMINED without "
        "application-specific voltage/current trajectories, exact component thermal impedance Z_theta(t), and pulse duration."
    ),
}


# Officially characterized TVS candidate catalog from manufacturer datasheets
TVS_CANDIDATE_CATALOG: dict[str, TVSSpec] = {
    "SMBJ15CA": TVSSpec(
        part_number="SMBJ15CA",
        manufacturer="Littelfuse",
        datasheet_reference=(
            "Littelfuse Silicon Avalanche Diodes SMBJ Series Datasheet (Catalog Rev 2005/04, "
            "pinned artifact docs/datasheets/Littelfuse_TVS_Diode_SMBJ_Datasheet.pdf, "
            "SHA-256: 8c61bd4e70a9cd0506005cc70cd1ba5b0c8e76c8545f18cbfd19a78e16b08aa4)"
        ),
        polarity="bidirectional",
        v_rwm_v=15.0,
        v_br_min_v=16.70,
        v_br_max_v=18.50,
        v_clamp_max_v=24.4,
        i_pp_a=24.6,
        pulse_waveform="10/1000us",
        p_ppm_w=600.0,
        is_verified=True,
    ),
    "SMBJ18CA": TVSSpec(
        part_number="SMBJ18CA",
        manufacturer="Littelfuse",
        datasheet_reference=(
            "Littelfuse Silicon Avalanche Diodes SMBJ Series Datasheet (Catalog Rev 2005/04, "
            "pinned artifact docs/datasheets/Littelfuse_TVS_Diode_SMBJ_Datasheet.pdf, "
            "SHA-256: 8c61bd4e70a9cd0506005cc70cd1ba5b0c8e76c8545f18cbfd19a78e16b08aa4)"
        ),
        polarity="bidirectional",
        v_rwm_v=18.0,
        v_br_min_v=20.00,
        v_br_max_v=22.10,
        v_clamp_max_v=29.2,
        i_pp_a=20.6,
        pulse_waveform="10/1000us",
        p_ppm_w=600.0,
        is_verified=True,
    ),
    "SMBJ30CA": TVSSpec(
        part_number="SMBJ30CA",
        manufacturer="Littelfuse",
        datasheet_reference=(
            "Littelfuse Silicon Avalanche Diodes SMBJ Series Datasheet (Catalog Rev 2005/04, "
            "pinned artifact docs/datasheets/Littelfuse_TVS_Diode_SMBJ_Datasheet.pdf, "
            "SHA-256: 8c61bd4e70a9cd0506005cc70cd1ba5b0c8e76c8545f18cbfd19a78e16b08aa4)"
        ),
        polarity="bidirectional",
        v_rwm_v=30.0,
        v_br_min_v=33.30,
        v_br_max_v=36.80,
        v_clamp_max_v=48.4,
        i_pp_a=12.4,
        pulse_waveform="10/1000us",
        p_ppm_w=600.0,
        is_verified=True,
    ),
    "SMBJ33CA": TVSSpec(
        part_number="SMBJ33CA",
        manufacturer="Littelfuse",
        datasheet_reference=(
            "Littelfuse Silicon Avalanche Diodes SMBJ Series Datasheet (Catalog Rev 2005/04, "
            "pinned artifact docs/datasheets/Littelfuse_TVS_Diode_SMBJ_Datasheet.pdf, "
            "SHA-256: 8c61bd4e70a9cd0506005cc70cd1ba5b0c8e76c8545f18cbfd19a78e16b08aa4)"
        ),
        polarity="bidirectional",
        v_rwm_v=33.0,
        v_br_min_v=36.70,
        v_br_max_v=40.60,
        v_clamp_max_v=53.3,
        i_pp_a=11.3,
        pulse_waveform="10/1000us",
        p_ppm_w=600.0,
        is_verified=True,
    ),
}


@dataclass(frozen=True)
class TVSEvaluationResult:
    """Outcome of electrical protection evaluation for a TVS candidate on TMC2209 bus."""

    status: str  # "EVALUATED_CANDIDATE — UNVERIFIED", "NORMAL_CONDUCTION_RISK", "DANGEROUS_UNDERPROTECTION", "UNRESOLVED — HARDWARE POWER-UP BLOCKED"
    is_safe: bool
    v_rwm_margin_v: float | None
    v_clamp_vs_abs_max_margin_v: float | None
    issues: list[str]
    notes: list[str]
    datasheet_peak_pulse_power_w: float | None = None
    mathematical_pulse_energy_estimate_j: float | None = None
    application_pulse_energy_capacity: str = "UNDETERMINED"
    standard_10_1000us_integral_energy_j: float | None = None


def evaluate_tvs_protection_envelope(
    tvs: TVSSpec | None,
    nominal_supply_voltage_v: float = 24.0,
    supply_tolerance_percent: float = 5.0,
    peak_ripple_v: float = 1.0,
    operating_temp_c: float = 25.0,
) -> TVSEvaluationResult:
    """Evaluate whether a TVS diode protects TMC2209 within datasheet operating and absolute maximum ratings.

    Datasheet Boundaries (TMC2209 Rev 1.09):
        - Section 20.1 Operational Range: VS up to 29.0 V DC.
        - Section 19 Absolute Maximum Ratings:
            * VS max operating with inductive load: 32.0 V DC
            * Bridge voltage max (including kickbacks/overshoots): 33.0 V DC

    Args:
        tvs: Specification of the candidate TVS diode. If None, returns UNRESOLVED.
        nominal_supply_voltage_v: Nominal DC bench power supply voltage [V] (typically 12V, 15V, or 24V).
        supply_tolerance_percent: Power supply regulation/accuracy tolerance [%].
        peak_ripple_v: Switching ripple and inductive line noise peak voltage [V].
        operating_temp_c: Expected operating temperature [deg C] for thermal clamping voltage derating.

    Returns:
        TVSEvaluationResult detailing protection envelope analysis.
    """
    if tvs is None or not tvs.is_verified:
        return TVSEvaluationResult(
            status="UNRESOLVED — HARDWARE POWER-UP BLOCKED",
            is_safe=False,
            v_rwm_margin_v=None,
            v_clamp_vs_abs_max_margin_v=None,
            issues=[
                "TVS protection component is unspecified, unverified, or lacks full manufacturer datasheet parameters.",
                "HARDWARE POWER-UP IS STRICTLY BLOCKED until a verified protection circuit is qualified."
            ],
            notes=[
                "Do not assume a generic 30V or 33V TVS protects TMC2209.",
                "On a 24V DC bus, discrete passive TVS diodes have V_CLAMP >= 48.4V, which destroys the 32V/33V-rated TMC2209.",
                "TMC2209 Absolute Maximum Ratings (Datasheet Rev 1.09, Section 19): VS = 32.0 V (inductive), VMAX = 33.0 V (bridge)."
            ],
        )

    issues: list[str] = []
    notes: list[str] = []

    # 1. Calculate maximum normal operational supply voltage
    v_supply_max = nominal_supply_voltage_v * (1.0 + supply_tolerance_percent / 100.0) + peak_ripple_v

    # 2. Check stand-off voltage V_RWM against maximum normal operating voltage
    v_rwm_margin = tvs.v_rwm_v - v_supply_max
    if v_rwm_margin <= 0.0:
        issues.append(
            f"NORMAL CONDUCTION HAZARD: V_RWM ({tvs.v_rwm_v:.2f} V) is less than maximum normal operational "
            f"supply voltage ({v_supply_max:.2f} V). TVS diode will enter conduction/breakdown during normal operation, "
            f"causing continuous power dissipation and fire/thermal burnout hazard."
        )

    # 3. Compute thermal derated clamping voltage
    v_clamp_actual = tvs.clamping_voltage_at_temp(operating_temp_c)

    # 4. Check clamping voltage V_CLAMP against TMC2209 Absolute Maximum Ratings (32.0 V inductive, 33.0 V bridge)
    v_clamp_margin_inductive = TMC2209_VS_ABS_MAX_INDUCTIVE_V - v_clamp_actual
    v_clamp_margin_bridge = TMC2209_VMAX_ABS_MAX_BRIDGE_V - v_clamp_actual

    if v_clamp_margin_inductive < 0.0:
        issues.append(
            f"DANGEROUS UNDERPROTECTION: Maximum clamping voltage V_CLAMP ({v_clamp_actual:.2f} V at {tvs.i_pp_a:.1f} A, {operating_temp_c:.0f} deg C) "
            f"exceeds TMC2209 Inductive Operating Absolute Maximum ({TMC2209_VS_ABS_MAX_INDUCTIVE_V:.1f} V) by {-v_clamp_margin_inductive:.2f} V."
        )
    if v_clamp_margin_bridge < 0.0:
        issues.append(
            f"FATAL BRIDGE BREAKDOWN: V_CLAMP ({v_clamp_actual:.2f} V) exceeds TMC2209 Bridge Maximum ({TMC2209_VMAX_ABS_MAX_BRIDGE_V:.1f} V) "
            f"by {-v_clamp_margin_bridge:.2f} V, destroying driver power MOSFETs!"
        )

    # 5. Polarity check: must be bidirectional for inductive stepper environments
    if tvs.polarity.lower() != "bidirectional":
        notes.append("Notice: Unidirectional TVS will conduct forward if ground bounce causes negative transient swings.")

    std_energy = tvs.calculate_double_exponential_energy_estimate() if tvs.is_verified else None
    math_energy = round(std_energy, 4) if std_energy is not None else None
    datasheet_p = tvs.p_ppm_w if tvs.is_verified else None

    notes.append(
        "TVS ENERGY TAXONOMY AUDIT: "
        f"[DATASHEET_PEAK_PULSE_POWER: {datasheet_p:.1f} W at 10/1000 us], "
        f"[MATHEMATICAL_PULSE_ENERGY_ESTIMATE: {math_energy:.4f} J from normalized double-exponential integral], "
        "[APPLICATION_PULSE_ENERGY_CAPACITY: UNDETERMINED]. "
        "The heuristic '600 W * 1 ms = 0.6 J' is a rectangular pulse fallacy; (1/ln 2)*P*t applies only to single exponential decay. "
        "Allowable energy capacity under real-world stepper back-EMF discharges (hypothetical unverified 10-100 ms duration scenario) is strictly UNDETERMINED "
        "from 10/1000 us datasheet ratings alone without transient thermal impedance curves Z_theta(t) and application current/voltage profiles. "
        "THERMAL CONDITION AUDIT: Littelfuse SMBJ Fig 1 (TA = 25 deg C) covers pulse widths up to 10 ms (~180 W approximate visual reading); "
        "ratings at 100 ms are UNAVAILABLE (not covered by Fig 1). The 5.0 W power dissipation rating applies strictly under an infinite heat sink "
        "at TL = 50 deg C (CONDITIONAL RATING — NOT DIRECTLY APPLICABLE TO A TYPICAL PCB WITHOUT THERMAL VALIDATION); "
        "on a standard PCB with 5.0x5.0 mm copper pad to each terminal, typical free-air continuous thermal estimate is ~1.25 W based on typical R_theta_JA ~ 100 deg C/W (dependent on mounting geometry and ambient conditions, not a universal guaranteed continuous-power limit); "
        "transient pulse derating curves reflect non-repetitive heat capacity and do not converge to the infinite-heat-sink rating."
    )

    if len(issues) == 0:
        is_safe = False
        status = "EVALUATED_CANDIDATE — UNVERIFIED"
        notes.append(
            "HARDWARE GATE BLOCKED: Marked as EVALUATED_CANDIDATE — UNVERIFIED. While nominal clamping voltage falls below TMC2209 "
            "absolute maximum limits (32.0 V / 33.0 V) on derated supply (12V-15V), passive silicon TVS diodes cannot dissipate "
            "continuous kinetic regeneration (application capacity is UNDETERMINED) and board parasitic inductance (L*di/dt) "
            "can produce unmodeled overshoots. Power-up remains STRICTLY BLOCKED pending laboratory oscilloscope qualification."
        )
    else:
        is_safe = False
        status = "NORMAL_CONDUCTION_RISK" if v_rwm_margin <= 0 else "DANGEROUS_UNDERPROTECTION"

    return TVSEvaluationResult(
        status=status,
        is_safe=is_safe,
        v_rwm_margin_v=round(v_rwm_margin, 3),
        v_clamp_vs_abs_max_margin_v=round(v_clamp_margin_inductive, 3),
        issues=issues,
        notes=notes,
        datasheet_peak_pulse_power_w=datasheet_p,
        mathematical_pulse_energy_estimate_j=math_energy,
        application_pulse_energy_capacity="UNDETERMINED",
        standard_10_1000us_integral_energy_j=math_energy,
    )


def calculate_single_phase_inductive_energy(
    inductance_h: float,
    current_a: float,
) -> float:
    """Calculate stored magnetic energy in a single motor phase inductor: E_L = 0.5 * L * I^2.

    For L = 0.004 H (4.0 mH) and I = 1.0 A:
        E_L,1-phase = 0.5 * 0.004 H * (1.0 A)^2 = 0.002 J = 2.0 mJ.

    Args:
        inductance_h: Single-phase inductance in Henries [H].
        current_a: Phase current in Amperes [A].

    Returns:
        Stored magnetic energy in one phase in Joules [J].
    """
    if inductance_h <= 0.0:
        raise ValueError(f"inductance_h must be positive, got {inductance_h}")
    if current_a < 0.0:
        raise ValueError(f"current_a must be non-negative, got {current_a}")
    return float(0.5 * inductance_h * (current_a ** 2))


def calculate_stored_inductive_energy(
    inductance_h: float,
    current_a: float,
    drive_mode: str = "microstepping",
    phase_currents_a: tuple[float, ...] | None = None,
) -> float:
    """Calculate magnetic energy stored in motor phase inductances: E = 1/2 * sum(L * I_k^2).

    PHASE RELATIONSHIP & ELECTRICAL REALITIES:
    0. Single-Phase Reference (drive_mode="single_phase"):
       Energy stored in a single inductor carrying current I:
           E_L,1-phase = 1/2 * L * I^2
       For L = 4.0 mH (0.004 H) and I = 1.0 A:
           E_L,1-phase = 0.5 * 0.004 * 1.0^2 = 0.002 J = 2.0 mJ.
       For two identical phases each carrying I = 1.0 A simultaneously:
           E_L,2-phase = 2 * (1/2 * L * I^2) = L * I^2 = 0.004 J = 4.0 mJ.
       Note: The claim "4.41 mJ per phase" is unsupported by L = 4.0 mH, I = 1.0 A
       (requires unconfigured parameters such as L = 4.0 mH, I = 1.4849 A or L = 2.0 mH, I = 2.1 A)
       and is classified as UNDETERMINED (UNSUPPORTED_4_41_MJ_PER_PHASE_STATUS).

    1. Sinusoidal Microstepping Mode (drive_mode="microstepping"):
       Coil currents obey quadrature sine/cosine relationship:
           i_A(t) = I_peak * cos(theta_e)
           i_B(t) = I_peak * sin(theta_e)
       Total stored magnetic energy across both phases is constant across electrical angle:
           E = 1/2 * L * (i_A^2 + i_B^2) = 1/2 * L * I_peak^2 * (cos^2 + sin^2) = 1/2 * L * I_peak^2
       Notice: This is 1/2 * L * I_peak^2 (or L * I_RMS^2 for I_peak = sqrt(2) * I_RMS),
       representing the TWO-PHASE TOTAL magnetic energy (4.0 mJ when L = 4.0 mH and I_RMS = 1.0 A / I_peak = 1.414 A;
       or 2.0 mJ when L = 4.0 mH and I_peak = 1.0 A).

    2. Full-Step Two-Phase-On Mode (drive_mode="full_step_two_phase_on" or "two_phase_dc"):
       Both identical phases carry current_a simultaneously (i_A = current_a, i_B = current_a):
           E = 1/2 * L * current_a^2 + 1/2 * L * current_a^2 = L * current_a^2
       For L = 4.0 mH and I = 1.0 A per phase, this two-phase total is 4.0 mJ;
       for I_peak = 1.414 A on both phases simultaneously, the worst-case upper bound is 8.0 mJ.

    3. Discrete Phase Currents:
       If phase_currents_a is provided (e.g. (i_A, i_B)), computes exact sum: E = sum(1/2 * L * I_k^2).

    Args:
        inductance_h: Motor phase inductance in Henries [H].
        current_a: Phase current (or peak phase current in microstepping) in Amperes [A].
        drive_mode: "single_phase", "microstepping", or "full_step_two_phase_on" ("two_phase_dc"). Default is "microstepping".
        phase_currents_a: Optional tuple of explicit individual phase currents [A].

    Returns:
        Stored magnetic energy in Joules [J].
    """
    if inductance_h <= 0.0:
        raise ValueError(f"inductance_h must be positive, got {inductance_h}")
    if current_a < 0.0:
        raise ValueError(f"current_a must be non-negative, got {current_a}")

    if phase_currents_a is not None:
        return float(sum(0.5 * inductance_h * (i_k ** 2) for i_k in phase_currents_a))

    mode = drive_mode.strip().lower()
    if mode in ("single_phase", "one_phase"):
        return calculate_single_phase_inductive_energy(inductance_h, current_a)
    elif mode in ("microstep", "microstepping", "sine_cosine"):
        return float(0.5 * inductance_h * (current_a ** 2))
    elif mode in ("full_step", "full_step_two_phase_on", "two_phase_dc", "conservative_worst_case"):
        return float(inductance_h * (current_a ** 2))
    else:
        raise ValueError(
            f"Unknown drive_mode '{drive_mode}'. Valid choices: 'single_phase', 'microstepping', 'full_step_two_phase_on'."
        )


def calculate_kinetic_energy(
    rotor_inertia_kg_m2: float,
    angular_velocity_rad_s: float,
    load_inertia_kg_m2: float = 0.0,
) -> float:
    """Calculate mechanical kinetic energy stored in spinning motor rotor and coupled load.

    Formula:
        E_kin = 1/2 * (J_rotor + J_load) * omega^2

    Args:
        rotor_inertia_kg_m2: Motor rotor moment of inertia in kg*m^2.
        angular_velocity_rad_s: Shaft angular velocity in rad/s.
        load_inertia_kg_m2: Coupled mechanical load inertia reflected to motor shaft in kg*m^2.

    Returns:
        Kinetic energy in Joules [J].
    """
    if rotor_inertia_kg_m2 < 0.0:
        raise ValueError(f"rotor_inertia_kg_m2 must be non-negative, got {rotor_inertia_kg_m2}")
    if load_inertia_kg_m2 < 0.0:
        raise ValueError(f"load_inertia_kg_m2 must be non-negative, got {load_inertia_kg_m2}")

    j_total = rotor_inertia_kg_m2 + load_inertia_kg_m2
    return float(0.5 * j_total * (angular_velocity_rad_s ** 2))


def calculate_bus_voltage_rise(
    initial_voltage_v: float,
    bulk_capacitance_f: float,
    energy_joules: float,
    capacitance_tolerance_percent: float = 20.0,
    calculation_type: str = "IDEALIZED_UPPER_BOUND_SCENARIO",
) -> float:
    """Calculate DC bus voltage rise when local bulk capacitor absorbs dumped energy.

    Formula (Lossless Idealized Conservation of Energy):
        1/2 * C_eff * (V_final^2 - V_initial^2) = E_dumped
        V_final = sqrt(V_initial^2 + 2 * E_dumped / C_eff)

    STANDARDIZED VOLTAGE CALCULATION TAXONOMY:
    1. IDEALIZED_UPPER_BOUND_SCENARIO (Default):
       Lossless mathematical ceiling assuming 100% of dumped energy is transferred into an ideal lumped capacitor.
       It represents a theoretical conservative upper bound, NOT a measured physical voltage or certain circuit prediction!
    2. MODEL_BASED_ESTIMATE:
       Dynamic ODE simulation accounting for coil resistance I^2*R, mechanical friction, and contactor trajectories.
    3. MEASURED_PHYSICAL_VOLTAGE:
       Empirical data recorded with an oscilloscope probe on physical bench hardware.

    MANDATORY DECLARED ASSUMPTIONS (for IDEALIZED_UPPER_BOUND_SCENARIO):
    - Assumption 1: 100% of net dumped energy E is transferred into the capacitor (lossless scenario).
    - Assumption 2: No other active power sources or sink loads affect the bus during the integration window.
    - Assumption 3: C_eff explicitly reflects worst-case lower manufacturer tolerance and DC bias derating.
    - Assumption 4: Local high-frequency transient spikes due to parasitic trace inductance (L_trace * di/dt)
      and capacitor ESR are omitted from this lumped scalar energy equation.

    CRITICAL ENGINEERING LIMITATIONS:
    1. Lossless Conservative Bound: Assumes 100% of dumped energy is transferred into the capacitor.
       In reality, copper winding resistance (I^2 * R_coil) and friction dissipate a portion of energy as heat.
    2. Effective Capacitance Derating: Electrolytic capacitors exhibit +/- 20% tolerance, temperature drop,
       and capacitance reduction under high DC voltage bias. C_eff = C_bulk * (1 - tol/100).
    3. ESR Transient Omitted: Pure capacitive absorption ignores the instantaneous IR_ESR voltage step
       produced across capacitor equivalent series resistance during fast pulse rise.
    4. Bench Supply Inability to Sink Current: Standard DC bench supplies use output rectifiers/diodes
       and CANNOT sink reverse current. When an upstream E-stop opens VM, the driver is completely isolated
       from the bench supply's large output capacitor, leaving ONLY the local bulk capacitor.
    5. Unit Test Disclaimer: Passing this mathematical calculation proves computational correctness;
       it DOES NOT certify that the physical electrical hardware is safe without laboratory oscilloscope measurements.

    Args:
        initial_voltage_v: Initial DC bus voltage before power disconnect [V].
        bulk_capacitance_f: Local bulk decoupling nominal capacitance in Farads [F] (e.g. 100e-6 for 100 uF).
        energy_joules: Total energy dumped onto the bus in Joules [J] (inductive and/or regenerative kinetic).
        capacitance_tolerance_percent: Worst-case capacitance lower tolerance [%] (default 20.0%).
        calculation_type: Standardized calculation taxonomy classification.

    Returns:
        Final worst-case bus voltage V_final in Volts [V].
    """
    valid_taxonomies = ("IDEALIZED_UPPER_BOUND_SCENARIO", "MODEL_BASED_ESTIMATE", "MEASURED_PHYSICAL_VOLTAGE")
    if calculation_type not in valid_taxonomies:
        raise ValueError(f"calculation_type must be one of {valid_taxonomies}, got '{calculation_type}'")
    if initial_voltage_v < 0.0:
        raise ValueError(f"initial_voltage_v must be non-negative, got {initial_voltage_v}")
    if bulk_capacitance_f <= 0.0:
        raise ValueError(f"bulk_capacitance_f must be positive, got {bulk_capacitance_f}")
    if energy_joules < 0.0:
        raise ValueError(f"energy_joules must be non-negative, got {energy_joules}")
    if not (0.0 <= capacitance_tolerance_percent < 100.0):
        raise ValueError(f"capacitance_tolerance_percent must be in [0, 100), got {capacitance_tolerance_percent}")

    c_effective = bulk_capacitance_f * (1.0 - capacitance_tolerance_percent / 100.0)
    v_sq = (initial_voltage_v ** 2) + (2.0 * energy_joules / c_effective)
    return float(math.sqrt(v_sq))


@dataclass(frozen=True)
class BrakeChopperEvaluation:
    """Rigorous evaluation of active brake chopper transient dissipation circuit."""

    status: str  # strictly "CONCEPTUAL_DESIGN — UNVERIFIED"
    is_qualified: bool  # strictly False until physical hardware qualification
    turn_on_voltage_v: float
    turn_off_voltage_v: float
    resistor_ohm: float
    resistor_continuous_w: float
    peak_current_a: float
    peak_instantaneous_power_w: float
    overload_power_ratio: float
    is_continuous_wattage_sufficient_proof: bool  # strictly False!
    pulse_energy_rating_j: float | None
    energy_absorption_adequate: bool | None
    mosfet_vds_rating_v: float
    mosfet_id_rating_a: float
    logic_supply_independent: bool
    application_energy_status: str = "UNDETERMINED"
    required_resistor_pulse_energy_status: str = "UNDETERMINED"
    alternative_architectures_considered: list[str] = field(default_factory=list)
    issues: list[str] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)


def evaluate_active_brake_chopper(
    turn_on_voltage_v: float = 28.5,
    turn_off_voltage_v: float = 27.0,
    resistor_ohm: float = 5.0,
    resistor_continuous_w: float = 25.0,
    pulse_energy_rating_j: float | None = None,
    target_energy_to_dissipate_j: float | None = 0.036125,
    mosfet_vds_rating_v: float = 60.0,
    mosfet_id_rating_a: float = 30.0,
    logic_supply_independent: bool = False,
) -> BrakeChopperEvaluation:
    """Evaluate active brake chopper circuit parameters against transient stress.

    Physics & Engineering Principles:
    1. Resistor Peak Power Overload Fallacy:
       - Peak instantaneous current: I_peak = V_on / R = 28.5 V / 5.0 Ohm = 5.7 A.
       - Peak instantaneous power: P_peak = V_on^2 / R = (28.5 V)^2 / 5.0 Ohm = 162.45 W.
       - Overload ratio: P_peak / P_continuous = 162.45 W / 25.0 W = 6.498 (approx 6.5x nominal rating).
       - Continuous 25W rating is thermal steady-state under convection; it DOES NOT prove pulse survival.
         Wirewound resistors can suffer wire melting or localized hot-spot blowout under high-energy pulses
         unless validated by manufacturer pulse-overload energy rating (E_pulse in Joules).
    2. MOSFET Voltage & Current Headroom:
       - V_DS rating (e.g. 60V) must exceed max bus voltage with inductive kickback margin (>= 48 V).
       - I_D pulsed rating must exceed 2x peak chopper current (>= 11.4 A).
    3. Failure Mode on Loss of Logic Supply:
       - If logic power (+5V or +3.3V) is lost or drops before motor kinetic energy is dissipated,
          the comparator gate drive collapses to 0V. The chopper ceases conduction, leaving the bus
          unprotected against freewheeling back-EMF pumping!
    4. Readiness Gate:
       - Status is strictly CONCEPTUAL_DESIGN — UNVERIFIED until physical hardware bench qualification.

    Args:
        turn_on_voltage_v: Comparator turn-on threshold [V] (default 28.5 V).
        turn_off_voltage_v: Comparator turn-off threshold [V] (default 27.0 V).
        resistor_ohm: Dump resistor resistance in Ohms [Ohm] (default 5.0 Ohm).
        resistor_continuous_w: Manufacturer nominal continuous power rating [W] (default 25.0 W).
        pulse_energy_rating_j: Manufacturer single-pulse energy capacity [J] (if specified).
        target_energy_to_dissipate_j: Estimated total dumped energy [J] (e.g. 36.125 mJ). If None, UNDETERMINED.
        mosfet_vds_rating_v: Switching MOSFET drain-source breakdown voltage [V] (default 60.0 V).
        mosfet_id_rating_a: Switching MOSFET continuous/pulsed drain current [A] (default 30.0 A).
        logic_supply_independent: True if chopper has autonomous boot supply derived from VM.

    Returns:
        BrakeChopperEvaluation dataclass.
    """
    if turn_on_voltage_v <= turn_off_voltage_v:
        raise ValueError(
            f"turn_on_voltage_v ({turn_on_voltage_v}) must exceed turn_off_voltage_v ({turn_off_voltage_v}) for hysteresis"
        )
    if resistor_ohm <= 0.0:
        raise ValueError(f"resistor_ohm must be positive, got {resistor_ohm}")
    if resistor_continuous_w <= 0.0:
        raise ValueError(f"resistor_continuous_w must be positive, got {resistor_continuous_w}")

    peak_current = turn_on_voltage_v / resistor_ohm
    peak_power = (turn_on_voltage_v ** 2) / resistor_ohm
    overload_ratio = peak_power / resistor_continuous_w

    issues: list[str] = []
    notes: list[str] = []

    # Check 1: Continuous wattage fallacy
    is_continuous_wattage_sufficient_proof = False
    issues.append(
        f"RESISTOR PULSE OVERLOAD: Peak instantaneous power {peak_power:.2f} W is {overload_ratio:.2f}x "
        f"the nominal continuous rating ({resistor_continuous_w:.1f} W). Continuous power rating is NOT proof "
        f"of pulse survival without manufacturer pulse energy specification (Joules)."
    )

    # Check 2: Pulse energy rating verification
    energy_absorption_adequate: bool | None = None
    required_resistor_pulse_energy_status = "UNDETERMINED"
    if target_energy_to_dissipate_j is None:
        application_energy_status = "UNDETERMINED"
        notes.append(
            "Target energy to dissipate is unspecified (UNDETERMINED). Application velocity profile and payload "
            "inertia must be characterized before final component sizing. Required resistor pulse rating is UNDETERMINED."
        )
    else:
        application_energy_status = "SPECIFIED_FOR_EVALUATION"
        notes.append(
            f"Target dumped energy specified as {target_energy_to_dissipate_j:.4f} J for evaluation. "
            "Heuristic requirements such as 'E_pulse >= 2.0 J' represent an arbitrary safety factor (~55x over 36 mJ baseline); "
            "actual pulse rating requirement remains UNDETERMINED without measured duty cycle and deceleration trajectory."
        )
        if pulse_energy_rating_j is not None:
            if pulse_energy_rating_j < target_energy_to_dissipate_j:
                energy_absorption_adequate = False
                issues.append(
                    f"INSUFFICIENT PULSE ENERGY CAPACITY: Resistor pulse rating ({pulse_energy_rating_j:.3f} J) "
                    f"is less than dumped energy ({target_energy_to_dissipate_j:.3f} J)."
                )
            else:
                energy_absorption_adequate = True
                notes.append(
                    f"Pulse energy rating ({pulse_energy_rating_j:.3f} J) exceeds target dumped energy ({target_energy_to_dissipate_j:.3f} J)."
                )
        else:
            notes.append("Pulse energy rating is unspecified; requires manufacturer pulse rating validation before hardware sign-off.")

    # Check 3: MOSFET ratings
    if mosfet_vds_rating_v < TMC2209_VS_ABS_MAX_INDUCTIVE_V * 1.5:
        issues.append(
            f"MOSFET V_DS rating ({mosfet_vds_rating_v:.1f} V) provides insufficient margin over bus max (recommended >= 48 V)."
        )
    if mosfet_id_rating_a < peak_current * 1.5:
        issues.append(
            f"MOSFET I_D rating ({mosfet_id_rating_a:.1f} A) provides insufficient margin over peak chopper current ({peak_current:.1f} A)."
        )

    # Check 4: Logic supply dependency
    if not logic_supply_independent:
        notes.append(
            "CRITICAL HAZARD: Active chopper depends on external logic supply (+5V/+3.3V). If logic power is lost during "
            "an E-stop while motor continues spinning, chopper gate drive collapses to 0V, disabling protection!"
        )

    alternative_architectures = [
        "Architecture 1 (Supply Voltage Derating): Operating DC bus at 12V-15V DC lowers peak back-EMF below 32V and provides wider safe margin for standard TVS clamping. Status: EVALUATED_CANDIDATE — UNVERIFIED.",
        "Architecture 2 (Active Brake Chopper): Hysteretic comparator (28.5V/27.0V) and power MOSFET dumping energy into pulse-rated power resistor. Requires autonomous bootstrap gate power to survive logic rail loss. Status: CONCEPTUAL_DESIGN — UNVERIFIED.",
        "Architecture 3 (High-Side Electronic Circuit Breaker & Reverse-Blocking Diode): Electronic switch isolating power supply and stopping back-feed. Notice: Does NOT eliminate supply energy during finite detection (t_detect) and switch-off (t_switch_off) latency (E_supply = integral(V*I dt) > 0); also leaves residual energy on downstream capacitors.",
        "Architecture 4 (Electromechanical Holding Brake): Mandatory for gravity/vertical axes to prevent mechanical back-driving.",
    ]

    return BrakeChopperEvaluation(
        status="CONCEPTUAL_DESIGN — UNVERIFIED",
        is_qualified=False,
        turn_on_voltage_v=turn_on_voltage_v,
        turn_off_voltage_v=turn_off_voltage_v,
        resistor_ohm=resistor_ohm,
        resistor_continuous_w=resistor_continuous_w,
        peak_current_a=round(peak_current, 3),
        peak_instantaneous_power_w=round(peak_power, 3),
        overload_power_ratio=round(overload_ratio, 3),
        is_continuous_wattage_sufficient_proof=is_continuous_wattage_sufficient_proof,
        pulse_energy_rating_j=pulse_energy_rating_j,
        energy_absorption_adequate=energy_absorption_adequate,
        mosfet_vds_rating_v=mosfet_vds_rating_v,
        mosfet_id_rating_a=mosfet_id_rating_a,
        logic_supply_independent=logic_supply_independent,
        application_energy_status=application_energy_status,
        required_resistor_pulse_energy_status=required_resistor_pulse_energy_status,
        alternative_architectures_considered=alternative_architectures,
        issues=issues,
        notes=notes,
    )


@dataclass(frozen=True)
class EnergyAccountingResult:
    """Rigorous multi-pathway energy accounting for DC bus transient overvoltage."""

    case_category: str  # "CASE_1_MAGNETIC_ONLY", "CASE_2_MAGNETIC_AND_KINETIC", "CASE_3_CONTINUOUS_SUPPLY_INJECTION", "CASE_4_DISSIPATIVE_PATHWAYS", "CASE_5_INDETERMINATE_MISSING_PARAMETERS"
    voltage_calculation_type: str  # "IDEALIZED_UPPER_BOUND_SCENARIO", "MODEL_BASED_ESTIMATE", "MEASURED_PHYSICAL_VOLTAGE"
    e_magnetic_j: float | None
    e_kinetic_j: float | None
    e_power_supply_j: float | None
    e_dissipated_j: float | None
    e_net_dumped_j: float | None
    c_effective_uf: float | None
    v_initial_v: float
    v_final_v: float | None
    voltage_exceeds_operational_max: bool | None
    voltage_exceeds_inductive_abs_max: bool | None
    voltage_exceeds_bridge_abs_max: bool | None
    is_safe: bool
    status: str
    fault_detection_delay_s: float | None = None
    switch_off_delay_s: float | None = None
    contactor_opening_delay_s: float | None = None
    fault_duration_s: float | None = None
    power_supply_current_a: float | None = None
    residual_stored_energy_j: float = 0.0
    assumptions_declared: list[str] = field(default_factory=list)
    issues: list[str] = field(default_factory=list)
    reconciliation_notes: list[str] = field(default_factory=list)


def compute_energy_accounting(
    inductance_h: float | None = 0.004,
    current_a: float | None = 1.414,
    drive_mode: str = "microstepping",
    rotor_inertia_kg_m2: float | None = None,
    angular_velocity_rad_s: float | None = None,
    load_inertia_kg_m2: float | None = None,
    bulk_capacitance_f: float | None = 100e-6,
    capacitance_tolerance_percent: float = 20.0,
    initial_bus_voltage_v: float = 24.0,
    dissipated_energy_j: float = 0.0,
    power_supply_active: bool = False,
    power_supply_current_a: float | None = 0.0,
    power_supply_current_limit_a: float | None = None,
    fault_detection_delay_s: float | None = 0.0,
    switch_off_delay_s: float | None = None,
    contactor_opening_delay_s: float | None = 0.0,
    fault_duration_s: float | None = None,
    residual_stored_energy_j: float = 0.0,
    voltage_calculation_type: str = "IDEALIZED_UPPER_BOUND_SCENARIO",
) -> EnergyAccountingResult:
    """Compute comprehensive multi-pathway energy accounting and bus voltage rise.

    Categories:
        - CASE 1: Pure magnetic energy only (rotor stationary, omega = 0).
        - CASE 2: Magnetic + Kinetic energy (rotor spinning, omega > 0).
        - CASE 3: Continuous power supply injection (fault isolation delayed or contactor unopened).
        - CASE 4: Dissipative pathways accounted for (copper resistance I^2*R, mechanical friction).
        - CASE 5: Indeterminate / missing parameters (capacitance, velocity, or fault timing undefined).

    Taxonomy Classification:
        - IDEALIZED_UPPER_BOUND_SCENARIO (Default): Theoretical lossless scalar ceiling assuming 100% of dumped
          energy is absorbed by local bulk capacitor without parasitic line inductance or ESR damping.
        - MODEL_BASED_ESTIMATE: Dynamic ODE solution incorporating resistive decay and friction.
        - MEASURED_PHYSICAL_VOLTAGE: Empirical physical laboratory oscilloscope acquisition.

    Baseline Physical Motor Values (NEMA 17 Benchmark):
        - Single-Phase Reference (L = 4.0 mH, I = 1.0 A): E_L,1-phase = 0.5 * L * I^2 = 2.0 mJ.
        - Two Identical Energized Phases (L = 4.0 mH, I = 1.0 A per phase): E_L,2-phase = 2 * 2.0 mJ = 4.0 mJ.
        - Sinusoidal Microstepping (L = 4.0 mH, I_RMS = 1.0 A, I_peak = 1.414 A): Two-phase total E_mag = 4.0 mJ (or 8.0 mJ full-step at 1.414 A).
        - Unsupported Claim: "4.41 mJ per phase" is unsupported by L = 4.0 mH, I = 1.0 A and classified as UNDETERMINED.
        - J_rotor = 5.7e-6 kg*m^2, J_load = 20.0e-6 kg*m^2, omega = 50.0 rad/s -> E_kin = 32.125 mJ.
        - Baseline Total Energy: 36.125 mJ (two-phase 4.0 mJ + 32.125 mJ), 34.125 mJ (single-phase 2.0 mJ + 32.125 mJ), or 40.125 mJ (full-step 8.0 mJ + 32.125 mJ).
        - On 80 uF effective (100 uF - 20%), 36.125 mJ surges bus from 24 V to 38.46 V (> 33.0 V); 34.125 mJ surges bus to 37.80 V (> 33.0 V).
        - Note on 98.0 mJ: The previously reported 98.0 mJ was an artificial test fixture value
          (producing 55.01 V on 80 uF). To physically generate 98 mJ requires omega = 85.5 rad/s (816 RPM)
          or heavier load inertia J_load = 69.5e-6 kg*m^2.

    Returns:
        EnergyAccountingResult with complete energy terms, voltage predictions, and safety evaluation.
    """
    valid_taxonomies = ("IDEALIZED_UPPER_BOUND_SCENARIO", "MODEL_BASED_ESTIMATE", "MEASURED_PHYSICAL_VOLTAGE")
    if voltage_calculation_type not in valid_taxonomies:
        raise ValueError(f"voltage_calculation_type must be one of {valid_taxonomies}, got '{voltage_calculation_type}'")

    # Physical parameter bounds check
    if power_supply_current_a is not None and power_supply_current_a < 0.0:
        raise ValueError(f"power_supply_current_a must be non-negative, got {power_supply_current_a}")
    if power_supply_current_limit_a is not None and power_supply_current_limit_a < 0.0:
        raise ValueError(f"power_supply_current_limit_a must be non-negative, got {power_supply_current_limit_a}")
    if fault_detection_delay_s is not None and fault_detection_delay_s < 0.0:
        raise ValueError(f"fault_detection_delay_s must be non-negative, got {fault_detection_delay_s}")
    if switch_off_delay_s is not None and switch_off_delay_s < 0.0:
        raise ValueError(f"switch_off_delay_s must be non-negative, got {switch_off_delay_s}")
    if contactor_opening_delay_s is not None and contactor_opening_delay_s < 0.0:
        raise ValueError(f"contactor_opening_delay_s must be non-negative, got {contactor_opening_delay_s}")
    if fault_duration_s is not None and fault_duration_s < 0.0:
        raise ValueError(f"fault_duration_s must be non-negative, got {fault_duration_s}")
    if residual_stored_energy_j < 0.0:
        raise ValueError(f"residual_stored_energy_j must be non-negative, got {residual_stored_energy_j}")
    if dissipated_energy_j < 0.0:
        raise ValueError(f"dissipated_energy_j must be non-negative, got {dissipated_energy_j}")

    # Resolve power supply current (assumed upper bound based on power supply current limit)
    i_ps: float = (
        power_supply_current_limit_a
        if power_supply_current_limit_a is not None
        else (power_supply_current_a if power_supply_current_a is not None else 0.0)
    )

    t_detect = fault_detection_delay_s if fault_detection_delay_s is not None else 0.0
    t_open_contactor = contactor_opening_delay_s if contactor_opening_delay_s is not None else 0.0
    t_switch = switch_off_delay_s if switch_off_delay_s is not None else 0.0
    t_disconnect = max(t_open_contactor, t_switch)

    if power_supply_active:
        if fault_duration_s is not None and fault_duration_s > 0.0:
            total_fault_s = fault_duration_s
        else:
            total_fault_s = t_detect + t_disconnect
    else:
        total_fault_s = 0.0

    missing_case_3 = power_supply_active and (i_ps <= 0.0 or total_fault_s <= 0.0)

    # Check Case 5: Missing or invalid parameters
    if (
        bulk_capacitance_f is None
        or bulk_capacitance_f <= 0.0
        or inductance_h is None
        or inductance_h <= 0.0
        or current_a is None
        or current_a < 0.0
        or angular_velocity_rad_s is None
        or (angular_velocity_rad_s > 0.0 and rotor_inertia_kg_m2 is None)
        or missing_case_3
    ):
        c_eff_uf = (
            round(bulk_capacitance_f * 1e6 * (1.0 - capacitance_tolerance_percent / 100.0), 2)
            if (bulk_capacitance_f and bulk_capacitance_f > 0.0)
            else None
        )
        issues = []
        if missing_case_3:
            issues.append(
                "Case 3 (continuous supply injection) requested but power supply current or fault duration "
                "(fault_detection_delay_s + contactor/switch opening delay) is missing or zero. Parameters are UNDETERMINED."
            )
        else:
            issues.append(
                "Missing essential physical parameters (inductance, current, inertia, velocity, or bulk capacitance) to compute energy accounting."
            )
        return EnergyAccountingResult(
            case_category="CASE_5_INDETERMINATE_MISSING_PARAMETERS",
            voltage_calculation_type=voltage_calculation_type,
            e_magnetic_j=None,
            e_kinetic_j=None,
            e_power_supply_j=None,
            e_dissipated_j=None,
            e_net_dumped_j=None,
            c_effective_uf=c_eff_uf,
            v_initial_v=initial_bus_voltage_v,
            v_final_v=None,
            voltage_exceeds_operational_max=None,
            voltage_exceeds_inductive_abs_max=None,
            voltage_exceeds_bridge_abs_max=None,
            is_safe=False,
            status="INDETERMINATE_MISSING_PARAMETERS",
            fault_detection_delay_s=t_detect if power_supply_active else None,
            switch_off_delay_s=switch_off_delay_s if power_supply_active else None,
            contactor_opening_delay_s=contactor_opening_delay_s if power_supply_active else None,
            fault_duration_s=total_fault_s if power_supply_active else None,
            power_supply_current_a=i_ps if power_supply_active else None,
            residual_stored_energy_j=residual_stored_energy_j,
            assumptions_declared=[],
            issues=issues,
            reconciliation_notes=[
                "Energy accounting cannot be safely computed without verified motor electrical, mechanical, and protection properties."
            ],
        )

    # 1. Stored inductive magnetic energy
    e_mag = calculate_stored_inductive_energy(inductance_h, current_a, drive_mode)

    # 2. Kinetic energy
    if angular_velocity_rad_s > 0.0:
        j_rotor = rotor_inertia_kg_m2 if rotor_inertia_kg_m2 is not None else 0.0
        j_load = load_inertia_kg_m2 if load_inertia_kg_m2 is not None else 0.0
        e_kin = calculate_kinetic_energy(j_rotor, angular_velocity_rad_s, j_load)
    else:
        e_kin = 0.0

    # 3. Power supply energy injection: E_supply = integral(V_source(t) * I_source(t) dt)
    # Under model assumption of constant source voltage and constant upper-bound current limit:
    if power_supply_active and i_ps > 0.0 and total_fault_s > 0.0:
        e_ps = initial_bus_voltage_v * i_ps * total_fault_s
    else:
        e_ps = 0.0

    # 4. Net energy dumped to capacitor (including residual stored energy from downstream capacitors)
    e_net = max(0.0, e_mag + e_kin + e_ps + residual_stored_energy_j - dissipated_energy_j)

    # 5. Effective bulk capacitance
    c_eff = bulk_capacitance_f * (1.0 - capacitance_tolerance_percent / 100.0)
    c_eff_uf = c_eff * 1e6

    # 6. Final bus voltage
    v_final = calculate_bus_voltage_rise(
        initial_voltage_v=initial_bus_voltage_v,
        bulk_capacitance_f=bulk_capacitance_f,
        energy_joules=e_net,
        capacitance_tolerance_percent=capacitance_tolerance_percent,
        calculation_type=voltage_calculation_type,
    )

    # 7. Check voltage thresholds
    v_exceeds_op = v_final > TMC2209_VS_MAX_RECOMMENDED_V
    v_exceeds_ind = v_final > TMC2209_VS_ABS_MAX_INDUCTIVE_V
    v_exceeds_brg = v_final > TMC2209_VMAX_ABS_MAX_BRIDGE_V

    is_safe = not v_exceeds_op

    # 8. Status and Case categorization
    if v_exceeds_brg:
        status = "DESTRUCTIVE_BRIDGE_OVERVOLTAGE"
    elif v_exceeds_ind:
        status = "EXCEEDS_INDUCTIVE_ABS_MAX"
    elif v_exceeds_op:
        status = "EXCEEDS_OPERATIONAL_RANGE"
    else:
        status = "WITHIN_OPERATIONAL_LIMITS"

    if power_supply_active and e_ps > 0.0:
        case_category = "CASE_3_CONTINUOUS_SUPPLY_INJECTION"
    elif dissipated_energy_j > 0.0:
        case_category = "CASE_4_DISSIPATIVE_PATHWAYS"
    elif e_kin > 0.0:
        case_category = "CASE_2_MAGNETIC_AND_KINETIC"
    else:
        case_category = "CASE_1_MAGNETIC_ONLY"

    issues: list[str] = []
    if v_exceeds_brg:
        issues.append(
            f"FATAL BRIDGE BREAKDOWN: Bus voltage spikes to {v_final:.2f} V, exceeding TMC2209 Absolute Maximum "
            f"Bridge Voltage ({TMC2209_VMAX_ABS_MAX_BRIDGE_V:.1f} V) by {v_final - TMC2209_VMAX_ABS_MAX_BRIDGE_V:.2f} V. "
            f"Power MOSFETs will suffer dielectric rupture!"
        )
    elif v_exceeds_ind:
        issues.append(
            f"EXCEEDS INDUCTIVE LIMIT: Bus voltage spikes to {v_final:.2f} V, exceeding {TMC2209_VS_ABS_MAX_INDUCTIVE_V:.1f} V "
            f"inductive operating ceiling."
        )
    elif v_exceeds_op:
        issues.append(
            f"EXCEEDS OPERATIONAL RANGE: Bus voltage reaches {v_final:.2f} V (> {TMC2209_VS_MAX_RECOMMENDED_V:.1f} V recommendation)."
        )

    assumptions_declared = [
        "Assumption 1: 100% of net dumped energy is transferred into the capacitor (lossless scenario; zero mechanical friction, air drag, or iron core losses).",
        f"Assumption 2: Power supply energy E_supply = integral(V*I dt) assumes constant bus voltage ({initial_bus_voltage_v:.1f} V) and assumed upper-bound supply current limit (I_ps = {i_ps:.2f} A [ASSUMED — NOT MEASURED], NOT measured physical current).",
        f"Assumption 2b: Fault detection delay ({t_detect * 1e3:.2f} ms), contactor opening delay ({t_open_contactor * 1e3:.2f} ms), and switch-off delay ({t_switch * 1e3:.2f} ms) are classified as ASSUMED — NOT MEASURED engineering scenario values, NOT verified physical timings.",
        f"Assumption 3: Bus voltage rise ({v_final:.2f} V) is an IDEALIZED_UPPER_BOUND_SCENARIO valid ONLY when all modeled energy is dumped into an initial {c_eff_uf:.1f} uF capacitor at {initial_bus_voltage_v:.1f} V without resistive I^2*R dissipation, load consumption, or active voltage clamping.",
        f"Assumption 4: C_eff ({c_eff_uf:.1f} uF) accounts for -{capacitance_tolerance_percent:.1f}% manufacturer tolerance on nominal {bulk_capacitance_f * 1e6:.1f} uF bulk capacitor.",
        "Assumption 5: High-side electronic disconnect does NOT eliminate supply energy if detection latency or switch-off latency is non-zero (t_fault > 0) or if residual energy remains in downstream capacitors.",
        f"Taxonomy Classification: Classified as {voltage_calculation_type}. It represents a theoretical conservative upper bound, NOT a measured physical voltage.",
    ]

    reconciliation_notes = [
        "Phase-energy scope distinction (L=4mH, I=1A): Single phase stores E_L,1-phase = 0.5*L*I^2 = 2.0 mJ; "
        "two identical energized phases store E_L,2-phase = 2*2.0 mJ = 4.0 mJ; sinusoidal microstepping at Irms=1.0A (Ipeak=1.414A) "
        "stores 4.0 mJ across both phases total. The figure '4.41 mJ per phase' is unsupported and classified as UNDETERMINED.",
        "Baseline physical parameters (NEMA 17, L=4mH, Irms=1.0A, J_total=25.7e-6 kg*m^2, omega=50 rad/s) "
        "produce 4.0 mJ magnetic (two-phase total) + 32.125 mJ kinetic = 36.125 mJ total (34.125 mJ if single-phase 2.0 mJ; 40.125 mJ in full-step at 1.414A).",
        "On 80 uF effective capacitance (100 uF - 20%), 36.125 mJ dumps bus from 24 V to 38.46 V (> 33.0 V); single-phase 34.125 mJ dumps bus to 37.80 V (> 33.0 V).",
        "The previously recorded 98.0 mJ was an artificial test fixture value (spiking to 55.01 V). "
        "To physically generate 98 mJ requires omega = 85.5 rad/s (816 RPM) or high load inertia J_load = 69.5e-6 kg*m^2.",
        "Case 3 Continuous Supply Injection Parameter Traceability: At V_bus=24V, assumed upper-bound current limit I_supply=1A [ASSUMED — NOT MEASURED], "
        "t_detect=1ms [ASSUMED — NOT MEASURED], t_contactor=5ms [ASSUMED — NOT MEASURED] (total fault duration 6ms), energy injected is 144 mJ. Combined with baseline two-phase motor energy (36.125 mJ), "
        "net dumped energy is 180.125 mJ (> 100 mJ), resulting in an idealized upper-bound voltage rise of 71.27 V (> 60 V) on 80 uF (or 178.125 mJ -> 70.92 V under single-phase 2.0 mJ; IDEALIZED_UPPER_BOUND_SCENARIO).",
        "High-side disconnect non-zero energy rule: Disconnect switches with finite opening/switch-off time (t_detect + t_switch > 0) "
        "continue to conduct during fault clearance, meaning E_supply > 0 even with solid-state disconnect.",
        "Bulk decoupling capacitors (TMC2209 §21.3 recommendation of 47 uF or larger + ceramic decoupling close to VS) serve supply filtering and do NOT prove transient-energy absorption sufficiency.",
        "Crucial physical insight: pulling ENABLE HIGH does NOT prevent back-EMF overvoltage if rotor spins fast, "
        "because MOSFET body diodes act as a full-wave bridge rectifier pumping generated voltage back onto VM.",
    ]

    return EnergyAccountingResult(
        case_category=case_category,
        voltage_calculation_type=voltage_calculation_type,
        e_magnetic_j=round(e_mag, 6),
        e_kinetic_j=round(e_kin, 6),
        e_power_supply_j=round(e_ps, 6),
        e_dissipated_j=round(dissipated_energy_j, 6),
        e_net_dumped_j=round(e_net, 6),
        c_effective_uf=round(c_eff_uf, 2),
        v_initial_v=round(initial_bus_voltage_v, 2),
        v_final_v=round(v_final, 3),
        voltage_exceeds_operational_max=v_exceeds_op,
        voltage_exceeds_inductive_abs_max=v_exceeds_ind,
        voltage_exceeds_bridge_abs_max=v_exceeds_brg,
        is_safe=is_safe,
        status=status,
        fault_detection_delay_s=t_detect if power_supply_active else None,
        switch_off_delay_s=switch_off_delay_s if power_supply_active else None,
        contactor_opening_delay_s=contactor_opening_delay_s if power_supply_active else None,
        fault_duration_s=total_fault_s if power_supply_active else None,
        power_supply_current_a=i_ps if power_supply_active else None,
        residual_stored_energy_j=residual_stored_energy_j,
        assumptions_declared=assumptions_declared,
        issues=issues,
        reconciliation_notes=reconciliation_notes,
    )


def calculate_tmc2209_irms(
    vref_v: float,
    r_sense_ohm: float = 0.110,
    r_brd_ohm: float = DEFAULT_R_BRD_OHM,
    v_fs_v: float = DEFAULT_V_FS_V,
) -> float:
    """Calculate RMS coil current [A] from measured V_REF [V] in standalone mode.

    Args:
        vref_v: Measured potentiometer reference voltage [V] (typically 0.0 - 2.5 V).
        r_sense_ohm: Value of SMD sense resistors [Ohm] (e.g. 0.110 for 'R110', 0.075 for 'R075').
        r_brd_ohm: PCB trace parasitic resistance [Ohm] (default 0.020 Ohm).
        v_fs_v: Full-scale reference voltage [V] (default 0.325 V).

    Returns:
        I_RMS current in Amperes [A].

    Raises:
        ValueError: If any parameter is negative or resistance is non-positive.
    """
    if vref_v < 0.0:
        raise ValueError(f"vref_v must be non-negative, got {vref_v}")
    if r_sense_ohm <= 0.0:
        raise ValueError(f"r_sense_ohm must be positive, got {r_sense_ohm}")
    if r_brd_ohm < 0.0:
        raise ValueError(f"r_brd_ohm must be non-negative, got {r_brd_ohm}")

    r_total = r_sense_ohm + r_brd_ohm
    i_fs = (v_fs_v / r_total) * (vref_v / DEFAULT_V_REF_MAX_V)
    i_rms = i_fs / math.sqrt(2.0)
    return float(i_rms)


def calculate_tmc2209_vref(
    i_rms_a: float,
    r_sense_ohm: float = 0.110,
    r_brd_ohm: float = DEFAULT_R_BRD_OHM,
    v_fs_v: float = DEFAULT_V_FS_V,
) -> float:
    """Calculate target V_REF [V] required for a desired RMS coil current [A].

    Args:
        i_rms_a: Target RMS phase current [A].
        r_sense_ohm: Value of SMD sense resistors [Ohm].
        r_brd_ohm: PCB trace parasitic resistance [Ohm] (default 0.020 Ohm).
        v_fs_v: Full-scale reference voltage [V] (default 0.325 V).

    Returns:
        Target V_REF voltage in Volts [V].

    Raises:
        ValueError: If any parameter is negative or resistance is non-positive.
    """
    if i_rms_a < 0.0:
        raise ValueError(f"i_rms_a must be non-negative, got {i_rms_a}")
    if r_sense_ohm <= 0.0:
        raise ValueError(f"r_sense_ohm must be positive, got {r_sense_ohm}")
    if r_brd_ohm < 0.0:
        raise ValueError(f"r_brd_ohm must be non-negative, got {r_brd_ohm}")

    r_total = r_sense_ohm + r_brd_ohm
    v_ref = i_rms_a * math.sqrt(2.0) * r_total * (DEFAULT_V_REF_MAX_V / v_fs_v)
    return float(v_ref)


def calculate_derated_current(
    i_rated_a: float,
    derating_factor: float = 0.70,
) -> float:
    """Calculate target motor current applying an open-loop thermal derating factor.

    WARNING (ENGINEERING HEURISTIC):
    Derating factors (typically 0.70 continuous, 0.85 intermittent) are empirical
    guidelines to maintain stepper motor surface temperature below 70 deg C under
    static holding torque conditions. They are NOT physical hardware safety invariants
    and must be validated against the specific motor thermal resistance and active airflow.

    Args:
        i_rated_a: Manufacturer nameplate rated current [A].
        derating_factor: Dimensionless multiplier (0.50 to 1.00). Default is 0.70.

    Returns:
        Target derated current in Amperes [A].
    """
    if i_rated_a <= 0.0:
        raise ValueError(f"i_rated_a must be positive, got {i_rated_a}")
    if not (0.30 <= derating_factor <= 1.00):
        raise ValueError(f"derating_factor must be in [0.30, 1.00], got {derating_factor}")
    return float(i_rated_a * derating_factor)


def validate_tmc2209_operating_limits(
    vref_v: float,
    i_rms_a: float,
) -> tuple[bool, list[str]]:
    """Validate calculated electrical parameters against TMC2209 silicon operating envelope.

    Limits (TMC2209 Datasheet Rev 1.09):
        V_REF max: 2.50 V (linear scaling saturation ceiling)
        I_RMS continuous: 1.40 A (with standard heatsink and passive convection)
        I_RMS maximum peak: 2.00 A (short transient / forced cooling only)

    Returns:
        tuple (is_safe: bool, warnings_or_violations: list[str])
    """
    issues = []
    if vref_v > DEFAULT_V_REF_MAX_V:
        issues.append(
            f"V_REF {vref_v:.3f} V exceeds maximum linear analog scaling range ({DEFAULT_V_REF_MAX_V:.2f} V)."
        )
    if i_rms_a > 2.0:
        issues.append(
            f"I_RMS {i_rms_a:.2f} A exceeds absolute silicon peak rating (2.00 A)."
        )
    elif i_rms_a > 1.4:
        issues.append(
            f"I_RMS {i_rms_a:.2f} A exceeds continuous thermal rating (1.40 A); active cooling fan mandatory."
        )

    return len(issues) == 0, issues

