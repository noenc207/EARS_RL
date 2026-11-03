"""Unit tests for TMC2209 VREF, voltage envelope, and transient safety (Datasheet Rev 1.09)."""

from __future__ import annotations

import math

import pytest

from ears_rl.application.commissioning import (
    get_default_commissioning_gates,
)
from ears_rl.hardware.safety.tmc2209 import (
    EARS_RL_PROJECT_TARGET_BULK_CAP_UF,
    TMC2209_5VOUT_MAX_CERAMIC_CAP_UF,
    TMC2209_5VOUT_MIN_CERAMIC_CAP_UF,
    TMC2209_DATASHEET_MIN_BULK_CAP_UF,
    TMC2209_VIO_ABS_MAX_V,
    TMC2209_VIO_MAX_OPERATIONAL_V,
    TMC2209_VIO_MIN_OPERATIONAL_V,
    TMC2209_VMAX_ABS_MAX_BRIDGE_V,
    TMC2209_VS_ABS_MAX_INDUCTIVE_V,
    TMC2209_VS_CERAMIC_DECOUPLING_MIN_UF,
    TMC2209_VS_MAX_RECOMMENDED_V,
    TMC2209_VS_MIN_BRIDGED_V,
    TMC2209_VS_MIN_OPERATIONAL_V,
    TVS_CANDIDATE_CATALOG,
    TVS_ENERGY_METRIC_TAXONOMY,
    UNSUPPORTED_4_41_MJ_PER_PHASE_STATUS,
    BrakeChopperEvaluation,
    DoubleExponentialPulseParameters,
    EnergyAccountingResult,
    TVSSpec,
    calculate_bus_voltage_rise,
    calculate_derated_current,
    calculate_double_exponential_pulse_integral,
    calculate_kinetic_energy,
    calculate_single_phase_inductive_energy,
    calculate_stored_inductive_energy,
    calculate_tmc2209_irms,
    calculate_tmc2209_vref,
    compute_double_exponential_pulse_parameters,
    compute_energy_accounting,
    evaluate_active_brake_chopper,
    evaluate_tvs_protection_envelope,
    validate_tmc2209_operating_limits,
)


def test_tmc2209_official_voltage_envelope_constants():
    """Verify official TMC2209 voltage envelope strictly matches Datasheet Rev 1.09.

    Section 19: Absolute Maximum Ratings
        - VS (operating with inductive load): max 32.0 V
        - VMAX (bridge voltage max / kickbacks / overshoots): max 33.0 V
        - VIO: max 5.5 V
    Section 20.1: Operational Range
        - VS (internal +5V regulator): 5.5 V to 29.0 V
        - VS (internal regulator bridged 5VOUT=VS): 4.7 V to 5.4 V
        - VIO: 3.00 V to 5.25 V
    """
    assert TMC2209_VS_ABS_MAX_INDUCTIVE_V == 32.0
    assert TMC2209_VMAX_ABS_MAX_BRIDGE_V == 33.0
    assert TMC2209_VIO_ABS_MAX_V == 5.5

    assert TMC2209_VS_MIN_OPERATIONAL_V == 5.5
    assert TMC2209_VS_MIN_BRIDGED_V == 4.7
    assert TMC2209_VS_MAX_RECOMMENDED_V == 29.0
    assert TMC2209_VIO_MIN_OPERATIONAL_V == 3.00
    assert TMC2209_VIO_MAX_OPERATIONAL_V == 5.25


def test_tmc2209_vref_inverse_substitution():
    """Verify calculating VREF and then calculating IRMS returns the exact original current."""
    test_currents = [0.2, 0.5, 0.8, 1.2, 1.6]
    test_r_senses = [0.075, 0.110, 0.150, 0.220]

    for i_target in test_currents:
        for r_s in test_r_senses:
            vref = calculate_tmc2209_vref(i_rms_a=i_target, r_sense_ohm=r_s)
            i_recovered = calculate_tmc2209_irms(vref_v=vref, r_sense_ohm=r_s)
            assert pytest.approx(i_recovered, rel=1e-6) == i_target


def test_tmc2209_known_numerical_values():
    """Verify manual calculation against formula."""
    vref_1a = calculate_tmc2209_vref(i_rms_a=1.0, r_sense_ohm=0.110)
    expected_vref = math.sqrt(2.0)
    assert pytest.approx(vref_1a, rel=1e-5) == expected_vref

    vref_08a = calculate_tmc2209_vref(i_rms_a=0.8, r_sense_ohm=0.110)
    assert pytest.approx(vref_08a, rel=1e-5) == 0.8 * expected_vref


def test_tmc2209_sense_resistor_sensitivity():
    """Verify lower R_SENSE (0.075 Ohm) requires lower VREF for identical RMS current."""
    vref_r110 = calculate_tmc2209_vref(i_rms_a=0.8, r_sense_ohm=0.110)
    vref_r075 = calculate_tmc2209_vref(i_rms_a=0.8, r_sense_ohm=0.075)

    assert vref_r075 < vref_r110
    assert pytest.approx(vref_r075 / vref_r110, rel=1e-5) == (0.095 / 0.130)


def test_tmc2209_zero_current():
    """Verify 0.0 A current requires 0.0 V VREF."""
    assert calculate_tmc2209_vref(0.0) == 0.0
    assert calculate_tmc2209_irms(0.0) == 0.0


def test_tmc2209_invalid_negative_values():
    """Verify negative inputs raise ValueError."""
    with pytest.raises(ValueError, match="non-negative"):
        calculate_tmc2209_vref(-0.5)

    with pytest.raises(ValueError, match="non-negative"):
        calculate_tmc2209_irms(-1.0)

    with pytest.raises(ValueError, match="positive"):
        calculate_tmc2209_vref(0.8, r_sense_ohm=0.0)


def test_tmc2209_derating_factors_are_engineering_heuristics():
    """Verify derating factor calculations and range enforcement."""
    i_target = calculate_derated_current(i_rated_a=1.5, derating_factor=0.70)
    assert pytest.approx(i_target, rel=1e-5) == 1.05

    i_intermittent = calculate_derated_current(i_rated_a=1.5, derating_factor=0.85)
    assert pytest.approx(i_intermittent, rel=1e-5) == 1.275

    with pytest.raises(ValueError, match="positive"):
        calculate_derated_current(-1.0)
    with pytest.raises(ValueError, match="derating_factor"):
        calculate_derated_current(1.5, derating_factor=1.5)


def test_tmc2209_operating_limits_validation():
    """Verify silicon operating limits flagging VREF > 2.5V and excessive currents."""
    safe, issues = validate_tmc2209_operating_limits(vref_v=1.13, i_rms_a=0.8)
    assert safe is True
    assert len(issues) == 0

    safe, issues = validate_tmc2209_operating_limits(vref_v=2.65, i_rms_a=1.2)
    assert safe is False
    assert any("exceeds maximum linear" in s for s in issues)

    safe, issues = validate_tmc2209_operating_limits(vref_v=2.2, i_rms_a=1.6)
    assert safe is False
    assert any("active cooling fan mandatory" in s for s in issues)

    safe, issues = validate_tmc2209_operating_limits(vref_v=2.2, i_rms_a=2.2)
    assert safe is False
    assert any("absolute silicon peak rating" in s for s in issues)


def test_tmc2209_formula_does_not_prove_physical_board_safety():
    """SCIENTIFIC INVARIANT: Verifying the mathematical formula is NOT proof of physical hardware safety."""
    vref = calculate_tmc2209_vref(i_rms_a=0.8, r_sense_ohm=0.110)
    assert vref > 0.0
    is_hardware_physically_verified = False
    assert is_hardware_physically_verified is False


def test_tvs_unverified_spec_blocks_hardware_power_up():
    """SAFETY INVARIANT: An unspecified or unverified TVS blocks hardware power-up."""
    res_none = evaluate_tvs_protection_envelope(None)
    assert res_none.is_safe is False
    assert res_none.status == "UNRESOLVED — HARDWARE POWER-UP BLOCKED"
    assert any("HARDWARE POWER-UP IS STRICTLY BLOCKED" in issue for issue in res_none.issues)

    unverified_tvs = TVSSpec(
        part_number="GENERIC_30V_TVS",
        manufacturer="UNKNOWN",
        datasheet_reference="NONE",
        polarity="bidirectional",
        v_rwm_v=30.0,
        v_br_min_v=33.3,
        v_br_max_v=36.8,
        v_clamp_max_v=48.4,
        i_pp_a=12.4,
        is_verified=False,
    )
    res_unverified = evaluate_tvs_protection_envelope(unverified_tvs)
    assert res_unverified.is_safe is False
    assert res_unverified.status == "UNRESOLVED — HARDWARE POWER-UP BLOCKED"


def test_tvs_30v_or_33v_fails_tmc2209_absolute_max_rating():
    """ELECTRICAL SAFETY PROOF: Littelfuse SMBJ30CA and SMBJ33CA fail to protect TMC2209 under surge.

    Datasheet Rev 1.09 Limits:
        - VS (operating with inductive load): 32.0 V DC
        - VMAX (bridge voltage max): 33.0 V DC

    Littelfuse SMBJ30CA:
        - V_RWM = 30.0 V
        - V_BR = 33.30 V to 36.80 V @ 1 mA
        - V_CLAMP = 48.4 V @ 12.4 A (10/1000 us)
        -> Exceeds 32.0 V by 16.4 V, and exceeds 33.0 V bridge max by 15.4 V!
    """
    smbj30ca = TVS_CANDIDATE_CATALOG["SMBJ30CA"]
    eval_res = evaluate_tvs_protection_envelope(smbj30ca, nominal_supply_voltage_v=24.0)

    assert eval_res.is_safe is False
    assert eval_res.status == "DANGEROUS_UNDERPROTECTION"
    assert eval_res.v_clamp_vs_abs_max_margin_v == pytest.approx(-16.4, rel=1e-3)
    assert any("Inductive Operating Absolute Maximum (32.0 V)" in issue for issue in eval_res.issues)
    assert any("exceeds TMC2209 Bridge Maximum (33.0 V)" in issue for issue in eval_res.issues)

    smbj33ca = TVS_CANDIDATE_CATALOG["SMBJ33CA"]
    eval_33 = evaluate_tvs_protection_envelope(smbj33ca, nominal_supply_voltage_v=24.0)
    assert eval_33.is_safe is False
    assert eval_33.v_clamp_vs_abs_max_margin_v == pytest.approx(-21.3, rel=1e-3)


def test_tvs_temperature_derating():
    """Verify TVS clamping voltage increases at elevated operating temperatures."""
    smbj30ca = TVS_CANDIDATE_CATALOG["SMBJ30CA"]
    v_clamp_25c = smbj30ca.clamping_voltage_at_temp(25.0)
    assert v_clamp_25c == 48.4

    # At 75 deg C (50 deg C rise), positive temperature coefficient (+0.10%/deg C) increases V_CLAMP
    v_clamp_75c = smbj30ca.clamping_voltage_at_temp(75.0)
    assert v_clamp_75c == pytest.approx(48.4 * (1.0 + 0.05), rel=1e-4)
    assert v_clamp_75c > v_clamp_25c


def test_tvs_low_standoff_causes_normal_conduction_risk():
    """ELECTRICAL SAFETY: TVS with V_RWM <= V_supply_max risks continuous avalanche and thermal fire."""
    low_vrwm_tvs = TVSSpec(
        part_number="HYPOTHETICAL_24V",
        manufacturer="TEST",
        datasheet_reference="TEST",
        polarity="bidirectional",
        v_rwm_v=24.0,
        v_br_min_v=26.7,
        v_br_max_v=29.5,
        v_clamp_max_v=38.9,
        i_pp_a=10.3,
        is_verified=True,
    )

    eval_res = evaluate_tvs_protection_envelope(low_vrwm_tvs, nominal_supply_voltage_v=24.0)
    assert eval_res.is_safe is False
    assert eval_res.status == "NORMAL_CONDUCTION_RISK"
    assert any("NORMAL CONDUCTION HAZARD" in issue for issue in eval_res.issues)


def test_stored_inductive_energy_calculation():
    """Verify stored inductive energy under sinusoidal microstepping vs full-step 2-phase on."""
    # L = 4 mH, I_peak = 1.414 A
    # In sinusoidal microstepping: i_A^2 + i_B^2 = I_peak^2, so E = 1/2 * L * I_peak^2 = 4.0 mJ
    e_micro = calculate_stored_inductive_energy(
        inductance_h=0.004, current_a=1.414, drive_mode="microstepping"
    )
    expected_micro = 0.5 * 0.004 * (1.414 ** 2)  # ≈ 0.003998 J ≈ 4.0 mJ
    assert pytest.approx(e_micro, rel=1e-3) == expected_micro

    # In full-step two-phase-on (worst-case simultaneous peak current): E = L * I_peak^2 = 8.0 mJ
    e_full = calculate_stored_inductive_energy(
        inductance_h=0.004, current_a=1.414, drive_mode="full_step_two_phase_on"
    )
    expected_full = 0.004 * (1.414 ** 2)  # ≈ 0.007997 J ≈ 8.0 mJ
    assert pytest.approx(e_full, rel=1e-3) == expected_full

    # Custom phase currents: i_A = 1.0 A, i_B = 1.0 A -> E = 1/2 * 0.004 * (1 + 1) = 0.004 J
    e_custom = calculate_stored_inductive_energy(
        inductance_h=0.004, current_a=1.0, phase_currents_a=(1.0, 1.0)
    )
    assert pytest.approx(e_custom, rel=1e-4) == 0.004

    with pytest.raises(ValueError, match="inductance_h must be positive"):
        calculate_stored_inductive_energy(0.0, 1.0)
    with pytest.raises(ValueError, match="current_a must be non-negative"):
        calculate_stored_inductive_energy(0.004, -1.0)


def test_kinetic_energy_calculation():
    """Verify mechanical kinetic energy calculation: E_kin = 1/2 * (J_rotor + J_load) * omega^2."""
    # J_rotor = 5.7e-6 kg*m^2, J_load = 20.0e-6 kg*m^2, omega = 50.0 rad/s
    j_rotor = 5.7e-6
    j_load = 20.0e-6
    omega = 50.0
    e_kin = calculate_kinetic_energy(rotor_inertia_kg_m2=j_rotor, angular_velocity_rad_s=omega, load_inertia_kg_m2=j_load)
    expected_e = 0.5 * (25.7e-6) * (2500.0)  # 0.032125 J = 32.125 mJ
    assert pytest.approx(e_kin, rel=1e-5) == expected_e

    with pytest.raises(ValueError, match="rotor_inertia_kg_m2 must be non-negative"):
        calculate_kinetic_energy(-1.0, 10.0)


def test_bus_voltage_rise_calculation_demonstrates_capacitor_and_tolerance_limits():
    """Verify DC bus voltage rise calculation: V_final = sqrt(V_init^2 + 2*E/C_eff).

    Demonstrates:
    - Pure inductive dump (4 mJ) on 100 uF cap (with 20% tolerance -> 80 uF) rises from 24 V to 25.96 V (safe).
    - Inertial back-EMF dump (98 mJ) spikes bus voltage to 55.0 V (far exceeding 32.0 V / 33.0 V limits)!
    """
    # Case 1: Pure inductive dump (4 mJ) with 20% tolerance (80 uF effective)
    v_final_ind = calculate_bus_voltage_rise(
        initial_voltage_v=24.0, bulk_capacitance_f=100e-6, energy_joules=0.004, capacitance_tolerance_percent=20.0
    )
    # sqrt(576 + 2 * 0.004 / 80e-6) = sqrt(576 + 100) = sqrt(676) = 26.0 V
    assert pytest.approx(v_final_ind, rel=1e-4) == 26.0
    assert v_final_ind < TMC2209_VS_ABS_MAX_INDUCTIVE_V  # Safe below 32.0 V

    # Case 2: Inertial back-EMF dump (98 mJ) with 20% tolerance (80 uF effective)
    v_final_inertial = calculate_bus_voltage_rise(
        initial_voltage_v=24.0, bulk_capacitance_f=100e-6, energy_joules=0.098, capacitance_tolerance_percent=20.0
    )
    # sqrt(576 + 2 * 0.098 / 80e-6) = sqrt(576 + 2450) = sqrt(3026) = 55.009 V
    assert pytest.approx(v_final_inertial, rel=1e-4) == math.sqrt(3026.0)
    assert v_final_inertial > TMC2209_VMAX_ABS_MAX_BRIDGE_V  # Exceeds 33.0 V bridge max by 22.0 V!


def test_mock_commissioning_gates_block_physical_requirements():
    """SAFETY INVARIANT: Mock commissioning MUST mark physical gates as BLOCKED or NOT_EXECUTED."""
    gates_mock = get_default_commissioning_gates(is_mock=True)
    gate_map = {g.gate_id: g for g in gates_mock}

    # Gate A (Physical component audit): BLOCKED in mock
    assert gate_map["gate_a"].status == "BLOCKED"
    assert "mock" in gate_map["gate_a"].status_reason.lower()

    # Gate B (Power rails, TVS protection, star GND): strictly BLOCKED
    assert gate_map["gate_b"].status == "BLOCKED"
    assert "HARDWARE POWER-UP BLOCKED" in gate_map["gate_b"].status_reason

    # Gate C (Firmware protocol loopback): PASSED in mock
    assert gate_map["gate_c"].status == "PASSED"

    # Gate D (Encoder rotation): NOT_EXECUTED in mock
    assert gate_map["gate_d"].status == "NOT_EXECUTED"

    # Gate E (Physical motion): strictly NOT_EXECUTED
    assert gate_map["gate_e"].status == "NOT_EXECUTED"


def test_tmc2209_supply_filtering_capacitance_constants():
    """Verify official TMC2209 capacitor recommendations from Datasheet Rev 1.09, Section 21.3.

    - Section 21.3 specifies: "decoupled with an electrolytic capacitor (47 µF or larger is recommended
      alongside ceramic surface-mount capacitors)"
    - EARS-RL Project target: 100 µF is an engineering choice for added ripple and energy margin.
    - 5VOUT pin: 2.2 µF to 4.7 µF ceramic MLCC to GND.
    - VS pin: 100 nF low-ESR ceramic MLCC close to driver pins.
    """
    assert TMC2209_DATASHEET_MIN_BULK_CAP_UF == 47.0
    assert EARS_RL_PROJECT_TARGET_BULK_CAP_UF == 100.0
    assert TMC2209_5VOUT_MIN_CERAMIC_CAP_UF == 2.2
    assert TMC2209_5VOUT_MAX_CERAMIC_CAP_UF == 4.7
    assert TMC2209_VS_CERAMIC_DECOUPLING_MIN_UF == 0.1


def test_smbj18ca_hardware_power_up_strictly_blocked():
    """Verify that SMBJ18CA is NEVER marked as safe or allowed to unblock hardware power-up.

    1. On 24V supply:
       Maximum operational voltage is 24 * 1.05 + 1.0 = 26.2 V.
       V_RWM is 18.0 V, so the TVS diode enters continuous conduction (fire/burnout hazard).
    2. On 15V supply:
       Even if V_RWM > V_supply and V_clamp <= 32.0 V, discrete passive silicon TVS diodes
       cannot absorb sustained kinetic back-EMF (energy capacity is UNDETERMINED for motor waveforms),
       and board parasitic trace inductance causes unmodeled voltage spikes. Power-up remains BLOCKED.
    """
    smbj18ca = TVS_CANDIDATE_CATALOG["SMBJ18CA"]

    # On 24V bus:
    res_24 = evaluate_tvs_protection_envelope(smbj18ca, nominal_supply_voltage_v=24.0)
    assert res_24.is_safe is False
    assert res_24.status == "NORMAL_CONDUCTION_RISK"
    assert any("NORMAL CONDUCTION HAZARD" in issue for issue in res_24.issues)

    # On 15V bus:
    res_15 = evaluate_tvs_protection_envelope(smbj18ca, nominal_supply_voltage_v=15.0)
    assert res_15.is_safe is False
    assert res_15.status == "EVALUATED_CANDIDATE — UNVERIFIED"
    assert any("HARDWARE GATE BLOCKED" in note for note in res_15.notes)
    assert any("TVS ENERGY TAXONOMY AUDIT" in note for note in res_15.notes)
    assert res_15.datasheet_peak_pulse_power_w == 600.0
    assert res_15.application_pulse_energy_capacity == "UNDETERMINED"
    assert res_15.mathematical_pulse_energy_estimate_j == pytest.approx(0.8617, rel=1e-3)
    assert res_15.standard_10_1000us_integral_energy_j == pytest.approx(0.8617, rel=1e-3)


def test_active_brake_chopper_resistor_overload_and_conceptual_status():
    """Verify active brake chopper peak power overload calculation and unverified status.

    Circuit Parameters:
        V_on = 28.5 V, V_off = 27.0 V, R = 5.0 Ohm, P_continuous = 25.0 W.
    Calculations:
        I_peak = 28.5 V / 5.0 Ohm = 5.7 A.
        P_peak = (28.5 V)^2 / 5.0 Ohm = 162.45 W.
        Overload ratio = 162.45 W / 25.0 W = 6.498 (approx 6.5x continuous rating).
    Scientific Rule:
        Continuous 25W rating is steady-state thermal convection; it DOES NOT prove pulse survival.
        Status is strictly CONCEPTUAL_DESIGN — UNVERIFIED.
    """
    eval_ch = evaluate_active_brake_chopper(
        turn_on_voltage_v=28.5,
        turn_off_voltage_v=27.0,
        resistor_ohm=5.0,
        resistor_continuous_w=25.0,
    )
    assert isinstance(eval_ch, BrakeChopperEvaluation)
    assert eval_ch.peak_current_a == pytest.approx(5.7, rel=1e-3)
    assert eval_ch.peak_instantaneous_power_w == pytest.approx(162.45, rel=1e-3)
    assert eval_ch.overload_power_ratio == pytest.approx(6.498, rel=1e-3)
    assert eval_ch.is_continuous_wattage_sufficient_proof is False
    assert eval_ch.is_qualified is False
    assert eval_ch.status == "CONCEPTUAL_DESIGN — UNVERIFIED"
    assert any("RESISTOR PULSE OVERLOAD" in s for s in eval_ch.issues)
    assert any("6.50x" in s for s in eval_ch.issues)


def test_active_brake_chopper_pulse_energy_capacity_and_logic_power_loss():
    """Verify pulse energy rating checking, logic supply dependency, and invalid parameters."""
    # Insufficient pulse capacity
    eval_low = evaluate_active_brake_chopper(
        pulse_energy_rating_j=0.010, target_energy_to_dissipate_j=0.036125
    )
    assert eval_low.energy_absorption_adequate is False
    assert any("INSUFFICIENT PULSE ENERGY CAPACITY" in s for s in eval_low.issues)

    # Adequate pulse capacity
    eval_high = evaluate_active_brake_chopper(
        pulse_energy_rating_j=2.0, target_energy_to_dissipate_j=0.036125
    )
    assert eval_high.energy_absorption_adequate is True

    # Logic supply dependency note
    eval_logic = evaluate_active_brake_chopper(logic_supply_independent=False)
    assert any("depends on external logic supply" in s for s in eval_logic.notes)

    # Invalid thresholds raise ValueError
    with pytest.raises(ValueError, match="must exceed turn_off_voltage_v"):
        evaluate_active_brake_chopper(turn_on_voltage_v=27.0, turn_off_voltage_v=28.5)
    with pytest.raises(ValueError, match="resistor_ohm must be positive"):
        evaluate_active_brake_chopper(resistor_ohm=0.0)


def test_energy_accounting_case_1_pure_magnetic_stationary():
    """Verify Case 1: Pure magnetic energy only when rotor is stationary (omega = 0).

    L = 4 mH, I_peak = 1.414 A -> E_mag = 4.0 mJ.
    On 80 uF effective (100 uF - 20%), 24V bus rises to:
    sqrt(576 + 2 * 0.004 / 80e-6) = sqrt(576 + 100) = sqrt(676) = 26.0 V.
    Status is WITHIN_OPERATIONAL_LIMITS, is_safe is True.
    """
    res = compute_energy_accounting(
        inductance_h=0.004,
        current_a=1.414,
        angular_velocity_rad_s=0.0,
        bulk_capacitance_f=100e-6,
        capacitance_tolerance_percent=20.0,
    )
    assert isinstance(res, EnergyAccountingResult)
    assert res.case_category == "CASE_1_MAGNETIC_ONLY"
    assert pytest.approx(res.e_magnetic_j, rel=1e-3) == 0.004
    assert res.e_kinetic_j == 0.0
    assert pytest.approx(res.v_final_v, rel=1e-3) == 26.0
    assert res.voltage_exceeds_operational_max is False
    assert res.voltage_exceeds_inductive_abs_max is False
    assert res.voltage_exceeds_bridge_abs_max is False
    assert res.is_safe is True
    assert res.status == "WITHIN_OPERATIONAL_LIMITS"


def test_energy_accounting_case_2_baseline_motor_kinetic_hazard():
    """Verify Case 2: Magnetic + Kinetic energy on baseline physical NEMA 17 test bench.

    Baseline parameters:
        L = 4.0 mH, I_peak = 1.414 A -> E_mag = 4.0 mJ
        J_rotor = 5.7e-6 kg*m^2, J_load = 20.0e-6 kg*m^2, omega = 50.0 rad/s -> E_kin = 32.125 mJ
        Total physical energy = 36.125 mJ
    On 80 uF effective:
        sqrt(576 + 2 * 0.036125 / 80e-6) = sqrt(576 + 903.125) = sqrt(1479.125) = 38.459 V
        Exceeds TMC2209 Absolute Maximum Bridge Voltage (33.0 V) by 5.46 V!
    Status is DESTRUCTIVE_BRIDGE_OVERVOLTAGE, is_safe is False.
    """
    res = compute_energy_accounting(
        inductance_h=0.004,
        current_a=1.414,
        rotor_inertia_kg_m2=5.7e-6,
        angular_velocity_rad_s=50.0,
        load_inertia_kg_m2=20.0e-6,
        bulk_capacitance_f=100e-6,
        capacitance_tolerance_percent=20.0,
    )
    assert res.case_category == "CASE_2_MAGNETIC_AND_KINETIC"
    assert pytest.approx(res.e_magnetic_j, rel=1e-3) == 0.004
    assert pytest.approx(res.e_kinetic_j, rel=1e-3) == 0.032125
    assert pytest.approx(res.e_net_dumped_j, rel=1e-3) == 0.036125
    assert res.c_effective_uf == 80.0
    assert pytest.approx(res.v_final_v, rel=1e-3) == 38.459
    assert res.voltage_exceeds_bridge_abs_max is True
    assert res.is_safe is False
    assert res.status == "DESTRUCTIVE_BRIDGE_OVERVOLTAGE"
    assert any("FATAL BRIDGE BREAKDOWN" in s for s in res.issues)


def test_energy_accounting_case_2_reconcile_98mj_test_value():
    """Verify reconciliation of the previously reported 98.0 mJ test value.

    The 98.0 mJ value spikes the 24V bus to 55.01 V on 80 uF:
        sqrt(576 + 2 * 0.098 / 80e-6) = sqrt(576 + 2450) = sqrt(3026) = 55.009 V.
    To physically generate 98 mJ from NEMA 17 (4 mJ magnetic + 94 mJ kinetic), the shaft
    speed must reach omega = 85.528 rad/s (approx 816.7 RPM).
    """
    res = compute_energy_accounting(
        inductance_h=0.004,
        current_a=1.414,
        rotor_inertia_kg_m2=5.7e-6,
        angular_velocity_rad_s=85.5276,
        load_inertia_kg_m2=20.0e-6,
        bulk_capacitance_f=100e-6,
        capacitance_tolerance_percent=20.0,
    )
    assert pytest.approx(res.e_net_dumped_j, rel=1e-2) == 0.098
    assert pytest.approx(res.v_final_v, rel=1e-2) == 55.01
    assert res.status == "DESTRUCTIVE_BRIDGE_OVERVOLTAGE"


def test_energy_accounting_case_3_and_case_4_power_supply_and_dissipation():
    """Verify Case 3 (Continuous power supply injection) and Case 4 (Dissipative pathways)."""
    # Case 3: Power supply injecting energy during fault
    res_ps = compute_energy_accounting(
        angular_velocity_rad_s=0.0,
        power_supply_active=True,
        power_supply_current_a=2.0,
        fault_duration_s=0.001,
    )
    assert res_ps.case_category == "CASE_3_CONTINUOUS_SUPPLY_INJECTION"
    assert pytest.approx(res_ps.e_power_supply_j, rel=1e-3) == 0.048  # 24 V * 2 A * 1 ms

    # Case 4: Copper coil resistance and friction dissipating energy
    res_diss = compute_energy_accounting(
        angular_velocity_rad_s=0.0,
        dissipated_energy_j=0.002,
    )
    assert res_diss.case_category == "CASE_4_DISSIPATIVE_PATHWAYS"
    assert pytest.approx(res_diss.e_net_dumped_j, rel=1e-3) == 0.002  # 4 mJ - 2 mJ


def test_energy_accounting_case_5_indeterminate_missing_parameters():
    """Verify Case 5: Incomplete parameters returns INDETERMINATE_MISSING_PARAMETERS and is_safe=False."""
    # Missing capacitance
    res_no_cap = compute_energy_accounting(bulk_capacitance_f=None)
    assert res_no_cap.case_category == "CASE_5_INDETERMINATE_MISSING_PARAMETERS"
    assert res_no_cap.is_safe is False
    assert res_no_cap.status == "INDETERMINATE_MISSING_PARAMETERS"

    # Missing inertia when rotating
    res_no_j = compute_energy_accounting(angular_velocity_rad_s=50.0, rotor_inertia_kg_m2=None)
    assert res_no_j.case_category == "CASE_5_INDETERMINATE_MISSING_PARAMETERS"
    assert res_no_j.is_safe is False

    # Missing angular velocity
    res_no_omega = compute_energy_accounting(angular_velocity_rad_s=None)
    assert res_no_omega.case_category == "CASE_5_INDETERMINATE_MISSING_PARAMETERS"
    assert res_no_omega.is_safe is False


def test_tvs_peak_power_vs_pulse_energy_fallacy():
    """SCIENTIFIC INVARIANT: Verify distinction between peak power (W) and pulse energy (J).

    - P_PPM = 600 W is the peak power ceiling under a 10/1000 us double exponential waveform.
    - Multiplying 600 W * 1 ms = 0.6 J is a mathematical fallacy assuming a constant rectangular pulse.
    - Normalized double-exponential integral for standard 10/1000 us pulse (tau1=1426 us, tau2=1.45 us) is ~0.862 J.
    - (1/ln 2)*P*t applies only to single exponential decay, not general double exponential waveforms.
    - For motor regenerative discharges (non-standard waveforms lasting 10-100 ms), the allowable
      pulse energy capacity is strictly UNDETERMINED from datasheet 10/1000 us specs alone.
    """
    smbj18ca = TVS_CANDIDATE_CATALOG["SMBJ18CA"]
    assert smbj18ca.p_ppm_w == 600.0

    integral_j = smbj18ca.calculate_10_1000us_integral_energy_j()
    # Normalized double-exponential integral: ~0.8617 J
    assert pytest.approx(integral_j, rel=1e-3) == 0.8617
    assert integral_j != pytest.approx(0.600, rel=1e-2)

    eval_res = evaluate_tvs_protection_envelope(smbj18ca, nominal_supply_voltage_v=15.0)
    assert eval_res.datasheet_peak_pulse_power_w == 600.0
    assert eval_res.mathematical_pulse_energy_estimate_j == pytest.approx(0.8617, rel=1e-3)
    assert eval_res.application_pulse_energy_capacity == "UNDETERMINED"


def test_energy_accounting_case_3_traceable_parameters():
    """Verify Case 3 (Continuous supply injection) has fully traceable, reproducible parameters.

    Physical Test Bench Parameters:
        V_bus = 24.0 V, I_supply = 1.0 A (assumed upper bound based on power supply current limit)
        t_detect = 1.0 ms (fault detection comparator latency)
        t_contactor = 5.0 ms (mechanical contactor opening and arc extinction latency)
        t_fault = t_detect + t_contactor = 6.0 ms
    Energy Calculations:
        E_supply = integral(V_source(t) * I_source(t) dt) = 24.0 V * 1.0 A * 0.006 s = 144.0 mJ
        E_motor (baseline NEMA 17) = 36.125 mJ
        E_net = 36.125 mJ + 144.0 mJ = 180.125 mJ (> 100 mJ)
    Idealized Bus Voltage Rise (on C_eff = 80 uF):
        V_final = sqrt(576 + 2 * 0.180125 / 80e-6) = sqrt(576 + 4503.125) = sqrt(5079.125) = 71.268 V (> 60 V)
    """
    res = compute_energy_accounting(
        inductance_h=0.004,
        current_a=1.414,
        rotor_inertia_kg_m2=5.7e-6,
        angular_velocity_rad_s=50.0,
        load_inertia_kg_m2=20.0e-6,
        bulk_capacitance_f=100e-6,
        capacitance_tolerance_percent=20.0,
        power_supply_active=True,
        power_supply_current_a=1.0,
        fault_detection_delay_s=0.001,
        contactor_opening_delay_s=0.005,
        voltage_calculation_type="IDEALIZED_UPPER_BOUND_SCENARIO",
    )
    assert res.case_category == "CASE_3_CONTINUOUS_SUPPLY_INJECTION"
    assert res.fault_detection_delay_s == 0.001
    assert res.contactor_opening_delay_s == 0.005
    assert res.fault_duration_s == 0.006
    assert res.power_supply_current_a == 1.0
    assert pytest.approx(res.e_power_supply_j, rel=1e-3) == 0.144
    assert pytest.approx(res.e_net_dumped_j, rel=1e-3) == 0.180125
    assert res.e_net_dumped_j > 0.100  # Formally verifies "> 100 mJ"
    assert res.v_final_v > 60.0  # Formally verifies "> 60 V"
    assert pytest.approx(res.v_final_v, rel=1e-3) == 71.268
    assert res.voltage_calculation_type == "IDEALIZED_UPPER_BOUND_SCENARIO"
    assert len(res.assumptions_declared) >= 5
    assert any("lossless scenario" in a for a in res.assumptions_declared)
    assert any("NOT measured physical current" in a for a in res.assumptions_declared)
    assert any("IDEALIZED_UPPER_BOUND_SCENARIO valid ONLY when all modeled energy is dumped" in a for a in res.assumptions_declared)


def test_energy_accounting_case_3_missing_parameters_returns_indeterminate():
    """Verify that requesting Case 3 without supplying current or duration returns INDETERMINATE."""
    # Active supply but 0 current
    res_no_current = compute_energy_accounting(
        angular_velocity_rad_s=0.0,
        power_supply_active=True,
        power_supply_current_a=0.0,
        fault_duration_s=0.010,
    )
    assert res_no_current.case_category == "CASE_5_INDETERMINATE_MISSING_PARAMETERS"
    assert res_no_current.is_safe is False
    assert any("Case 3 (continuous supply injection) requested" in issue for issue in res_no_current.issues)

    # Active supply but 0 duration
    res_no_time = compute_energy_accounting(
        angular_velocity_rad_s=0.0,
        power_supply_active=True,
        power_supply_current_a=1.0,
        fault_detection_delay_s=0.0,
        contactor_opening_delay_s=0.0,
    )
    assert res_no_time.case_category == "CASE_5_INDETERMINATE_MISSING_PARAMETERS"
    assert res_no_time.is_safe is False


def test_voltage_calculation_taxonomy_enforcement():
    """Verify voltage calculation taxonomy enforcement across calculate_bus_voltage_rise and compute_energy_accounting."""
    v_ideal = calculate_bus_voltage_rise(24.0, 100e-6, 0.004, calculation_type="IDEALIZED_UPPER_BOUND_SCENARIO")
    assert pytest.approx(v_ideal, rel=1e-3) == 26.0

    v_model = calculate_bus_voltage_rise(24.0, 100e-6, 0.004, calculation_type="MODEL_BASED_ESTIMATE")
    assert pytest.approx(v_model, rel=1e-3) == 26.0

    v_meas = calculate_bus_voltage_rise(24.0, 100e-6, 0.004, calculation_type="MEASURED_PHYSICAL_VOLTAGE")
    assert pytest.approx(v_meas, rel=1e-3) == 26.0

    with pytest.raises(ValueError, match="calculation_type must be one of"):
        calculate_bus_voltage_rise(24.0, 100e-6, 0.004, calculation_type="UNVALIDATED_GUESS")

    with pytest.raises(ValueError, match="voltage_calculation_type must be one of"):
        compute_energy_accounting(angular_velocity_rad_s=0.0, voltage_calculation_type="INVALID_TYPE")


def test_active_brake_chopper_alternative_architectures_and_undetermined_energy():
    """Verify active brake chopper reports UNDETERMINED when application energy is not specified and lists alternatives."""
    eval_undetermined = evaluate_active_brake_chopper(target_energy_to_dissipate_j=None)
    assert eval_undetermined.application_energy_status == "UNDETERMINED"
    assert eval_undetermined.required_resistor_pulse_energy_status == "UNDETERMINED"
    assert any("UNDETERMINED" in note for note in eval_undetermined.notes)
    assert len(eval_undetermined.alternative_architectures_considered) == 4
    assert any("Supply Voltage Derating" in a for a in eval_undetermined.alternative_architectures_considered)
    assert any("Holding Brake" in a for a in eval_undetermined.alternative_architectures_considered)
    assert eval_undetermined.status == "CONCEPTUAL_DESIGN — UNVERIFIED"
    assert eval_undetermined.is_qualified is False


def test_double_exponential_pulse_integral_mathematics():
    """Verify normalized double-exponential waveform integral and peak power normalization."""
    # Standard 10/1000 us parameters
    tau1 = 1426.18e-6
    tau2 = 1.4496e-6
    p_ppm = 600.0

    energy = calculate_double_exponential_pulse_integral(p_ppm, tau1, tau2)
    assert pytest.approx(energy, rel=1e-3) == 0.8617

    # Verify peak time calculation
    t_peak = (tau1 * tau2) / (tau1 - tau2) * math.log(tau1 / tau2)
    assert pytest.approx(t_peak, rel=1e-2) == 10e-6

    # Verify normalization amplitude
    norm = math.exp(-t_peak / tau1) - math.exp(-t_peak / tau2)
    amp = p_ppm / norm
    p_at_peak = amp * (math.exp(-t_peak / tau1) - math.exp(-t_peak / tau2))
    assert pytest.approx(p_at_peak, rel=1e-5) == p_ppm

    # Verify input validation
    with pytest.raises(ValueError, match="p_ppm_w must be positive"):
        calculate_double_exponential_pulse_integral(0.0)
    with pytest.raises(ValueError, match="Invalid time constants"):
        calculate_double_exponential_pulse_integral(600.0, tau1_s=1e-6, tau2_s=2e-6)


def test_double_exponential_intermediate_parameters_rigorous_regression():
    """Verify all intermediate quantities (t_peak, norm, A, E) of normalized double-exponential pulse."""
    tau1 = 1426.18e-6
    tau2 = 1.4496e-6
    p_ppm = 600.0

    params = compute_double_exponential_pulse_parameters(p_ppm, tau1, tau2)
    assert isinstance(params, DoubleExponentialPulseParameters)

    # 1. Peak time: ~10.0 us (10.000035 us)
    assert pytest.approx(params.t_peak_s, abs=1e-8) == 10.000035e-6
    assert pytest.approx(params.t_peak_s * 1e6, rel=1e-3) == 10.0

    # 2. Normalization factor: ~0.9920 (0.99200344), strictly NOT 0.9990
    assert pytest.approx(params.norm_factor, rel=1e-4) == 0.992003
    assert pytest.approx(round(params.norm_factor, 4)) == 0.9920
    assert params.norm_factor != 0.9990  # Regression: disprove erroneous 0.9990

    # 3. Amplitude factor A: ~604.84 W (604.8366 W), strictly NOT 600.6 W
    assert pytest.approx(params.amplitude_a_w, rel=1e-4) == 604.837
    assert pytest.approx(round(params.amplitude_a_w, 2)) == 604.84
    assert params.amplitude_a_w != 600.6  # Regression: disprove erroneous 600.6 W

    # 4. Integrated pulse energy: ~0.8617 J (0.861729 J)
    assert pytest.approx(params.integral_energy_j, rel=1e-4) == 0.861729
    assert pytest.approx(round(params.integral_energy_j, 4)) == 0.8617

    # 5. Mutual consistency: A * norm == P_PPM exactly
    assert pytest.approx(params.amplitude_a_w * params.norm_factor, rel=1e-6) == p_ppm

    # 6. Mutual consistency: E == A * (tau1 - tau2)
    assert pytest.approx(params.amplitude_a_w * (tau1 - tau2), rel=1e-6) == params.integral_energy_j

    # 7. Derivative at peak dp/dt = 0 (first-order stationary condition)
    dp_dt = params.amplitude_a_w * (
        -(1.0 / tau1) * math.exp(-params.t_peak_s / tau1)
        + (1.0 / tau2) * math.exp(-params.t_peak_s / tau2)
    )
    assert abs(dp_dt) < 1e-3

    # 8. Declared model taxonomy
    assert params.waveform_model_type == "INSTANTANEOUS_POWER_MODEL"
    assert "Assumes instantaneous power p(t)" in params.assumed_clamping_behavior

    # 9. Validation exceptions
    with pytest.raises(ValueError, match="p_ppm_w must be positive"):
        compute_double_exponential_pulse_parameters(-10.0)
    with pytest.raises(ValueError, match="Invalid time constants"):
        compute_double_exponential_pulse_parameters(600.0, tau1_s=1e-6, tau2_s=2e-6)
    with pytest.raises(ValueError, match="Invalid time constants"):
        compute_double_exponential_pulse_parameters(600.0, tau1_s=1e-6, tau2_s=-1e-6)


def test_case_3_traceability_and_parameter_bounds_validation():
    """Verify Case 3 traceability labeling, parameter validation, and disconnect dynamics."""
    # 1. Parameter bounds validation (negative parameters must be rejected)
    with pytest.raises(ValueError, match="power_supply_current_a must be non-negative"):
        compute_energy_accounting(angular_velocity_rad_s=0.0, power_supply_current_a=-1.0)

    with pytest.raises(ValueError, match="power_supply_current_limit_a must be non-negative"):
        compute_energy_accounting(angular_velocity_rad_s=0.0, power_supply_current_limit_a=-0.5)

    with pytest.raises(ValueError, match="fault_detection_delay_s must be non-negative"):
        compute_energy_accounting(angular_velocity_rad_s=0.0, fault_detection_delay_s=-0.001)

    with pytest.raises(ValueError, match="switch_off_delay_s must be non-negative"):
        compute_energy_accounting(angular_velocity_rad_s=0.0, switch_off_delay_s=-0.001)

    with pytest.raises(ValueError, match="contactor_opening_delay_s must be non-negative"):
        compute_energy_accounting(angular_velocity_rad_s=0.0, contactor_opening_delay_s=-0.001)

    with pytest.raises(ValueError, match="fault_duration_s must be non-negative"):
        compute_energy_accounting(angular_velocity_rad_s=0.0, fault_duration_s=-0.001)

    with pytest.raises(ValueError, match="residual_stored_energy_j must be non-negative"):
        compute_energy_accounting(angular_velocity_rad_s=0.0, residual_stored_energy_j=-0.001)

    with pytest.raises(ValueError, match="dissipated_energy_j must be non-negative"):
        compute_energy_accounting(angular_velocity_rad_s=0.0, dissipated_energy_j=-0.001)

    # 2. Case 3 parameter traceability: verify ASSUMED — NOT MEASURED labels
    res = compute_energy_accounting(
        inductance_h=0.004,
        current_a=1.414,
        rotor_inertia_kg_m2=5.7e-6,
        angular_velocity_rad_s=50.0,
        load_inertia_kg_m2=20.0e-6,
        bulk_capacitance_f=100e-6,
        power_supply_active=True,
        power_supply_current_a=1.0,
        fault_detection_delay_s=0.001,
        contactor_opening_delay_s=0.005,
        voltage_calculation_type="IDEALIZED_UPPER_BOUND_SCENARIO",
    )
    assert res.case_category == "CASE_3_CONTINUOUS_SUPPLY_INJECTION"
    assert any("[ASSUMED — NOT MEASURED]" in a for a in res.assumptions_declared)
    assert any("ASSUMED — NOT MEASURED engineering scenario values" in a for a in res.assumptions_declared)
    assert any("IDEALIZED_UPPER_BOUND_SCENARIO" in a for a in res.assumptions_declared)
    assert res.voltage_calculation_type == "IDEALIZED_UPPER_BOUND_SCENARIO"
    assert pytest.approx(res.v_final_v, rel=1e-3) == 71.268

    # 3. Disconnect with zero latency but downstream residual energy still produces net dumped energy
    res_residual = compute_energy_accounting(
        inductance_h=0.004,
        current_a=1.414,
        angular_velocity_rad_s=0.0,
        bulk_capacitance_f=100e-6,
        power_supply_active=False,
        residual_stored_energy_j=0.005,  # 5 mJ residual energy downstream
    )
    # E_net = E_mag (4 mJ) + E_residual (5 mJ) = 9 mJ
    assert pytest.approx(res_residual.e_net_dumped_j, rel=1e-3) == 0.008998
    assert res_residual.e_net_dumped_j > 0.004


def test_tvs_energy_metric_taxonomy():
    """Verify TVS energy metric taxonomy classifications and completeness."""
    assert "DATASHEET_PEAK_PULSE_POWER" in TVS_ENERGY_METRIC_TAXONOMY
    assert "MATHEMATICAL_PULSE_ENERGY_ESTIMATE" in TVS_ENERGY_METRIC_TAXONOMY
    assert "APPLICATION_PULSE_ENERGY_CAPACITY" in TVS_ENERGY_METRIC_TAXONOMY

    smbj18ca = TVS_CANDIDATE_CATALOG["SMBJ18CA"]
    eval_res = evaluate_tvs_protection_envelope(smbj18ca, nominal_supply_voltage_v=15.0)
    assert eval_res.datasheet_peak_pulse_power_w == 600.0
    assert eval_res.mathematical_pulse_energy_estimate_j == pytest.approx(0.8617, rel=1e-3)
    assert eval_res.application_pulse_energy_capacity == "UNDETERMINED"


def test_high_side_disconnect_energy_nonzero_with_latency():
    """Verify that high-side electronic disconnect does NOT default power supply energy to zero when latency is non-zero."""
    # Fast electronic disconnect: 10 us detection delay, 5 us switch-off delay
    t_detect = 10e-6
    t_switch = 5e-6
    v_bus = 24.0
    i_limit = 1.0  # assumed upper-bound current limit
    c_out_dump = 0.0005  # 0.5 mJ residual stored energy in downstream decoupling

    res = compute_energy_accounting(
        inductance_h=0.004,
        current_a=1.414,
        angular_velocity_rad_s=0.0,
        bulk_capacitance_f=100e-6,
        initial_bus_voltage_v=v_bus,
        power_supply_active=True,
        power_supply_current_limit_a=i_limit,
        fault_detection_delay_s=t_detect,
        switch_off_delay_s=t_switch,
        residual_stored_energy_j=c_out_dump,
    )
    assert res.case_category == "CASE_3_CONTINUOUS_SUPPLY_INJECTION"
    # E_supply = 24 V * 1.0 A * 15 us = 0.00036 J = 0.36 mJ (NOT zero!)
    assert pytest.approx(res.e_power_supply_j, rel=1e-3) == 0.00036
    assert res.e_power_supply_j > 0.0
    # Net energy includes magnetic + supply + residual
    expected_net = 0.003998 + 0.00036 + 0.0005
    assert pytest.approx(res.e_net_dumped_j, rel=1e-3) == expected_net
    assert any("Assumption 5: High-side electronic disconnect does NOT eliminate supply energy" in a for a in res.assumptions_declared)


def test_brake_chopper_resistor_pulse_rating_undetermined_without_profile():
    """Verify brake chopper required pulse energy rating is UNDETERMINED without profile."""
    eval_no_target = evaluate_active_brake_chopper(target_energy_to_dissipate_j=None)
    assert eval_no_target.required_resistor_pulse_energy_status == "UNDETERMINED"

    eval_with_target = evaluate_active_brake_chopper(target_energy_to_dissipate_j=0.036125)
    assert eval_with_target.required_resistor_pulse_energy_status == "UNDETERMINED"
    assert any("actual pulse rating requirement remains UNDETERMINED" in note for note in eval_with_target.notes)


def test_smbj_datasheet_thermal_conditions_and_provenance():
    """Verify SMBJ18CA thermal condition de-conflation, artifact pinning, and characteristic curve limits."""
    smbj18ca = TVS_CANDIDATE_CATALOG["SMBJ18CA"]

    # 1. Source artifact pinning verification
    assert "Littelfuse_TVS_Diode_SMBJ_Datasheet.pdf" in smbj18ca.datasheet_reference
    assert "8c61bd4e70a9cd0506005cc70cd1ba5b0c8e76c8545f18cbfd19a78e16b08aa4" in smbj18ca.datasheet_reference

    # 2. Evaluation notes verification
    eval_res = evaluate_tvs_protection_envelope(smbj18ca, nominal_supply_voltage_v=15.0)
    assert eval_res.application_pulse_energy_capacity == "UNDETERMINED"

    note_text = " ".join(eval_res.notes)
    # Fig 1 coverage: up to 10 ms
    assert "Fig 1 (TA = 25 deg C) covers pulse widths up to 10 ms" in note_text
    # 100 ms is unavailable / not covered
    assert "ratings at 100 ms are UNAVAILABLE" in note_text
    # 5.0 W is strictly infinite heat sink at TL = 50 deg C (conditional rating)
    assert "5.0 W power dissipation rating applies strictly under an infinite heat sink at TL = 50 deg C" in note_text
    assert "CONDITIONAL RATING — NOT DIRECTLY APPLICABLE TO A TYPICAL PCB WITHOUT THERMAL VALIDATION" in note_text
    # On 5x5mm copper pad to each terminal in still air, typical free-air thermal estimate is ~1.25 W
    assert "typical free-air continuous thermal estimate is ~1.25 W based on typical R_theta_JA ~ 100 deg C/W" in note_text
    assert "not a universal guaranteed continuous-power limit" in note_text
    # No convergence assumption between pulse derating curve and infinite-heat-sink rating
    assert "transient pulse derating curves reflect non-repetitive heat capacity and do not converge to the infinite-heat-sink rating" in note_text


def test_timing_parameter_distinction_mechanical_vs_solid_state():
    """Verify distinct parameter handling for electromechanical contactors vs solid-state electronic switches."""
    # Run with both mechanical contactor delay and solid-state switch delay
    t_contactor = 0.005  # 5.0 ms (mechanical drop-out)
    t_switch = 0.000010  # 10.0 us (solid-state FET turn-off)

    res = compute_energy_accounting(
        inductance_h=0.004,
        current_a=1.414,
        rotor_inertia_kg_m2=5.7e-6,
        angular_velocity_rad_s=50.0,
        load_inertia_kg_m2=20.0e-6,
        bulk_capacitance_f=100e-6,
        power_supply_active=True,
        power_supply_current_a=1.0,
        fault_detection_delay_s=0.001,
        contactor_opening_delay_s=t_contactor,
        switch_off_delay_s=t_switch,
        voltage_calculation_type="IDEALIZED_UPPER_BOUND_SCENARIO",
    )
    assert res.contactor_opening_delay_s == 0.005
    assert res.switch_off_delay_s == 0.000010
    assert res.fault_detection_delay_s == 0.001
    # Mechanical contactor dominates disconnect time (max(0.005, 0.000010) = 0.005)
    assert res.fault_duration_s == 0.006

    # Verify both delays are labeled [ASSUMED — NOT MEASURED]
    assert any("contactor opening delay" in a and "switch-off delay" in a and "ASSUMED — NOT MEASURED" in a for a in res.assumptions_declared)
    assert res.voltage_calculation_type == "IDEALIZED_UPPER_BOUND_SCENARIO"
    assert pytest.approx(res.v_final_v, rel=1e-3) == 71.268


def test_phase_energy_single_vs_two_phase_and_case_3_reconciliation():
    """Verify E_L = 0.5 * L * I^2 single-phase vs two-phase scope, UNDETERMINED 4.41 mJ claim, and Case 3 reconciliation.

    1. For L = 4.0 mH (0.004 H) and I = 1.0 A:
       - Energy in one phase: E_L,1-phase = 0.5 * 0.004 * 1.0^2 = 0.002 J = 2.0 mJ.
       - Total energy for two identical energized phases: E_L,2-phase = 2 * 2.0 mJ = 0.004 J = 4.0 mJ.
       - Sinusoidal microstepping at I_RMS = 1.0 A (I_peak = sqrt(2) A): two-phase total = 0.004 J = 4.0 mJ.
    2. The claim '4.41 mJ per phase' is unsupported by L = 4.0 mH, I = 1.0 A and classified as UNDETERMINED.
    3. Downstream Case 3 reconciliation (V_bus = 24 V, C_eff = 80 uF, E_kin = 32.125 mJ, E_supply = 144.0 mJ):
       - Single-phase (2.0 mJ): E_net = 2.0 + 32.125 + 144.0 = 178.125 mJ -> V_final = sqrt(576 + 4453.125) = 70.916 V.
       - Two-phase total (4.0 mJ): E_net = 4.0 + 32.125 + 144.0 = 180.125 mJ -> V_final = sqrt(576 + 4503.125) = 71.268 V.
    """
    inductance_h = 0.004
    current_a = 1.0

    # 1. Single-phase energy: 0.5 * L * I^2 = 2.0 mJ
    e_one_phase = calculate_single_phase_inductive_energy(inductance_h=inductance_h, current_a=current_a)
    assert e_one_phase == pytest.approx(0.002, rel=1e-9)
    assert calculate_stored_inductive_energy(
        inductance_h=inductance_h, current_a=current_a, drive_mode="single_phase"
    ) == pytest.approx(0.002, rel=1e-9)

    # 2. Two identical energized phases at I = 1.0 A each: 2 * 2.0 mJ = 4.0 mJ
    e_two_phases = calculate_stored_inductive_energy(
        inductance_h=inductance_h, current_a=current_a, drive_mode="two_phase_dc"
    )
    assert e_two_phases == pytest.approx(0.004, rel=1e-9)
    assert calculate_stored_inductive_energy(
        inductance_h=inductance_h, current_a=current_a, phase_currents_a=(1.0, 1.0)
    ) == pytest.approx(0.004, rel=1e-9)

    # 3. Sinusoidal microstepping at I_RMS = 1.0 A (I_peak = sqrt(2) A): two-phase total = 4.0 mJ
    e_microstep_total = calculate_stored_inductive_energy(
        inductance_h=inductance_h, current_a=math.sqrt(2.0), drive_mode="microstepping"
    )
    assert e_microstep_total == pytest.approx(0.004, rel=1e-9)

    # 4. Unsupported "4.41 mJ per phase" claim is formally UNDETERMINED
    assert UNSUPPORTED_4_41_MJ_PER_PHASE_STATUS == "UNDETERMINED"

    # 5. Downstream Case 3 reconciliation: single-phase (2.0 mJ) vs two-phase total (4.0 mJ)
    res_single_phase = compute_energy_accounting(
        inductance_h=0.004,
        current_a=1.0,
        drive_mode="single_phase",
        rotor_inertia_kg_m2=5.7e-6,
        angular_velocity_rad_s=50.0,
        load_inertia_kg_m2=20.0e-6,
        bulk_capacitance_f=100e-6,
        capacitance_tolerance_percent=20.0,
        power_supply_active=True,
        power_supply_current_a=1.0,
        fault_detection_delay_s=0.001,
        contactor_opening_delay_s=0.005,
        voltage_calculation_type="IDEALIZED_UPPER_BOUND_SCENARIO",
    )
    assert res_single_phase.e_magnetic_j == pytest.approx(0.002, rel=1e-6)
    assert res_single_phase.e_kinetic_j == pytest.approx(0.032125, rel=1e-6)
    assert res_single_phase.e_power_supply_j == pytest.approx(0.144, rel=1e-6)
    assert res_single_phase.e_net_dumped_j == pytest.approx(0.178125, rel=1e-6)
    expected_v_single = math.sqrt(24.0**2 + 2.0 * 0.178125 / 80e-6)
    assert res_single_phase.v_final_v == pytest.approx(expected_v_single, rel=1e-4)
    assert res_single_phase.v_final_v == pytest.approx(70.916, rel=1e-3)

    res_two_phase = compute_energy_accounting(
        inductance_h=0.004,
        current_a=1.0,
        drive_mode="two_phase_dc",
        rotor_inertia_kg_m2=5.7e-6,
        angular_velocity_rad_s=50.0,
        load_inertia_kg_m2=20.0e-6,
        bulk_capacitance_f=100e-6,
        capacitance_tolerance_percent=20.0,
        power_supply_active=True,
        power_supply_current_a=1.0,
        fault_detection_delay_s=0.001,
        contactor_opening_delay_s=0.005,
        voltage_calculation_type="IDEALIZED_UPPER_BOUND_SCENARIO",
    )
    assert res_two_phase.e_magnetic_j == pytest.approx(0.004, rel=1e-6)
    assert res_two_phase.e_net_dumped_j == pytest.approx(0.180125, rel=1e-6)
    expected_v_two = math.sqrt(24.0**2 + 2.0 * 0.180125 / 80e-6)
    assert res_two_phase.v_final_v == pytest.approx(expected_v_two, rel=1e-4)
    assert res_two_phase.v_final_v == pytest.approx(71.268, rel=1e-3)

    # Verify explicit assumptions about copper losses, current decay, freewheeling paths, and energy capture
    assumptions_text = " ".join(res_two_phase.assumptions_declared)
    reconciliation_text = " ".join(res_two_phase.reconciliation_notes)
    assert "100% of net dumped energy is transferred into the capacitor (lossless scenario" in assumptions_text
    assert "without resistive I^2*R dissipation" in assumptions_text
    assert "MOSFET body diodes act as a full-wave bridge rectifier" in reconciliation_text
    assert "4.41 mJ per phase' is unsupported and classified as UNDETERMINED" in reconciliation_text





