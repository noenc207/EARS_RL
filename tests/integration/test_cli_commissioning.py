"""Integration tests for safe hardware commissioning CLI and diagnostic runner."""

from __future__ import annotations

import json
from pathlib import Path

from typer.testing import CliRunner

from ears_rl.cli import app
from ears_rl.hardware.protocol import MockBenchTransceiver
from ears_rl.hardware.serial_transceiver import run_commissioning_diagnostic

runner = CliRunner()


def test_cli_hardware_commission_mock(tmp_path: Path):
    """Verify that CLI commission command executes cleanly in mock mode and produces report."""
    report_file = tmp_path / "test_commission_report.json"
    result = runner.invoke(
        app,
        ["hardware", "commission", "--mock", "--output", str(report_file)],
    )
    assert result.exit_code == 0
    assert "Initiating Safe Commissioning Handshake" in result.stdout
    assert "Hardware Commissioning Diagnostic Results" in result.stdout
    assert "PASS" in result.stdout
    assert report_file.exists()

    with open(report_file, "r", encoding="utf-8") as f:
        data = json.load(f)
    assert data["overall_status"] == "PASSED"
    assert data["mode"] == "MOCK / SYNTHETIC"
    assert data["checks"]["ping_pong"]["status"] == "PASS"
    assert data["checks"]["motion_rejection_safety_invariant"]["status"] == "PASS"


def test_commissioning_diagnostic_mock_transceiver_pass():
    """Verify programmatic execution of commissioning diagnostic passes on standard MockBenchTransceiver."""
    mock = MockBenchTransceiver(commissioning_mode=True)
    report = run_commissioning_diagnostic(mock, is_mock=True)

    assert report["overall_status"] == "PASSED"
    assert report["checks"]["arm_controller"]["state"] == "ENABLED_ARMED"
    assert report["checks"]["motion_rejection_safety_invariant"]["status"] == "PASS"
    assert report["checks"]["disarm_controller"]["state"] == "IDLE_DISARMED"


def test_commissioning_diagnostic_fails_if_motion_not_rejected():
    """SAFETY INVARIANT: If firmware somehow accepts motion in commissioning mode, report flags CRITICAL_FAIL."""
    # Transceiver with commissioning_mode=False simulates rogue firmware accepting motion commands
    mock = MockBenchTransceiver(commissioning_mode=False)
    report = run_commissioning_diagnostic(mock, is_mock=True)

    assert report["overall_status"] == "FAILED"
    rej_check = report["checks"]["motion_rejection_safety_invariant"]
    assert rej_check["status"] == "CRITICAL_FAIL"
    assert "SAFETY VIOLATION" in rej_check["reason"]


def test_cli_hardware_commission_nonexistent_port():
    """Verify that specifying a non-existent physical port handles errors gracefully with exit code 1."""
    result = runner.invoke(
        app,
        ["hardware", "commission", "--no-mock", "--port", "COM999"],
    )
    assert result.exit_code == 1
    assert "Hardware Commissioning Error" in result.stdout
