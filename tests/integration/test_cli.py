"""Integration smoke tests for EARS-RL command-line interface."""

from pathlib import Path

from typer.testing import CliRunner

from ears_rl.cli import app

runner = CliRunner()


def test_cli_help():
    """Verify ears-rl --help returns exit code 0 and lists subcommands."""
    result = runner.invoke(app, ["--help"])
    assert result.exit_code == 0
    assert "EARS-RL" in result.stdout
    assert "doctor" in result.stdout
    assert "bench" in result.stdout
    assert "config" in result.stdout


def test_cli_doctor():
    """Verify ears-rl doctor executes and audits environment."""
    result = runner.invoke(app, ["doctor"])
    assert result.exit_code == 0
    assert "Diagnostic" in result.stdout
    assert "Python Runtime" in result.stdout


def test_cli_config_validate():
    """Verify ears-rl config validate succeeds on sample config."""
    cfg_file = "configs/hardware/bench_mock.yaml"
    result = runner.invoke(app, ["config", "validate", "--config", cfg_file])
    assert result.exit_code == 0
    assert "Validated Successfully" in result.stdout


def test_cli_sim_check():
    """Verify ears-rl sim check handles optional Isaac Lab isolation gracefully."""
    result = runner.invoke(app, ["sim", "check"])
    assert result.exit_code == 0
    assert "Isaac Lab" in result.stdout


def test_cli_hardware_status_read_only():
    """Verify ears-rl hardware status reports disabled invariant without hardware command."""
    result = runner.invoke(app, ["hardware", "status"])
    assert result.exit_code == 0
    assert "DISABLED" in result.stdout
    assert "Zero pulse generation" in result.stdout


def test_cli_bench_mock_and_data_validate(tmp_path: Path):
    """Verify ears-rl bench mock followed by ears-rl data validate."""
    out_dir = tmp_path / "cli_bench_run"
    bench_result = runner.invoke(app, [
        "bench", "mock",
        "--config", "configs/hardware/bench_mock.yaml",
        "--duration-s", "0.5",
        "--output-dir", str(out_dir),
    ])
    assert bench_result.exit_code == 0
    assert "Completed Successfully" in bench_result.stdout

    data_file = out_dir / "data.csv"
    assert data_file.exists()

    val_result = runner.invoke(app, ["data", "validate", "--path", str(data_file)])
    assert val_result.exit_code == 0
    assert "Dataset Valid: YES" in val_result.stdout
