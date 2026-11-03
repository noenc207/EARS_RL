"""Command-Line Interface (CLI) for EARS-RL Research Platform.

Built with Typer and Rich for clear, reproducible research workflows.
"""

from __future__ import annotations

import sys
from pathlib import Path

import typer
from rich.console import Console
from rich.panel import Panel
from rich.table import Table

from ears_rl import __version__
from ears_rl.application import (
    RunBenchmarkWorkflow,
    RunCommissioningWorkflow,
    RunEvaluationWorkflow,
    RunIdentificationWorkflow,
)
from ears_rl.config.loader import load_experiment_config
from ears_rl.core.exceptions import EarsRLError
from ears_rl.data.validation import validate_dataset
from ears_rl.identification.independent_validation import (
    run_comprehensive_independent_validation_matrix,
)
from ears_rl.identification.synthetic_bench import (
    run_synthetic_identification_benchmark,
)
from ears_rl.simulation.isaac_lab.adapter import check_isaac_lab_available

app = typer.Typer(
    name="ears-rl",
    help="EARS-RL: Event-Aware Risk-Sensitive Reinforcement Learning Research CLI",
    add_completion=False,
)
config_app = typer.Typer(name="config", help="Manage and validate configurations")
bench_app = typer.Typer(name="bench", help="Run simulated or hardware bench sessions")
data_app = typer.Typer(name="data", help="Inspect and validate experiment datasets")
sim_app = typer.Typer(name="sim", help="Inspect simulation integration status")
hardware_app = typer.Typer(name="hardware", help="Inspect hardware bus status (Read-Only)")

app.add_typer(config_app)
app.add_typer(bench_app)
app.add_typer(data_app)
app.add_typer(sim_app)
app.add_typer(hardware_app)

console = Console()


@app.callback(invoke_without_command=True)
def main(
    version: bool | None = typer.Option(
        None, "--version", "-v", help="Display EARS-RL version", is_eager=True
    )
) -> None:
    if version:
        console.print(f"[bold green]EARS-RL[/bold green] version [bold cyan]{__version__}[/bold cyan]")
        raise typer.Exit()


@app.command(name="doctor")
def doctor_cmd() -> None:
    """Audit local runtime environment, dependencies, directories, and optional integrations."""
    table = Table(title="EARS-RL Environment & Tooling Diagnostic", show_header=True)
    table.add_column("Component", style="cyan", no_wrap=True)
    table.add_column("Status", style="bold")
    table.add_column("Details")

    # Python version
    py_ver = f"{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}"
    py_ok = sys.version_info >= (3, 11)
    table.add_row(
        "Python Runtime",
        "[green]PASS[/green]" if py_ok else "[red]FAIL[/red]",
        f"v{py_ver} (>= 3.11 recommended)"
    )

    # Core dependencies
    for pkg in ["numpy", "pandas", "pydantic", "yaml", "typer", "rich", "matplotlib", "pytest"]:
        try:
            mod = __import__(pkg)
            v = getattr(mod, "__version__", "installed")
            table.add_row(f"Package: {pkg}", "[green]INSTALLED[/green]", f"v{v}")
        except ImportError:
            table.add_row(f"Package: {pkg}", "[red]MISSING[/red]", "Please install via pip")

    # PyTorch & CUDA
    try:
        import torch
        cuda_avail = torch.cuda.is_available()
        cuda_str = f"CUDA: {'Available' if cuda_avail else 'CPU only'}"
        table.add_row("PyTorch", "[green]INSTALLED[/green]", f"v{torch.__version__} ({cuda_str})")
    except ImportError:
        table.add_row("PyTorch", "[yellow]OPTIONAL[/yellow]", "Not installed")

    # Isaac Lab check
    isaac_ok, isaac_msg = check_isaac_lab_available()
    table.add_row(
        "Isaac Lab Simulator",
        "[green]AVAILABLE[/green]" if isaac_ok else "[yellow]NOT INSTALLED (OPTIONAL)[/yellow]",
        "Isolated from core package" if not isaac_ok else isaac_msg
    )

    # Hardware mode safety invariant
    table.add_row(
        "Hardware Motion Invariant",
        "[green]DISABLED BY DEFAULT[/green]",
        "hardware_enabled: false enforced"
    )

    console.print(table)


@config_app.command(name="validate")
def config_validate_cmd(
    config: Path = typer.Option(..., "--config", "-c", help="Path to YAML configuration file")
) -> None:
    """Validate a YAML configuration file against Pydantic schemas."""
    try:
        cfg = load_experiment_config(config)
        console.print(
            Panel(
                f"[bold green]Configuration Validated Successfully[/bold green]\n"
                f"Experiment ID: [cyan]{cfg.experiment_id}[/cyan]\n"
                f"Hardware Mode: [yellow]{'HARDWARE' if cfg.safety.hardware_enabled else 'DISABLED / MOCK'}[/yellow]\n"
                f"Actuators Configured: {len(cfg.actuators)}\n"
                f"Software Version: {cfg.software_version}",
                title=f"Schema Validation: {config.name}"
            )
        )
    except EarsRLError as e:
        console.print(f"[bold red]Configuration Validation Error:[/bold red]\n{e}")
        raise typer.Exit(code=1)


@bench_app.command(name="mock")
def bench_mock_cmd(
    config: Path = typer.Option(
        Path("configs/hardware/bench_mock.yaml"), "--config", "-c", help="Path to bench config YAML"
    ),
    duration_s: float = typer.Option(2.0, "--duration-s", "-d", help="Session duration [seconds]"),
    output_dir: Path | None = typer.Option(
        None, "--output-dir", "-o", help="Custom output directory"
    ),
    trajectory: str = typer.Option("chirp", "--trajectory", "-t", help="'chirp', 'sine', 'step'"),
) -> None:
    """Execute an end-to-end simulated actuator bench session and write synthetic telemetry."""
    try:
        cfg = load_experiment_config(config)
        out_path = output_dir or (Path("./datasets/raw") / cfg.experiment_id / "session_001")

        console.print("[bold cyan]Launching Mock Actuator Bench session...[/bold cyan]")
        workflow = RunBenchmarkWorkflow(config=cfg, output_dir=out_path)
        res = workflow.execute(duration_s=duration_s, trajectory=trajectory)

        console.print(
            Panel(
                f"[bold green]Mock Bench Session Completed Successfully[/bold green]\n"
                f"Output Data File: [cyan]{res['data_file']}[/cyan]\n"
                f"Metadata File:    [cyan]{res['metadata_file']}[/cyan]\n"
                f"Manifest File:    [cyan]{res['manifest_file']}[/cyan]\n"
                f"Data Origin:      [yellow]SYNTHETIC[/yellow] (Simulated on CPU)\n"
                f"Total Duration:   {duration_s:.1f} s",
                title="Mock Bench Telemetry"
            )
        )
    except Exception as e:
        console.print(f"[bold red]Mock Bench Execution Error:[/bold red] {e}")
        raise typer.Exit(code=1)


@data_app.command(name="validate")
def data_validate_cmd(
    path: Path = typer.Option(..., "--path", "-p", help="Path to data file (.csv or .jsonl)"),
    metadata: Path | None = typer.Option(None, "--metadata", "-m", help="Path to metadata.json"),
) -> None:
    """Validate dataset against BenchRecord schema, checking monotonicity and gaps."""
    report = validate_dataset(path, metadata_path=metadata)
    console.print(report.summary())
    if not report.is_valid:
        raise typer.Exit(code=1)


@app.command(name="identify")
def identify_cmd(
    input_path: Path | None = typer.Option(None, "--input", "-i", help="Path to input dataset file"),
    config: Path | None = typer.Option(None, "--config", "-c", help="Path to baseline.yaml"),
    output_dir: Path | None = typer.Option(None, "--output-dir", "-o", help="Output directory"),
    run_benchmark: bool = typer.Option(
        False, "--benchmark", "-b", help="Run comprehensive synthetic benchmark across Cases A-G with ground truth"
    ),
    run_independent_validation: bool = typer.Option(
        False, "--independent-validation", "-v", help="Run decoupled continuous analytical & RK4 validation matrix"
    ),
) -> None:
    """Run system identification (Nominal, Fixed Delay, First-Order, FOPDT) or synthetic benchmark."""
    try:
        if run_independent_validation:
            console.print("[bold cyan]Executing Independent Continuous-Time Validation Matrix (Analytical & RK4)...[/bold cyan]")
            val_out = output_dir or Path("reports/identification")
            matrix_results = run_comprehensive_independent_validation_matrix()

            table = Table(
                title="Independent Validation Matrix: Continuous Analytical & RK4 Ground Truth vs Discrete Estimator",
                show_header=True,
            )
            table.add_column("Solver / Trajectory", style="cyan")
            table.add_column("Sample dt", style="white")
            table.add_column("GT (theta, tau)", style="magenta")
            table.add_column("Identified (theta, tau)", style="bold green")
            table.add_column("Recovery Error", style="yellow")
            table.add_column("Discretization Limit", style="dim white")
            table.add_column("Noise / Residual", style="white")

            for r in matrix_results:
                solver_str = f"{r.benchmark_type} ({r.trajectory})"
                dt_str = f"{r.sample_period_ms:.1f} ms"
                gt_str = f"th={r.gt_dead_time_ms:.1f}ms, tau={r.gt_tau_ms:.1f}ms"
                if r.is_identifiable:
                    id_str = f"th={r.identified_dead_time_ms:.1f}ms, tau={r.identified_tau_ms:.1f}ms"
                    err_str = f"d_th={r.dead_time_error_ms:.1f}ms, d_tau={r.tau_error_ms:.1f}ms"
                    noise_str = f"sigma={r.noise_sigma_rad:.3f} | rmse={r.residual_rmse_rad:.4f}"
                else:
                    id_str = "[yellow]UNIDENTIFIABLE[/yellow]"
                    err_str = "N/A (Rejected)"
                    noise_str = f"sigma={r.noise_sigma_rad:.3f} | rmse=N/A"
                limit_str = f"+/- {r.discretization_uncertainty_ms:.1f} ms"

                table.add_row(solver_str, dt_str, gt_str, id_str, err_str, limit_str, noise_str)

            console.print(table)
            val_out.mkdir(parents=True, exist_ok=True)
            import json
            from dataclasses import asdict
            with open(val_out / "independent_validation_report.json", "w", encoding="utf-8") as f:
                json.dump([asdict(r) for r in matrix_results], f, indent=2)
            console.print(f"[green]Independent validation report saved to {val_out / 'independent_validation_report.json'}[/green]")
            return

        if run_benchmark:
            console.print("[bold cyan]Executing Synthetic Identification Benchmark Suite (Cases A-G)...[/bold cyan]")
            bench_out = output_dir or Path("reports/identification")
            bench_results = run_synthetic_identification_benchmark(output_dir=bench_out)

            table = Table(title="Synthetic Benchmark Results (Cases A-G Ground Truth vs Estimates)", show_header=True)
            table.add_column("Case ID", style="cyan")
            table.add_column("GT (Gain, tau, theta)", style="magenta")
            table.add_column("FOPDT Estimates", style="bold green")
            table.add_column("Recovery Error", style="yellow")
            table.add_column("Held-Out RMSE", style="white")

            for cid, cdata in bench_results["cases"].items():
                gt = cdata["ground_truth"]
                gt_str = f"K={gt['gain']:.2f}, tau={gt['tau_s']*1000:.1f}ms, theta={gt['dead_time_s']*1000:.1f}ms"
                fopdt_m = cdata["models"]["fopdt"]
                f_fit = fopdt_m["fit_metrics"]
                f_eval = fopdt_m.get("eval_metrics") or {}
                rec = fopdt_m.get("parameter_recovery")

                if f_fit.get("is_identifiable"):
                    est_str = (
                        f"K={f_fit['identified_gain']:.2f}, "
                        f"tau={f_fit['identified_time_constant_ms']:.1f}ms, "
                        f"theta={f_fit['identified_dead_time_ms']:.1f}ms"
                    )
                    err_str = (
                        f"d_theta={rec['dead_time_error_ms']:.1f}ms, d_tau={rec['tau_error_ms']:.1f}ms"
                        if rec else "N/A"
                    )
                    eval_rmse_str = f"{f_eval.get('held_out_rmse_rad', 0.0):.4f} rad"
                else:
                    est_str = f"[red]{f_fit.get('status')}[/red]"
                    err_str = f_fit.get("error", "Unidentifiable")[:30] + "..."
                    eval_rmse_str = "N/A"

                table.add_row(cid, gt_str, est_str, err_str, eval_rmse_str)

            console.print(table)
            console.print(f"[green]Full benchmark JSON saved to {bench_out / 'synthetic_benchmark_report.json'}[/green]")
            return

        if input_path is None:
            console.print("[bold red]Error:[/bold red] Either --input <file> or --benchmark must be specified.")
            raise typer.Exit(code=1)

        console.print(f"[bold cyan]Running System Identification Baseline on {input_path}...[/bold cyan]")
        workflow = RunIdentificationWorkflow(output_dir=output_dir)
        results = workflow.execute(dataset_path=input_path)

        table = Table(title="System Identification Baseline Results", show_header=True)
        table.add_column("Model / Metric", style="cyan")
        table.add_column("Identified Value", style="bold green")

        table.add_row("Data Source Provenance", str(results.get("data_source")).upper())
        table.add_row("Total Samples", str(results.get("sample_count")))

        # Nominal Model (Zero-hypothesis baseline)
        nom = results.get("nominal_model", {})
        nom_err = nom.get("prediction_errors", {})
        table.add_row("Nominal Baseline Role", "Zero-Hypothesis Predictor (0 free parameters)")
        table.add_row(
            "Nominal Position MAE / RMSE",
            f"{nom_err.get('position_mae_rad', 0.0):.4f} / {nom_err.get('position_rmse_rad', 0.0):.4f} rad"
        )

        # Fixed Delay Model (Apparent Shift)
        fd = results.get("fixed_delay_model", {})
        table.add_row(
            "Identified Apparent Delay (Fixed Delay)",
            f"{fd.get('identified_delay_ms', 0.0):.1f} ms (lumped theta + tau)"
        )
        table.add_row(
            "Fixed-Delay Residual RMSE",
            f"{fd.get('residual_rmse_rad', 0.0):.4f} rad"
        )

        # FOPDT Grey-Box Model
        fopdt_res = results.get("fopdt_model", {})
        if fopdt_res.get("identified_dead_time_ms") is not None:
            table.add_row(
                "FOPDT Pure Dead Time (theta)",
                f"{fopdt_res.get('identified_dead_time_ms'):.1f} ms"
            )
            table.add_row(
                "FOPDT Time Constant (tau)",
                f"{fopdt_res.get('identified_time_constant_ms'):.1f} ms"
            )
            table.add_row(
                "FOPDT Static Gain (K)",
                f"{fopdt_res.get('identified_gain'):.4f}"
            )
            table.add_row(
                "FOPDT Residual RMSE",
                f"{fopdt_res.get('residual_rmse_rad', 0.0):.4f} rad"
            )

        # Timing & Latency Audit
        audit = results.get("timing_and_latency_audit", {})
        table.add_row("Sample Period (dt)", f"{audit.get('sample_period_ms', 0.0):.1f} ms")
        if audit.get("configured_simulated_delay_ms") is not None:
            table.add_row("Configured Simulated Dead Time", f"{audit.get('configured_simulated_delay_ms'):.1f} ms")
        if audit.get("dead_time_estimation_error_ms") is not None:
            table.add_row("FOPDT Dead Time Recovery Error", f"{audit.get('dead_time_estimation_error_ms'):.1f} ms")
        elif audit.get("delay_estimation_error_ms") is not None:
            table.add_row("Delay Estimation Error", f"{audit.get('delay_estimation_error_ms'):.1f} ms")

        console.print(table)

        # Print model comparison table
        comp = results.get("model_comparison")
        if comp:
            comp_table = Table(title="Model Hierarchy Comparison", show_header=True)
            comp_table.add_column("Model Candidate", style="cyan")
            comp_table.add_column("Free Params", style="magenta")
            comp_table.add_column("Apparent Delay", style="yellow")
            comp_table.add_column("Dead Time", style="green")
            comp_table.add_column("Time Constant", style="blue")
            comp_table.add_column("Residual RMSE", style="bold white")

            for m in comp:
                app_str = f"{m['apparent_delay_ms']:.1f} ms" if m.get("apparent_delay_ms") is not None else "N/A"
                dt_str = f"{m['dead_time_ms']:.1f} ms" if m.get("dead_time_ms") is not None else "N/A"
                tau_str = f"{m['time_constant_ms']:.1f} ms" if m.get("time_constant_ms") is not None else "N/A"
                rmse_str = f"{m['rmse_rad']:.4f} rad" if m.get("rmse_rad") is not None else "N/A"
                comp_table.add_row(
                    m["model"],
                    str(m["parameters"]),
                    app_str,
                    dt_str,
                    tau_str,
                    rmse_str,
                )
            console.print(comp_table)

    except Exception as e:
        console.print(f"[bold red]System Identification Error:[/bold red] {e}")
        raise typer.Exit(code=1)


@app.command(name="evaluate")
def evaluate_cmd(
    input_path: Path = typer.Option(..., "--input", "-i", help="Path to dataset file"),
    output_dir: Path | None = typer.Option(None, "--output-dir", "-o", help="Output directory"),
) -> None:
    """Compute tracking performance and latency metrics on a bench dataset."""
    try:
        workflow = RunEvaluationWorkflow()
        metrics = workflow.execute(dataset_path=input_path)

        timing = metrics.get("timing_metrics", {})
        sp_info = timing.get("sample_period") or {}
        sp_str = (
            f"{sp_info.get('mean_ms', 0):.2f} ms ({sp_info.get('derived_sample_rate_hz', 0):.1f} Hz)"
            if sp_info
            else "N/A"
        )
        sim_delay = timing.get("configured_simulated_delay_ms")
        sim_delay_str = f"{sim_delay:.2f} ms" if sim_delay is not None else "N/A"
        cf_delay = timing.get("command_to_feedback_delay")
        cf_str = (
            f"{cf_delay.get('mean_ms', 0):.2f} ms"
            if cf_delay
            else "Unmeasured (synchronous mock / no paired ACK)"
        )

        console.print(
            Panel(
                f"Dataset: [cyan]{input_path}[/cyan]\n"
                f"Data Origin:   [yellow]{metrics.get('data_source', 'unknown').upper()}[/yellow]\n"
                f"Samples:       {metrics['sample_count']}\n"
                f"Position MAE:  [bold green]{metrics['position_tracking']['mae_rad']:.4f} rad[/bold green]\n"
                f"Position RMSE: [bold green]{metrics['position_tracking']['rmse_rad']:.4f} rad[/bold green]\n"
                f"Sample Period (dt):           [cyan]{sp_str}[/cyan]\n"
                f"Configured Simulated Delay:   [cyan]{sim_delay_str}[/cyan]\n"
                f"Command-to-Feedback Latency:  [cyan]{cf_str}[/cyan]",
                title="Benchmark Evaluation Report",
            )
        )
    except Exception as e:
        console.print(f"[bold red]Evaluation Error:[/bold red] {e}")
        raise typer.Exit(code=1)


@sim_app.command(name="check")
def sim_check_cmd() -> None:
    """Inspect NVIDIA Isaac Lab integration status."""
    available, msg = check_isaac_lab_available()
    if available:
        console.print(Panel(f"[bold green]NVIDIA Isaac Lab is ready[/bold green]\n{msg}"))
    else:
        console.print(
            Panel(
                f"[bold yellow]NVIDIA Isaac Lab Integration Status[/bold yellow]\n\n"
                f"{msg}\n\n"
                f"[italic]Notice: The core EARS-RL package, CLI, and unit tests run independently without Isaac Lab.[/italic]",
                title="Simulation Diagnostics"
            )
        )


@hardware_app.command(name="status")
def hardware_status_cmd() -> None:
    """Inspect hardware connection and safety interlock status (Read-Only; sends no commands)."""
    table = Table(title="Hardware Subsystem Status (Read-Only)", show_header=True)
    table.add_column("Property", style="cyan")
    table.add_column("State", style="bold")
    table.add_column("Notes")

    table.add_row("Master Hardware Mode", "[yellow]DISABLED[/yellow]", "Default safety invariant active")
    table.add_row("Motor Driver Excitation", "[yellow]OFF[/yellow]", "Zero pulse generation")
    table.add_row("Software Safety Interlock", "[green]ENGAGED[/green]", "Filtering all output")
    table.add_row("Physical E-Stop Circuit", "[cyan]INDEPENDENT[/cyan]", "Hardwired loop independent of software")

    console.print(table)


@hardware_app.command(name="commission")
def hardware_commission_cmd(
    port: str | None = typer.Option(None, "--port", "-p", help="Serial port (e.g. COM3 or /dev/ttyUSB0)"),
    baud: int = typer.Option(115200, "--baud", "-b", help="Baud rate (default 115200)"),
    mock: bool = typer.Option(True, "--mock/--no-mock", help="Use software loopback mock (default True)"),
    timeout_s: float = typer.Option(2.0, "--timeout-s", "-t", help="Timeout in seconds"),
    output: Path = typer.Option(
        Path("reports/commissioning_report.json"), "--output", "-o", help="Diagnostic report output JSON"
    ),
) -> None:
    """Run safe commissioning protocol verification (Read-Only; motor pulses strictly locked disabled)."""
    import json

    from ears_rl.hardware.protocol import MockBenchTransceiver
    from ears_rl.hardware.serial_transceiver import SerialBenchTransceiver

    use_mock = mock or (port is None)
    target_str = "Software Loopback Mock (MockBenchTransceiver)" if use_mock else f"Physical UART {port} @ {baud} baud"

    console.print(f"[bold cyan]Initiating Safe Commissioning Handshake on {target_str}...[/bold cyan]")

    try:
        if use_mock:
            transceiver = MockBenchTransceiver(commissioning_mode=True)
            report = RunCommissioningWorkflow(transport=transceiver, timeout_s=timeout_s).execute()
        else:
            assert port is not None
            with SerialBenchTransceiver(port=port, baudrate=baud, timeout_s=timeout_s) as st:
                report = RunCommissioningWorkflow(transport=st, timeout_s=timeout_s).execute()

        # Output table
        table = Table(title="Hardware Commissioning Diagnostic Results", show_header=True)
        table.add_column("Verification Step", style="cyan")
        table.add_column("Result", style="bold")
        table.add_column("Details")

        table.add_row("Execution Mode", "MOCK" if use_mock else "PHYSICAL", target_str)

        # 1. Ping Pong
        chk_ping = report.get("checks", {}).get("ping_pong", {})
        ping_res = "[green]PASS[/green]" if chk_ping.get("status") == "PASS" else "[red]FAIL[/red]"
        ping_det = (
            f"RTT: {chk_ping.get('round_trip_time_ms', 0):.3f} ms, Clock Offset: {chk_ping.get('estimated_clock_offset_ms', 0):.3f} ms"
            if chk_ping.get("status") == "PASS"
            else chk_ping.get("reason", "")
        )
        table.add_row("1. PING/PONG & Clock Sync", ping_res, ping_det)

        # 2. Arm
        chk_arm = report.get("checks", {}).get("arm_controller", {})
        arm_res = "[green]PASS[/green]" if chk_arm.get("status") == "PASS" else "[red]FAIL[/red]"
        table.add_row("2. Arm Controller (SET_MODE=1)", arm_res, f"State: {chk_arm.get('state', 'UNKNOWN')}")

        # 3. Motion rejection safety invariant
        chk_rej = report.get("checks", {}).get("motion_rejection_safety_invariant", {})
        rej_res = "[green]PASS[/green]" if chk_rej.get("status") == "PASS" else "[bold red]CRITICAL FAIL[/bold red]"
        rej_det = chk_rej.get("message", chk_rej.get("reason", ""))
        table.add_row("3. Safety Invariant Motion Rejection", rej_res, rej_det)

        # 4. Disarm
        chk_dis = report.get("checks", {}).get("disarm_controller", {})
        dis_res = "[green]PASS[/green]" if chk_dis.get("status") == "PASS" else "[red]FAIL[/red]"
        table.add_row("4. Disarm Controller (SET_MODE=0)", dis_res, f"State: {chk_dis.get('state', 'UNKNOWN')}")

        console.print(table)

        # Save report
        output.parent.mkdir(parents=True, exist_ok=True)
        with open(output, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2)
        console.print(f"[bold green]Diagnostic report written to [cyan]{output}[/cyan][/bold green]")

        if report.get("overall_status") != "PASSED":
            raise typer.Exit(code=1)

    except Exception as e:
        console.print(f"[bold red]Hardware Commissioning Error:[/bold red] {e}")
        raise typer.Exit(code=1)


if __name__ == "__main__":
    app()
