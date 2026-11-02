#!/usr/bin/env python3
"""Run isolated DS-vs-NT and 20ms-vs-2ms local robot simulations.

The matrix uses the already-built native robot through sim_headless.sh. Each
case gets a fresh robot process, local NT/WS ports, logs, and a new artifact
file. Existing artifacts are never overwritten. Only simulator processes
started by this script are stopped.
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

try:
    from websockets.sync.client import connect as ws_connect
except ImportError as exc:
    raise SystemExit("Install websockets in the selected Python environment") from exc


ROOT = Path(__file__).resolve().parents[1]
SIM_LAUNCHER = ROOT / "scripts" / "sim_headless.sh"
COMPARE_SCRIPT = ROOT / "scripts" / "nt_gamepad_compare.py"
DEFAULT_ARTIFACTS = ROOT / "artifacts" / "frequency-matrix"


def proc_children(pid: int) -> set[int]:
    """Return PID plus all descendants using Linux /proc child lists."""
    found: set[int] = set()
    pending = [pid]
    while pending:
        current = pending.pop()
        if current in found or not Path(f"/proc/{current}").exists():
            continue
        found.add(current)
        try:
            child_text = Path(f"/proc/{current}/task/{current}/children").read_text()
            pending.extend(int(item) for item in child_text.split())
        except (OSError, ValueError):
            continue
    return found


def listening_sockets(*ports: int) -> list[dict]:
    wanted = set(ports)
    found = []
    for path, family in (("/proc/net/tcp", socket.AF_INET), ("/proc/net/tcp6", socket.AF_INET6)):
        try:
            lines = Path(path).read_text().splitlines()[1:]
        except OSError:
            continue
        for line in lines:
            fields = line.split()
            addr_hex, port_hex = fields[1].split(":")
            port = int(port_hex, 16)
            if port not in wanted or fields[3] != "0A":
                continue
            raw = bytes.fromhex(addr_hex)[::-1]
            found.append({"port": port, "address": socket.inet_ntop(family, raw),
                          "inode": fields[9]})
    return found


def socket_owners(inodes: set[str], pids: set[int]) -> set[int]:
    owners = set()
    for pid in pids:
        fd_dir = Path(f"/proc/{pid}/fd")
        try:
            for fd in fd_dir.iterdir():
                try:
                    target = os.readlink(fd)
                except OSError:
                    continue
                if target.startswith("socket:[") and target[8:-1] in inodes:
                    owners.add(pid)
                    break
        except OSError:
            pass
    return owners


def wait_for_owned_listeners(process: subprocess.Popen, ports: tuple[int, int], timeout: float,
                             log_path: Path) -> tuple[list[dict], dict[str, list[int]]]:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"Simulator exited early; see {log_path}")
        pids = proc_children(process.pid)
        listeners = listening_sockets(*ports)
        owners_by_port = {
            str(port): sorted({pid for entry in listeners if entry["port"] == port
                               for pid in socket_owners({entry["inode"]}, pids)})
            for port in ports
        }
        if all(owners_by_port.values()):
            return listeners, owners_by_port
        time.sleep(0.05)
    raise TimeoutError(f"Simulator did not own both local listeners {ports} within {timeout:.0f}s; see {log_path}")


def ensure_ports_free(ports: tuple[int, int]) -> None:
    listeners = listening_sockets(*ports)
    if listeners:
        details = ", ".join(f"{x['address']}:{x['port']}" for x in listeners)
        raise RuntimeError(f"Refusing to start: requested local port is already listening ({details})")


def disable_hal(ws_port: int) -> None:
    """Best-effort disabled/neutral packet over this run's local HAL endpoint."""
    uri = f"ws://127.0.0.1:{ws_port}/wpilibws"
    try:
        with ws_connect(uri, open_timeout=0.5, close_timeout=0.5,
                        ping_interval=None, proxy=None) as ws:
            ws.send(json.dumps({"type": "Joystick", "device": "0", "data": {
                ">axes": [0.0] * 6, ">povs": [], ">buttons": [False] * 12,
            }}, separators=(",", ":")))
            ws.send(json.dumps({"type": "DriverStation", "device": "", "data": {
                ">new_data": True, ">enabled": False, ">autonomous": False,
                ">test": False, ">estop": False, ">fms": False, ">ds": False,
                ">station": "blue1", ">match_time": -1.0, ">game_data": "",
            }}, separators=(",", ":")))
    except Exception:
        pass


def stop_simulator(process: subprocess.Popen, ws_port: int) -> None:
    disable_hal(ws_port)
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=3)
    except ProcessLookupError:
        pass


def make_run_dir(base: Path) -> Path:
    base.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    target = base / stamp
    suffix = 0
    while True:
        candidate = target if suffix == 0 else base / f"{stamp}-{suffix}"
        try:
            candidate.mkdir(exist_ok=False)
            return candidate
        except FileExistsError:
            suffix += 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=8.0,
                        help="measurement interval per case (6 to 10 seconds recommended)")
    parser.add_argument("--include-vision-load", action="store_true",
                        help="run all eight cadence/load combinations; default is four unloaded cases")
    parser.add_argument("--nt-port", type=int, default=5810)
    parser.add_argument("--ws-port", type=int, default=3300)
    parser.add_argument("--timeout-seconds", type=float, default=150.0,
                        help="per-simulator timeout; maximum 165 seconds plus cleanup allowance")
    parser.add_argument("--artifacts", type=Path, default=DEFAULT_ARTIFACTS)
    args = parser.parse_args()
    if not 6 <= args.seconds <= 30:
        parser.error("--seconds must be between 6 and 30")
    if not 10 <= args.timeout_seconds <= 165:
        parser.error("--timeout-seconds must be between 10 and 165")
    if not 1024 <= args.nt_port <= 65533 or not 1024 <= args.ws_port <= 65535:
        parser.error("ports must be between 1024 and 65535")
    if not SIM_LAUNCHER.is_file() or not COMPARE_SCRIPT.is_file():
        parser.error("expected simulator and comparison scripts are missing from this checkout")

    loads = ("none", "vision") if args.include_vision_load else ("none",)
    configs = [(mode, fast, load) for fast in (False, True)
               for mode in ("ds", "nt") for load in loads]
    run_dir = make_run_dir(args.artifacts)
    matrix = {
        "experiment": "local robot input/control frequency matrix",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "worktree": str(ROOT),
        "duration_s": args.seconds,
        "per_simulator_timeout_s": args.timeout_seconds,
        "physics_period_ms": 20,
        "cases": [],
    }
    summary_path = run_dir / "matrix-summary.json"
    for index, (mode, fast, load) in enumerate(configs):
        nt_port = args.nt_port + index * 2
        ws_port = args.ws_port + index
        case_name = f"{mode}-robot{'2' if fast else '20'}-load-{load}"
        result_path = run_dir / f"{case_name}.json"
        sim_log = run_dir / f"{case_name}.sim.log"
        compare_log = run_dir / f"{case_name}.compare.log"
        case = {"name": case_name, "mode": mode, "robot_period_ms": 2 if fast else 20,
                "nt_input_period_ms": 5 if mode == "nt" else 20,
                "fast_control_enabled": fast, "physics_period_ms": 20,
                "synthetic_load": load, "nt_port": nt_port, "ws_port": ws_port,
                "result": str(result_path), "sim_log": str(sim_log),
                "compare_log": str(compare_log), "started_utc": datetime.now(timezone.utc).isoformat()}
        matrix["cases"].append(case)
        proc = None
        try:
            ensure_ports_free((nt_port, ws_port))
            env = os.environ.copy()
            env.update({
                "FRC_SIM_NT_PORT": str(nt_port),
                "FRC_SIM_NT_GAMEPAD": "1" if mode == "nt" else "0",
                "FRC_SIM_LATENCY_PROBE": "1",
                "HALSIMWS_PORT": str(ws_port),
            })
            if fast:
                env["FRC_FAST_CONTROL"] = "1"
            else:
                env["FRC_FAST_CONTROL"] = "0"
            case_deadline = time.monotonic() + args.timeout_seconds
            with sim_log.open("wb") as log:
                proc = subprocess.Popen([str(SIM_LAUNCHER)], cwd=ROOT, env=env,
                                        stdin=subprocess.DEVNULL, stdout=log,
                                        stderr=subprocess.STDOUT, start_new_session=True)
            startup_timeout = min(45.0, max(5.0, case_deadline - time.monotonic()))
            listeners, owners = wait_for_owned_listeners(proc, (nt_port, ws_port), startup_timeout, sim_log)
            case["simulator_pid"] = proc.pid
            case["listener_addresses"] = [{"port": entry["port"], "address": entry["address"]}
                                          for entry in listeners if entry["port"] in (nt_port, ws_port)]
            case["listener_owner_pids_by_port"] = owners
            remaining = case_deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("per-simulator timeout expired during startup")
            command = [sys.executable, str(COMPARE_SCRIPT), "--mode", mode, "--load", load,
                       "--seconds", str(args.seconds), "--robot-period-ms", "2" if fast else "20",
                       "--nt-port", str(nt_port), "--ws-port", str(ws_port), "--output", str(result_path)]
            with compare_log.open("wb") as log:
                comparison = subprocess.Popen(command, cwd=ROOT, env=env,
                                               stdin=subprocess.DEVNULL, stdout=log,
                                               stderr=subprocess.STDOUT)
                try:
                    comparison.wait(timeout=remaining)
                except subprocess.TimeoutExpired:
                    comparison.send_signal(signal.SIGINT)
                    try:
                        comparison.wait(timeout=4)
                    except subprocess.TimeoutExpired:
                        comparison.terminate()
                        try:
                            comparison.wait(timeout=2)
                        except subprocess.TimeoutExpired:
                            comparison.kill()
                            comparison.wait(timeout=2)
                    raise TimeoutError("comparison exceeded the per-simulator timeout")
            case["compare_exit_code"] = comparison.returncode
            case["status"] = "complete" if comparison.returncode == 0 else "comparison_failed"
            if result_path.exists():
                run_result = json.loads(result_path.read_text())
                case["quality"] = run_result.get("quality", {})
                case["latency_summary_ms"] = run_result.get("latency_summary_ms", {})
                case["control_loop_period_ms"] = run_result.get("control_loop_period_ms", {})
                case["physics_samples"] = run_result.get("physics_samples", {})
                case["mechanism_cadence"] = run_result.get("mechanism_cadence", {})
                case["active_command_period_ms"] = run_result.get("active_command_period_ms", {})
                case["fast_loop"] = run_result.get("fast_loop", {})
        except Exception as exc:
            case["status"] = "failed"
            case["error"] = str(exc)
        finally:
            if proc is not None:
                stop_simulator(proc, ws_port)
        case["finished_utc"] = datetime.now(timezone.utc).isoformat()
        summary_path.write_text(json.dumps(matrix, indent=2) + "\n")
        print(f"{case_name}: {case['status']} ({result_path})", flush=True)

    failures = [case for case in matrix["cases"] if case["status"] != "complete"]
    matrix["completed_utc"] = datetime.now(timezone.utc).isoformat()
    matrix["status"] = "complete" if not failures else "completed_with_failures"
    summary_path.write_text(json.dumps(matrix, indent=2) + "\n")
    print(json.dumps({"status": matrix["status"], "case_count": len(configs),
                      "artifact_directory": str(run_dir),
                      "failed_cases": [case["name"] for case in failures]}, indent=2))
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
