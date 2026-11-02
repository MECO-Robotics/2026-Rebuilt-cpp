#!/usr/bin/env python3
"""Compare a simulated DS joystick with the robot's 5 ms NT gamepad input.

This script controls only the local WPILib HAL simulation server at
127.0.0.1:3300 and the local robot NT4 server at 127.0.0.1:5810. It never
starts the robot and refuses non-loopback connections. Run after launching the
headless robot with the requested telemetry build and environment flags.
"""
from __future__ import annotations

import argparse
import json
import math
import socket
import struct
import threading
import time
from collections import defaultdict
from pathlib import Path

try:
    import ntcore
    from websockets.sync.client import connect as ws_connect
except ImportError as exc:
    raise SystemExit(
        "Install RobotPy pyntcore and websockets in the selected Python environment"
    ) from exc


WS_HOST = "127.0.0.1"
WS_PORT = 3300
WS_URI = f"ws://{WS_HOST}:{WS_PORT}/wpilibws"
NT_HOST = "127.0.0.1"
NT_PORT = 5810
INPUT_TOPIC = "/Input/Gamepad"
PROBE_TOPIC = "/Diagnostics/Latency/RobotLoop"
NT_COMMAND_TOPIC = "/Simulation/NTGamepad/Consumed"
DS_COMMAND_TOPIC = "/Simulation/NTGamepad/DriverStationCommand"
DRIVE_MAX_MPS = 5.24
STEP_PERIOD_S = 0.073
STEP_VALUES = (-0.65, 0.65, 0.0, 0.35, -0.35, 0.0)
LOAD_STREAMS = {
    "questnav": (120.0, 12),
    "perception_a": (60.0, 20),
    "perception_b": (60.0, 20),
    "apriltag": (60.0, 16),
}
LOAD_ROOT = "/Simulation/LatencyLoad/"


def pct(values: list[float], p: float) -> float | None:
    if not values:
        return None
    x = (len(values := sorted(values)) - 1) * p
    lo, hi = math.floor(x), math.ceil(x)
    return values[lo] + (values[hi] - values[lo]) * (x - lo)


def listener_bindings(port: int) -> list[str]:
    """Report Linux listener addresses; the harness always connects to loopback."""
    matches: list[str] = []
    proc_tables = (("/proc/net/tcp", socket.AF_INET), ("/proc/net/tcp6", socket.AF_INET6))
    for path, family in proc_tables:
        try:
            lines = Path(path).read_text().splitlines()[1:]
        except OSError:
            continue
        for line in lines:
            fields = line.split()
            local_hex, state = fields[1], fields[3]
            addr_hex, port_hex = local_hex.split(":")
            if int(port_hex, 16) != port or state != "0A":
                continue
            raw = bytes.fromhex(addr_hex)[::-1]
            address = socket.inet_ntop(family, raw)
            matches.append(address)
    if not matches:
        raise RuntimeError(f"Cannot verify a listener on local port {port}")
    return matches


def wait_until(predicate, timeout_s: float, detail: str) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise TimeoutError(detail)


class HalSession:
    def __init__(self, server_now_us):
        self.server_now_us = server_now_us
        self.ws = ws_connect(WS_URI, open_timeout=3, close_timeout=2,
                             ping_interval=10, proxy=None, max_queue=64)
        self.stop_reader = threading.Event()
        self.reader_error: str | None = None
        self.received_messages = 0
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()
        self.send_lock = threading.Lock()

    def _read(self) -> None:
        while not self.stop_reader.is_set():
            try:
                message = self.ws.recv()
                json.loads(message)
                self.received_messages += 1
            except Exception as exc:
                if not self.stop_reader.is_set():
                    self.reader_error = str(exc)
                return

    def _send(self, message: dict) -> None:
        with self.send_lock:
            self.ws.send(json.dumps(message, separators=(",", ":")))

    def set_joystick(self, left_y: float, left_x: float = 0.0, right_x: float = 0.0,
                     new_data: bool = True, enabled: bool = True, a_button: bool = False) -> int:
        source_us = self.server_now_us()
        axes = [left_x, left_y, 0.0, 0.0, right_x, 0.0]
        buttons = [False] * 12
        buttons[0] = a_button
        self._send({"type": "Joystick", "device": "0", "data": {
            ">axes": axes, ">povs": [], ">buttons": buttons,
        }})
        if new_data:
            self._send({"type": "DriverStation", "device": "", "data": {
                ">new_data": True, ">enabled": enabled, ">autonomous": False,
                ">test": False, ">estop": False, ">fms": False, ">ds": True,
                ">station": "blue1", ">match_time": -1.0, ">game_data": "",
            }})
        return source_us

    def disable(self) -> None:
        try:
            self._send({"type": "Joystick", "device": "0", "data": {
                ">axes": [0.0] * 6, ">povs": [], ">buttons": [False] * 12,
            }})
            self._send({"type": "DriverStation", "device": "", "data": {
                ">new_data": True, ">enabled": False, ">autonomous": False,
                ">test": False, ">estop": False, ">fms": False, ">ds": False,
                ">station": "blue1", ">match_time": -1.0, ">game_data": "",
            }})
        except Exception:
            pass

    def close(self) -> None:
        self.stop_reader.set()
        try:
            self.ws.close()
        finally:
            self.reader.join(timeout=1)


class NtSession:
    def __init__(self):
        self.inst = ntcore.NetworkTableInstance.create()
        self.inst.startClient4("nt-gamepad-compare")
        self.inst.setServer(NT_HOST, NT_PORT)
        wait_until(lambda: self.inst.isConnected(), 5,
                   f"No local robot NT4 server at {NT_HOST}:{NT_PORT}")
        wait_until(lambda: self.inst.getServerTimeOffset() is not None, 5,
                   "NT4 client/server clock synchronization did not complete")
        self.initial_server_offset_us = self.inst.getServerTimeOffset()
        self.gamepad = self.inst.getDoubleArrayTopic(INPUT_TOPIC).publish(
            ntcore.PubSubOptions(periodic=0.005, sendAll=True)
        )
        opts = ntcore.PubSubOptions(periodic=0.005, sendAll=True, pollStorage=20000)
        self.robot_probe = self.inst.getDoubleArrayTopic(PROBE_TOPIC).subscribe([], opts)
        self.nt_command = self.inst.getDoubleArrayTopic(NT_COMMAND_TOPIC).subscribe([], opts)
        self.ds_command = self.inst.getDoubleArrayTopic(DS_COMMAND_TOPIC).subscribe([], opts)
        self.load_publishers = {}
        self.load_sequences = defaultdict(int)
        self.load_stop = threading.Event()
        self.load_thread: threading.Thread | None = None

    def set_gamepad(self, seq: int, left_y: float, left_x: float, right_x: float) -> int:
        local_us = ntcore._now()
        offset = self.inst.getServerTimeOffset()
        if offset is None:
            raise RuntimeError("NT server time synchronization was lost")
        source_us = local_us + offset
        # NTCore converts explicit publisher timestamps from client to server
        # time. Embed the converted value, but pass local time to Publisher.set.
        self.gamepad.set([1.0, float(seq), float(source_us), left_y, left_x, right_x], local_us)
        self.inst.flush()
        return source_us

    def server_now_us(self) -> int:
        offset = self.inst.getServerTimeOffset()
        if offset is None:
            raise RuntimeError("NT server time synchronization was lost")
        return ntcore._now() + offset

    def start_load(self) -> None:
        for name, (hz, _) in LOAD_STREAMS.items():
            self.load_publishers[name] = self.inst.getDoubleArrayTopic(LOAD_ROOT + name).publish(
                ntcore.PubSubOptions(periodic=1.0 / hz, sendAll=True)
            )
        self.load_stop.clear()
        self.load_thread = threading.Thread(target=self._publish_load, daemon=True)
        self.load_thread.start()

    def _publish_load(self) -> None:
        due = {name: time.monotonic_ns() for name in LOAD_STREAMS}
        while not self.load_stop.is_set():
            now = time.monotonic_ns()
            for name, (hz, width) in LOAD_STREAMS.items():
                if now < due[name]:
                    continue
                self.load_sequences[name] += 1
                local_us = ntcore._now()
                offset = self.inst.getServerTimeOffset()
                if offset is None:
                    raise RuntimeError("NT server time synchronization was lost")
                publish_us = local_us + offset
                self.load_publishers[name].set([
                    float(self.load_sequences[name]), float(publish_us),
                    *([0.125] * (width - 2)),
                ], local_us)
                due[name] += int(1e9 / hz)
                if due[name] <= now:
                    due[name] = now + int(1e9 / hz)
            time.sleep(0.0005)

    def stop_load(self) -> None:
        self.load_stop.set()
        if self.load_thread:
            self.load_thread.join(timeout=1)

    def close(self) -> None:
        self.stop_load()
        try:
            self.gamepad.close()
            for pub in self.load_publishers.values():
                pub.close()
            self.inst.stopClient()
        finally:
            ntcore.NetworkTableInstance.destroy(self.inst)


def expected_command_magnitude(raw_left_y: float) -> float:
    axis = abs(raw_left_y)
    shaped = 0.0 if axis < 0.1 else (axis - 0.1) / 0.9
    return shaped * DRIVE_MAX_MPS


def drain(subscriber) -> list[list[float]]:
    return [list(item.value) for item in subscriber.readQueue()]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("ds", "nt"), required=True,
                        help="simulated Driver Station joystick or NT4 gamepad publisher")
    parser.add_argument("--load", choices=("none", "vision"), default="none",
                        help="optional concurrent synthetic NT4 vision-rate traffic")
    parser.add_argument("--seconds", type=float, default=8.0,
                        help="run duration; 6 to 10 seconds is recommended")
    parser.add_argument("--output", type=Path, default=Path("nt-gamepad-compare.json"))
    args = parser.parse_args()
    if not 6 <= args.seconds <= 30:
        parser.error("--seconds must be between 6 and 30")

    nt = NtSession()
    hal: HalSession | None = None
    result: dict = {
        "experiment": "actual local WPILib robot HAL DS-vs-NT gamepad comparison",
        "mode": args.mode,
        "synthetic_vision_traffic": args.load == "vision",
        "robot_period_ms": 20,
        "nt4_input_publish_period_ms": 5,
        "nt_server": f"{NT_HOST}:{NT_PORT}",
        "hal_websocket": WS_URI,
        "clock": "NT server monotonic timestamp in microseconds, client local time adjusted by getServerTimeOffset()",
        "duration_s": args.seconds,
        "steps": [],
        "input_to_command_ms": [],
        "step_to_command_ms": [],
        "command_trace": [],
        "robot_loop_probe": {"samples": 0, "ds_attached_and_enabled": False},
        "errors": [],
    }
    try:
        nt_bindings = listener_bindings(NT_PORT)
        hal_bindings = listener_bindings(WS_PORT)
        result["listener_bind_addresses"] = {str(NT_PORT): nt_bindings, str(WS_PORT): hal_bindings}
        result["listener_scope_note"] = (
            "The harness connects only to 127.0.0.1. HALSim's bundled server may bind beyond loopback; "
            "verify the owning robot process before running this input harness."
        )
        result["nt_time_sync_offset_us_at_start"] = nt.initial_server_offset_us
        hal = HalSession(nt.server_now_us)
        # Ensure clean neutral input before authorizing teleop.
        hal.set_joystick(0.0, new_data=True, enabled=False)
        if args.load == "vision":
            nt.start_load()

        # Require the actual robot loop diagnostics before enabling. The
        # environment must opt in with FRC_SIM_LATENCY_PROBE=1.
        wait_until(lambda: bool(drain(nt.robot_probe)), 5,
                   f"Robot telemetry {PROBE_TOPIC} missing; launch with FRC_SIM_LATENCY_PROBE=1")

        probe_rows: list[list[float]] = []
        ds_rows: list[list[float]] = []
        nt_rows: list[list[float]] = []
        hal.set_joystick(0.0, new_data=True, enabled=True)
        wait_until(lambda: any(len(row) >= 9 and row[7] >= 0.5 and row[8] >= 0.5
                               for row in drain(nt.robot_probe)), 3,
                   "Robot telemetry did not confirm attached, enabled teleop")
        command_sub = nt.nt_command if args.mode == "nt" else nt.ds_command
        wait_until(lambda: bool(drain(command_sub)), 3,
                   "Robot command telemetry did not appear; confirm the comparison build is running")

        source_period_s = 0.020 if args.mode == "ds" else 0.005
        started_us = nt.server_now_us()
        end_us = started_us + int(args.seconds * 1e6)
        next_source_us = started_us
        next_ds_us = started_us
        # Seed from synchronized time so reconnecting this publisher does not
        # regress the sequence retained by the still-running robot adapter.
        seq = nt.server_now_us() if args.mode == "nt" else 0
        first_sequence = seq
        source_send_times_us: list[int] = []
        step_index = -1
        active_left_y = 0.0
        step_period_us = int(STEP_PERIOD_S * 1e6)
        next_step_us = started_us
        ds_confirmed = False
        while nt.server_now_us() < end_us:
            now_us = nt.server_now_us()
            while now_us >= next_step_us:
                step_index += 1
                active_left_y = STEP_VALUES[step_index % len(STEP_VALUES)]
                event = {"index": step_index, "left_y": active_left_y,
                         "scheduled_nt_us": next_step_us, "source_nt_us": None, "mode": args.mode,
                         "first_sequence": None, "command_nt_us": None}
                result["steps"].append(event)
                next_step_us += step_period_us

            if args.mode == "nt" and now_us >= next_source_us:
                seq += 1
                sent_us = nt.set_gamepad(seq, active_left_y, 0.0, 0.0)
                source_send_times_us.append(sent_us)
                if result["steps"][-1]["first_sequence"] is None:
                    result["steps"][-1]["first_sequence"] = seq
                    result["steps"][-1]["source_nt_us"] = sent_us
                next_source_us += int(source_period_s * 1e6)
                if next_source_us <= now_us:
                    next_source_us = now_us + int(source_period_s * 1e6)
            elif args.mode == "ds" and now_us >= next_ds_us:
                seq += 1
                sent_us = hal.set_joystick(active_left_y, new_data=True, enabled=True)
                source_send_times_us.append(sent_us)
                if result["steps"][-1]["first_sequence"] is None:
                    result["steps"][-1]["first_sequence"] = seq
                    result["steps"][-1]["source_nt_us"] = sent_us
                next_ds_us += 20_000
                if next_ds_us <= now_us:
                    next_ds_us = now_us + 20_000
            else:
                time.sleep(0.0005)

            probes = drain(nt.robot_probe)
            probe_rows.extend(probes)
            ds_confirmed = ds_confirmed or any(len(row) >= 9 and row[7] >= 0.5 and row[8] >= 0.5
                                                for row in probes)
            if args.mode == "nt":
                rows = drain(nt.nt_command)
                nt_rows.extend(rows)
                for row in rows:
                    if len(row) < 7:
                        continue
                    accepted = bool(row[6] >= 0.5)
                    result["command_trace"].append({"sequence": int(row[0]),
                        "source_nt_us": int(row[1]), "command_nt_us": int(row[2]),
                        "command_x_mps": float(row[3]), "command_y_mps": float(row[4]),
                        "command_omega_rad_s": float(row[5]), "accepted": accepted})
                    if not accepted:
                        continue
                    source_us, consume_us = int(row[1]), int(row[2])
                    result["input_to_command_ms"].append((consume_us - source_us) / 1000.0)
                    # Match sequence telemetry to the first robot command sample
                    # after each input step, without using NT subscriber arrival.
                    command_seq = int(row[0])
                    candidates = [e for e in result["steps"]
                                  if e["first_sequence"] is not None and
                                  e["first_sequence"] <= command_seq and e["command_nt_us"] is None and
                                  consume_us >= e["source_nt_us"] and
                                  abs(math.hypot(float(row[3]), float(row[4])) -
                                      expected_command_magnitude(e["left_y"])) < 0.10]
                    if candidates:
                        event = candidates[-1]
                        event["command_nt_us"] = consume_us
                        result["step_to_command_ms"].append((consume_us - event["scheduled_nt_us"]) / 1000.0)
            else:
                rows = drain(nt.ds_command)
                ds_rows.extend(rows)
                for row in rows:
                    if len(row) < 7:
                        continue
                    consume_us = int(row[0])
                    raw_y = float(row[1])
                    command_x = float(row[4])
                    result["command_trace"].append({"command_nt_us": consume_us,
                        "raw_left_y": raw_y, "raw_left_x": float(row[2]),
                        "raw_right_x": float(row[3]), "command_x_mps": command_x,
                        "command_y_mps": float(row[5]), "command_omega_rad_s": float(row[6])})
                    # Select the latest edge with a matching measured command.
                    # The output itself is robot telemetry, timestamped inside
                    # the real teleop command path.
                    for event in reversed(result["steps"]):
                        if (event["command_nt_us"] is not None or event["source_nt_us"] is None or
                                consume_us < event["source_nt_us"]):
                            continue
                        actual_magnitude = math.hypot(command_x, float(row[5]))
                        if abs(raw_y - event["left_y"]) < 0.03 and \
                           abs(actual_magnitude - expected_command_magnitude(event["left_y"])) < 0.10:
                            event["command_nt_us"] = consume_us
                            result["input_to_command_ms"].append((consume_us - event["source_nt_us"]) / 1000.0)
                            result["step_to_command_ms"].append((consume_us - event["scheduled_nt_us"]) / 1000.0)
                            break

        # Capture the final queued robot samples before making the input safe.
        probe_rows.extend(drain(nt.robot_probe))
        if args.mode == "nt":
            nt_rows.extend(drain(nt.nt_command))
        else:
            ds_rows.extend(drain(nt.ds_command))
        result["robot_loop_probe"] = {
            "samples": len(probe_rows),
            "ds_attached_and_enabled": ds_confirmed,
        }
        result["robot_command_samples"] = len(nt_rows if args.mode == "nt" else ds_rows)
        result["hal_websocket_messages_received"] = hal.received_messages if hal else 0
        if hal and hal.reader_error:
            result["errors"].append(f"HAL websocket reader: {hal.reader_error}")
        result["publisher_frames_attempted"] = seq - first_sequence
        requested_rate = 50.0 if args.mode == "ds" else 200.0
        result["source_rate_frames_per_second"] = {"actual": (seq - first_sequence) / args.seconds, "requested": requested_rate}
        send_intervals_ms = [(b - a) / 1000.0 for a, b in zip(source_send_times_us, source_send_times_us[1:])]
        result["input_publish_rate"] = {
            "frames_per_second": (seq - first_sequence) / args.seconds,
            "period_ms_p50": pct(send_intervals_ms, .50),
            "period_ms_p95": pct(send_intervals_ms, .95),
        }
        result["steps_observed_by_robot"] = sum(e["command_nt_us"] is not None for e in result["steps"])
        loop_periods_ms = [(float(b[0]) - float(a[0])) / 1000.0 for a, b in zip(probe_rows, probe_rows[1:])
                           if len(a) >= 2 and len(b) >= 2 and b[0] > a[0]]
        result["robot_loop_period_ms"] = {"samples": len(loop_periods_ms),
            "p50": pct(loop_periods_ms, .50), "p95": pct(loop_periods_ms, .95),
            "p99": pct(loop_periods_ms, .99)}
        result["quality"] = {
            "robot_telemetry_observed": bool(probe_rows),
            "driver_station_enabled_teleop_confirmed": ds_confirmed,
            "robot_command_telemetry_observed": bool(nt_rows if args.mode == "nt" else ds_rows),
            "robot_loop_p50_within_10pct_of_20ms": result["robot_loop_period_ms"]["p50"] is not None and
                abs(result["robot_loop_period_ms"]["p50"] - 20.0) <= 2.0,
            "source_cadence_within_10pct": abs((seq - first_sequence) / args.seconds - requested_rate) <= requested_rate * 0.10,
            "latencies_finite": bool(result["input_to_command_ms"]) and
                                all(math.isfinite(x) for x in result["input_to_command_ms"]),
            "steps_observed": result["steps_observed_by_robot"] >= len(result["steps"]) * 0.9,
            "all_robot_commands_after_first_input_accepted": args.mode != "nt" or
                any(row["sequence"] > 0 and row["accepted"] for row in result["command_trace"]) and
                all(row[6] >= 0.5 for row in nt_rows if len(row) >= 7 and row[0] > first_sequence),
            "no_errors": not result["errors"],
        }
        result["latency_summary_ms"] = {
            "input_to_actual_command": {"samples": len(result["input_to_command_ms"]),
                "p50": pct(result["input_to_command_ms"], .50), "p95": pct(result["input_to_command_ms"], .95),
                "p99": pct(result["input_to_command_ms"], .99),
                "max": max(result["input_to_command_ms"]) if result["input_to_command_ms"] else None},
            "scheduled_step_to_actual_command": {"samples": len(result["step_to_command_ms"]),
                "p50": pct(result["step_to_command_ms"], .50), "p95": pct(result["step_to_command_ms"], .95),
                "p99": pct(result["step_to_command_ms"], .99),
                "max": max(result["step_to_command_ms"]) if result["step_to_command_ms"] else None},
        }
        result["synthetic_load"] = ({"enabled": True, "streams": {
            name: {"target_hz": hz, "payload_width": width, "frames_published": nt.load_sequences[name],
                   "observed_publish_hz": nt.load_sequences[name] / args.seconds}
            for name, (hz, width) in LOAD_STREAMS.items()
        }} if args.load == "vision" else {"enabled": False})
        result["limitations"] = [
            "This exercises the full local desktop robot loop and its command telemetry; it is not physical Driver Station, USB, radio, roboRIO, motor controller, or field evidence.",
            "Synthetic vision load is NT4 traffic only; the robot's simulation vision sources are injected local callbacks, so this does not reproduce camera processing or perception CPU work.",
            "DS input-to-command time uses the harness HAL write timestamp; NT input-to-command time uses the embedded source NT timestamp and robot consumer command timestamp.",
        ]
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + "\n")
        summary = {
            "mode": result["mode"], "load": result["synthetic_load"],
            "input_publish_rate": result["input_publish_rate"],
            "robot_loop_period_ms": result["robot_loop_period_ms"],
            "latency_summary_ms": result["latency_summary_ms"],
            "steps_observed_by_robot": result["steps_observed_by_robot"],
            "quality": result["quality"], "output": str(args.output),
        }
        print(json.dumps(summary, indent=2))
        return 0 if all(result["quality"].values()) else 1
    finally:
        if hal:
            hal.disable()
        if args.mode == "nt":
            try:
                seq += 1
                nt.set_gamepad(seq, 0.0, 0.0, 0.0)
            except Exception:
                pass
        nt.stop_load()
        if hal:
            hal.close()
        nt.close()


if __name__ == "__main__":
    raise SystemExit(main())
