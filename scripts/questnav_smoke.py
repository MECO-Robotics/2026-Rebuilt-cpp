#!/usr/bin/env python3
"""Disabled-only synthetic QuestNav/Limelight fixture for an existing native sim.

Requires the same pyntcore 2026 + websockets environment as sim_smoke.py. Talks
only to localhost NT4 and the desktop HAL websocket; never enables the DS.
The protobuf fixture matches QuestNav 2026-2.2.0 and WPILib geometry3d schemas.
This verifies decoding, mounting conversion, alignment/fallback and diagnostics;
it does not model headset sensor fidelity or establish hardware calibration.
"""
import argparse
import asyncio
import contextlib
import json
import math
from pathlib import Path
import struct
import time

import ntcore
from websockets.asyncio.client import connect


def varint(value):
    result = bytearray()
    while value > 127:
        result.append((value & 127) | 128)
        value >>= 7
    result.append(value)
    return bytes(result)


def scalar(tag, value):
    return varint(tag << 3) + varint(value)


def number(tag, value):
    return varint((tag << 3) | 1) + struct.pack("<d", value)


def message(tag, payload):
    return varint((tag << 3) | 2) + varint(len(payload)) + payload


def frame_payload(count, tracking=True, robot_x=3.0):
    # Java season mount: (-13.5 in, 0, 8.5 in), roll=pi, yaw=pi.
    # For robot heading zero its headset quaternion is (w=0,x=0,y=1,z=0).
    translation = number(1, robot_x - 13.5 * .0254) + number(2, 2.) + number(3, 8.5 * .0254)
    quaternion = b"".join(number(tag, value) for tag, value in enumerate((0., 0., 1., 0.), 1))
    pose = message(1, translation) + message(2, message(1, quaternion))
    # Uptime is intentionally unrelated to FPGA time: NT capture time must win.
    return scalar(1, count) + number(2, 123. + count * .01) + message(3, pose) + scalar(4, int(tracking))


async def exercise(args):
    result = {"scope": "Disabled localhost native simulator; synthetic protobuf and Limelight observations",
              "hardwareValidation": False, "passed": False, "checks": {}, "samples": {}}
    instance = ntcore.NetworkTableInstance.create()
    instance.startClient4("questnav-disabled-fixture")
    instance.setServer("127.0.0.1", args.nt_port)
    dashboard = instance.getTable("SmartDashboard")
    options = ntcore.PubSubOptions(periodic=.01, sendAll=True)
    quest = instance.getRawTopic("/QuestNav/frameData").publish(
        "proto:questnav.protos.data.ProtobufQuestNavFrameData", options)
    device = instance.getRawTopic("/QuestNav/deviceData").publish(
        "proto:questnav.protos.data.ProtobufQuestNavDeviceData", options)
    version = instance.getStringTopic("/QuestNav/version").publish(options)
    poses = instance.getDoubleArrayTopic("/limelight/botpose_wpiblue").publish(options)
    heartbeat = instance.getDoubleTopic("/limelight/hb").publish(options)
    subscriptions = {
        "ready": dashboard.getBooleanTopic("QuestNav/PoseReady").subscribe(False),
        "connected": dashboard.getBooleanTopic("QuestNav/Connected").subscribe(False),
        "aligned": dashboard.getBooleanTopic("QuestNav/FieldAligned").subscribe(False),
        "combined": dashboard.getBooleanTopic("QuestNav/CombinedConnected").subscribe(False),
        "fallback": dashboard.getBooleanTopic("QuestNav/BypassingToAbsolute").subscribe(False),
        "versionMatches": dashboard.getBooleanTopic("QuestNav/VersionMatches").subscribe(False),
        "battery": dashboard.getDoubleTopic("QuestNav/BatteryPercent").subscribe(-1),
        "rejected": dashboard.getDoubleTopic("QuestNav/RejectedFrames").subscribe(-1),
        "pose": dashboard.getDoubleArrayTopic("Drive/Pose").subscribe([]),
        "speeds": dashboard.getDoubleArrayTopic("Drive/Speeds").subscribe([]),
    }
    sequence = 0
    async with connect(f"ws://127.0.0.1:{args.hal_port}/wpilibws", max_size=None) as websocket:
        async def drain():
            async for _ in websocket:
                pass
        reader = asyncio.create_task(drain())

        async def disabled():
            await websocket.send(json.dumps({"type": "DriverStation", "device": "", "data": {
                ">enabled": False, ">autonomous": False, ">test": False,
                ">estop": False, ">fms": False, ">ds": True, ">new_data": True}}))

        def snapshot():
            return {key: list(topic.get()) if key in ("pose", "speeds") else topic.get()
                    for key, topic in subscriptions.items()}

        async def phase(name, seconds=.8, absolute=False, inertial=False, tracking=True, robot_x=3., malformed=False):
            nonlocal sequence
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                await disabled()
                sequence += 1
                if absolute:
                    heartbeat.set(float(sequence))
                    # Limelight MT1: xyz, roll/pitch/yaw degrees, latency ms,
                    # tag count, span, mean distance, mean area.
                    # Vary unused area so NT dedup cannot suppress fresh captures.
                    poses.set([3., 2., 0., 0., 0., 0., 0., 2., 0., 1., 1. + sequence * 1e-6])
                if inertial:
                    quest.set(b"\x1a\xff" if malformed else frame_payload(sequence, tracking, robot_x))
                instance.flush()
                await asyncio.sleep(.02)
            sample = snapshot()
            result["samples"][name] = sample
            return sample

        def require(name, passed):
            result["checks"][name] = bool(passed)
            if not passed:
                raise AssertionError(f"QuestNav fixture check failed: {name}; samples={result['samples']}")

        try:
            deadline = time.monotonic() + 10
            while not instance.isConnected() and time.monotonic() < deadline:
                await disabled()
                await asyncio.sleep(.02)
            require("localhost_nt_connected", instance.isConnected())
            version.set("2026-2.2.0")
            device.set(scalar(1, 3) + scalar(3, 85))
            sample = await phase("absolute_fallback", absolute=True)
            require("absolute_fallback_without_quest", sample["fallback"] and not sample["ready"] and sample["combined"])
            sample = await phase("aligned_tracking", absolute=True, inertial=True)
            require("field_aligned_tracking", sample["ready"] and sample["aligned"] and sample["connected"] and not sample["fallback"])
            require("pinned_version_and_battery_diagnostics", sample["versionMatches"] and sample["battery"] == 85)
            # Drain the last absolute capture before imposing a synthetic step;
            # no old absolute frame should align away that subsequent motion.
            await phase("absolute_capture_drained", seconds=.1, inertial=True)
            sample = await phase("relative_headset_motion", inertial=True, robot_x=3.4)
            require("mounted_headset_motion_fused", len(sample["pose"]) == 3 and
                    abs(sample["pose"][0]-3.4) < .35 and abs(sample["pose"][1]-2.) < .35)
            sample = await phase("tracking_lost", absolute=True, inertial=True, tracking=False)
            require("tracking_loss_uses_absolute_fallback", not sample["ready"] and not sample["connected"] and sample["fallback"])
            await phase("tracking_recovered", absolute=True, inertial=True)
            sample = await phase("stale_frames", seconds=.6)
            require("stale_tracking_not_ready", not sample["ready"] and not sample["connected"] and sample["fallback"] and not sample["combined"])
            before = sample["rejected"]
            sample = await phase("field_bounds_rejection", inertial=True, robot_x=30.)
            require("out_of_field_not_ready", not sample["ready"] and sample["rejected"] > before)
            version.set("wrong-version-fixture")
            sample = await phase("version_mismatch", seconds=.4)
            require("version_mismatch_visible", not sample["versionMatches"])
            speed = sample["speeds"]
            require("disabled_stationary", len(speed) == 3 and all(math.isfinite(v) and abs(v) < .05 for v in speed))
            # Driver reset is intentionally not exercised: the source binding is
            # teleop-enabled only. This fixture keeps the DS disabled throughout.
            result["resetObserver"] = "Not exercised; driver reset requires enabled teleop"
            result["passed"] = all(result["checks"].values())
        finally:
            await disabled()
            reader.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await reader
            instance.stopClient()
            ntcore.NetworkTableInstance.destroy(instance)
            path = Path(args.output)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(result, indent=2) + "\n")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--nt-port", type=int, default=5810)
    parser.add_argument("--hal-port", type=int, default=3300)
    parser.add_argument("--output", default="artifacts/questnav-smoke.json")
    args = parser.parse_args()
    result = asyncio.run(exercise(args))
    print(json.dumps(result["checks"], indent=2))


if __name__ == "__main__":
    main()
