#!/usr/bin/env python3
"""Exercise an already-running desktop frcUserProgram through HAL simulation only.

Requires pyntcore (2026) and websockets. The simulator must load the official
halsim_ws_server extension (localhost:3300) and publish its normal NT server.
No roboRIO address or physical DS transport is used.
"""
import argparse
import asyncio
import contextlib
import json
import math
from pathlib import Path
import statistics
import sys
import time
import ntcore
from websockets.asyncio.client import connect


async def exercise(args):
    evidence = {"transport": "localhost HAL websocket + NT4",
                "scope": "Native simulator direction, steady speed tracking, pose change, and disable behavior; excludes physical calibration",
                "samples": {}, "checks": {}}
    instance = ntcore.NetworkTableInstance.create()
    instance.startClient4("base-cpp-sim-smoke")
    instance.setServer("127.0.0.1", args.nt_port)
    table = instance.getTable("SmartDashboard")
    speed = table.getDoubleArrayTopic("Drive/Speeds").subscribe([])
    pose = table.getDoubleArrayTopic("Drive/Pose").subscribe([])
    async with connect(f"ws://127.0.0.1:{args.hal_port}/wpilibws", max_size=None) as websocket:
        async def drain():
            async for packet in websocket:
                # Consume simulation output so the socket cannot stall the robot.
                try:
                    data = json.loads(packet)
                    if data.get("type") == "DriverStation":
                        evidence["driverStationSchema"] = sorted(data.get("data", {}))
                except (ValueError, AttributeError):
                    pass
        reader = asyncio.create_task(drain())

        async def control(enabled, forward=0., turn=0., reset=False):
            buttons=[False]*10
            if reset: buttons[args.reset_button-1]=True
            await websocket.send(json.dumps({"type": "Joystick", "device": "0", "data": {
                ">axes": [0., -forward, 0., 0., -turn, 0.], ">buttons": buttons, ">povs": [-1]}}))
            await websocket.send(json.dumps({"type": "DriverStation", "device": "", "data": {
                ">enabled": enabled, ">autonomous": False, ">test": False,
                ">estop": False, ">fms": False, ">ds": True, ">new_data": True}}))

        async def phase(name, enabled, forward=0., turn=0., duration=2.0):
            readings = []
            start = time.monotonic()
            while time.monotonic()-start < duration:
                await control(enabled, forward, turn)
                reading = list(speed.get())
                if len(reading) == 3 and all(math.isfinite(v) for v in reading):
                    readings.append(reading)
                await asyncio.sleep(.02)
            tail = readings[-15:]
            if not tail:
                raise RuntimeError(f"No Drive/Speeds NT telemetry during {name}")
            median = [statistics.median(r[i] for r in tail) for i in range(3)]
            evidence["samples"][name] = {"speeds": median, "pose": list(pose.get()), "count": len(readings)}
            return median
        try:
            deadline=time.monotonic()+10
            while not instance.isConnected() and time.monotonic()<deadline:
                await control(False)
                await asyncio.sleep(.02)
            if not instance.isConnected():
                raise RuntimeError("Local simulator NetworkTables server did not connect")
            # Hold the configured reset binding across several robot loops.
            # Base B supports disabled reset; season Start requires --reset-enabled.
            for _ in range(6):
                await control(args.reset_enabled, reset=True)
                await asyncio.sleep(.02)
            await control(False)
            initial=await phase("disabled_initial",False,duration=1.)
            forward=await phase("forward",True,forward=.5)
            reverse=await phase("reverse",True,forward=-.5)
            turn=await phase("turn",True,turn=.5)
            stopped=await phase("disabled_final",False,duration=2.)
            evidence["checks"]={
                "initial_disabled_stationary": max(abs(v) for v in initial)<.1,
                "forward_velocity": forward[0]>.2,
                "forward_speed_tracking": abs(forward[0]-args.max_speed*4/9)<args.max_speed*4/9*.10,
                "reverse_speed_tracking": abs(reverse[0]+args.max_speed*4/9)<args.max_speed*4/9*.10,
                "turn_speed_tracking": abs(turn[2]-args.max_angular_rate*4/9)<args.max_angular_rate*4/9*.10,
                "pose_changes_on_drive": evidence["samples"]["forward"]["pose"][0]-evidence["samples"]["disabled_initial"]["pose"][0]>.3,
                "reverse_velocity": reverse[0]<-.2,
                "turn_velocity": turn[2]>.2,
                "final_disabled_stationary": max(abs(v) for v in stopped)<.1,
            }
            evidence["passed"]=all(evidence["checks"].values())
        finally:
            await control(False)
            reader.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await reader
    instance.stopClient()
    return evidence


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hal-port",type=int,default=3300)
    parser.add_argument("--nt-port",type=int,default=5810)
    parser.add_argument("--output",type=Path)
    parser.add_argument("--reset-button",type=int,choices=range(1,11),default=2)
    parser.add_argument("--reset-enabled",action="store_true")
    parser.add_argument("--max-speed",type=float,default=4.572)
    parser.add_argument("--max-angular-rate",type=float,default=7.)
    args=parser.parse_args()
    try:
        result=asyncio.run(exercise(args))
    except Exception as exception:
        result={"passed":False,"error":str(exception)}
    rendered=json.dumps(result,indent=2)
    print(rendered)
    if args.output:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(rendered+"\n")
    return 0 if result.get("passed") else 1


if __name__=="__main__":
    sys.exit(main())
