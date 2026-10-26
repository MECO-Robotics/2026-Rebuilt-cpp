#!/usr/bin/env python3
"""Exercise an already-running desktop frcUserProgram through HAL simulation only.

Requires pyntcore, robotpy-wpimath (2026) and websockets. The simulator must load the official
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
import struct
import sys
import time
import ntcore
from websockets.asyncio.client import connect
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState


async def exercise(args):
    evidence = {"transport": "localhost HAL websocket + NT4",
                "scope": "Native simulator joystick response, module azimuth travel, 3D floor support, direction, speed tracking, pose change, and disable behavior; excludes physical calibration",
                "samples": {}, "checks": {}}
    instance = ntcore.NetworkTableInstance.create()
    instance.startClient4("base-cpp-sim-smoke")
    instance.setServer("127.0.0.1", args.nt_port)
    speed = instance.getStructTopic("/AdvantageScope/Drive/ChassisSpeeds", ChassisSpeeds).subscribe(ChassisSpeeds(), ntcore.PubSubOptions(periodic=.02))
    pose = instance.getStructTopic("/AdvantageScope/Drive/Pose", Pose2d).subscribe(Pose2d(), ntcore.PubSubOptions(periodic=.02))
    module_states = instance.getStructArrayTopic("/AdvantageScope/Drive/ModuleStates", SwerveModuleState).subscribe([], ntcore.PubSubOptions(periodic=.02))
    module_targets = instance.getStructArrayTopic("/AdvantageScope/Drive/ModuleTargets", SwerveModuleState).subscribe([], ntcore.PubSubOptions(periodic=.02))
    grounded = instance.getDoubleTopic("/SmartDashboard/FieldSimulation/GroundedBodyHeight").subscribe(.30)
    truth = instance.getRawTopic("/Simulation/PhysicsBodyTruth").subscribe("struct:Pose3d", b"", ntcore.PubSubOptions(periodic=.02))
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

        async def control(enabled, forward=0., lateral=0., turn=0., reset=False):
            buttons=[False]*10
            if reset: buttons[args.reset_button-1]=True
            await websocket.send(json.dumps({"type": "Joystick", "device": "0", "data": {
                ">axes": [-lateral, -forward, 0., 0., -turn, 0.], ">buttons": buttons, ">povs": [-1]}}))
            await websocket.send(json.dumps({"type": "DriverStation", "device": "", "data": {
                ">enabled": enabled, ">autonomous": False, ">test": False,
                ">estop": False, ">fms": False, ">ds": True, ">new_data": True}}))

        async def phase(name, enabled, forward=0., lateral=0., turn=0., duration=2.0):
            readings, heights, module_state_samples, module_target_samples = [], [], 0, 0
            truth_samples = []
            field_readings = []
            module_angle_errors = [[] for _ in range(4)]
            previous_angles, target_previous_angles = [None]*4, [None]*4
            module_travel, target_travel = [0.]*4, [0.]*4
            raw_target_previous, raw_target_travel = [None]*4, [0.]*4
            start = time.monotonic()
            while time.monotonic()-start < duration:
                await control(enabled, forward, lateral, turn)
                measured = speed.get()
                reading = [measured.vx, measured.vy, measured.omega]
                if len(reading) == 3 and all(math.isfinite(v) for v in reading):
                    readings.append(reading)
                    heading = pose.get().rotation().radians()
                    field_readings.append([measured.vx*math.cos(heading)-measured.vy*math.sin(heading),
                                           measured.vx*math.sin(heading)+measured.vy*math.cos(heading),
                                           measured.omega])
                truth_atomic = truth.getAtomic()
                raw_truth = truth_atomic.value
                if len(raw_truth) == 56:
                    coordinates = struct.unpack("<7d", raw_truth)
                    if all(math.isfinite(v) for v in coordinates[:3]):
                        heights.append(coordinates[2])
                        if truth_atomic.time and (not truth_samples or truth_atomic.time != truth_samples[-1][0]):
                            w, qx, qy, qz = coordinates[3:]
                            yaw = math.atan2(2*(w*qz+qx*qy), 1-2*(qy*qy+qz*qz))
                            truth_samples.append((truth_atomic.time, coordinates[0], coordinates[1], yaw))
                states = module_states.get()
                if len(states) == 4:
                    module_state_samples += 1
                    targets = module_targets.get()
                    for index, state in enumerate(states):
                        angle = state.angle.radians()
                        if math.isfinite(angle):
                            previous = previous_angles[index]
                            if previous is not None:
                                delta = math.atan2(math.sin(angle-previous), math.cos(angle-previous))
                                module_travel[index] += abs(delta)
                            previous_angles[index] = angle
                        if len(targets) == 4:
                            error = math.atan2(
                                math.sin(targets[index].angle.radians() - angle),
                                math.cos(targets[index].angle.radians() - angle),
                            )
                            if math.isfinite(error):
                                module_angle_errors[index].append(abs(error))
                targets = module_targets.get()
                if len(targets) == 4:
                    module_target_samples += 1
                    for index, target in enumerate(targets):
                        raw_angle = target.angle.radians()
                        previous_raw = raw_target_previous[index]
                        if previous_raw is not None:
                            raw_target_travel[index] += abs(math.remainder(raw_angle-previous_raw, 2*math.pi))
                        raw_target_previous[index] = raw_angle
                        # Optimization may flip azimuth by pi and reverse wheel
                        # speed without changing the requested rolling direction.
                        # Measure that direction, not its incidental representation.
                        if abs(target.speed) < .01:
                            target_previous_angles[index] = None
                            continue
                        angle = target.angle.radians() + (math.pi if target.speed < 0 else 0.)
                        if math.isfinite(angle):
                            previous = target_previous_angles[index]
                            if previous is not None:
                                delta = math.atan2(math.sin(angle-previous), math.cos(angle-previous))
                                target_travel[index] += abs(delta)
                            target_previous_angles[index] = angle
                await asyncio.sleep(.02)
            tail = readings[-15:]
            if not tail:
                raise RuntimeError(f"No Drive/Speeds NT telemetry during {name}")
            median = [statistics.median(r[i] for r in tail) for i in range(3)]
            field_median = [statistics.median(r[i] for r in field_readings[-15:]) for i in range(3)]
            measured_pose = pose.get()
            physical_speeds = [math.hypot(b[1]-a[1], b[2]-a[2])/((b[0]-a[0])/1e6)
                               for a,b in zip(truth_samples,truth_samples[1:]) if b[0]>a[0]]
            evidence["samples"][name] = {
                "speeds": median,
                "fieldSpeeds": field_median,
                "pose": [measured_pose.X(), measured_pose.Y(), measured_pose.rotation().radians()],
                "count": len(readings),
                "truthZRange": [min(heights), max(heights)] if heights else None,
                "peakSpeed": max(math.hypot(row[0], row[1]) for row in readings),
                "peakPhysicalSpeed": max(physical_speeds) if physical_speeds else None,
                "peakAngularRate": max(abs(row[2]) for row in readings),
                "truthYawChange": sum(math.remainder(b[3]-a[3], 2*math.pi)
                                      for a,b in zip(truth_samples,truth_samples[1:])),
                "moduleStateSamples": module_state_samples,
                "moduleAzimuthTravel": module_travel,
                "moduleTargetSamples": module_target_samples,
                "moduleTargetAzimuthTravel": target_travel,
                "moduleRawTargetAzimuthTravel": raw_target_travel,
                "moduleAzimuthFinalError": [
                    statistics.median(errors[-10:]) if errors else None for errors in module_angle_errors
                ],
            }
            return field_median
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
            small=await phase("small_forward",True,forward=args.small_input,duration=1.5)
            await phase("small_input_release",False,duration=.5)
            forward=await phase("forward",True,forward=.5)
            reverse=await phase("reverse",True,forward=-.5)
            turn=await phase("turn",True,turn=.5,duration=2.5)
            await phase("turn_release",False,duration=.5)
            small_turn=await phase("small_turn",True,turn=args.small_input,duration=1.5)
            await phase("small_turn_release",False,duration=.5)
            await phase("medium_diagonal",True,forward=.4,lateral=.4,duration=1.2)
            await phase("medium_lateral",True,lateral=.4,duration=1.2)
            await phase("medium_turn",True,turn=.4,duration=1.2)
            stopped=await phase("disabled_final",False,duration=2.)
            evidence["checks"]={
                "initial_disabled_stationary": max(abs(v) for v in initial)<.1,
                "small_input_speed_tracks_deadband": abs(small[0]-args.max_speed*max(0.,(args.small_input-.1)/.9))<max(.12,args.max_speed*max(0.,(args.small_input-.1)/.9)*.45),
                # Wheel encoders include tire slip and steering/drive coupling during
                # reorientation. Bound actual chassis motion using Jolt truth.
                "small_input_peak_speed_bounded": evidence["samples"]["small_forward"]["peakPhysicalSpeed"] is not None and evidence["samples"]["small_forward"]["peakPhysicalSpeed"]<max(.35,args.max_speed*max(0.,(args.small_input-.1)/.9)*1.5),
                "small_input_truth_stays_above_floor": bool(evidence["samples"]["small_forward"]["truthZRange"] and min(evidence["samples"]["small_forward"]["truthZRange"])>grounded.get()-.01 and max(evidence["samples"]["small_forward"]["truthZRange"])<grounded.get()+.10),
                "small_turn_rate_tracks_deadband": abs(small_turn[2]-args.max_angular_rate*max(0.,(args.small_input-.1)/.9))<max(.15,args.max_angular_rate*max(0.,(args.small_input-.1)/.9)*.50),
                "small_turn_rate_bounded": evidence["samples"]["small_turn"]["peakAngularRate"]<max(.65,args.max_angular_rate*max(0.,(args.small_input-.1)/.9)*1.5),
                "forward_velocity": forward[0]>.2,
                "forward_speed_tracking": abs(forward[0]-args.max_speed*4/9)<args.max_speed*4/9*.10,
                "reverse_speed_tracking": abs(reverse[0]+args.max_speed*4/9)<args.max_speed*4/9*.10,
                "turn_truth_rotates": evidence["samples"]["turn"]["truthYawChange"] > args.max_angular_rate*4/9*2.5*.5,
                "turn_speed_tracking": abs(turn[2]-args.max_angular_rate*4/9)<args.max_angular_rate*4/9*.10,
                "joystick_module_azimuth_bounded": all(
                    evidence["samples"][name]["moduleStateSamples"] > 0
                    and evidence["samples"][name]["moduleTargetSamples"] > 0
                    # A native optimizer can retarget the opposite azimuth while
                    # reversing wheel speed. Permit that commanded travel, but
                    # reject a full physical revolution or >0.75 rad extra hunting.
                    and all(travel < min(2*math.pi, max(math.pi, target_travel+.75))
                            for travel,target_travel in zip(
                                evidence["samples"][name]["moduleAzimuthTravel"],
                                evidence["samples"][name]["moduleRawTargetAzimuthTravel"]))
                    and all(travel <= math.pi+1e-6 for travel in evidence["samples"][name]["moduleTargetAzimuthTravel"])
                    for name in ("small_forward", "small_turn", "medium_diagonal", "medium_lateral", "medium_turn")
                ),
                "joystick_module_azimuth_tracks_target": all(
                    all(error is not None and error < .25 for error in evidence["samples"][name]["moduleAzimuthFinalError"])
                    for name in ("small_forward", "small_turn", "medium_diagonal", "medium_lateral", "medium_turn")
                ),
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
    parser.add_argument("--small-input",type=float,default=.15)
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
