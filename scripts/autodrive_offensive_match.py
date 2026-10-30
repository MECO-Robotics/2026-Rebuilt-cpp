#!/usr/bin/env python3
"""Observe the onboard offensive command during a local simulated practice match.

The only driver input is holding the command's enable button during teleop.
The script never chooses goals, intake, aim, or feed actions.
"""
import argparse
import asyncio
import contextlib
import hashlib
import json
import math
import time
from pathlib import Path

import ntcore
from wpimath.geometry import Pose3d
from wpimath.kinematics import SwerveModuleState
from websockets.asyncio.client import connect
from sim_evidence import running_sim_binary


async def run(args):
    binary = running_sim_binary(args.port, args.hal_port)
    nt = ntcore.NetworkTableInstance.create()
    nt.setServer('127.0.0.1', args.port)
    nt.startClient4('offensive-match-observer')
    opts = ntcore.PubSubOptions(periodic=.02)
    def number(name):
        return nt.getDoubleTopic(name).subscribe(0, opts)
    def boolean(name):
        return nt.getBooleanTopic(name).subscribe(False, opts)
    def string(name):
        return nt.getStringTopic(name).subscribe('', opts)
    truth = nt.getStructTopic('/Simulation/RobotTruth', Pose3d).subscribe(Pose3d(), opts)
    modules = nt.getStructArrayTopic('/AdvantageScope/Drive/ModuleTargets', SwerveModuleState).subscribe([], opts)
    stored = number('/SmartDashboard/FieldSimulation/StoredFuelCount')
    launched = number('/SmartDashboard/FieldSimulation/LaunchedFuelCount')
    scored = number('/SmartDashboard/FieldSimulation/SuccessfulScoreCount')
    clusters = number('/SmartDashboard/AutoDrive/ClusterCount')
    front = number('/SmartDashboard/AutoDrive/SimFrontDetections')
    rear = number('/SmartDashboard/AutoDrive/SimRearDetections')
    offense_phase = string('/SmartDashboard/AutoDrive/OffensePhase')
    estimated_fuel = number('/SmartDashboard/AutoDrive/EstimatedFuel')
    phase = string('/Match/Phase')
    remaining = number('/Match/Time')
    hub_known = boolean('/SmartDashboard/AutoDrive/HubKnown')
    hub_active = boolean('/SmartDashboard/AutoDrive/HubActive')
    feeding = boolean('/SmartDashboard/Shooter/Feeding')
    state = string('/Sim/Practice/State')
    running = boolean('/Sim/Practice/Running')
    command = nt.getStringTopic('/Sim/Practice/Command').publish(ntcore.PubSubOptions(keepDuplicates=True))
    chooser = nt.getStringTopic('/SmartDashboard/Auto Choices/selected').publish()
    result = {'scenario': 'onboard-offensive-practice-match', 'alliance': args.alliance,
              'auto_result': args.auto_result, 'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
              'trace': [], 'checks': {}, 'error': None, 'passed': False}
    async with connect(f'ws://127.0.0.1:{args.hal_port}/wpilibws') as ws:
        async def drain():
            async for _ in ws:
                pass
        reader = asyncio.create_task(drain())
        async def joystick(enabled):
            buttons = [False] * 10
            buttons[4] = enabled
            await ws.send(json.dumps({'type': 'Joystick', 'device': '0', 'data': {
                '>axes': [0, 0, 0, 0, 0, 0], '>buttons': buttons, '>povs': [-1]}}))
        async def disable():
            await ws.send(json.dumps({'type': 'DriverStation', 'device': '', 'data': {
                '>enabled': False, '>autonomous': False, '>test': False, '>ds': True,
                '>fms': False, '>estop': False, '>new_data': True}}))
        try:
            await asyncio.sleep(1)
            if not nt.isConnected() or not truth.getAtomic().time:
                raise RuntimeError('no robot telemetry')
            await disable()
            await joystick(False)
            await asyncio.sleep(.3)
            nt.getEntry('/Sim/Practice/Config/AllianceStation').setString(args.alliance.title() + '1')
            nt.getEntry('/Sim/Practice/Config/AutoResult').setString(args.auto_result)
            chooser.set(args.auto_choice)
            nt.getEntry('/SmartDashboard/AutoDrive/OffenseEnabled').setBoolean(True)
            nt.flush()
            await asyncio.sleep(.3)
            command.set('start')
            nt.flush()
            start = time.monotonic()
            prev_state = ''
            started = False
            peak_clusters = peak_detections = peak_stored = 0
            saw_feed = False
            last_sample = 0
            score_by_phase = {}
            teleop_start_score = None
            teleop_feed = False
            teleop_collected = 0
            previous_stored = None
            while time.monotonic() - start < 170:
                now = time.monotonic()
                current = state.get()
                if current != prev_state:
                    print(f'{now-start:.1f}s practice={current}', flush=True)
                    prev_state = current
                started |= current == 'autonomous'
                if current == 'teleop' and teleop_start_score is None:
                    teleop_start_score = scored.get()
                if current == 'teleop':
                    if previous_stored is not None:
                        teleop_collected += max(0, stored.get() - previous_stored)
                    previous_stored = stored.get()
                if current == 'aborted':
                    raise RuntimeError('practice aborted: ' + nt.getEntry('/Sim/Practice/Reason').getString(''))
                if started and current == 'complete':
                    break
                await joystick(current == 'teleop')
                peak_clusters = max(peak_clusters, clusters.get())
                peak_detections = max(peak_detections, front.get() + rear.get())
                peak_stored = max(peak_stored, stored.get())
                saw_feed |= feeding.get()
                teleop_feed |= current == 'teleop' and feeding.get()
                score_by_phase[current] = scored.get()
                if now - last_sample >= .25:
                    p = truth.get()
                    result['trace'].append({'elapsed': round(now-start, 2), 'state': current,
                        'phase': phase.get(), 'offense_phase': offense_phase.get(), 'time': remaining.get(),
                        'pose': [p.X(), p.Y(), p.rotation().Z()], 'height_m': p.Z(),
                        'roll_rad': p.rotation().X(), 'pitch_rad': p.rotation().Y(),
                        'stored': stored.get(), 'estimated_fuel': estimated_fuel.get(),
                        'launched': launched.get(), 'scored': scored.get(), 'feeding': feeding.get(),
                        'clusters': clusters.get(), 'detections': [front.get(), rear.get()],
                        'hub_known': hub_known.get(), 'hub_active': hub_active.get()})
                    last_sample = now
                await asyncio.sleep(.02)
            await joystick(False)
            await asyncio.sleep(.5)
            result.update({'peak_clusters': peak_clusters, 'peak_detections': peak_detections,
                           'peak_stored': peak_stored, 'final_launched': launched.get(),
                           'final_scored': scored.get(), 'teleop_collected': teleop_collected, 'teleop_score_delta': (scored.get() - teleop_start_score if teleop_start_score is not None else None),
                           'score_by_phase': score_by_phase})
            result['checks'] = {'match_complete': state.get() == 'complete' and not running.get(),
                'simulated_camera_observed_fuel': peak_detections > 0 and peak_clusters > 0,
                'collected_fuel': peak_stored > 0, 'feeding_seen': saw_feed,
                'launched_fuel': launched.get() > 0, 'scored_fuel': scored.get() > 0,
                'teleop_collected': teleop_collected > 0,
                'teleop_scoring': teleop_start_score is not None and scored.get() > teleop_start_score and teleop_feed,
                'stopped_on_complete': len(modules.get()) == 4 and max(abs(m.speed) for m in modules.get()) < .05}
            result['passed'] = all(result['checks'].values())
        except Exception as exc:
            result['error'] = str(exc)
            raise
        finally:
            with contextlib.suppress(Exception):
                command.set('abort')
                nt.flush()
                await joystick(False)
                await asyncio.sleep(.1)
                await disable()
            reader.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await reader
            nt.stopClient()
            ntcore.NetworkTableInstance.destroy(nt)
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({k: v for k, v in result.items() if k != 'trace'}, indent=2))
    return result['passed']


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=5811)
    parser.add_argument('--hal-port', type=int, default=3301)
    parser.add_argument('--alliance', choices=['blue', 'red'], default='blue')
    parser.add_argument('--auto-result', choices=['R', 'B'], default='R')
    parser.add_argument('--auto-choice', default='AutoDrive offensive')
    parser.add_argument('--output', type=Path, required=True)
    raise SystemExit(0 if asyncio.run(run(parser.parse_args())) else 1)
