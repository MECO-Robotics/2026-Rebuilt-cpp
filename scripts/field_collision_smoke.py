#!/usr/bin/env python3
"""Check legal field spawns, deferred odometry reset and live wall contact.

Local HAL/NT only. Requires pyntcore, robotpy-wpimath and websockets. Disables on
exit and restores the blue scene spawn. Intended for the field-enabled simulator.
"""
import argparse
import asyncio
import contextlib
import json
import math
from pathlib import Path
import time

import ntcore
from websockets.asyncio.client import connect
from wpimath.geometry import Pose2d, Pose3d


async def exercise():
    client = ntcore.NetworkTableInstance.create()
    client.startClient4('field-collision-check')
    client.setServer('127.0.0.1', 5810)
    options = ntcore.PubSubOptions(periodic=.02, sendAll=True)
    pose = client.getStructTopic('/AdvantageScope/Drive/Pose', Pose2d).subscribe(Pose2d(), options)
    truth = client.getStructTopic('/Simulation/RobotTruth', Pose3d).subscribe(Pose3d(), options)
    count = client.getDoubleTopic('/SmartDashboard/FieldSimulation/ColliderCount').subscribe(0)
    result = {'transport': 'localhost HAL websocket + NT4', 'samples': {}, 'checks': {}}
    async with connect('ws://127.0.0.1:3300/wpilibws', max_size=None) as websocket:
        async def drain():
            async for _ in websocket:
                pass
        reader = asyncio.create_task(drain())
        async def hold(duration, enabled=False, lateral=0., reset=False, red=False, turn=0.):
            end = time.monotonic() + duration
            rows = []
            while time.monotonic() < end:
                buttons = [False]*10
                buttons[6] = reset
                await websocket.send(json.dumps({'type': 'Joystick', 'device': '0', 'data': {
                    '>axes': [-lateral, 0, 0, 0, -turn, 0], '>buttons': buttons, '>povs': [-1]}}))
                await websocket.send(json.dumps({'type': 'DriverStation', 'device': '', 'data': {
                    '>enabled': enabled, '>autonomous': False, '>test': False, '>ds': True,
                    '>station': 'red1' if red else 'blue1', '>new_data': True}}))
                p, t = pose.get(), truth.get()
                rows.append({'time': ntcore._now(), 'pose': [p.X(), p.Y(), p.rotation().radians()],
                             'truth': [t.X(), t.Y(), t.Z(), t.rotation().X(), t.rotation().Y(), t.rotation().Z()]})
                await asyncio.sleep(.02)
            return rows
        async def reset_scene(red):
            await hold(.25, enabled=True, red=red)
            await hold(.15, enabled=True, reset=True, red=red)
            return await hold(.8, red=red)
        try:
            await hold(1.)
            initial = truth.get()
            result['samples']['startup'] = [initial.X(), initial.Y(), initial.Z()]
            result['collider_count'] = count.get()
            result['checks']['startup_clear_blue_spawn'] = math.hypot(initial.X()-3.65, initial.Y()-.639318)<.08 and .25<initial.Z()<.35
            for red in (False, True):
                label = 'red_reset' if red else 'blue_reset'
                rows = await reset_scene(red)
                result['samples'][label] = rows
                last = rows[-1]
                expected = [16.541-3.65, 8.069-.639318, math.pi] if red else [3.65, .639318, 0]
                result['checks'][label+'_matches_truth_and_odometry'] = all(
                    math.hypot(last[key][0]-expected[0], last[key][1]-expected[1])<.08 and
                    abs(math.remainder(last[key][-1]-expected[2], 2*math.pi))<.08 for key in ('pose','truth'))
            await reset_scene(False)
            rotation = await hold(2.5, enabled=True, turn=.35)
            result['samples']['turn_at_spawn'] = rotation
            yaw_change = sum(math.remainder(b['truth'][-1]-a['truth'][-1], 2*math.pi)
                             for a,b in zip(rotation,rotation[1:]))
            result['checks']['can_turn_at_default_spawn'] = yaw_change>1.5 and min(r['truth'][2] for r in rotation)>.20
            await reset_scene(False)
            wall = await hold(2., enabled=True, lateral=-.35)
            result['samples']['south_wall'] = wall
            result['checks']['actual_wall_contact_blocks_motion'] = .35<wall[-1]['truth'][1]<.52 and min(r['truth'][1] for r in wall)>.30
            result['checks']['field_installed'] = count.get()>0
        finally:
            await reset_scene(False)
            await hold(.2)
            reader.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await reader
            client.stopClient()
    result['passed'] = all(result['checks'].values())
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = asyncio.run(exercise())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({'checks': result['checks'], 'passed': result['passed']}, indent=2))
    raise SystemExit(0 if result['passed'] else 1)
