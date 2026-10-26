#!/usr/bin/env python3
"""Check hard-mounted support, legal spawns, turning and field contacts.

Local HAL/NT only. Requires pyntcore, robotpy-wpimath and websockets. Disables on
exit and restores the blue scene spawn. Intended for the field-enabled simulator.
"""
import argparse
import asyncio
import contextlib
import json
import hashlib
from sim_evidence import running_sim_binary
import math
from pathlib import Path
import time

import ntcore
from websockets.asyncio.client import connect
from wpimath.geometry import Pose2d, Pose3d


async def exercise(args):
    binary=running_sim_binary(nt_port=args.nt_port)
    client = ntcore.NetworkTableInstance.create()
    client.startClient4('field-collision-check')
    client.setServer('127.0.0.1', args.nt_port)
    options = ntcore.PubSubOptions(periodic=.02, sendAll=True)
    pose = client.getStructTopic('/AdvantageScope/Drive/Pose', Pose2d).subscribe(Pose2d(), options)
    truth = client.getStructTopic('/Simulation/PhysicsBodyTruth', Pose3d).subscribe(Pose3d(), options)
    model = client.getStructTopic('/Simulation/RobotTruth', Pose3d).subscribe(Pose3d(), options)
    grounded = client.getDoubleTopic('/SmartDashboard/FieldSimulation/GroundedBodyHeight').subscribe(.30)
    count = client.getDoubleTopic('/SmartDashboard/FieldSimulation/ColliderCount').subscribe(0)
    result = {'transport': 'localhost HAL websocket + NT4', 'running_binary': str(binary),
              'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
              'samples': {}, 'checks': {}}
    async with connect('ws://127.0.0.1:3300/wpilibws', max_size=None) as websocket:
        async def drain():
            async for _ in websocket:
                pass
        reader = asyncio.create_task(drain())
        async def hold(duration, enabled=False, lateral=0., reset=False, red=False, turn=0., forward=0.):
            end = time.monotonic() + duration
            rows = []
            while time.monotonic() < end:
                buttons = [False]*10
                buttons[6] = reset
                await websocket.send(json.dumps({'type': 'Joystick', 'device': '0', 'data': {
                    '>axes': [-lateral, -forward, 0, 0, -turn, 0], '>buttons': buttons, '>povs': [-1]}}))
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
            result['samples']['startup_model_pose'] = [model.get().X(),model.get().Y(),model.get().Z()]
            result['checks']['grounded_model_height_is_zero'] = model.getAtomic().time>0 and abs(model.get().Z())<.003 and math.hypot(model.get().X()-initial.X(),model.get().Y()-initial.Y())<.08
            result['checks']['hard_mounted_body_has_no_static_sag'] = abs(initial.Z()-grounded.get())<.001
            result['collider_count'] = count.get()
            result['grounded_body_height_m'] = grounded.get()
            result['checks']['startup_clear_blue_spawn'] = math.hypot(initial.X()-3.65, initial.Y()-.639318)<.08 and abs(initial.Z()-grounded.get())<.01
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
            result['checks']['can_turn_at_default_spawn'] = yaw_change>1.5 and min(r['truth'][2] for r in rotation)>grounded.get()-.01
            await reset_scene(False)
            def bumper_south_clearance(row):
                # Project the actual shell's outer corners onto the wall normal.
                _, y, _, roll, pitch, yaw = row['truth']
                ryx=math.sin(yaw)*math.cos(pitch)
                ryy=math.sin(yaw)*math.sin(pitch)*math.sin(roll)+math.cos(yaw)*math.cos(roll)
                ryz=math.sin(yaw)*math.sin(pitch)*math.cos(roll)-math.cos(yaw)*math.sin(roll)
                return y-.43*(abs(ryx)+abs(ryy))-.06985*abs(ryz)
            # Contact acceptance waits for a bounded physical approach instead
            # of assuming an unloaded 90-degree azimuth step finishes in 2 s.
            wall=[]
            approach=time.monotonic()
            while time.monotonic()-approach<8.:
                wall.extend(await hold(.1, enabled=True, lateral=-.35))
                if bumper_south_clearance(wall[-1])<.025:
                    break
            wall.extend(await hold(.5, enabled=True, lateral=-.35))
            result['samples']['south_wall'] = wall
            result['wall_approach_s']=time.monotonic()-approach
            clearances=[bumper_south_clearance(row) for row in wall]
            result['wall_bumper_clearance_m']={'final':clearances[-1], 'minimum':min(clearances)}
            result['checks']['actual_wall_contact_blocks_motion'] = (
                -.01<clearances[-1]<.025 and min(clearances)>-.01)
            result['checks']['field_installed'] = count.get()>0
            await reset_scene(False)
            # Approach the blue lower bump through clear alliance-zone space.
            def stick(velocity):
                return 0. if abs(velocity)<.03 else math.copysign(.1+.9*min(.7,abs(velocity))/5.24,velocity)
            deadline=time.monotonic()+8
            while time.monotonic()<deadline:
                current=truth.get()
                dx,dy=2.8-current.X(),2.5106376-current.Y()
                if math.hypot(dx,dy)<.04:
                    break
                await hold(.04,enabled=True,forward=stick(1.5*dx),lateral=stick(1.5*dy))
            await hold(.4,enabled=True)
            bump=await hold(8.,enabled=True,forward=.18)
            result['samples']['bumper_bump_approach']=bump
            # At a 0.25-inch floor gap, the bumper meets the bump's leading
            # plate before the wheels. The physical shell must block penetration.
            result['checks']['low_bumper_contacts_bump_before_wheels_climb'] = (
                3.55<bump[-1]['truth'][0]<3.72 and max(r['truth'][0] for r in bump)<3.75 and
                abs(bump[-1]['truth'][2]-grounded.get())<.01 and
                max(abs(bump[-1]['truth'][i]) for i in (3,4))<.1)
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
    parser.add_argument('--nt-port', type=int, default=5810)
    args = parser.parse_args()
    result = asyncio.run(exercise(args))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({'checks': result['checks'], 'passed': result['passed']}, indent=2))
    raise SystemExit(0 if result['passed'] else 1)
