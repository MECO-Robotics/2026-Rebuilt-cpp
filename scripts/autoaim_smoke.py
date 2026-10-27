#!/usr/bin/env python3
"""Verify hub aim and Start's driver frame through the local HAL/NT simulator.

Requires field-enabled scripts/sim_headless.sh, pyntcore, robotpy-wpimath,
and websockets. Tests both alliances and disables simulation inputs on exit.
"""
import argparse
import asyncio
import contextlib
import hashlib
import json
import math
from pathlib import Path

import ntcore
from wpimath.geometry import Pose2d, Pose3d
from wpimath.kinematics import SwerveModuleState
from websockets.asyncio.client import connect
from sim_evidence import running_sim_binary


async def main():
    binary = running_sim_binary()
    result = {'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
              'samples': [], 'checks': {}}
    client = ntcore.NetworkTableInstance.create()
    client.startClient4('autoaim-smoke')
    client.setServer('127.0.0.1', 5810)
    options = ntcore.PubSubOptions(periodic=.02)
    pose = client.getStructTopic('/AdvantageScope/Drive/Pose', Pose2d).subscribe(Pose2d(), options)
    targets = client.getStructArrayTopic('/AdvantageScope/Drive/ModuleTargets', SwerveModuleState).subscribe([], options)
    hood = client.getDoubleTopic('/SmartDashboard/Hood/Target').subscribe(float('nan'))
    shooter_velocity = client.getDoubleTopic('/SmartDashboard/ShooterFlywheel/Velocity').subscribe(float('nan'))
    shooter = client.getDoubleTopic('/SmartDashboard/ShooterFlywheel/Target').subscribe(float('nan'))
    truth = client.getStructTopic('/Simulation/PhysicsBodyTruth', Pose3d).subscribe(Pose3d(), options)
    async with connect('ws://127.0.0.1:3300/wpilibws') as ws:
        async def drain():
            async for _ in ws:
                pass
        reader = asyncio.create_task(drain())

        async def hold(seconds, red=False, button=0, forward=0., enabled=True):
            for _ in range(round(seconds / .02)):
                buttons = [False] * 10
                if button:
                    buttons[button - 1] = True
                await ws.send(json.dumps({'type': 'Joystick', 'device': '0', 'data': {
                    '>axes': [0, -forward, 0, 0, 0, 0], '>buttons': buttons, '>povs': [-1]}}))
                await ws.send(json.dumps({'type': 'DriverStation', 'device': '', 'data': {
                    '>enabled': enabled, '>autonomous': False, '>test': False, '>ds': True,
                    '>station': 'red1' if red else 'blue1', '>new_data': True}}))
                await asyncio.sleep(.02)

        def sample(label, red):
            if not client.isConnected() or not pose.getAtomic().time or not truth.getAtomic().time:
                raise RuntimeError('Missing connected odometry/physics telemetry')
            p, q = pose.get(), truth.get()
            hub = ((469.115 if red else 182.105) * .0254, 158.845 * .0254)
            bearing = math.atan2(hub[1] - (q.Y() - .19 * math.sin(q.rotation().Z())),
                                 hub[0] - (q.X() - .19 * math.cos(q.rotation().Z())))
            row = dict(label=('red_' if red else 'blue_') + label,
                       pose=[p.X(), p.Y(), p.rotation().radians()],
                       truth=[q.X(), q.Y(), q.rotation().Z()], hub_bearing=bearing,
                       error=math.remainder(bearing - q.rotation().Z(), 2 * math.pi))
            states = targets.get()
            if len(states) != 4:
                raise RuntimeError('Missing four module targets')
            # Symmetric module locations cancel the rotational velocity terms.
            vx = sum(m.speed * m.angle.cos() for m in states) / 4
            vy = sum(m.speed * m.angle.sin() for m in states) / 4
            h = p.rotation().radians()
            row['requested_field_translation'] = [vx * math.cos(h) - vy * math.sin(h),
                                                  vx * math.sin(h) + vy * math.cos(h)]
            row['hood_target_rotations'] = hood.get()
            row['shooter_target_rps'] = shooter.get()
            result['samples'].append(row)
            return row

        try:
            await hold(1, enabled=False)
            for red in (False, True):
                await hold(.2, red, button=7)
                await hold(.8, red, enabled=False)
                sample('reset', red)
                await hold(4, red, button=1)
                before = sample('aim', red)
                await hold(.2, red, button=8)
                reset = sample('start_reset', red)
                await hold(4, red, button=1)
                after = sample('aim_after_start', red)
                label = 'red' if red else 'blue'
                # Independent interpolation check for the calibration interval
                # around these spawns (123.24, 144.60 and 153.64 inches).
                for aim_sample in (before, after):
                    px, py, ph = aim_sample['pose']
                    hx = (469.115 if red else 182.105) * .0254
                    distance = math.hypot(hx - (px - .19 * math.cos(ph)),
                                          158.845 * .0254 - (py - .19 * math.sin(ph)))
                    if not 123.24 * .0254 <= distance <= 153.64 * .0254:
                        raise RuntimeError('Spawn shot outside regression interpolation interval')
                    lower = (123.24 * .0254, .019, 37.9)
                    upper = (144.60 * .0254, .020, 40.)
                    if distance > upper[0]:
                        lower, upper = upper, (153.64 * .0254, .020, 41.)
                    fraction = (distance - lower[0]) / (upper[0] - lower[0])
                    expected_hood = lower[1] + fraction * (upper[1] - lower[1])
                    expected_rps = lower[2] + fraction * (upper[2] - lower[2])
                    result['checks'][aim_sample['label'] + '_interpolated_setpoints'] = (
                        abs(aim_sample['hood_target_rotations'] - expected_hood) < .0002 and
                        abs(aim_sample['shooter_target_rps'] - expected_rps) < .1)
                spinning = abs(shooter_velocity.get())
                await hold(2., red)
                released = abs(shooter_velocity.get())
                result['samples'].append({'label': label + '_release',
                                          'before_rps': spinning, 'after_rps': released})
                # Stop retains the last target as telemetry; actual flywheel
                # decay distinguishes neutral output from a latched velocity loop.
                result['checks'][label + '_release_stops_shooter'] = spinning > 20 and released < spinning * .5

                # Allow plant/odometry tracking error. The original Start reset
                # produced errors over 60 degrees while odometry claimed alignment.
                result['checks'][label + '_initial_hub_aim'] = abs(before['error']) < math.radians(3)
                result['checks'][label + '_start_preserves_field_heading'] = abs(math.remainder(
                    reset['pose'][2] - before['pose'][2], 2 * math.pi)) < math.radians(2)
                result['checks'][label + '_hub_aim_after_start'] = abs(after['error']) < math.radians(3)
                # Verify the driver transform in commanded wheel vectors;
                # field obstacles at the spawn can block physical translation.
                for button in (0, 1):
                    await hold(.25, red, button=button, forward=.25)
                    end = sample('translation_aim' if button else 'translation_manual', red)
                    dx, dy = end['requested_field_translation']
                    heading = reset['pose'][2]
                    along = dx * math.cos(heading) + dy * math.sin(heading)
                    across = -dx * math.sin(heading) + dy * math.cos(heading)
                    result['checks'][label + ('_aim' if button else '_manual') + '_driver_forward_targets'] = (
                        along > .3 and abs(across) < .05)
                    await hold(.5, red)
        finally:
            await hold(.2, enabled=False)
            reader.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await reader
            client.stopClient()
    result['passed'] = all(result['checks'].values())
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('artifacts/autoaim-validation.json'))
    args = parser.parse_args()
    result = asyncio.run(main())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['passed'] else 1)
