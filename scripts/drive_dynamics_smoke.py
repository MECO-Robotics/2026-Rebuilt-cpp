#!/usr/bin/env python3
"""Measure local HAL joystick acceleration and three braking modes, using body truth.

Run with FRC_SIM_FIELD_COLLIDERS=0. Always disables the desktop robot on exit.
"""
import argparse
import asyncio
import contextlib
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import time
import ntcore
from sim_evidence import running_sim_binary
from websockets.asyncio.client import connect


async def measure():
    nt = ntcore.NetworkTableInstance.create()
    nt.startClient4('drive-dynamics')
    nt.setServer('127.0.0.1', 5810)
    truth = nt.getRawTopic('/Simulation/PhysicsBodyTruth').subscribe('struct:Pose3d', b'', ntcore.PubSubOptions(periodic=.02, sendAll=True))
    electrical = nt.getDoubleArrayTopic('/Diagnostics/Drive/MotorElectrical').subscribe([], ntcore.PubSubOptions(periodic=.02, sendAll=True))
    requested = nt.getRawTopic('/AdvantageScope/Drive/RequestedChassisSpeeds').subscribe('struct:ChassisSpeeds', b'', ntcore.PubSubOptions(periodic=.02, sendAll=True))
    colliders = nt.getBooleanTopic('/SmartDashboard/FieldSimulation/CollidersEnabled').subscribe(True)
    binary = running_sim_binary()
    result = {'running_binary': str(binary), 'worktree_dirty': bool(subprocess.check_output(['git', 'status', '--porcelain'], text=True).strip()), 'revision': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
              'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
              'scope': 'Desktop physical body truth; not hardware calibration', 'runs': {}}
    try:
        async with connect('ws://127.0.0.1:3300/wpilibws') as ws:
            async def drain():
                async for _ in ws:
                    pass
            reader = asyncio.create_task(drain())
            async def control(enabled, forward=0., reset=False):
                buttons = [False]*10
                buttons[6] = reset
                await ws.send(json.dumps({'type': 'Joystick', 'device': '0', 'data': {
                    '>axes': [0., -forward, 0., 0., 0., 0.], '>buttons': buttons, '>povs': [-1]}}))
                await ws.send(json.dumps({'type': 'DriverStation', 'device': '', 'data': {
                    '>enabled': enabled, '>autonomous': False, '>test': False, '>estop': False,
                    '>fms': False, '>ds': True, '>new_data': True}}))
            async def hold(seconds, enabled=False, forward=0., reset=False, rows=None):
                start = time.monotonic()
                while time.monotonic()-start < seconds:
                    await control(enabled, forward, reset)
                    raw = truth.getAtomic()
                    if rows is not None and len(raw.value) == 56 and (not rows or raw.time != rows[-1]['timestamp']):
                        p = struct.unpack('<7d', raw.value)
                        rows.append({'timestamp': raw.time, 'x': p[0], 'y': p[1], 'z': p[2],
                                     'enabled': enabled, 'stick': forward, 'electrical': electrical.get(), 'requested': list(struct.unpack('<3d', requested.get())) if len(requested.get()) == 24 else []})
                    await asyncio.sleep(.02)
            try:
                await hold(1.)
                if not nt.isConnected() or colliders.get():
                    raise RuntimeError('Need connected local simulator with field colliders disabled')
                for mode in ('release', 'reverse', 'disabled_brake'):
                    await hold(.2, True, reset=True)
                    await hold(.8)
                    rows = []
                    await hold(2., True, 1., rows=rows)
                    switch = rows[-1]['timestamp']
                    await hold(3., mode != 'disabled_brake', -1. if mode == 'reverse' else 0., rows=rows)
                    # Centred 80 ms position differences measure chassis motion, not wheel speed.
                    velocity = []
                    for i in range(2, len(rows)-2):
                        a, b = rows[i-2], rows[i+2]
                        dt = (b['timestamp']-a['timestamp'])/1e6
                        velocity.append({'t': (rows[i]['timestamp']-switch)/1e6,
                                         'vx': (b['x']-a['x'])/dt, 'x': rows[i]['x']})
                    accel = [r for r in velocity if r['t'] < -.1]
                    brake = [r for r in velocity if r['t'] > .1]
                    def crossing(data, target, rising):
                        return next((r for r in data if (r['vx'] >= target if rising else r['vx'] <= target)), None)
                    low, high = crossing(accel, .5, True), crossing(accel, 4., True)
                    bhigh, blow = crossing(brake, 4., False), crossing(brake, .5, False)
                    stopped = crossing(brake, .1, False)
                    m = {'peak_speed_mps': max(r['vx'] for r in accel),
                         'accel_0p5_to_4_s': high['t']-low['t'] if low and high else None,
                         'brake_4_to_0p5_s': blow['t']-bhigh['t'] if bhigh and blow else None,
                         'stop_after_command_s': stopped['t'] if stopped else None,
                         'stop_distance_m': stopped['x']-rows[next(i for i,r in enumerate(rows) if r['timestamp'] == switch)]['x'] if stopped else None}
                    result['runs'][mode] = {'metrics': m, 'velocity': velocity, 'samples': rows}
                    print(mode, json.dumps(m), flush=True)
                    await hold(1.)
            finally:
                await control(False)
                reader.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await reader
    finally:
        nt.stopClient()
    # At 67.6 kg, 50 A stator and the configured wheel inertia, the ideal
    # current-limited acceleration ceiling is about 5.4 m/s². Allow transport,
    # tire slip, voltage saturation and 80 ms differentiation, but reject the
    # original >2 s / >7 m enabled stop. These are simulation regression bands.
    result['checks'] = {}
    for mode, run in result['runs'].items():
        m = run['metrics']
        result['checks'][mode] = (
            4.8 < m['peak_speed_mps'] < 5.5 and
            m['accel_0p5_to_4_s'] is not None and .65 < m['accel_0p5_to_4_s'] < 1.5 and
            m['brake_4_to_0p5_s'] is not None and .3 < m['brake_4_to_0p5_s'] < 1.1 and
            m['stop_after_command_s'] is not None and m['stop_after_command_s'] < 1.6 and
            0 < m['stop_distance_m'] < 4.5)
    result['passed'] = all(result['checks'].values())
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    evidence = asyncio.run(measure())
    args.output.write_text(json.dumps(evidence, indent=2)+'\n')
    raise SystemExit(0 if evidence['passed'] else 1)
