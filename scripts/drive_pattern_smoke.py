#!/usr/bin/env python3
"""Validate square/circle HAL joystick sweeps against physical truth and odometry.

Targets only the local desktop HAL/NT servers. Always disables on exit.
"""
import argparse
import asyncio
import contextlib
import hashlib
import json
import math
from pathlib import Path
import struct
import subprocess
import time

import ntcore
from sim_evidence import running_sim_binary
from websockets.asyncio.client import connect
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState


def quantile(values, fraction):
    ordered = sorted(values)
    return ordered[round((len(ordered) - 1) * fraction)]


def wrap(angle):
    return math.remainder(angle, 2 * math.pi)


def analyze_pattern(name, rows, final, max_speed, grounded_height=.30):
    # Compare velocity after a 0.75 s corner/start settling allowance for a current-limited 90-degree step.
    active = [r for r in rows if r['t'] > .75 and
              (name != 'square' or r['t'] % 2 > .75)]
    errors, heading, azimuth, physical_errors = [], [], [], []
    for r in active:
        expected = [max_speed * (math.copysign(max(0., (abs(a)-.1)/.9), a))
                    for a in r['joystick'][:2]]
        vx, vy, _ = r['speeds']
        a = r['pose'][2]
        field = [vx*math.cos(a)-vy*math.sin(a), vx*math.sin(a)+vy*math.cos(a)]
        errors.append(math.hypot(*(field[i]-expected[i] for i in range(2))))
        heading.append(abs(wrap(r['pose'][2]-r['truth'][3])))
        azimuth.extend(abs(wrap(m[1]-target[1])) for m,target in zip(r['states'],r['targets']))
    # Physical velocity comes from a 0.1 s truth-position difference.
    for i,r in enumerate(rows):
        if i < 5 or r['t'] <= .75 or (name == 'square' and r['t'] % 2 <= .75):
            continue
        prev = rows[i-5]
        dt = (r['truthTimestamp']-prev['truthTimestamp'])/1e6
        if dt <= 0:
            raise RuntimeError('Stale truth telemetry')
        physical = [(r['truth'][j]-prev['truth'][j])/dt for j in range(2)]
        a = r['pose'][2]; vx,vy,_ = r['speeds']
        physical_errors.append(math.hypot(physical[0]-(vx*math.cos(a)-vy*math.sin(a)),
                                         physical[1]-(vx*math.sin(a)+vy*math.cos(a))))
    xy = [math.hypot(r['pose'][0]-r['truth'][0],r['pose'][1]-r['truth'][1]) for r in rows]
    result = {'samples': rows, 'disabled': final, 'metrics': {
        'field_velocity_error_p95_mps': quantile(errors,.95),
        'physical_vs_encoder_velocity_p95_mps': quantile(physical_errors,.95),
        'azimuth_error_p95_rad': quantile(azimuth,.95),
        'heading_error_max_rad': max(heading), 'pose_error_max_m': max(xy),
        'uncommanded_yaw_max_rad': max(abs(r['truth'][3]) for r in rows) if name != 'circle_with_yaw' else None,
        'truth_z_range_m': [min(r['truth'][2] for r in rows),max(r['truth'][2] for r in rows)]}}
    m = result['metrics']
    # 0.5 m/s allows 0.1s telemetry/plant lag during continuous sweeps.
    passed = (
        m['field_velocity_error_p95_mps'] < .5 and
        m['physical_vs_encoder_velocity_p95_mps'] < .5 and
        m['azimuth_error_p95_rad'] < .25 and m['heading_error_max_rad'] < .15 and
        m['pose_error_max_m'] < .5 and
        (m['uncommanded_yaw_max_rad'] is None or m['uncommanded_yaw_max_rad'] < .15) and grounded_height-.01 < m['truth_z_range_m'][0] and
        m['truth_z_range_m'][1] < grounded_height+.10 and max(abs(v) for v in final['speeds']) < .1)
    return result, passed


def analyze_bumper_stress(rows):
    # Current 0.86 m square / 5.5-inch-high shell; all outer corners are material.
    lowest, tilt, before_corner_speed = [], [], []
    for i, r in enumerate(rows):
        w, x, y, z = r['quaternion']
        zz = 1-2*(x*x+y*y)
        tilt.append(math.degrees(math.acos(max(-1., min(1., zz)))))
        lowest.append(r['truth'][2]-.43*abs(2*(x*z-w*y))
                      -.43*abs(2*(y*z+w*x))-.06985*abs(zz))
        if i >= 5 and 1.0 < r['t'] < 1.8:
            prev = rows[i-5]
            dt = (r['truthTimestamp']-prev['truthTimestamp'])/1e6
            if dt > 0:
                before_corner_speed.append((r['truth'][0]-prev['truth'][0])/dt)
    metrics = {'max_tilt_deg': max(tilt), 'minimum_bumper_corner_height_m': min(lowest),
               'approach_physical_speed_mps': quantile(before_corner_speed, .5)}
    # Require a real speed/tilt stimulus, shallow numerical penetration (<5 mm),
    # and a ground catch well before the former approximately 70-degree lean.
    passed = (.5 < metrics['max_tilt_deg'] < 15 and min(lowest) > -.005 and
              metrics['approach_physical_speed_mps'] > 4.)
    return metrics, passed


async def exercise(args):
    nt = ntcore.NetworkTableInstance.create()
    nt.startClient4('season-drive-patterns')
    nt.setServer('127.0.0.1', 5810)
    options = ntcore.PubSubOptions(periodic=.02, sendAll=True)
    pose = nt.getStructTopic('/AdvantageScope/Drive/Pose', Pose2d).subscribe(Pose2d(), options)
    speed = nt.getStructTopic('/AdvantageScope/Drive/ChassisSpeeds', ChassisSpeeds).subscribe(ChassisSpeeds(), options)
    states = nt.getStructArrayTopic('/AdvantageScope/Drive/ModuleStates', SwerveModuleState).subscribe([], options)
    targets = nt.getStructArrayTopic('/AdvantageScope/Drive/ModuleTargets', SwerveModuleState).subscribe([], options)
    chooser = nt.getStringTopic('/SmartDashboard/Auto Choices/selected').publish()
    active_auto = nt.getStringTopic('/SmartDashboard/Auto Choices/active').subscribe('')
    previous_auto = ''
    grounded = nt.getDoubleTopic('/SmartDashboard/FieldSimulation/GroundedBodyHeight').subscribe(.30)
    truth = nt.getRawTopic('/Simulation/PhysicsBodyTruth').subscribe('struct:Pose3d', b'', options)
    binary = running_sim_binary()
    evidence = {'revision': subprocess.check_output(['git','rev-parse','HEAD'], text=True).strip(),
                'worktree_dirty': bool(subprocess.check_output(['git','status','--porcelain'], text=True).strip()),
                'running_binary': str(binary), 'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
                'dependencies': {name: json.loads(Path('vendordeps', name+'.json').read_text())['version']
                                 for name in ('MecoBase','MecoSimulation')},
                'settling_allowance_s': .75,
                'scope': 'Local desktop joystick/physics/sensor integration; no hardware calibration',
                'patterns': {}, 'checks': {}}
    try:
        async with connect('ws://127.0.0.1:3300/wpilibws', max_size=None) as ws:
            async def drain():
                async for _ in ws:
                    pass
            reader = asyncio.create_task(drain())

            async def control(enabled, x=0., y=0., omega=0., reset=False, autonomous=False):
                buttons = [False] * 10
                buttons[7] = reset
                await ws.send(json.dumps({'type': 'Joystick', 'device': '0', 'data': {
                    '>axes': [-y, -x, 0., 0., -omega, 0.], '>buttons': buttons, '>povs': [-1]}}))
                await ws.send(json.dumps({'type': 'DriverStation', 'device': '', 'data': {
                    '>enabled': enabled, '>autonomous': autonomous, '>test': False, '>estop': False,
                    '>fms': False, '>ds': True, '>new_data': True}}))

            async def hold(duration, enabled=False, reset=False):
                start = time.monotonic()
                while time.monotonic() - start < duration:
                    await control(enabled, reset=reset)
                    await asyncio.sleep(.02)

            def sample(t, joystick):
                raw = truth.getAtomic()
                if len(raw.value) != 56 or not raw.time:
                    raise RuntimeError('Missing physical truth telemetry')
                v = struct.unpack('<7d', raw.value)
                qw, qx, qy, qz = v[3:]
                yaw = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
                pa, sa = pose.getAtomic(), speed.getAtomic()
                p, s = pa.value, sa.value
                ms, mt = states.get(), targets.get()
                if len(ms) != 4 or len(mt) != 4:
                    raise RuntimeError('Missing module telemetry')
                row = {'t': t, 'joystick': joystick, 'truthTimestamp': raw.time,
                       'truth': [*v[:3], yaw], 'quaternion': list(v[3:]),
                       'pose': [p.X(), p.Y(), p.rotation().radians()],
                       'poseTimestamp': pa.time, 'speedTimestamp': sa.time,
                       'speeds': [s.vx, s.vy, s.omega],
                       'states': [[m.speed, m.angle.radians()] for m in ms],
                       'targets': [[m.speed, m.angle.radians()] for m in mt]}
                if not all(math.isfinite(n) for k in ('truth','pose','speeds') for n in row[k]):
                    raise RuntimeError('Nonfinite chassis feedback')
                return row

            try:
                deadline = time.monotonic() + 10
                while not nt.isConnected() and time.monotonic() < deadline:
                    await hold(.1)
                if not nt.isConnected():
                    raise RuntimeError('Local NT server unavailable')
                previous_auto = active_auto.get()
                chooser.set('You better hit the A stop before this -Manny (none)')
                nt.flush()
                await hold(1.)
                for name in (('square',) if args.bumper_stress else ('square', 'circle', 'circle_with_yaw')):
                    # AutonomousInit uses the season's physical+estimator reset.
                    # Immediately disable before the default autonomous can drive.
                    await control(True, autonomous=True)
                    await asyncio.sleep(.06)
                    await hold(1.)
                    rows = []
                    start = time.monotonic()
                    duration = 8. if name == 'square' else 12.
                    while (t := time.monotonic() - start) < duration:
                        if name == 'square':
                            level = 1. if args.bumper_stress else .4
                            x, y = ((level,0.), (0.,level), (-level,0.), (0.,-level))[min(3, int(t/2))]
                        else:
                            angle = 2*math.pi*t/duration
                            x, y = .4*math.cos(angle), .4*math.sin(angle)
                        turn = .25 if name == 'circle_with_yaw' else 0.
                        await control(True, x, y, turn)
                        row = sample(t, [x,y,turn])
                        if not rows or row['truthTimestamp'] != rows[-1]['truthTimestamp']:
                            rows.append(row)
                        await asyncio.sleep(.02)
                    await hold(1.5)
                    final = sample(duration+1.5, [0.,0.,0.])
                    result, passed = analyze_pattern(name, rows, final, args.max_speed, grounded.get())
                    if args.bumper_stress:
                        result['bumper_stress_metrics'], passed = analyze_bumper_stress(rows)
                        evidence['scope'] = 'Full-stick bumper-to-ground stress check; tracking metrics are informational'
                        print('bumper stress', json.dumps(result['bumper_stress_metrics']), flush=True)
                    evidence['patterns'][name] = result
                    evidence['checks'][name] = passed
                    print(name, json.dumps(result['metrics']), flush=True)
            finally:
                await control(False)
                reader.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await reader
    finally:
        if previous_auto:
            chooser.set(previous_auto)
            nt.flush()
        nt.stopClient()
    evidence['passed'] = all(evidence['checks'].values())
    return evidence


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bumper-stress', action='store_true', help='Check ground catch during a full-stick square; not a tracking acceptance run')
    parser.add_argument('--max-speed', type=float, default=5.24)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    try:
        result = asyncio.run(exercise(args))
    except Exception as error:
        result = {'passed': False, 'error': str(error), 'patterns': {}}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({name: value['metrics'] for name,value in result['patterns'].items()},indent=2))
    print('PASS' if result['passed'] else 'FAIL')
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
