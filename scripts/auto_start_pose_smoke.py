#!/usr/bin/env python3
"""Check a PathPlanner auto seeds physical and estimated simulation poses."""
import json
import math
import time
import argparse
from pathlib import Path

import ntcore
from wpimath.geometry import Pose2d, Pose3d
from websockets.sync.client import connect
from sim_evidence import running_sim_binary

running_sim_binary(15811, 13301)
parser = argparse.ArgumentParser()
parser.add_argument('--alliance', choices=['blue', 'red'], default='blue')
args = parser.parse_args()
path = json.loads(Path('src/main/deploy/pathplanner/paths/Left Trench Bump 0.path').read_text())
start = path['waypoints'][0]['anchor']
heading = math.radians(path['idealStartingState']['rotation'])
if args.alliance == 'red':
    # PathPlanner's 2026 rotational field flip.
    start = {'x': 16.54 - start['x'], 'y': 8.07 - start['y']}
    heading -= math.pi
n = ntcore.NetworkTableInstance.create()
n.setServer('127.0.0.1', 15811)
n.startClient4('auto-start-pose-smoke')
opts = ntcore.PubSubOptions(periodic=.01)
truth = n.getStructTopic('/Simulation/RobotTruth', Pose3d).subscribe(Pose3d(), opts)
pose = n.getStructTopic('/AdvantageScope/Drive/Pose', Pose2d).subscribe(Pose2d(), opts)
state = n.getStringTopic('/Sim/Practice/State').subscribe('', opts)
status = n.getStringTopic('/SmartDashboard/Simulation/AutoStartPose').subscribe('', opts)
choice = n.getStringTopic('/SmartDashboard/Auto Choices/selected').publish()
button = n.getBooleanTopic('/SmartDashboard/Start Practice Match/running').publish()
abort = n.getStringTopic('/Sim/Practice/Command').publish(ntcore.PubSubOptions(keepDuplicates=True))
try:
    time.sleep(1)
    if not n.isConnected() or state.get() not in ('idle', 'aborted', 'complete'):
        raise RuntimeError('expected an idle isolated simulator')
    with connect('ws://127.0.0.1:13301/wpilibws') as ws:
        ws.send(json.dumps({'type': 'DriverStation', 'device': '', 'data': {
            '>enabled': False, '>station': args.alliance + '1', '>ds': True,
            '>new_data': True}}))
        time.sleep(.2)
    choice.set('Left Trench Bump')
    n.flush()
    time.sleep(.3)
    button.set(True)
    n.flush()
    deadline = time.monotonic() + 4
    while time.monotonic() < deadline:
        a, b = truth.get(), pose.get()
        error = lambda x, y: math.hypot(x - start['x'], y - start['y'])
        angle = abs(math.remainder(a.rotation().Z() - heading, 2 * math.pi))
        if state.get() == 'autonomous' and status.get() == 'applied' and error(a.X(), a.Y()) < .15 and error(b.X(), b.Y()) < .15 and angle < .15:
            time.sleep(.15)
            a, b = truth.get(), pose.get()
            if error(a.X(), a.Y()) < .15 and error(b.X(), b.Y()) < .15:
                print('PASS: physical pose and odometry stay at the selected auto start')
                break
        time.sleep(.02)
    else:
        raise RuntimeError(f'auto start failed: state={state.get()} status={status.get()} truth={truth.get()} odometry={pose.get()}')
finally:
    abort.set('abort')
    n.flush()
    time.sleep(.2)
    n.stopClient()
