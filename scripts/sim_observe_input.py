#!/usr/bin/env python3
"""Passively record physical-stick HAL, DS cache, command and requested-speed cadence.

Requires an already-running simulator with FRC_SIM_LATENCY_PROBE=1, pyntcore and
robotpy-wpimath. Does not inject inputs, enable, or disable the simulator.
"""
import argparse
import json
import time
from pathlib import Path

import ntcore
from wpimath.kinematics import ChassisSpeeds


def record(duration):
    client = ntcore.NetworkTableInstance.create()
    client.startClient4('passive-physical-input')
    client.setServer('127.0.0.1', 5810)
    options = ntcore.PubSubOptions(periodic=.01, sendAll=True, pollStorage=4096)
    probe = client.getDoubleArrayTopic('/Diagnostics/Latency/RobotLoop').subscribe([], options)
    requested = client.getStructTopic('/AdvantageScope/Drive/RequestedChassisSpeeds', ChassisSpeeds).subscribe(ChassisSpeeds(), options)
    rows, requests = [], []
    try:
        end = time.monotonic() + duration
        while time.monotonic() < end:
            rows.extend([e.time, *e.value] for e in probe.readQueue())
            requests.extend([e.time, e.value.vx, e.value.vy, e.value.omega] for e in requested.readQueue())
            time.sleep(.005)
    finally:
        client.stopClient()
    if not rows:
        raise RuntimeError('No probes received; start simulator with FRC_SIM_LATENCY_PROBE=1')
    return {'probe': rows, 'requested': requests}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=30)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error('--duration must be positive')
    result = record(args.duration)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result) + '\n')
    print(f'Recorded {len(result["probe"])} loop samples and {len(result["requested"])} requested-speed changes')
