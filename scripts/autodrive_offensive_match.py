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
from wpimath.geometry import Pose2d, Pose3d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState
from websockets.asyncio.client import connect
from sim_evidence import running_sim_binary


def summarize_trace(trace):
    """Summarize sampled requests; these are not continuous-time measurements."""
    collection = [s for s in trace if s['state'] == 'teleop' and s['offense_phase'] == 'collect']
    stopped = lambda s: math.hypot(*s['requested_speeds'][:2]) < .05 and abs(s['requested_speeds'][2]) < .05
    pauses = []
    begin = previous = None
    for sample in trace:
        eligible = sample['state'] == 'teleop' and sample['offense_phase'] == 'collect' and stopped(sample)
        if eligible:
            if begin is None:
                begin = sample['elapsed']
            previous = sample['elapsed']
        elif begin is not None:
            pauses.append(round(previous - begin, 2))
            begin = previous = None
    if begin is not None:
        pauses.append(round(previous - begin, 2))
    # Ground-truth displacement is diagnostic only and is sampled, not a
    # drivetrain feedback signal used by the robot.
    measured = []
    for previous, sample in zip(trace, trace[1:]):
        dt = sample['elapsed'] - previous['elapsed']
        if (previous['state'] == sample['state'] == 'teleop' and
                previous['offense_phase'] == sample['offense_phase'] == 'collect' and dt > 0):
            speed = math.hypot(sample['pose'][0] - previous['pose'][0],
                               sample['pose'][1] - previous['pose'][1]) / dt
            measured.append(speed)
    full_spans = []
    begin = previous = None
    for sample in trace + [None]:
        eligible = (sample is not None and sample['state'] == 'teleop' and
                    sample['offense_phase'] == 'collect' and sample.get('hopper_fill_ratio', 0) >= .999)
        if eligible:
            if begin is None:
                begin = sample['elapsed']
            previous = sample['elapsed']
        elif begin is not None:
            full_spans.append(round(previous - begin, 2))
            begin = previous = None
    ferry = []
    window = []
    for sample in trace + [None]:
        if sample is not None and sample['state'] == 'teleop' and sample['offense_phase'] == 'ferry':
            window.append(sample)
        elif window:
            ferry.append({'start_elapsed_s': window[0]['elapsed'], 'end_elapsed_s': window[-1]['elapsed'],
                          'stored_start': window[0]['stored'], 'stored_end': window[-1]['stored'],
                          'launched_delta': window[-1]['launched'] - window[0]['launched'],
                          'feed_samples': sum(s['feeding'] for s in window),
                          'peak_airborne_in_alliance_zone': max((s.get('airborne_in_alliance_zone', 0) for s in window), default=0),
                          'samples': len(window)})
            window = []
    return {'collection': {'samples': len(collection), 'near_zero_request_samples': sum(stopped(s) for s in collection),
                           'near_zero_request_fraction': (sum(stopped(s) for s in collection) / len(collection) if collection else None),
                           'sampled_pause_spans_s': pauses, 'longest_sampled_pause_span_s': max(pauses, default=0),
                           'sampled_full_hopper_spans_s': full_spans,
                           'measured_motion_samples': len(measured),
                           'near_zero_measured_fraction': (sum(v < .05 for v in measured) / len(measured) if measured else None)},
            'ferry_windows': ferry}


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
    projectiles = nt.getStructArrayTopic('/FieldSimulation/FuelProjectilePositions', Pose3d).subscribe([], opts)
    estimate = nt.getStructTopic('/AdvantageScope/Drive/Pose', Pose2d).subscribe(Pose2d(), opts)
    requested = nt.getStructTopic('/AdvantageScope/Drive/RequestedChassisSpeeds', ChassisSpeeds).subscribe(ChassisSpeeds(), opts)
    modules = nt.getStructArrayTopic('/AdvantageScope/Drive/ModuleTargets', SwerveModuleState).subscribe([], opts)
    stored = number('/SmartDashboard/FieldSimulation/StoredFuelCount')
    hopper_fill_ratio = number('/SmartDashboard/FieldSimulation/HopperFillRatio')
    motor_currents = [number('/SmartDashboard/FieldSimulation/' + name + 'CurrentAmps')
                      for name in ('Shooter', 'Conveyor', 'BottomIndexer', 'TopIndexer')]
    transport_velocity = [number('/SmartDashboard/' + name + '/Velocity')
                          for name in ('BottomIndexer', 'TopIndexer', 'Conveyor', 'Intake')]
    launched = number('/SmartDashboard/FieldSimulation/LaunchedFuelCount')
    scored = number('/SmartDashboard/FieldSimulation/SuccessfulScoreCount')
    clusters = number('/SmartDashboard/AutoDrive/ClusterCount')
    front = number('/SmartDashboard/AutoDrive/SimFrontDetections')
    rear = number('/SmartDashboard/AutoDrive/SimRearDetections')
    offense_phase = string('/SmartDashboard/AutoDrive/OffensePhase')
    scoring_goal_x = number('/SmartDashboard/AutoDrive/ScoringGoalX')
    scoring_goal_y = number('/SmartDashboard/AutoDrive/ScoringGoalY')
    estimated_fuel = number('/SmartDashboard/AutoDrive/EstimatedFuel')
    collection_goal_x = number('/SmartDashboard/AutoDrive/CollectionGoalX')
    collection_goal_y = number('/SmartDashboard/AutoDrive/CollectionGoalY')
    collection_plan = string('/SmartDashboard/AutoDrive/CollectionPlan')
    collection_chain_waypoints = number('/SmartDashboard/AutoDrive/CollectionChainWaypoints')
    transport_latency = number('/SmartDashboard/AutoDrive/ObservedTransportLatencySeconds')
    ferry_transport_latency = number('/SmartDashboard/AutoDrive/ObservedFerryTransportLatencySeconds')
    ferry_cycle_seconds = number('/SmartDashboard/AutoDrive/FerryCycleSeconds')
    refill_seconds = number('/SmartDashboard/AutoDrive/EstimatedRefillSeconds')
    ferry_cycle_feasible = boolean('/SmartDashboard/AutoDrive/FerryCycleFeasible')
    hub_setup_latency = number('/SmartDashboard/AutoDrive/ObservedHubSetupSeconds')
    return_latency = number('/SmartDashboard/AutoDrive/ObservedReturnSeconds')
    return_deferred = boolean('/SmartDashboard/AutoDrive/ReturnDeferredForShift')
    current_empty = boolean('/SmartDashboard/AutoDrive/CurrentIndicatesEmpty')
    empty_noise = number('/SmartDashboard/AutoDrive/EmptyDetectorRollerNoiseAmps')
    empty_shooter = number('/SmartDashboard/AutoDrive/EmptyDetectorShooterAmps')
    empty_quiet = number('/SmartDashboard/AutoDrive/EmptyDetectorQuietSeconds')
    phase = string('/Match/Phase')
    remaining = number('/Match/Time')
    hub_known = boolean('/SmartDashboard/AutoDrive/HubKnown')
    hub_active = boolean('/SmartDashboard/AutoDrive/HubActive')
    hub_next_change = number('/SmartDashboard/AutoDrive/HubNextChangeSeconds')
    feeding = boolean('/SmartDashboard/Shooter/Feeding')
    shooter_ready = boolean('/SmartDashboard/Shooter/Ready')
    shooter_readiness = string('/SmartDashboard/Shooter/Readiness')
    transport_active = boolean('/SmartDashboard/Shooter/TransportActive')
    agitating = boolean('/SmartDashboard/Shooter/Agitating')
    rack_position = number('/SmartDashboard/IntakeRack/Position')
    rack_target = number('/SmartDashboard/IntakeRack/Target')
    sim_feeding = boolean('/SmartDashboard/FieldSimulation/Feeding')
    feed_volts = [number('/SmartDashboard/FieldSimulation/' + name + 'Volts')
                  for name in ('BottomIndexer', 'TopIndexer', 'Conveyor')]
    motor_current = [number('/SmartDashboard/FieldSimulation/' + name + 'CurrentAmps')
                     for name in ('BottomIndexer', 'TopIndexer', 'Conveyor', 'Shooter')]
    shooter_velocity = number('/SmartDashboard/ShooterFlywheel/Velocity')
    shooter_target = number('/SmartDashboard/ShooterFlywheel/Target')
    drive_status = string('/AutoDrive/Status')
    state = string('/Sim/Practice/State')
    running = boolean('/Sim/Practice/Running')
    command = nt.getStringTopic('/Sim/Practice/Command').publish(ntcore.PubSubOptions(keepDuplicates=True))
    start_button = nt.getBooleanTopic('/SmartDashboard/Start Practice Match/running').publish()
    chooser = nt.getStringTopic('/SmartDashboard/Auto Choices/selected').publish()
    active_auto = string('/SmartDashboard/Auto Choices/active')
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
                '>fms': False, '>estop': False, '>station': args.alliance + '1',
                '>new_data': True}}))
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
            result['active_auto'] = active_auto.get()
            if args.dashboard_start:
                start_button.set(True)
            else:
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
            score_at_match_end = None
            while time.monotonic() - start < 170:
                now = time.monotonic()
                current = state.get()
                if current != prev_state:
                    print(f'{now-start:.1f}s practice={current}', flush=True)
                    prev_state = current
                started |= current == 'autonomous'
                if current == 'autonomous' and active_auto.get():
                    result['active_auto'] = active_auto.get()
                if current == 'teleop' and teleop_start_score is None:
                    teleop_start_score = scored.get()
                if current == 'teleop':
                    if previous_stored is not None:
                        teleop_collected += max(0, stored.get() - previous_stored)
                    previous_stored = stored.get()
                if current == 'aborted':
                    raise RuntimeError('practice aborted: ' + nt.getEntry('/Sim/Practice/Reason').getString(''))
                if started and current == 'complete':
                    score_at_match_end = scored.get()
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
                    e = estimate.get()
                    r = requested.get()
                    # Observer-only trajectory evidence; these are airborne
                    # positions, not a count of confirmed landings.
                    boundary = 158.6 * .0254
                    airborne = projectiles.get()
                    zone_projectiles = sum(
                        q.X() <= boundary if args.alliance == 'blue' else q.X() >= 16.541 - boundary
                        for q in airborne)
                    # These low projectile samples are approaching the floor;
                    # they are not distinct tracked landings and may repeat.
                    low_projectiles = [[q.X(), q.Y(), q.Z()] for q in airborne if q.Z() < .3]
                    result['trace'].append({'elapsed': round(now-start, 2), 'state': current,
                        'phase': phase.get(), 'offense_phase': offense_phase.get(), 'time': remaining.get(),
                        'scoring_goal': [scoring_goal_x.get(), scoring_goal_y.get()],
                        'collection_goal': [collection_goal_x.get(), collection_goal_y.get()],
                        'collection_plan': collection_plan.get(), 'collection_chain_waypoints': collection_chain_waypoints.get(),
                        'pose': [p.X(), p.Y(), p.rotation().Z()], 'height_m': p.Z(),
                        'roll_rad': p.rotation().X(), 'pitch_rad': p.rotation().Y(),
                        'estimate': [e.X(), e.Y(), e.rotation().radians()],
                        'requested_speeds': [r.vx, r.vy, r.omega],
                        'stored': stored.get(), 'hopper_fill_ratio': hopper_fill_ratio.get(),
                        'estimated_fuel': estimated_fuel.get(),
                        'observed_transport_latency_s': transport_latency.get(),
                        'observed_ferry_transport_latency_s': ferry_transport_latency.get(),
                        'estimated_refill_s': refill_seconds.get(), 'ferry_cycle_s': ferry_cycle_seconds.get(),
                        'ferry_cycle_feasible': ferry_cycle_feasible.get(),
                        'observed_hub_setup_s': hub_setup_latency.get(),
                        'observed_return_s': return_latency.get(), 'return_deferred_for_shift': return_deferred.get(),
                        'current_indicates_empty': current_empty.get(),
                        'empty_detector': [empty_shooter.get(), empty_noise.get(), empty_quiet.get()],
                        'motor_currents_amps': [v.get() for v in motor_currents],
                        'transport_velocity_rps': [v.get() for v in transport_velocity],
                        'launched': launched.get(), 'scored': scored.get(), 'feeding': feeding.get(),
                        'airborne_in_alliance_zone': zone_projectiles,
                        'low_projectiles': low_projectiles,
                        'shooter_ready': shooter_ready.get(), 'sim_feeding': sim_feeding.get(),
                        'shooter_readiness': shooter_readiness.get(), 'transport_active': transport_active.get(),
                        'agitating': agitating.get(), 'rack_position': rack_position.get(), 'rack_target': rack_target.get(),
                        'feed_volts': [v.get() for v in feed_volts],
                        'motor_current_amps': [v.get() for v in motor_current],
                        'shooter_velocity': shooter_velocity.get(), 'shooter_target': shooter_target.get(),
                        'drive_status': drive_status.get(),
                        'clusters': clusters.get(), 'detections': [front.get(), rear.get()],
                        'hub_known': hub_known.get(), 'hub_active': hub_active.get(),
                        'hub_next_change_s': hub_next_change.get()})
                    last_sample = now
                await asyncio.sleep(.02)
            await joystick(False)
            await asyncio.sleep(.5)
            result.update({'peak_clusters': peak_clusters, 'peak_detections': peak_detections,
                           'peak_stored': peak_stored, 'final_launched': launched.get(),
                           'final_scored': scored.get(), 'score_at_match_end': score_at_match_end,
                           'teleop_collected': teleop_collected,
                           'teleop_score_delta': (score_at_match_end - teleop_start_score if score_at_match_end is not None and teleop_start_score is not None else None),
                           'score_by_phase': score_by_phase})
            ferry_launched = any(
                current['state'] == 'teleop' and current['offense_phase'] == 'ferry' and
                current['hub_known'] and not current['hub_active'] and current['launched'] > previous['launched']
                for previous, current in zip(result['trace'], result['trace'][1:]))
            result['checks'] = {'match_complete': state.get() == 'complete' and not running.get(),
                'requested_auto_selected': result['active_auto'] == args.auto_choice,
                'score_target_met_at_match_end': score_at_match_end is not None and score_at_match_end >= 200,
                'ferry_launched_when_transport_active': ferry_launched or not any(
                    s['state'] == 'teleop' and s['offense_phase'] == 'ferry' and s['sim_feeding']
                    for s in result['trace']),
                'simulated_camera_observed_fuel': peak_detections > 0 and peak_clusters > 0,
                'collected_fuel': peak_stored > 0, 'feeding_seen': saw_feed,
                'launched_fuel': launched.get() > 0, 'scored_fuel': scored.get() > 0,
                'shot_agitation_seen': any(s['agitating'] and s['feeding'] and s['rack_target'] < .2
                                           for s in result['trace']),
                'agitation_stopped_on_complete': not agitating.get(),
                'teleop_collected': teleop_collected > 0,
                'teleop_scoring': teleop_start_score is not None and scored.get() > teleop_start_score and teleop_feed,
                'stopped_on_complete': len(modules.get()) == 4 and max(abs(m.speed) for m in modules.get()) < .05}
            result['passed'] = all(result['checks'].values())
        except Exception as exc:
            result['error'] = str(exc)
            raise
        finally:
            result['diagnostics'] = summarize_trace(result['trace'])
            with contextlib.suppress(Exception):
                command.set('abort')
                nt.flush()
                await joystick(False)
                await asyncio.sleep(.1)
                await disable()
            reader.cancel()
            with contextlib.suppress(asyncio.CancelledError, Exception):
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
    parser.add_argument('--dashboard-start', action='store_true',
                        help='start using the desktop SmartDashboard command')
    parser.add_argument('--output', type=Path, required=True)
    raise SystemExit(0 if asyncio.run(run(parser.parse_args())) else 1)
