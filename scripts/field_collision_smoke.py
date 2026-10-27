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
    rounding = client.getDoubleTopic('/SmartDashboard/FieldSimulation/BumperCornerRadius').subscribe(.005)
    compliant = client.getBooleanTopic('/SmartDashboard/FieldSimulation/CompliantBumpers').subscribe(False)
    noodle = client.getDoubleTopic('/SmartDashboard/FieldSimulation/NoodleDiameter').subscribe(0.)
    foam_compression = client.getDoubleArrayTopic('/Simulation/Contact/FoamCompressionM').subscribe([],options)
    foam_forces = client.getDoubleArrayTopic('/Simulation/Contact/FoamNormalForcesN').subscribe([],options)
    rigid_contacts = client.getDoubleArrayTopic('/Simulation/Contact/RigidContactsPanBacking').subscribe([],options)
    loads = client.getDoubleArrayTopic('/Diagnostics/Drive/AzimuthLoad').subscribe([],options)
    count = client.getDoubleTopic('/SmartDashboard/FieldSimulation/ColliderCount').subscribe(0)
    result = {'transport': 'localhost HAL websocket + NT4', 'running_binary': str(binary),
              'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
              'samples': {}, 'checks': {}, 'bump_command_speed_mps': args.bump_speed}
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
                if not client.isConnected():
                    raise RuntimeError('NT disconnected during field validation; no physical result inferred')
                sample=truth.getAtomic()
                if sample.time==0:
                    raise RuntimeError('Missing timestamped physical truth')
                p, t = pose.get(), sample.value
                rows.append({'time': ntcore._now(), 'pose': [p.X(), p.Y(), p.rotation().radians()],
                             'truth': [t.X(), t.Y(), t.Z(), t.rotation().X(), t.rotation().Y(), t.rotation().Z()],
                             'loads': list(loads.get()), 'foam_compression': list(foam_compression.get()),
                             'foam_forces': list(foam_forces.get()), 'rigid_contacts': list(rigid_contacts.get())})
                await asyncio.sleep(.02)
            return rows
        async def reset_scene(red):
            await hold(.25, enabled=True, red=red)
            await hold(.15, enabled=True, reset=True, red=red)
            return await hold(.8, red=red)
        try:
            connected_deadline=time.monotonic()+10
            while not client.isConnected() and time.monotonic()<connected_deadline:
                await asyncio.sleep(.02)
            await asyncio.sleep(.2)
            await hold(1.)
            initial = truth.get()
            result['samples']['startup'] = [initial.X(), initial.Y(), initial.Z()]
            result['samples']['startup_model_pose'] = [model.get().X(),model.get().Y(),model.get().Z()]
            result['checks']['grounded_model_height_is_zero'] = model.getAtomic().time>0 and abs(model.get().Z())<.003 and math.hypot(model.get().X()-initial.X(),model.get().Y()-initial.Y())<.08
            result['checks']['hard_mounted_body_has_no_static_sag'] = abs(initial.Z()-grounded.get())<.001
            result['collider_count'] = count.get()
            result['grounded_body_height_m'] = grounded.get()
            result['compliant_bumpers'] = compliant.get()
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
                r=rounding.get()
                return y-(.43-r)*(abs(ryx)+abs(ryy))-(.06985-r)*abs(ryz)-r
            # Contact acceptance waits for a bounded physical approach instead
            # of assuming an unloaded 90-degree azimuth step finishes in 2 s.
            wall=[]
            approach=time.monotonic()
            while time.monotonic()-approach<8.:
                wall.extend(await hold(.1, enabled=True, lateral=-.35))
                if bumper_south_clearance(wall[-1])<.025:
                    break
            wall.extend(await hold(.5, enabled=True, lateral=-.35))
            if compliant.get():
                settling_deadline=time.monotonic()+3.
                while time.monotonic()<settling_deadline:
                    recent=wall[-min(10,len(wall)):]
                    contacted=any(max(row['foam_forces'],default=0)>10 or sum(row['rigid_contacts'])>0 for row in recent)
                    if contacted and abs(recent[-1]['truth'][1]-recent[0]['truth'][1])<.005:
                        break
                    wall.extend(await hold(.1,enabled=True,lateral=-.35))
            result['samples']['south_wall'] = wall
            result['wall_approach_s']=time.monotonic()-approach
            clearances=[bumper_south_clearance(row) for row in wall]
            result['wall_bumper_clearance_m']={'final':clearances[-1], 'minimum':min(clearances)}
            result['checks']['actual_wall_contact_blocks_motion'] = (
                -.01<clearances[-1]<.025 and min(clearances)>-.01)
            if compliant.get():
                contacted=any(max(row['foam_forces'],default=0)>10 or sum(row['rigid_contacts'])>0 for row in wall)
                # Foam may compress; use backing depth as the penetration bound.
                # The final half-second must show bounded motion against contact.
                result['checks']['actual_wall_contact_blocks_motion'] = (
                    contacted and min(clearances)>-noodle.get()-.01 and
                    abs(wall[-1]['truth'][1]-wall[-min(10,len(wall))]['truth'][1])<.025)
            result['checks']['field_installed'] = count.get()>0
            await reset_scene(False)
            # Approach the blue lower bump through clear alliance-zone space.
            def stick(velocity):
                return 0. if abs(velocity)<.03 else math.copysign(.1+.9*min(.7,abs(velocity))/5.24,velocity)
            deadline=time.monotonic()+12
            while time.monotonic()<deadline:
                current=truth.get()
                dx,dy=.6-current.X(),2.5106376-current.Y()
                if math.hypot(dx,dy)<.04:
                    break
                yaw=math.remainder(current.rotation().Z(),2*math.pi)
                omega=max(-1.,min(1.,-2*yaw))
                turn=0. if abs(yaw)<.015 else math.copysign(.1+.9*abs(omega)/7,omega)
                await hold(.04,enabled=True,forward=stick(1.5*dx),lateral=stick(1.5*dy),turn=turn)
            await hold(.4,enabled=True)
            if len(loads.get())!=25:
                raise RuntimeError('Bump validation requires current module-load diagnostics')
            bump=[]
            deadline=time.monotonic()+6
            while time.monotonic()<deadline:
                bump.extend(await hold(.04,enabled=True,forward=.1+.9*args.bump_speed/5.24))
                if bump[-1]['truth'][0]>6.2:
                    break
            bump.extend(await hold(.8,enabled=True))
            # A full-speed release takes longer than 0.8 s to brake. Confirm
            # landing after bounded physical settling, not during load transfer.
            settling_deadline=time.monotonic()+3.
            while time.monotonic()<settling_deadline:
                recent=bump[-min(10,len(bump)):]
                last=recent[-1]['truth']
                tilt=math.acos(max(-1.,min(1.,math.cos(last[3])*math.cos(last[4]))))
                if (abs(last[2]-grounded.get())<.003 and tilt<math.radians(5) and
                    math.hypot(last[0]-recent[0]['truth'][0],last[1]-recent[0]['truth'][1])<.005):
                    break
                bump.extend(await hold(.1,enabled=True))
            result['samples']['bumper_bump_approach']=bump
            lifted=[row for row in bump if 3.6<row['truth'][0]<5.7 and
                    row['truth'][2]>grounded.get()+.03 and len(row['loads'])==25 and
                    max(row['loads'][1::6])<1.0]
            result['bump_metrics']={'peak_body_height_m':max(row['truth'][2] for row in bump),
                                    'lifted_samples':len(lifted),'final_x_m':bump[-1]['truth'][0],
                                    'cleared_entire_bump':bump[-1]['truth'][0]>5.8 and abs(bump[-1]['truth'][2]-grounded.get())<.02}
            result['bump_metrics'].update({
                'maximum_foam_compression_m':max((max(row['foam_compression'],default=0) for row in bump),default=0),
                'maximum_foam_force_n':max((max(row['foam_forces'],default=0) for row in bump),default=0),
                'pan_contact_samples':sum(bool(row['rigid_contacts']) and row['rigid_contacts'][0]>0 for row in bump),
                'backing_contact_samples':sum(len(row['rigid_contacts'])==2 and row['rigid_contacts'][1]>0 for row in bump)})
            # Bumper collision lifts the chassis; a running approach carries it
            # onto the ramp while wheel traction correctly disappears when unloaded.
            # Lift/contact and complete upright crossing have separate checks.
            # A lift alone is not traversal acceptance.
            result['checks']['bumper_slides_up_and_lifts_chassis'] = (
                bool(lifted) and max(row['truth'][0] for row in lifted)>4.0 and
                grounded.get()+.08<max(row['truth'][2] for row in bump)<.35 and len(lifted)>=3)
            maximum_tilt=max(math.acos(max(-1.,min(1.,math.cos(row['truth'][3])*math.cos(row['truth'][4])))) for row in bump)
            result['bump_metrics']['maximum_tilt_deg']=math.degrees(maximum_tilt)
            final_loads=bump[-1]['loads'][1::6]
            result['checks']['complete_upright_bump_crossing']=(
                result['bump_metrics']['cleared_entire_bump'] and maximum_tilt<math.radians(45) and
                result['bump_metrics']['peak_body_height_m']>grounded.get()+.08 and
                max(abs(row['truth'][1]-2.5106376) for row in bump)<.45 and
                len(final_loads)==4 and sum(v>1 for v in final_loads)>=3 and
                .8*67.6*9.80665<sum(final_loads)<1.2*67.6*9.80665 and
                math.acos(max(-1.,min(1.,math.cos(bump[-1]['truth'][3])*math.cos(bump[-1]['truth'][4]))))<math.radians(5))
            if compliant.get():
                del result['checks']['bumper_slides_up_and_lifts_chassis']
                # Complete upright crossing is required by its own check above.
                result['checks']['compliant_bump_contact_is_finite_and_unilateral'] = (
                    result['bump_metrics']['maximum_foam_compression_m']>0 and
                    all(len(row['foam_forces'])==8 and all(math.isfinite(v) and v>=0 for v in row['foam_forces'])
                        and all(math.isfinite(v) for v in row['truth']) and -.02<row['truth'][2]<2 for row in bump))
        finally:
            try:
                if client.isConnected():
                    await reset_scene(False)
            finally:
                # Disable does not depend on NT data being available.
                with contextlib.suppress(Exception):
                    await websocket.send(json.dumps({'type':'Joystick','device':'0','data':{
                        '>axes':[0]*6,'>buttons':[False]*10,'>povs':[-1]}}))
                    await websocket.send(json.dumps({'type':'DriverStation','device':'','data':{
                        '>enabled':False,'>autonomous':False,'>test':False,'>ds':True,'>new_data':True}}))
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
    parser.add_argument('--bump-speed', type=float, default=5.24, help='Joystick-requested approach speed in m/s, (0,5.24]')
    args = parser.parse_args()
    if not 0<args.bump_speed<=5.24:
        parser.error('--bump-speed must be in (0,5.24] m/s')
    result = asyncio.run(exercise(args))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({'checks': result['checks'], 'passed': result['passed']}, indent=2))
    raise SystemExit(0 if result['passed'] else 1)
