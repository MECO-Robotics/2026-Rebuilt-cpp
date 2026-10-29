#!/usr/bin/env python3
"""Verify onboard C++ AutoDrive through localhost HAL/NT; no companion process."""
import argparse
import asyncio
import contextlib
import hashlib
import json
import math
from pathlib import Path
import ntcore
from wpimath.geometry import Pose2d, Pose3d
from wpimath.kinematics import SwerveModuleState, ChassisSpeeds
from websockets.asyncio.client import connect
from sim_evidence import running_sim_binary


async def run(port,hal_port):
    binary = running_sim_binary(nt_port=port,hal_port=hal_port)
    result = dict(scenario='onboard-native-autodrive',seed=3737,
                  binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),checks={},cases=[])
    client = ntcore.NetworkTableInstance.create()
    client.startClient4('onboard-autodrive-validation');client.setServer('127.0.0.1',port)
    opts=ntcore.PubSubOptions(periodic=.02)
    pose=client.getStructTopic('/AdvantageScope/Drive/Pose',Pose2d).subscribe(Pose2d(),opts)
    truth=client.getStructTopic('/Simulation/RobotTruth',Pose3d).subscribe(Pose3d(),opts)
    targets=client.getStructArrayTopic('/AdvantageScope/Drive/ModuleTargets',SwerveModuleState).subscribe([],opts)
    active=client.getBooleanTopic('/AutoDrive/Active').subscribe(False)
    requested=client.getStructTopic('/AdvantageScope/Drive/RequestedChassisSpeeds',ChassisSpeeds).subscribe(ChassisSpeeds(),opts)
    fresh=client.getBooleanTopic('/SmartDashboard/AutoDrive/DetectionFresh').subscribe(False)
    stored=client.getDoubleTopic('/SmartDashboard/FieldSimulation/StoredFuelCount').subscribe(0)
    status=client.getStringTopic('/AutoDrive/Status').subscribe('')
    x=client.getDoubleTopic('/SmartDashboard/AutoDrive/TargetX').publish()
    y=client.getDoubleTopic('/SmartDashboard/AutoDrive/TargetY').publish()
    heading=client.getDoubleTopic('/SmartDashboard/AutoDrive/TargetHeading').publish()
    follow=client.getBooleanTopic('/SmartDashboard/AutoDrive/FollowFuel').publish()
    selection=client.getStringTopic('/SmartDashboard/Auto Choices/selected').publish()
    selected=client.getStringTopic('/SmartDashboard/Auto Choices/active').subscribe('')
    red=False
    old_selection=None
    peak_requested=0;peak_omega=0
    async with connect(f'ws://127.0.0.1:{hal_port}/wpilibws') as ws:
        async def drain():
            async for _ in ws: pass
        reader=asyncio.create_task(drain())
        async def hold(seconds,bumper=False,enabled=True,reset=False,autonomous=False,intake=False):
            nonlocal peak_requested,peak_omega
            for _ in range(round(seconds/.02)):
                buttons=[False]*10;buttons[4]=bumper;buttons[6]=reset
                for device in ('0','1'):
                    await ws.send(json.dumps(dict(type='Joystick',device=device,data={
                        '>axes':[0,0,0,.8 if intake and device=='0' else 0,0,0],'>buttons':buttons if device=='0' else [False]*10,'>povs':[-1]})))
                await ws.send(json.dumps(dict(type='DriverStation',device='',data={
                    '>enabled':enabled,'>autonomous':autonomous,'>test':False,'>ds':True,
                    '>fms':False,'>estop':False,'>station':'red1' if red else 'blue1','>new_data':True})))
                await asyncio.sleep(.02)
                if active.get():
                    v=requested.get();peak_requested=max(peak_requested,math.hypot(v.vx,v.vy));peak_omega=max(peak_omega,abs(v.omega))
        def speed():
            modules=targets.get()
            if len(modules)!=4:raise RuntimeError('missing four module targets')
            return max(abs(m.speed) for m in modules)
        async def reset_goal():
            await hold(.2,enabled=False)
            await hold(.15,reset=True)
            await hold(.8,enabled=False)
            p=pose.get();goal=(p.X()+(-1.8 if red else 1.8),p.Y())
            x.set(goal[0]);y.set(goal[1]);heading.set(math.pi if red else 0);follow.set(False);client.flush()
            await hold(.2,enabled=False)
            return goal
        try:
            await hold(1,enabled=False);old_selection=selected.get()
            if not client.isConnected() or not pose.getAtomic().time:raise RuntimeError('missing robot telemetry')
            for alliance in (False,True):
                red=alliance;goal=await reset_goal()
                await hold(.6,bumper=True)
                result['checks'][('red' if red else 'blue')+'_native_motion']=active.get() and speed()>.1
                seen=fresh.get()
                for _ in range(40):
                    await hold(.1,bumper=True)
                    seen=seen or fresh.get()
                if not red:result['checks']['typed_simulated_camera_detection']=seen
                q=truth.get();error=math.hypot(q.X()-goal[0],q.Y()-goal[1])
                result['cases'].append(dict(alliance='red' if red else 'blue',goal=goal,
                    truth=[q.X(),q.Y(),q.rotation().Z()],error_m=error,status=status.get()))
                result['checks'][('red' if red else 'blue')+'_goal']=error<.25
                await hold(.2,enabled=False)
            red=False
            await reset_goal()
            p=pose.get();close_goal=(p.X()+.35,p.Y())
            x.set(close_goal[0]);y.set(close_goal[1]);heading.set(0);client.flush()
            await hold(.2,enabled=False);await hold(3,bumper=True)
            q=truth.get();close_error=math.hypot(q.X()-close_goal[0],q.Y()-close_goal[1])
            result['cases'].append(dict(mode='short_approach',goal=close_goal,error_m=close_error,status=status.get()))
            result['checks']['short_approach_finishes']=close_error<.1 and not active.get()
            await reset_goal()
            p=pose.get();x.set(p.X()+2.55);y.set(p.Y());heading.set(0);follow.set(False);client.flush()
            await hold(.2,enabled=False);await hold(4,bumper=True)
            await hold(1.5,intake=True);before=stored.get();start=truth.get();visible=fresh.get()
            follow.set(True);client.flush();await hold(.2,intake=True);await hold(3,bumper=True,intake=True)
            end=truth.get();movement=math.hypot(end.X()-start.X(),end.Y()-start.Y())
            result['cases'].append(dict(mode='deployed_camera_fuel_follow',movement_m=movement,
                collected=stored.get()-before,detection_fresh=visible,status=status.get()))
            result['checks']['deployed_camera_fuel_follow']=visible and movement>.1 and stored.get()>before
            goal=await reset_goal()
            await hold(.6,bumper=True);await hold(.2)
            result['checks']['release_cancels_owner']=not active.get() and speed()<.01
            await hold(.4,bumper=True);await hold(.3,bumper=True,enabled=False)
            result['checks']['disable_cancels_owner']=not active.get() and speed()<.01
            selection.set('AutoDrive target');client.flush()
            await hold(.3,enabled=False,autonomous=True)
            await hold(.2,autonomous=True)
            auto_start=pose.get()
            auto_started=active.get()
            await hold(4.3,autonomous=True)
            q=truth.get();error=math.hypot(q.X()-goal[0],q.Y()-goal[1])
            result['cases'].append(dict(mode='autonomous',goal=goal,error_m=error,
                selected=selected.get(),active_at_start=auto_started,status=status.get(),
                start_pose=[auto_start.X(),auto_start.Y(),auto_start.rotation().radians()]))
            result['checks']['autonomous_native_goal']=selected.get()=='AutoDrive target' and error<.25
            result['checks']['bounded_autodrive_requests']=peak_requested<=2.01 and peak_omega<=3.01
            result['peak_requested_mps']=peak_requested;result['peak_requested_radps']=peak_omega
            result['checks']['no_external_command_stream']=not client.getTopics('/AutoDrive/Command')
            result['passed']=all(result['checks'].values())
        finally:
            with contextlib.suppress(Exception):await hold(.2,enabled=False)
            if old_selection:selection.set(old_selection);client.flush()
            reader.cancel()
            with contextlib.suppress(asyncio.CancelledError,Exception):await reader
            client.stopClient()
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',type=int,default=5810);parser.add_argument('--hal-port',type=int,default=3300);parser.add_argument('--output',type=Path)
    args=parser.parse_args();result=asyncio.run(run(args.port,args.hal_port));text=json.dumps(result,indent=2);print(text)
    if args.output:args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(text+'\n')
    raise SystemExit(0 if result.get('passed') else 1)
