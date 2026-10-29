#!/usr/bin/env python3
"""Exercise the experimental footprint command through the desktop HAL/NT path."""
import argparse
import asyncio
import contextlib
import hashlib
import json
import math
from pathlib import Path

import ntcore
from wpimath.geometry import Pose2d, Pose3d
from wpimath.kinematics import ChassisSpeeds
from websockets.asyncio.client import connect
from sim_evidence import running_sim_binary


async def run(port, hal_port):
    binary=running_sim_binary(nt_port=port,hal_port=hal_port)
    client=ntcore.NetworkTableInstance.create()
    client.startClient4('footprint-sim-validation');client.setServer('127.0.0.1',port)
    opts=ntcore.PubSubOptions(periodic=.02)
    pose=client.getStructTopic('/AdvantageScope/Drive/Pose',Pose2d).subscribe(Pose2d(),opts)
    truth=client.getStructTopic('/Simulation/RobotTruth',Pose3d).subscribe(Pose3d(),opts)
    requested=client.getStructTopic('/AdvantageScope/Drive/RequestedChassisSpeeds',ChassisSpeeds).subscribe(ChassisSpeeds(),opts)
    active=client.getBooleanTopic('/AutoDrive/Active').subscribe(False)
    status=client.getStringTopic('/AutoDrive/Status').subscribe('')
    x=client.getDoubleTopic('/SmartDashboard/AutoDrive/TargetX').publish()
    y=client.getDoubleTopic('/SmartDashboard/AutoDrive/TargetY').publish()
    heading=client.getDoubleTopic('/SmartDashboard/AutoDrive/TargetHeading').publish()
    mode=client.getBooleanTopic('/SmartDashboard/AutoDrive/UseFootprintSim').publish()
    follow=client.getBooleanTopic('/SmartDashboard/AutoDrive/FollowFuel').publish()
    result={'binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),'cases':[],'checks':{}}
    red=False;peak=0.
    async with connect(f'ws://127.0.0.1:{hal_port}/wpilibws') as ws:
        async def drain():
            async for _ in ws: pass
        reader=asyncio.create_task(drain())
        async def hold(seconds,bumper=False,enabled=True,reset=False):
            nonlocal peak
            for _ in range(round(seconds/.02)):
                buttons=[False]*10;buttons[4]=bumper;buttons[6]=reset
                for device in ('0','1'):
                    await ws.send(json.dumps({'type':'Joystick','device':device,'data':{
                        '>axes':[0]*6,'>buttons':buttons if device=='0' else [False]*10,'>povs':[-1]}}))
                await ws.send(json.dumps({'type':'DriverStation','device':'','data':{
                    '>enabled':enabled,'>autonomous':False,'>test':False,'>ds':True,'>fms':False,
                    '>estop':False,'>station':'red1' if red else 'blue1','>new_data':True}}))
                await asyncio.sleep(.02)
                if active.get():
                    velocity=requested.get();peak=max(peak,math.hypot(velocity.vx,velocity.vy))
        try:
            await hold(1,enabled=False)
            if not client.isConnected() or not pose.getAtomic().time:raise RuntimeError('missing robot telemetry')
            mode.set(True);follow.set(False);client.flush()
            for red in (False,True):
                await hold(.2,enabled=False);await hold(.15,reset=True);await hold(.8,enabled=False)
                p=pose.get();goal=(p.X()+(-1.0 if red else 1.0),p.Y())
                x.set(goal[0]);y.set(goal[1]);heading.set(math.pi if red else 0);client.flush()
                await hold(.2,enabled=False)
                await hold(8,bumper=True)
                q=truth.get();error=math.hypot(q.X()-goal[0],q.Y()-goal[1])
                label='red' if red else 'blue'
                result['cases'].append({'alliance':label,'goal':goal,'truth':[q.X(),q.Y(),q.rotation().Z()],
                                        'error_m':error,'status':status.get()})
                result['checks'][label+'_arrived']=error<.1 and status.get()=='arrived' and not active.get()
                await hold(.2,enabled=False)
            red=False
            await hold(.2,enabled=False);await hold(.15,reset=True);await hold(.8,enabled=False)
            goal=(5.9,2.4)
            x.set(goal[0]);y.set(goal[1]);heading.set(0);client.flush()
            await hold(.2,enabled=False);await hold(25,bumper=True)
            q=truth.get();error=math.hypot(q.X()-goal[0],q.Y()-goal[1])
            result['cases'].append({'mode':'field_obstacle_detour','goal':goal,
                                    'truth':[q.X(),q.Y(),q.rotation().Z()],
                                    'error_m':error,'status':status.get()})
            result['checks']['field_obstacle_detour']=error<.1 and status.get()=='arrived' and not active.get()
            await hold(.2,enabled=False);await hold(.15,reset=True);await hold(.8,enabled=False)
            p=pose.get();x.set(p.X()+1);y.set(p.Y());heading.set(0);client.flush()
            await hold(.2,enabled=False);await hold(1,bumper=True);await hold(.3)
            velocity=requested.get()
            result['checks']['release_stops']=not active.get() and math.hypot(velocity.vx,velocity.vy)<.01
            result['checks']['bounded_requests']=peak<=1.01
            result['peak_requested_mps']=peak
            result['passed']=all(result['checks'].values())
        finally:
            with contextlib.suppress(Exception):await hold(.2,enabled=False)
            mode.set(False);client.flush();reader.cancel()
            with contextlib.suppress(asyncio.CancelledError,Exception):await reader
            client.stopClient()
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',type=int,default=5811)
    parser.add_argument('--hal-port',type=int,default=3301)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args();result=asyncio.run(run(args.port,args.hal_port));text=json.dumps(result,indent=2)
    print(text)
    if args.output:args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(text+'\n')
    raise SystemExit(0 if result.get('passed') else 1)
