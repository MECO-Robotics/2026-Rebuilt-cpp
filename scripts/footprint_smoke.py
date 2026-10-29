#!/usr/bin/env python3
"""Exercise the experimental footprint command through the desktop HAL/NT path."""
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
from wpimath.kinematics import ChassisSpeeds
from websockets.asyncio.client import connect
from sim_evidence import running_sim_binary


async def run(port, hal_port, profile=False, ramp=False, stall_check=False, avoid_bumps=False):
    binary=running_sim_binary(nt_port=port,hal_port=hal_port)
    client=ntcore.NetworkTableInstance.create()
    client.startClient4('footprint-sim-validation');client.setServer('127.0.0.1',port)
    opts=ntcore.PubSubOptions(periodic=.02)
    pose=client.getStructTopic('/AdvantageScope/Drive/Pose',Pose2d).subscribe(Pose2d(),opts)
    truth=client.getStructTopic('/Simulation/RobotTruth',Pose3d).subscribe(Pose3d(),opts)
    requested=client.getStructTopic('/AdvantageScope/Drive/RequestedChassisSpeeds',ChassisSpeeds).subscribe(ChassisSpeeds(),opts)
    measured=client.getStructTopic('/AdvantageScope/Drive/ChassisSpeeds',ChassisSpeeds).subscribe(ChassisSpeeds(),opts)
    active=client.getBooleanTopic('/AutoDrive/Active').subscribe(False)
    status=client.getStringTopic('/AutoDrive/Status').subscribe('')
    x=client.getDoubleTopic('/SmartDashboard/AutoDrive/TargetX').publish()
    y=client.getDoubleTopic('/SmartDashboard/AutoDrive/TargetY').publish()
    heading=client.getDoubleTopic('/SmartDashboard/AutoDrive/TargetHeading').publish()
    heading_readback=client.getDoubleTopic('/SmartDashboard/AutoDrive/TargetHeading').subscribe(float('nan'))
    mode=client.getBooleanTopic('/SmartDashboard/AutoDrive/UseFootprintSim').publish()
    profile_mode=client.getBooleanTopic('/SmartDashboard/AutoDrive/UseFootprintProfileSim').publish()
    avoid_mode=client.getBooleanTopic('/SmartDashboard/AutoDrive/AvoidBumpsSim').publish()
    follow=client.getBooleanTopic('/SmartDashboard/AutoDrive/FollowFuel').publish()
    injected=client.getDoubleArrayTopic('/Simulation/FootprintObstacle').publish()
    follower_trace=client.getDoubleArrayTopic('/Simulation/FootprintFollowerTrace').subscribe([],opts)
    result={'binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),'cases':[],'checks':{}}
    red=False;peak=0.;peak_omega=0.;track_detour=False;detour_samples=[];track_rotation=False;rotation_samples=[];track_dynamic=False;dynamic_samples=[];track_ramp=False;ramp_samples=[]
    def rectangle_separation(xpos,ypos,yaw,cx,cy,hx,hy):
        c,s=math.cos(yaw),math.sin(yaw);dx=xpos-cx;dy=ypos-cy
        half=.43
        ex=half*(abs(c)+abs(s))
        return max(abs(dx)-hx-ex,abs(dy)-hy-ex,
                   abs(dx*c+dy*s)-half-hx*abs(c)-hy*abs(s),
                   abs(-dx*s+dy*c)-half-hx*abs(s)-hy*abs(c))
    def support_separation(xpos,ypos,yaw):
        # The blue near-wall trench support in the simulation field model.
        return rectangle_separation(xpos,ypos,yaw,4.6256194,(50.34+6)*.0254,53*.0254/2,12*.0254/2)
    async with connect(f'ws://127.0.0.1:{hal_port}/wpilibws') as ws:
        async def drain():
            async for _ in ws: pass
        reader=asyncio.create_task(drain())
        async def hold(seconds,bumper=False,enabled=True,reset=False):
            nonlocal peak,peak_omega
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
                    velocity=requested.get();peak=max(peak,math.hypot(velocity.vx,velocity.vy));peak_omega=max(peak_omega,abs(velocity.omega))
                if track_detour:
                    p=truth.get();detour_samples.append((p.X(),p.Y(),p.rotation().Z()))
                if track_ramp:
                    p=truth.get();v=requested.get();m=measured.get();e=pose.get()
                    ramp_samples.append((p.X(),p.Y(),p.Z(),p.rotation().Y(),v.vx,v.vy,v.omega,m.vx,m.vy,m.omega,e.X(),e.Y(),e.rotation().radians(),p.rotation().Z()))
                if track_rotation:
                    p=truth.get();e=pose.get();v=requested.get();m=measured.get()
                    rotation_samples.append({'truth':p.rotation().Z(),'estimated':e.rotation().radians(),
                                             'status':status.get(),'active':active.get(),'requested_omega':v.omega,
                                             'measured_omega':m.omega})
                if track_dynamic:
                    p=truth.get();v=requested.get();m=measured.get()
                    dynamic_samples.append((p.X(),p.Y(),p.rotation().Z(),math.hypot(v.vx,v.vy),active.get(),
                                            v.vx,v.vy,v.omega,m.vx,m.vy,m.omega,list(follower_trace.get())))
        try:
            await hold(1,enabled=False)
            if not client.isConnected() or not pose.getAtomic().time:raise RuntimeError('missing robot telemetry')
            mode.set(True);profile_mode.set(profile);avoid_mode.set(avoid_bumps);follow.set(False);injected.set([]);client.flush()
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
            start=truth.get();goal=(5.9,2.4)
            x.set(goal[0]);y.set(goal[1]);heading.set(0);client.flush()
            await hold(.2,enabled=False);track_detour=True
            detour_start=time.monotonic();await hold(.5,bumper=True)
            for _ in range(45):
                if status.get()=='arrived' and not active.get():break
                await hold(1,bumper=True)
            detour_seconds=time.monotonic()-detour_start;track_detour=False
            q=truth.get();error=math.hypot(q.X()-goal[0],q.Y()-goal[1])
            direct_blocked=any(support_separation(start.X()+(goal[0]-start.X())*i/200,
                                                 start.Y()+(goal[1]-start.Y())*i/200,0)<=0
                               for i in range(201))
            min_separation=min((support_separation(*sample) for sample in detour_samples),default=-1)
            result['cases'].append({'mode':'field_obstacle_detour','goal':goal,
                                    'truth':[q.X(),q.Y(),q.rotation().Z()],
                                    'error_m':error,'status':status.get(),
                                    'elapsed_seconds':detour_seconds,
                                    'samples':len(detour_samples),'minimum_support_separation_m':min_separation})
            result['checks']['field_obstacle_detour']=direct_blocked and min_separation>0 and error<.1 and status.get()=='arrived' and not active.get()
            if ramp:
                if not result['checks']['field_obstacle_detour']:
                    result['passed']=False
                    return result
                await hold(.3,enabled=False)
                ramp_goal=(3.5,2.4)
                x.set(ramp_goal[0]);y.set(ramp_goal[1]);heading.set(0);client.flush()
                await hold(.2,enabled=False);track_ramp=True;await hold(.5,bumper=True)
                ramp_start=time.monotonic()
                for _ in range(45):
                    if not active.get() and status.get() in ('arrived','stopped-before-goal','stopped-no-target'):break
                    await hold(1,bumper=True)
                track_ramp=False;q=truth.get();error=math.hypot(q.X()-ramp_goal[0],q.Y()-ramp_goal[1])
                result['cases'].append({'mode':'modeled_bump_crossing','goal':ramp_goal,
                    'truth':[q.X(),q.Y(),q.rotation().Z()],'error_m':error,'status':status.get(),
                    'elapsed_seconds':time.monotonic()-ramp_start,'samples':ramp_samples})
                if avoid_bumps:
                    # Plan-view envelope of the blue near-wall bump, including
                    # the same 5 mm margin used by the simulated obstacle map.
                    clearance=min((rectangle_separation(s[0],s[1],s[13],4.6256194,4.0346376-60*.0254,
                                                       44.4*.0254/2+.005,73*.0254/2+.005) for s in ramp_samples),default=-1)
                    result['cases'][-1]['minimum_sampled_bump_separation_m']=clearance
                    result['checks']['avoids_bump_and_arrives']=clearance>0 and error<.1 and status.get()=='arrived' and not active.get()
                elif stall_check:
                    v=requested.get()
                    result['checks']['stall_stops']=status.get()=='stopped-before-goal' and not active.get() and math.hypot(v.vx,v.vy)<.01 and abs(v.omega)<.01
                else:
                    result['checks']['modeled_bump_crossing']=error<.1 and status.get()=='arrived' and not active.get()
                result['passed']=all(result['checks'].values())
                return result
            await hold(.3)
            p=pose.get();rotation_goal=(p.X(),p.Y())
            x.set(rotation_goal[0]);y.set(rotation_goal[1]);heading.set(math.pi/2);client.flush()
            await hold(.2,enabled=False);track_rotation=True;await hold(25,bumper=True);track_rotation=False
            q=truth.get();estimate=pose.get();error=math.hypot(q.X()-rotation_goal[0],q.Y()-rotation_goal[1])
            heading_error=abs(math.remainder(q.rotation().Z()-math.pi/2,2*math.pi))
            rotation_start=next((i for i,sample in enumerate(rotation_samples) if sample['active']),len(rotation_samples))
            arrival_index=next((i for i in range(rotation_start,len(rotation_samples)) if rotation_samples[i]['status']=='arrived'),None)
            result['cases'].append({'mode':'rotate_in_place','goal':rotation_goal,
                                    'truth':[q.X(),q.Y(),q.rotation().Z()],
                                    'estimated_pose':[estimate.X(),estimate.Y(),estimate.rotation().radians()],
                                    'heading_readback':heading_readback.get(),
                                    'first_arrived':rotation_samples[arrival_index] if arrival_index is not None else None,
                                    'one_second_after_arrival':rotation_samples[arrival_index+50] if arrival_index is not None and arrival_index+50<len(rotation_samples) else None,
                                    'last_sample':rotation_samples[-1] if rotation_samples else None,
                                    'tail_samples':rotation_samples[-100:],
                                    'error_m':error,'heading_error_rad':heading_error,'status':status.get()})
            settled=rotation_samples[arrival_index+50] if arrival_index is not None and arrival_index+50<len(rotation_samples) else None
            result['checks']['rotate_in_place']=error<.1 and heading_error<.06 and settled is not None and abs(math.remainder(settled['truth']-math.pi/2,2*math.pi))<.06 and status.get()=='arrived' and not active.get()
            await hold(.3)
            dynamic_goal=(8.6,2.4)
            x.set(dynamic_goal[0]);y.set(dynamic_goal[1]);heading.set(0);client.flush()
            await hold(.2,enabled=False);await hold(1,bumper=True)
            v=requested.get();dynamic_started=active.get() and math.hypot(v.vx,v.vy)>.05
            # This is a perceived obstacle injected through NT, not a Jolt body.
            injected.set([7.5,1.9,7.8,2.9]);client.flush()
            track_dynamic=True;await hold(35,bumper=True);track_dynamic=False
            q=truth.get();error=math.hypot(q.X()-dynamic_goal[0],q.Y()-dynamic_goal[1])
            stopped=next((i for i,s in enumerate(dynamic_samples) if s[4] and s[3]<.01),None)
            resumed_index=next((i for i in range(stopped+1,len(dynamic_samples)) if dynamic_samples[i][4] and dynamic_samples[i][3]>.05),None) if stopped is not None else None
            resumed=resumed_index is not None
            separation=min((rectangle_separation(s[0],s[1],s[2],7.65,2.4,.15,.5) for s in dynamic_samples),default=-1)
            result['cases'].append({'mode':'perceived_obstacle_replan','goal':dynamic_goal,
                                    'truth':[q.X(),q.Y(),q.rotation().Z()],'error_m':error,
                                    'status':status.get(),'started':dynamic_started,'stopped':stopped is not None,
                                    'resumed':resumed,'first_sample':dynamic_samples[0] if dynamic_samples else None,
                                    'first_stopped_sample':dynamic_samples[stopped] if stopped is not None else None,
                                    'first_resumed_sample':dynamic_samples[resumed_index] if resumed_index is not None else None,
                                    'sampled_stop_duration_seconds':(resumed_index-stopped)*.02 if resumed_index is not None else None,
                                    'last_sample':dynamic_samples[-1] if dynamic_samples else None,
                                    'tail_samples':dynamic_samples[-100:],
                                    'minimum_sampled_separation_m':separation})
            result['checks']['dynamic_obstacle_replan']=dynamic_started and resumed and separation>0 and error<.1 and status.get()=='arrived' and not active.get()
            injected.set([]);client.flush()
            await hold(.2,enabled=False);await hold(.15,reset=True);await hold(.8,enabled=False)
            p=pose.get();x.set(p.X()+1);y.set(p.Y());heading.set(0);client.flush()
            await hold(.2,enabled=False);await hold(1,bumper=True)
            before_release=requested.get()
            started=active.get() and math.hypot(before_release.vx,before_release.vy)>.1
            await hold(.3)
            velocity=requested.get()
            result['checks']['release_stops']=started and not active.get() and math.hypot(velocity.vx,velocity.vy)<.01
            await hold(.2,enabled=False);await hold(.15,reset=True);await hold(.8,enabled=False)
            p=pose.get();x.set(p.X()+1);y.set(p.Y());heading.set(0);client.flush()
            await hold(.2,enabled=False);await hold(1,bumper=True)
            before_disable=requested.get()
            started=active.get() and math.hypot(before_disable.vx,before_disable.vy)>.1
            await hold(.3,bumper=True,enabled=False)
            velocity=requested.get()
            result['checks']['disable_stops']=started and not active.get() and math.hypot(velocity.vx,velocity.vy)<.01
            result['checks']['bounded_requests']=peak<=1.01 and peak_omega<=1.01
            result['peak_requested_mps']=peak;result['peak_requested_radps']=peak_omega
            result['passed']=all(result['checks'].values())
        finally:
            with contextlib.suppress(Exception):await hold(.2,enabled=False)
            injected.set([]);mode.set(False);profile_mode.set(False);avoid_mode.set(False);client.flush();reader.cancel()
            with contextlib.suppress(asyncio.CancelledError,Exception):await reader
            client.stopClient()
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',type=int,default=5811)
    parser.add_argument('--hal-port',type=int,default=3301)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--profile',action='store_true')
    parser.add_argument('--ramp',action='store_true',help='exercise the modeled bump after the detour')
    parser.add_argument('--stall-check',action='store_true',help='expect the ramp command to stop before the goal')
    parser.add_argument('--avoid-bumps',action='store_true',help='treat modeled bumps as obstacles')
    args=parser.parse_args();result=asyncio.run(run(args.port,args.hal_port,args.profile,args.ramp,args.stall_check,args.avoid_bumps));text=json.dumps(result,indent=2)
    print(text)
    if args.output:args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(text+'\n')
    raise SystemExit(0 if result.get('passed') else 1)
