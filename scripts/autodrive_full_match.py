#!/usr/bin/env python3
"""Exercise robot-native goal driving and intake for an entire 2026 practice match.

This is a HAL/NT test driver, not a planner or autonomous match strategy.
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
from wpimath.kinematics import SwerveModuleState, ChassisSpeeds
from websockets.asyncio.client import connect
from sim_evidence import running_sim_binary

async def run(args):
    binary=running_sim_binary(args.port,args.hal_port)
    nt=ntcore.NetworkTableInstance.create();nt.setServer('127.0.0.1',args.port);nt.startClient4('full-match-autodrive-test')
    opts=ntcore.PubSubOptions(periodic=.02)
    truth=nt.getStructTopic('/Simulation/RobotTruth',Pose3d).subscribe(Pose3d(),opts)
    pose=nt.getStructTopic('/AdvantageScope/Drive/Pose',Pose2d).subscribe(Pose2d(),opts)
    modules=nt.getStructArrayTopic('/AdvantageScope/Drive/ModuleTargets',SwerveModuleState).subscribe([],opts)
    speeds=nt.getStructTopic('/AdvantageScope/Drive/ChassisSpeeds',ChassisSpeeds).subscribe(ChassisSpeeds(),opts)
    requested=nt.getStructTopic('/AdvantageScope/Drive/RequestedChassisSpeeds',ChassisSpeeds).subscribe(ChassisSpeeds(),opts)
    active=nt.getBooleanTopic('/AutoDrive/Active').subscribe(False,opts)
    status=nt.getStringTopic('/AutoDrive/Status').subscribe('',opts)
    count=nt.getDoubleTopic('/SmartDashboard/FieldSimulation/StoredFuelCount').subscribe(0,opts)
    clusters=nt.getDoubleTopic('/SmartDashboard/AutoDrive/ClusterCount').subscribe(0,opts)
    nearest_fuel=nt.getDoubleTopic('/SmartDashboard/AutoDrive/SimNearestFuelDistance').subscribe(-1,opts)
    front_count=nt.getDoubleTopic('/SmartDashboard/AutoDrive/SimFrontDetections').subscribe(0,opts)
    rear_count=nt.getDoubleTopic('/SmartDashboard/AutoDrive/SimRearDetections').subscribe(0,opts)
    front_age=nt.getDoubleTopic('/SmartDashboard/AutoDrive/SimFrontFrameAge').subscribe(0,opts)
    rear_age=nt.getDoubleTopic('/SmartDashboard/AutoDrive/SimRearFrameAge').subscribe(0,opts)
    fuel_x=nt.getDoubleTopic('/SmartDashboard/AutoDrive/SelectedFuelX').subscribe(-1,opts)
    fuel_y=nt.getDoubleTopic('/SmartDashboard/AutoDrive/SelectedFuelY').subscribe(-1,opts)
    scored=nt.getDoubleTopic('/SmartDashboard/FieldSimulation/SuccessfulScoreCount').subscribe(0,opts)
    launched=nt.getDoubleTopic('/SmartDashboard/FieldSimulation/LaunchedFuelCount').subscribe(0,opts)
    projectiles=nt.getDoubleTopic('/SmartDashboard/FieldSimulation/FuelProjectileCount').subscribe(0,opts)
    feeding=nt.getBooleanTopic('/SmartDashboard/Shooter/Feeding').subscribe(False,opts)
    sim_feeding=nt.getBooleanTopic('/SmartDashboard/FieldSimulation/Feeding').subscribe(False,opts)
    bottom_volts=nt.getDoubleTopic('/SmartDashboard/FieldSimulation/BottomIndexerVolts').subscribe(0,opts)
    top_volts=nt.getDoubleTopic('/SmartDashboard/FieldSimulation/TopIndexerVolts').subscribe(0,opts)
    conveyor_volts=nt.getDoubleTopic('/SmartDashboard/FieldSimulation/ConveyorVolts').subscribe(0,opts)
    shooter_speed=nt.getDoubleTopic('/SmartDashboard/ShooterFlywheel/Velocity').subscribe(0,opts)
    shooter_target=nt.getDoubleTopic('/SmartDashboard/ShooterFlywheel/Target').subscribe(0,opts)
    hood=nt.getDoubleTopic('/SmartDashboard/Hood/Position').subscribe(0,opts)
    phase=nt.getStringTopic('/Match/Phase').subscribe('',opts)
    matchtime=nt.getDoubleTopic('/Match/Time').subscribe(-1,opts)
    hubknown=nt.getBooleanTopic('/SmartDashboard/AutoDrive/HubKnown').subscribe(False,opts)
    hubactive=nt.getBooleanTopic('/SmartDashboard/AutoDrive/HubActive').subscribe(False,opts)
    hubchange=nt.getDoubleTopic('/SmartDashboard/AutoDrive/HubNextChangeSeconds').subscribe(-1,opts)
    hubchangeknown=nt.getBooleanTopic('/SmartDashboard/AutoDrive/HubNextChangeKnown').subscribe(False,opts)
    station=nt.getStringTopic('/Sim/Practice/State').subscribe('',opts)
    runstate=nt.getBooleanTopic('/Sim/Practice/Running').subscribe(False,opts)
    command=nt.getStringTopic('/Sim/Practice/Command').publish(ntcore.PubSubOptions(keepDuplicates=True))
    x=nt.getDoubleTopic('/SmartDashboard/AutoDrive/TargetX').publish()
    y=nt.getDoubleTopic('/SmartDashboard/AutoDrive/TargetY').publish()
    heading=nt.getDoubleTopic('/SmartDashboard/AutoDrive/TargetHeading').publish()
    follow=nt.getBooleanTopic('/SmartDashboard/AutoDrive/FollowFuel').publish()
    select=nt.getStringTopic('/SmartDashboard/Auto Choices/selected').publish()
    red=args.alliance=='red'; lane_y=8.069-.75 if red else .75
    home=16.541-3.65 if red else 3.65; middle=16.541-7.35 if red else 7.35
    # Stop autonomous short of the neutral fuel grid; teleop approaches it with the intake active.
    auto_target=16.541-6.2 if red else 6.2
    angle=math.pi if red else 0
    result={'scenario':'practice-full-match-native-goal-driving','alliance':args.alliance,'auto_result':args.auto_result,
            'binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(), 'checks':{},'legs':[],
            'phase_seen':[],'auto':{},'error':None}
    def goal(v):x.set(v);y.set(lane_y);heading.set(angle);follow.set(False);nt.flush()
    def error(target):
        q=truth.get();return math.hypot(q.X()-target,q.Y()-lane_y)
    async with connect(f'ws://127.0.0.1:{args.hal_port}/wpilibws') as ws:
        async def drain():
            async for _ in ws:pass
        reader=asyncio.create_task(drain())
        async def joystick(bumper=False,intake=False,aim=False,feed=False):
            buttons=[False]*10;buttons[4]=bumper;buttons[0]=aim;buttons[5]=feed
            await ws.send(json.dumps({'type':'Joystick','device':'0','data':{'>axes':[0,0,0,.8 if intake else 0,0,0],'>buttons':buttons,'>povs':[-1]}}))
        async def disable():
            await ws.send(json.dumps({'type':'DriverStation','device':'','data':{'>enabled':False,'>autonomous':False,'>test':False,'>ds':True,'>fms':False,'>estop':False,'>new_data':True}}))
        try:
            await asyncio.sleep(1)
            if not nt.isConnected() or not truth.getAtomic().time:raise RuntimeError('no robot telemetry')
            await disable();await asyncio.sleep(.3)
            nt.getEntry('/Sim/Practice/Config/AllianceStation').setString('Red1' if red else 'Blue1')
            nt.getEntry('/Sim/Practice/Config/AutoResult').setString(args.auto_result)
            select.set('AutoDrive target');goal(auto_target);nt.flush();await asyncio.sleep(.3)
            command.set('start');nt.flush();start=time.monotonic();previous='';phases=set()
            auto_seen=False;auto_error=None;auto_status='';peak_count=0;peak_clusters=0;started=False
            mode='initial'; leg_target=home; leg_started=0; saw_active=False; leg_settle=0;leg_retries=0;leg_finish=None
            result['retries']=[]
            shot_done=False;shot_started=0;shot_before=0;launched_before=0;shot_feed_seen=False;shot_last_sample=0;shot_empty_at=None
            result['shots']=[];result['fuel_runs']=[]
            fuel_done=False;fuel_started=0;fuel_start=None;fuel_goal=None;fuel_before=0;fuel_active=False;fuel_goal_timestamp=0;fuel_last_sample=0
            hub_errors=[]; telemetry_samples=0;late_motion_seen=False;peak_request=0;peak_omega=0
            while time.monotonic()-start<170:
                now=time.monotonic();state=station.get();t=matchtime.get();p=phase.get()
                if state!=previous:
                    print(f'{now-start:.1f}s practice={state}',flush=True);previous=state
                if state=='autonomous':started=True;auto_seen |= active.get()
                if state=='assessment' and auto_error is None:
                    auto_error=error(auto_target);auto_status=status.get()
                if state=='teleop':
                    if active.get():
                        req=requested.get();peak_request=max(peak_request,math.hypot(req.vx,req.vy));peak_omega=max(peak_omega,abs(req.omega))
                    phases.add(p);peak_count=max(peak_count,count.get());peak_clusters=max(peak_clusters,clusters.get())
                    if 0<t<10 and active.get() and any(abs(m.speed)>.1 for m in modules.get()):late_motion_seen=True
                    if t>0 and all(abs(t-b)>.4 for b in [130,105,80,55,30]) and hubknown.get():
                        inactive=(args.auto_result=='R')==red
                        expected=True if t>130 or t<=30 else (not inactive if t>105 or 55<t<=80 else inactive)
                        telemetry_samples+=1
                        if hubactive.get()!=expected and len(hub_errors)<5:hub_errors.append({'time':t,'phase':p,'actual':hubactive.get(),'expected':expected})
                    if mode=='initial':mode='dwell';leg_started=now
                    if mode=='dwell' and now-leg_started>1.2 and t>(2 if args.continuous else 13):
                        goal(leg_target);mode='release';leg_started=now
                    if mode=='release' and now-leg_started>.2:mode='drive';leg_started=now;saw_active=False
                    if mode=='drive':
                        saw_active |= active.get()
                        if args.follow_fuel and shot_done and not fuel_done and .8<nearest_fuel.get()<1.5:
                            mode='fuel_wait';fuel_started=now
                        elif saw_active and not active.get():
                            q=truth.get();estimated=pose.get()
                            leg_finish={'truth':[q.X(),q.Y(),q.rotation().Z()],
                                'estimate':[estimated.X(),estimated.Y(),estimated.rotation().radians()],
                                'module_speed_max':max((abs(m.speed) for m in modules.get()),default=None),
                                'status':status.get()}
                            mode='settle';leg_settle=now
                        elif now-leg_started>10:
                            result['legs'].append({'target':leg_target,'error_m':error(leg_target),'status':status.get(),'failure':'timeout'});mode='failed'
                    if mode=='settle' and now-leg_settle>.5:
                        distance=error(leg_target)
                        actual=truth.get();estimated=pose.get();velocity=speeds.get()
                        heading_error=abs(math.remainder(actual.rotation().Z()-angle,2*math.pi))
                        settled_speed=math.hypot(velocity.vx,velocity.vy)
                        result['legs'].append({'target':leg_target,'error_m':distance,'status':status.get(),'time_remaining':t,'stored':count.get(),
                            'truth':[actual.X(),actual.Y(),actual.rotation().Z()],
                            'estimate':[estimated.X(),estimated.Y(),estimated.rotation().radians()],'command_finish':leg_finish,
                            'heading_error_rad':heading_error,'settled_speed_mps':settled_speed})
                        print(f"leg {len(result['legs'])}: error={distance:.3f}m stored={count.get():.0f}",flush=True)
                        failed_arrival=distance>(.10 if args.strict_arrival else .25) or (args.strict_arrival and
                            (heading_error>.12 or settled_speed>.2 or status.get()!='arrived'))
                        if failed_arrival:
                            if args.continuous and not args.strict_arrival and distance<1 and status.get() in ('arrived','stopped-before-goal') and leg_retries<2:
                                result['retries'].append(result['legs'].pop());leg_retries+=1
                                print(f'retry incomplete approach: error={distance:.3f}m',flush=True)
                                goal(leg_target);mode='release';leg_started=now
                            else:mode='failed'
                        elif (args.shoot_at_home and not shot_done and leg_target==home and hubknown.get()
                              and hubactive.get() and hubchangeknown.get() and hubchange.get()>17 and count.get()>0):
                            mode='aim';shot_started=now;shot_before=scored.get();launched_before=launched.get();shot_feed_seen=False;shot_empty_at=None
                        else:leg_target=middle if leg_target==home else home;mode='dwell';leg_started=now;leg_retries=0
                    if mode=='fuel_wait':
                        if now-fuel_started>.8 and clusters.get()>0:
                            follow.set(True);nt.flush();mode='fuel_release';fuel_started=now
                        elif now-fuel_started>3:result['fuel_follow']={'failure':'no_clusters'};mode='failed'
                    if mode=='fuel_release' and now-fuel_started>.1:
                        if clusters.get()==0:
                            follow.set(False);nt.flush();leg_target=middle;mode='dwell';leg_started=now
                        else:
                            q=truth.get();fuel_start=(q.X(),q.Y());fuel_before=count.get()
                            fuel_goal_timestamp=fuel_x.getAtomic().time
                            fuel_goal=None;fuel_active=False;fuel_started=now;mode='fuel_drive'
                    if mode=='fuel_drive':
                        fuel_active |= active.get()
                        if (fuel_active and fuel_goal is None and fuel_x.getAtomic().time>fuel_goal_timestamp
                                and fuel_x.get()>=0 and fuel_y.get()>=0):
                            fuel_goal=(fuel_x.get(),fuel_y.get())
                        q=truth.get();movement=math.hypot(q.X()-fuel_start[0],q.Y()-fuel_start[1])
                        if now-fuel_last_sample>=.1:
                            result.setdefault('fuel_trace',[]).append({'elapsed':now-fuel_started,'clusters':clusters.get(),
                                'active':active.get(),'status':status.get(),'position':[q.X(),q.Y()],
                                'selected':[fuel_x.get(),fuel_y.get()],'stored':count.get(),
                                'camera_counts':[front_count.get(),rear_count.get()],
                                'camera_frame_ages':[front_age.get(),rear_age.get()]})
                            fuel_last_sample=now
                        progress=(math.hypot(fuel_start[0]-fuel_goal[0],fuel_start[1]-fuel_goal[1])-
                                  math.hypot(q.X()-fuel_goal[0],q.Y()-fuel_goal[1])) if fuel_goal else 0
                        if fuel_active and count.get()>fuel_before and movement>.1 and progress>.1:
                            result['fuel_follow']={'start':fuel_start,'goal':fuel_goal,'end':[q.X(),q.Y()],
                                'progress_m':progress,'collected':count.get()-fuel_before,'status':status.get()}
                            print(f"fuel follow: {result['fuel_follow']}",flush=True)
                            result['fuel_runs'].append(result['fuel_follow'])
                            fuel_done=True
                            if args.continuous:shot_done=False
                            follow.set(False);nt.flush();leg_target=home;mode='dwell';leg_started=now;leg_retries=0
                        elif now-fuel_started>10:
                            result['fuel_follow']={'failure':'timeout','start':fuel_start,'goal':fuel_goal,
                                'end':[q.X(),q.Y()],'progress_m':progress,'collected':count.get()-fuel_before,
                                'active_seen':fuel_active,'status':status.get()};mode='failed'
                    if mode in ('aim','fire'):
                        if not hubknown.get() or not hubactive.get():
                            result['shot']={'failure':'hub_inactive','scored':scored.get()-shot_before};mode='failed'
                        elif now-shot_started>=2 and mode=='aim':mode='fire'
                        if mode=='fire':shot_feed_seen |= feeding.get()
                        if now-shot_last_sample>=.2:
                            q=truth.get()
                            result.setdefault('shot_trace',[]).append({'time':now-shot_started,'phase':p,
                                'pose':[q.X(),q.Y(),q.rotation().Z()],'shooter_rps':shooter_speed.get(),
                                'shooter_target':shooter_target.get(),'hood':hood.get(),
                                'feeding':feeding.get(),'sim_feeding':sim_feeding.get(),
                                'feed_volts':[bottom_volts.get(),top_volts.get(),conveyor_volts.get()],
                                'stored':count.get(),'launched':launched.get(),'scored':scored.get()})
                            shot_last_sample=now
                        if mode=='fire' and count.get()==0 and launched.get()>launched_before and shot_empty_at is None:
                            shot_empty_at=now
                        settled=shot_empty_at is not None and ((now-shot_empty_at>.5 and projectiles.get()==0) or now-shot_empty_at>2)
                        if mode=='fire' and (settled or now-shot_started>=16):
                            result['shot']={'scored':scored.get()-shot_before,'launched':launched.get(),
                                            'volley_launched':launched.get()-launched_before,'phase':p,'time_remaining':t,
                                            'feeding_seen':shot_feed_seen,'stored':count.get(), 'projectiles_remaining':projectiles.get()}
                            result['shots'].append(result['shot'])
                            print(f"shot: {result['shot']}",flush=True)
                            shot_done=True
                            if args.continuous:fuel_done=False
                            if result['shot']['scored']<=0 or not shot_feed_seen:mode='failed'
                            else:leg_target=middle;mode='dwell';leg_started=now;leg_retries=0
                    if mode=='failed':break
                    intake=mode not in ('aim','fire') and (not args.follow_fuel or not shot_done or fuel_done or mode in ('fuel_wait','fuel_release','fuel_drive'))
                    await joystick(mode in ('drive','fuel_drive'),intake,mode in ('aim','fire'),mode=='fire')
                else:await joystick(False,False)
                if started and state=='complete':break
                if started and state=='aborted':raise RuntimeError('practice aborted: '+nt.getEntry('/Sim/Practice/Reason').getString(''))
                await asyncio.sleep(.02)
            await joystick(False,False);await asyncio.sleep(.3)
            result['phase_seen']=sorted(phases);result['hub_errors']=hub_errors
            result['auto']={'active_seen':auto_seen,'goal_error_m':auto_error,'status':auto_status}
            result['checks']={'match_complete':station.get()=='complete' and not runstate.get(),
                'auto_goal':auto_seen and auto_error is not None and auto_error<.25,
                'all_hub_phases':{'transition','shift1','shift2','shift3','shift4','endgame'}.issubset(phases),
                'hub_context':telemetry_samples>100 and not hub_errors,
                'multiple_goal_legs':len(result['legs'])>=8 and all('failure' not in leg and leg['error_m']<.25 for leg in result['legs']),
                'observed_fuel':peak_clusters>0,'collected_fuel':peak_count>0,
                'stopped_on_complete':not active.get() and len(modules.get())==4 and max(abs(m.speed) for m in modules.get())<.05}
            if args.follow_fuel:
                result['checks']['fuel_follow_collected']=bool(result['fuel_runs']) and all(run['collected']>0 for run in result['fuel_runs'])
            if args.shoot_at_home:
                result['checks']['shot_scored']=bool(result['shots']) and all(shot['scored']>0 and shot['feeding_seen'] for shot in result['shots'])
            result['peak_stored_fuel']=peak_count;result['peak_clusters']=peak_clusters
            result['peak_requested_mps']=peak_request;result['peak_requested_radps']=peak_omega
            result['final_scored']=scored.get();result['final_launched']=launched.get()
            if args.continuous:
                result['checks']['repeated_scoring_and_collection']=len(result['shots'])>=2 and len(result['fuel_runs'])>=2
                result['checks']['driving_late_in_match']=late_motion_seen
            if args.strict_arrival:
                result['checks']['bounded_requests']=peak_request<=2.01 and peak_omega<=3.01
                result['checks']['settled_without_retries']=not result['retries'] and all(
                    leg['error_m']<.10 and leg['heading_error_rad']<.12 and leg['settled_speed_mps']<.2 and leg['status']=='arrived'
                    for leg in result['legs'] if 'failure' not in leg)
            result['passed']=all(result['checks'].values())
        except Exception as e:
            result['error']=str(e);result['passed']=False
            raise
        finally:
            with contextlib.suppress(Exception):
                command.set('abort');nt.flush();await joystick(False,False);await asyncio.sleep(.1);await disable()
            reader.cancel()
            with contextlib.suppress(asyncio.CancelledError):await reader
            nt.stopClient();ntcore.NetworkTableInstance.destroy(nt)
            args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2));return result['passed']

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',type=int,default=5811);p.add_argument('--hal-port',type=int,default=3301)
    p.add_argument('--alliance',choices=['blue','red'],default='blue');p.add_argument('--auto-result',choices=['R','B'],default='R')
    p.add_argument('--shoot-at-home',action='store_true')
    p.add_argument('--follow-fuel',action='store_true',help='test perception-driven collection after the first shot')
    p.add_argument('--continuous',action='store_true',help='repeat scoring and perception-driven collection through the match')
    p.add_argument('--strict-arrival',action='store_true',help='require bounded output and settled pose without retries')
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    if args.follow_fuel and not args.shoot_at_home:p.error('--follow-fuel requires --shoot-at-home')
    if args.continuous and not args.follow_fuel:p.error('--continuous requires --shoot-at-home and --follow-fuel')
    raise SystemExit(0 if asyncio.run(run(args)) else 1)
