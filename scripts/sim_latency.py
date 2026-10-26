#!/usr/bin/env python3
"""Measure local HAL -> scheduler -> NT latency (FRC_SIM_LATENCY_PROBE=1).

Does not measure pixels/display latency. Leaves the local simulator disabled.
"""
import argparse, asyncio, contextlib, hashlib, json, math, statistics, time
from pathlib import Path
import ntcore
from websockets.asyncio.client import connect
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds


def summary(values):
    s=sorted(values)
    return {'n':len(s),'min_ms':min(s),'median_ms':statistics.median(s),
            'p95_ms':s[round(.95*(len(s)-1))],'max_ms':max(s)} if s else None


async def run(args):
    clients=[]; subscriptions={}
    for label,period in [('fast',.02),('default',.1)]:
        inst=ntcore.NetworkTableInstance.create();inst.startClient4('latency-'+label)
        inst.setServer('127.0.0.1',5810);clients.append(inst)
        opts=ntcore.PubSubOptions(periodic=period,sendAll=True,pollStorage=2048)
        subscriptions[label]={
            'probe':inst.getDoubleArrayTopic('/Diagnostics/Latency/RobotLoop').subscribe([],opts),
            'background':inst.getDoubleArrayTopic('/Diagnostics/Latency/Background').subscribe([],opts),
            'request':inst.getStructTopic('/AdvantageScope/Drive/RequestedChassisSpeeds',ChassisSpeeds).subscribe(ChassisSpeeds(),opts),
            'speed':inst.getStructTopic('/AdvantageScope/Drive/ChassisSpeeds',ChassisSpeeds).subscribe(ChassisSpeeds(),opts),
            'pose':inst.getStructTopic('/AdvantageScope/Drive/Pose',Pose2d).subscribe(Pose2d(),opts)}
    trials=[]; scheduler=[]; cadence=[]; dispatch=[]; age={k:[] for k in subscriptions}; previous_loop=None
    async with connect('ws://127.0.0.1:3300/wpilibws',max_size=None) as ws:
        async def drain():
            async for _ in ws:pass
        reader=asyncio.create_task(drain())
        async def control(enabled,value,notify=True):
            await ws.send(json.dumps({'type':'Joystick','device':'0','data':{'>axes':[0.,-value,0.,0.,0.,0.],'>buttons':[False]*10,'>povs':[-1]}}))
            if notify or not enabled:await ws.send(json.dumps({'type':'DriverStation','device':'','data':{'>enabled':enabled,'>autonomous':False,'>test':False,'>estop':False,'>fms':False,'>ds':True,'>new_data':True}}))
        def collect(trial=None):
            nonlocal previous_loop
            for label,subs in subscriptions.items():
                heading=subs['pose'].get().rotation().radians()
                for kind in ('probe','background','request','speed'):
                    for e in subs[kind].readQueue():
                        arrived=ntcore._now(); value=e.value
                        if trial and e.time>=trial["sent_us"]:
                            age[label].append((arrived-e.time)/1000)
                        if kind=='probe' and len(value)>=9:
                            offset=e.time-e.serverTime
                            if label=='fast':
                                scheduler.append((value[1]-value[0])/1000)
                                if previous_loop is not None:cadence.append((value[0]-previous_loop)/1000)
                                previous_loop=value[0]
                            if trial and abs(value[2]-trial['input'])<1e-6:
                                trial.setdefault(label+'_HAL_sample_ms',(value[0]+offset-trial['sent_us'])/1000)
                                trial.setdefault(label+'_probe_arrival_ms',(arrived-trial['sent_us'])/1000)
                                if abs(value[3]-trial['input'])<1e-6:
                                    trial.setdefault(label+'_command_sample_ms',(value[4]+offset-trial['sent_us'])/1000)
                                    trial.setdefault('nt_minus_fpga_us',value[1]-value[5])
                        elif kind=='background' and len(value)>=10 and label=='fast':
                            dispatch.append((value[9]-value[1])/1000)
                        elif trial and e.time>=trial['sent_us'] and kind in ('request','speed'):
                            fieldx=value.vx*math.cos(heading)-value.vy*math.sin(heading)
                            if kind=='request' and abs(fieldx-trial['expected_mps'])<.15:
                                trial.setdefault(label+'_request_source_ms',(e.time-trial['sent_us'])/1000)
                                trial.setdefault(label+'_request_arrival_ms',(arrived-trial['sent_us'])/1000)
                            if kind=='speed':
                                for percent in (.1,.9):
                                    threshold=trial['previous_mps']+percent*(trial['expected_mps']-trial['previous_mps'])
                                    if (fieldx-threshold)*(trial['expected_mps']-trial['previous_mps'])>=0:
                                        trial.setdefault(label+f'_feedback_{int(percent*100)}pct_source_ms',(e.time-trial['sent_us'])/1000)
        try:
            deadline=time.monotonic()+12
            while time.monotonic()<deadline:
                await control(False,0);collect();await asyncio.sleep(.01)
                if all(c.isConnected() for c in clients) and subscriptions['fast']['probe'].get():break
            if not subscriptions['fast']['probe'].get():raise RuntimeError('Start simulator with FRC_SIM_LATENCY_PROBE=1')
            for _ in range(100):await control(True,0);collect();await asyncio.sleep(.01)
            previous=0.
            for i in range(args.steps):
                value=.35 if i%2==0 else -.35
                target=math.copysign((abs(value)-.1)/.9*5.24,value)
                collect()
                measured=subscriptions['fast']['speed'].get()
                heading=subscriptions['fast']['pose'].get().rotation().radians()
                previous=measured.vx*math.cos(heading)-measured.vy*math.sin(heading)
                trial={'input':value,'expected_mps':target,'previous_mps':previous,'sent_us':ntcore._now()}
                await control(True,value,notify=not args.raw_only)
                end=time.monotonic()+1.15+(i%5)*.013
                next_raw_send=0.
                while time.monotonic()<end:
                    if args.raw_only and time.monotonic()>=next_raw_send:
                        await control(True,value,notify=False)
                        next_raw_send=time.monotonic()+.01
                    collect(trial);await asyncio.sleep(.001)
                trials.append(trial);previous=target
        finally:
            await control(False,0);reader.cancel()
            with contextlib.suppress(asyncio.CancelledError):await reader
            for c in clients:c.stopClient()
    keys=sorted({k for t in trials for k in t if k.endswith('_ms')})
    result={'scope':'Local native simulator; send timestamps precede websocket send; NT clock-synchronized source times. No display/pixel latency measurement.',
            'raw_only':args.raw_only,'trials':trials,'latency':{k:summary([t[k] for t in trials if k in t]) for k in keys},
            'scheduler_duration':summary(scheduler),'robot_loop_interval':summary(cadence),
            'background_publish_wait':summary(dispatch),'NT_source_age_on_arrival':{k:summary(v) for k,v in age.items()},
            'binary_sha256':hashlib.sha256(Path('build/exe/frcUserProgram/linuxx86-64/release/frcUserProgram').read_bytes()).hexdigest()}
    result['complete_trials']=all(all(k in t for k in ('fast_command_sample_ms','fast_request_arrival_ms','default_request_arrival_ms')) for t in trials)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--steps',type=int,default=36);p.add_argument('--raw-only',action='store_true',help='Do not notify DS after raw joystick writes; exposes producer/cache latency');p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.steps<1:p.error('--steps must be positive')
    r=asyncio.run(run(a));a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(r,indent=2)+'\n')
    print(json.dumps({k:v for k,v in r.items() if k!='trials'},indent=2))
    raise SystemExit(0 if r['complete_trials'] else 1)
