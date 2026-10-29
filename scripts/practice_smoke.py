#!/usr/bin/env python3
"""Observe a real 163-second practice session over NT; never drives DS mode while running."""
import argparse
import asyncio
import contextlib
import hashlib
import json
import time
from pathlib import Path
import ntcore
from websockets.asyncio.client import connect
from sim_evidence import running_sim_binary

async def run(port, hal_port, output, controls_only=False, station="Blue1", auto_result="R"):
    binary=running_sim_binary(port,hal_port)
    nt=ntcore.NetworkTableInstance.create();nt.setServer('127.0.0.1',port);nt.startClient4('practice-validation')
    table=nt.getTable('/Sim/Practice')
    command=table.getStringTopic('Command').publish(ntcore.PubSubOptions(keepDuplicates=True))
    topics={name:table.getEntry(name) for name in ['State','Running','MatchTime','GameData','Reason']}
    context=nt.getTable('/Match')
    # Subscribe before the warm-up; first reads of late-created NT entries return defaults.
    probe={key:nt.getEntry(path) for key,path in {
        'phase':'/Match/Phase','time':'/Match/Time','active':'/Match/HubActive','known':'/Match/HubKnown',
        'auto_phase':'/SmartDashboard/AutoDrive/MatchPhase','auto_time':'/SmartDashboard/AutoDrive/MatchTime',
        'auto_active':'/SmartDashboard/AutoDrive/HubActive','auto_known':'/SmartDashboard/AutoDrive/HubKnown',
        'auto_order_known':'/SmartDashboard/AutoDrive/HubOrderKnown',
        'auto_inactive_first':'/SmartDashboard/AutoDrive/InactiveFirstAlliance',
        'auto_first_active':'/SmartDashboard/AutoDrive/FirstActiveAlliance',
        'auto_next_known':'/SmartDashboard/AutoDrive/HubNextChangeKnown',
        'auto_next':'/SmartDashboard/AutoDrive/HubNextChangeSeconds'}.items()}
    result={'binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),'checks':{},'transitions':[],'context_mismatches':[]}
    if controls_only:result['checks'].update(teleop_bumper_schedules=False,bumper_release_cancels=False,controls_preserve_practice=False,
        auto_order_unknown_before_game_data=False,hub_order_and_next_change=False,shift1_hub_state=False)
    selection=nt.getEntry('/SmartDashboard/Auto Choices/selected')
    active=nt.getEntry('/AutoDrive/Active')
    from wpimath.geometry import Pose2d
    pose=nt.getStructTopic('/AdvantageScope/Drive/Pose',Pose2d).subscribe(Pose2d())
    async with connect(f'ws://127.0.0.1:{hal_port}/wpilibws') as ws:
        async def drain():
            async for _ in ws: pass
        reader=asyncio.create_task(drain())
        async def ds_disabled():
            await ws.send(json.dumps({'type':'DriverStation','device':'','data':{'>enabled':False,'>autonomous':False,'>test':False,'>ds':True,'>fms':False,'>estop':False,'>new_data':True}}))
        def send(value):command.set(value);nt.flush()
        try:
            await asyncio.sleep(1)
            if not nt.isConnected():raise RuntimeError('simulator NT unavailable')
            await ds_disabled();await asyncio.sleep(.3)
            selection.setString('AutoDrive target');p=pose.get()
            nt.getEntry('/SmartDashboard/AutoDrive/TargetX').setDouble(p.X()+1.8)
            nt.getEntry('/SmartDashboard/AutoDrive/TargetY').setDouble(p.Y())
            nt.getEntry('/SmartDashboard/AutoDrive/FollowFuel').setBoolean(False)
            table.getEntry('Config/AllianceStation').setString(station);table.getEntry('Config/AutoResult').setString(auto_result)
            nt.flush();await asyncio.sleep(.3);send('start')
            begun=time.monotonic();previous=None;auto_seen=False;agreement=True;observed=set();times=[]
            while time.monotonic()-begun<(40 if controls_only else 170):
                await asyncio.sleep(.1)
                state=topics['State'].getString('');phase=probe['phase'].getString('')
                if state!=previous:
                    result['transitions'].append({'state':state,'elapsed':time.monotonic()-begun,'time':topics['MatchTime'].getDouble(-1),'data':topics['GameData'].getString('')});previous=state
                if state=='autonomous':
                    auto_seen |= active.getBoolean(False)
                    if time.monotonic()-begun>1:
                        result['checks']['auto_order_unknown_before_game_data'] = not probe['auto_order_known'].getBoolean(True)
                if state=='teleop':
                    observed.add(phase);times.append(topics['MatchTime'].getDouble(-1))
                    # Compare values away from network/phase boundaries.
                    m=probe['time'].getDouble(-1)
                    if all(abs(m-b)>.3 for b in [130,105,80,55,30]):
                        sample=dict(phase=phase,time=m,active=probe['active'].getBoolean(False),known=probe['known'].getBoolean(False),
                            auto_phase=probe['auto_phase'].getString(''),auto_time=probe['auto_time'].getDouble(-1),
                            auto_active=probe['auto_active'].getBoolean(False),auto_known=probe['auto_known'].getBoolean(False))
                        consistent=(sample['active']==sample['auto_active'] and sample['known']==sample['auto_known']
                            and sample['phase']==sample['auto_phase'] and abs(sample['time']-sample['auto_time'])<.3)
                        agreement &= consistent
                        if not consistent and len(result['context_mismatches'])<10:result['context_mismatches'].append(sample)
                if controls_only and state=='teleop':
                    await asyncio.sleep(.5)
                    own=station[:4].lower() if station.startswith('Blue') else 'red'
                    inactive='red' if auto_result=='R' else 'blue'
                    first_active='blue' if inactive=='red' else 'red'
                    match_time=probe['time'].getDouble(-1)
                    expected_next=(10 if own==inactive else 35)-(140-match_time)
                    result['hub_transition_observed']={
                        'station':station,'result':auto_result,'time':match_time,'expected_next':expected_next,
                        'order_known':probe['auto_order_known'].getBoolean(False),
                        'inactive_first':probe['auto_inactive_first'].getString(''),
                        'first_active':probe['auto_first_active'].getString(''),
                        'next_known':probe['auto_next_known'].getBoolean(False),
                        'next_seconds':probe['auto_next'].getDouble(-1)}
                    result['checks']['hub_order_and_next_change']=(
                        probe['auto_order_known'].getBoolean(False)
                        and probe['auto_inactive_first'].getString('')==inactive
                        and probe['auto_first_active'].getString('')==first_active
                        and probe['auto_next_known'].getBoolean(False)
                        and abs(probe['auto_next'].getDouble(-1)-expected_next)<.5)
                    p=pose.get()
                    nt.getEntry('/SmartDashboard/AutoDrive/TargetX').setDouble(p.X()+1)
                    nt.getEntry('/SmartDashboard/AutoDrive/TargetY').setDouble(p.Y())
                    nt.flush();await asyncio.sleep(.2)
                    held=False
                    for pressed,count in [(True,40),(False,12)]:
                        for _ in range(count):
                            buttons=[False]*10;buttons[4]=pressed
                            await ws.send(json.dumps({'type':'Joystick','device':'0','data':{'>axes':[0]*6,'>buttons':buttons,'>povs':[-1]}}))
                            await asyncio.sleep(.02)
                            if pressed:held |= active.getBoolean(False)
                    result['checks'].update(teleop_bumper_schedules=held,
                        bumper_release_cancels=not active.getBoolean(False),
                        controls_preserve_practice=topics['Running'].getBoolean(False),context_agreement=agreement)
                    for _ in range(120):
                        await asyncio.sleep(.1)
                        current=probe['time'].getDouble(-1)
                        if probe['phase'].getString('')=='shift1' and 125<current<129:
                            result['hub_shift1_observed']={'station':station,'result':auto_result,'time':current,
                                'phase':probe['auto_phase'].getString(''),
                                'hub_known':probe['auto_known'].getBoolean(False),'hub_active':probe['auto_active'].getBoolean(False),
                                'next_known':probe['auto_next_known'].getBoolean(False),'next_seconds':probe['auto_next'].getDouble(-1)}
                            result['checks']['shift1_hub_state']=(
                                probe['auto_phase'].getString('')=='shift1'
                                and probe['auto_known'].getBoolean(False)
                                and probe['auto_active'].getBoolean(False)==(own!=inactive)
                                and probe['auto_next_known'].getBoolean(False)
                                and abs(probe['auto_next'].getDouble(-1)-(current-105))<.5)
                            break
                    break
                if state=='complete':break
                if state=='aborted':raise RuntimeError(topics['Reason'].getString('unexpected abort'))
            if not controls_only:
                states=[t['state'] for t in result['transitions']]
                result['checks'].update(complete=topics['State'].getString('')=='complete',
                    phases=states==['autonomous','assessment','teleop','complete'],
                    autonomous_scheduled=auto_seen,context_agreement=agreement,
                    all_hub_phases=observed=={'transition','shift1','shift2','shift3','shift4','endgame'},
                    teleop_countdown=bool(times) and times[0]>139 and times[-1]<.3 and all(b<=a+.05 for a,b in zip(times,times[1:])),
                    autonomous_cancelled=not active.getBoolean(False),
                    game_data_delayed=all(t['data']=='' for t in result['transitions'] if t['state'] in ['autonomous','assessment']))
                send('start');await asyncio.sleep(.5);await ds_disabled();await asyncio.sleep(.4)
            else:
                await ds_disabled();await asyncio.sleep(.4)
            result['checks']['manual_disable_aborts']=topics['State'].getString('')=='aborted' and not topics['Running'].getBoolean(True)
            result['passed']=all(result['checks'].values())
        finally:
            with contextlib.suppress(Exception):send('abort');await asyncio.sleep(.1);await ds_disabled()
            reader.cancel()
            with contextlib.suppress(asyncio.CancelledError):await reader
            nt.stopClient();ntcore.NetworkTableInstance.destroy(nt)
            output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return result['passed']

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--port',type=int,default=5811);parser.add_argument('--hal-port',type=int,default=3301)
    parser.add_argument('--station',choices=['Blue1','Red1'],default='Blue1');parser.add_argument('--auto-result',choices=['R','B'],default='R');parser.add_argument('--controls-only',action='store_true');parser.add_argument('--output',type=Path,default=Path('artifacts/practice-match-validation.json'));args=parser.parse_args()
    raise SystemExit(0 if asyncio.run(run(args.port,args.hal_port,args.output,args.controls_only,args.station,args.auto_result)) else 1)
