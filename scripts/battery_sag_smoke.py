#!/usr/bin/env python3
"""Exercise shared battery sag through localhost simulated controller bindings.

Requires field colliders disabled for the straight drive phase. Disables on exit.
"""
import argparse
import asyncio
import contextlib
import hashlib
import json
import math
from pathlib import Path
import statistics
import time
import ntcore
from sim_evidence import running_sim_binary
from websockets.asyncio.client import connect


async def run():
    binary=running_sim_binary()
    result={'binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),
            'running_binary':str(binary),'scope':'Numerical desktop battery estimate, not hardware calibration',
            'samples':{},'checks':{}}
    nt=ntcore.NetworkTableInstance.create()
    nt.startClient4('battery-sag-smoke');nt.setServer('127.0.0.1',5810)
    power=nt.getDoubleArrayTopic('/Simulation/PowerSamples').subscribe([],ntcore.PubSubOptions(periodic=.02,sendAll=True))
    colliders=nt.getBooleanTopic('/SmartDashboard/FieldSimulation/CollidersEnabled').subscribe(True)
    async with connect('ws://127.0.0.1:3300/wpilibws') as ws:
        async def drain():
            async for _ in ws: pass
        reader=asyncio.create_task(drain())
        async def control(enabled=False,drive=0.,intake=False):
            await ws.send(json.dumps({'type':'Joystick','device':'0','data':{
                '>axes':[0,-drive,0,.8 if intake else 0,0,0],'>buttons':[False]*10,'>povs':[-1]}}))
            await ws.send(json.dumps({'type':'DriverStation','device':'','data':{
                '>enabled':enabled,'>autonomous':False,'>test':False,'>estop':False,
                '>fms':False,'>ds':True,'>new_data':True}}))
        async def phase(name,seconds,**controls):
            power.readQueue();rows=[];start=time.monotonic()
            while time.monotonic()-start<seconds:
                await control(**controls)
                rows.extend(list(s.value) for s in power.readQueue() if len(s.value)==3)
                await asyncio.sleep(.02)
            if len(rows)<10: raise RuntimeError('Missing power samples: '+name)
            result['samples'][name]=rows
            return rows
        try:
            await phase('settle',1)
            if colliders.get(): raise RuntimeError('Run with FRC_SIM_FIELD_COLLIDERS=0')
            idle=await phase('idle',1)
            mechanism=await phase('mechanism',2,enabled=True,intake=True)
            await phase('recover_mechanism',1.5)
            drive=await phase('drive',2,enabled=True,drive=.8)
            both=await phase('combined',2,enabled=True,drive=.8,intake=True)
            recovery=await phase('disabled_recovery',3)
            allrows=[r for rows in result['samples'].values() for r in rows]
            result['checks']={
                'finite_bounded':all(all(math.isfinite(v) for v in r) and r[1]>=0 and 0<=r[2]<=12 for r in allrows),
                'relaxed_battery_equation':all(abs(b[2]-.5*(a[2]+max(0,12-.02*b[1])))<1e-8
                    for a,b in zip(allrows,allrows[1:]) if 15000<b[0]-a[0]<25000),
                'no_drive_two_cycle':max(abs(b[2]-a[2]) for a,b in zip(drive[-25:],drive[-24:]))<.5,
                'no_combined_two_cycle':max(abs(b[2]-a[2]) for a,b in zip(both[-25:],both[-24:]))<.5,
                'mechanism_causes_sag':min(r[2] for r in mechanism)<11.8,
                'drive_causes_sag':min(r[2] for r in drive)<11.8,
                'combined_causes_sag':min(r[2] for r in both)<11.8,
                'recovers':statistics.median(r[2] for r in recovery[-20:])>11.95,
                'no_voltage_collapse':min(r[2] for r in allrows)>6,
            }
            result['summary']={name:{'min_voltage':min(r[2] for r in rows),
                'median_voltage':statistics.median(r[2] for r in rows),
                'peak_supply_current':max(r[1] for r in rows)} for name,rows in result['samples'].items()}
            result['passed']=all(result['checks'].values())
        finally:
            with contextlib.suppress(Exception): await control()
            reader.cancel()
            with contextlib.suppress(asyncio.CancelledError,Exception): await reader
            nt.stopClient()
    return result

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();result=asyncio.run(run())
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='samples'},indent=2))
    raise SystemExit(0 if result['passed'] else 1)
