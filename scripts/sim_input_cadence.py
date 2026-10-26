#!/usr/bin/env python3
"""Regress stale cached DS inputs after raw HAL joystick writes in a local simulator.

Run with FRC_SIM_LATENCY_PROBE=1. No physical Driver Station is contacted.
"""
import argparse,asyncio,contextlib,json,math,time,statistics
from pathlib import Path
import ntcore
from websockets.asyncio.client import connect
async def main(args):
 nt=ntcore.NetworkTableInstance.create();nt.startClient4('raw-vs-cached-input');nt.setServer('127.0.0.1',5810)
 sub=nt.getDoubleArrayTopic('/Diagnostics/Latency/RobotLoop').subscribe([],ntcore.PubSubOptions(periodic=.02,sendAll=True,pollStorage=2048))
 result={}
 async with connect('ws://127.0.0.1:3300/wpilibws',max_size=None) as ws:
  async def drain():
   async for _ in ws:pass
  reader=asyncio.create_task(drain())
  async def ds(enabled):
   await ws.send(json.dumps({'type':'DriverStation','device':'','data':{'>enabled':enabled,'>ds':True,'>autonomous':False,'>test':False,'>estop':False,'>fms':False,'>new_data':True}}))
  try:
   for label,notify in [('no_explicit_DS_notifications',False),('explicit_DS_notifications',True)]:
    await ds(True);start=time.monotonic();rows=[];sub.readQueue()
    while (t:=time.monotonic()-start)<8:
     v=.4*math.sin(t*2*math.pi)
     await ws.send(json.dumps({'type':'Joystick','device':'0','data':{'>axes':[0,-v,0,0,0,0],'>buttons':[False]*10,'>povs':[-1]}}))
     if notify:await ds(True)
     for e in sub.readQueue():
      if len(e.value)>=9:rows.append([e.time,*e.value])
     await asyncio.sleep(.01)
    active=rows[30:]
    changed=lambda index:sum(abs(b[index]-a[index])>.001 for a,b in zip(active,active[1:]))
    span=(active[-1][0]-active[0][0])/1e6
    diff=[abs(r[3]-r[7]) for r in active]
    result[label]={'rows':rows,'raw_HAL_changes_hz':changed(7)/span,'DS_cached_changes_hz':changed(3)/span,'median_difference':statistics.median(diff),'max_difference':max(diff),'DS_attached':sorted(set(r[8] for r in active)),'enabled':sorted(set(r[9] for r in active))}
    print(label,{k:v for k,v in result[label].items() if k!='rows'},flush=True)
  finally:
   await ds(False);reader.cancel()
   with contextlib.suppress(asyncio.CancelledError):await reader
 nt.stopClient()
 result['passed']=all(v['raw_HAL_changes_hz']>25 and v['DS_cached_changes_hz']>25 and v['median_difference']<.08 for v in result.values())
 args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(result,indent=2)+'\n')
 return result['passed']
if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 raise SystemExit(0 if asyncio.run(main(a)) else 1)
