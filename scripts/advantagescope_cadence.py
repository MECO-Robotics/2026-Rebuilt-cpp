#!/usr/bin/env python3
"""Observe actual AdvantageScope socket update cadence during a local circular HAL input.

Requires pyntcore, websockets, msgpack and Linux ss. Disables the simulator on exit.
Packet counts are not display frame counts; ss polling gives a lower-bound cadence.
"""
import argparse,asyncio,json,math,re,time
from pathlib import Path
import ntcore,msgpack
from websockets.asyncio.client import connect
async def main(args):
 nt=ntcore.NetworkTableInstance.create();nt.startClient4('AS-cadence-inspector');nt.setServer('127.0.0.1',5810)
 clients=nt.getRawTopic('$clients').subscribe('msgpack',b'')
 deadline=time.monotonic()+5
 while not clients.get() and time.monotonic()<deadline:await asyncio.sleep(.02)
 active=[c for c in msgpack.unpackb(clients.get(),raw=False) if c['id'].startswith('AdvantageScope@')]
 if len(active)!=1:raise RuntimeError('Expected one connected AdvantageScope client')
 client=active[0];port=int(client['conn'].rsplit(':',1)[1])
 subs=nt.getRawTopic('$clientsub$'+client['id']).subscribe('msgpack',b'')
 await asyncio.sleep(.2)
 metadata=msgpack.unpackb(subs.get(),raw=False) if subs.get() else []
 rows=[]
 async with connect('ws://127.0.0.1:3300/wpilibws',max_size=None) as ws:
  async def drain():
   async for _ in ws:pass
  reader=asyncio.create_task(drain())
  async def control(enabled,x=0,y=0):
   await ws.send(json.dumps({'type':'Joystick','device':'0','data':{'>axes':[-y,-x,0,0,0,0],'>buttons':[False]*10,'>povs':[-1]}}))
   await ws.send(json.dumps({'type':'DriverStation','device':'','data':{'>enabled':enabled,'>ds':True,'>autonomous':False,'>test':False,'>estop':False,'>fms':False,'>new_data':True}}))
  start=time.monotonic()
  async def send():
   while (t:=time.monotonic()-start)<args.duration:
    await control(True,.4*math.sin(t*2*math.pi),.4*math.cos(t*2*math.pi));await asyncio.sleep(.01)
  async def sample():
   while time.monotonic()-start<args.duration:
    p=await asyncio.create_subprocess_exec('ss','-tinp',f'( sport = :5810 and dport = :{port} )',stdout=asyncio.subprocess.PIPE)
    out,_=await p.communicate();s=out.decode();m=re.search(r'bytes_sent:(\d+)',s)
    if m:rows.append([time.monotonic()-start,int(m[1]),int(re.search(r'data_segs_out:(\d+)',s)[1]),int(re.search(r'lastsnd:(\d+)',s)[1]) if 'lastsnd:'in s else 0])
    await asyncio.sleep(.005)
  try:await asyncio.gather(send(),sample())
  finally:
   await control(False);reader.cancel()
 changes=[b[0] for a,b in zip(rows,rows[1:]) if b[1]!=a[1]]
 gaps=[b-a for a,b in zip(changes,changes[1:])]
 import statistics
 result={'client':client,'subscriptions':metadata,'scope':'Actual AdvantageScope NT connection server TCP send cadence during 1Hz circular HAL input; not display frame rate','samples':rows,'send_change_events':len(changes),'observed_event_rate_hz':len(changes)/(rows[-1][0]-rows[0][0]),'median_gap_ms':statistics.median(gaps)*1000,'max_gap_ms':max(gaps)*1000,'packets_sent':rows[-1][2]-rows[0][2]}
 nt.stopClient();args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(result,indent=2)+'\n');print({k:v for k,v in result.items() if k!='samples'})
if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--duration',type=float,default=10);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 if a.duration<2:p.error('--duration must be at least 2 seconds')
 asyncio.run(main(a))
