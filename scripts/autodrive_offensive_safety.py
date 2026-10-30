"""Check offense cleanup on match-data loss, button release, and DS disable.

Run only against a task-owned local desktop simulator after a practice match.
"""
import argparse
import asyncio,contextlib,hashlib,json,time,ntcore
from pathlib import Path
from sim_evidence import running_sim_binary
from wpimath.kinematics import ChassisSpeeds
from websockets.asyncio.client import connect
async def main(args):
 binary=running_sim_binary(args.port,args.hal_port)
 n=ntcore.NetworkTableInstance.create();n.setServer('127.0.0.1',args.port);n.startClient4('offense-safety-probe')
 phase=n.getStringTopic('/SmartDashboard/AutoDrive/OffensePhase').subscribe('');known=n.getBooleanTopic('/Match/HubKnown').subscribe(False)
 volts=[n.getDoubleTopic('/SmartDashboard/FieldSimulation/'+s+'Volts').subscribe(0) for s in ('BottomIndexer','TopIndexer','Conveyor')]
 req=n.getStructTopic('/AdvantageScope/Drive/RequestedChassisSpeeds',ChassisSpeeds).subscribe(ChassisSpeeds())
 async with connect(f'ws://127.0.0.1:{args.hal_port}/wpilibws') as ws:
  async def drain():
   async for _ in ws: pass
  reader=asyncio.create_task(drain())
  async def ds(enable, data='R'):
   await ws.send(json.dumps({'type':'DriverStation','device':'','data':{'>enabled':enable,'>autonomous':False,'>test':False,'>ds':True,'>fms':False,'>estop':False,'>station':'blue1','>match_time':120,'>game_data':data,'>new_data':True}}))
  async def joy(pressed):
   b=[False]*10;b[4]=pressed
   await ws.send(json.dumps({'type':'Joystick','device':'0','data':{'>axes':[0]*6,'>buttons':b,'>povs':[-1]}}))
  def snap():
   s=req.get();return {'phase':phase.get(),'known':known.get(),'voltages':[v.get() for v in volts],'request':[s.vx,s.vy,s.omega]}
  async def until(predicate, seconds=4):
   end=time.monotonic()+seconds
   while time.monotonic()<end:
    if predicate():return True
    await asyncio.sleep(.05)
   return False
  try:
   await asyncio.sleep(1);n.getEntry('/SmartDashboard/AutoDrive/OffenseEnabled').setBoolean(True);n.flush()
   await ds(False);await joy(False);await asyncio.sleep(.5)
   await ds(True);await joy(True);ok=await until(lambda:phase.get() not in ('idle','wait-for-match',''));await asyncio.sleep(.3);active=snap()
   await ds(True,'');ok_unknown=await until(lambda:not known.get() and phase.get()=='wait-for-match');await asyncio.sleep(.3);unknown=snap()
   await ds(True,'R');ok_resume=await until(lambda:phase.get() not in ('wait-for-match','idle',''));await asyncio.sleep(.3);resumed=snap()
   await joy(False);ok_release=await until(lambda:phase.get()=='idle');await asyncio.sleep(.3);released=snap()
   await joy(True);reengaged=await until(lambda:phase.get() not in ('idle','wait-for-match',''));await asyncio.sleep(.3)
   await ds(False,'R');disabled=await until(lambda:phase.get()=='idle');await asyncio.sleep(.3);disabled_snapshot=snap()
   def stopped(s):return max(map(abs,s['voltages']+s['request']))<.1
   result={'binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),'active_started':ok,'active':active,'unknown_seen':ok_unknown,'unknown':unknown,'unknown_stopped':stopped(unknown),'resumed':ok_resume,'resume_snapshot':resumed,'released':ok_release,'release_snapshot':released,'release_stopped':stopped(released),'reengaged':reengaged,'disabled':disabled,'disabled_snapshot':disabled_snapshot,'disable_stopped':stopped(disabled_snapshot)}
   result['passed']=all([ok,ok_unknown,stopped(unknown),ok_resume,ok_release,stopped(released),reengaged,disabled,stopped(disabled_snapshot)])
   args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(result,indent=2)+'\n')
   print(json.dumps(result,indent=2));assert result['passed']
  finally:
   await joy(False);await ds(False);reader.cancel()
   with contextlib.suppress(asyncio.CancelledError):await reader
   n.stopClient();ntcore.NetworkTableInstance.destroy(n)
if __name__=='__main__':
 parser=argparse.ArgumentParser(description=__doc__)
 parser.add_argument('--port',type=int,default=5811)
 parser.add_argument('--hal-port',type=int,default=3301)
 parser.add_argument('--output',type=Path,required=True)
 asyncio.run(main(parser.parse_args()))
