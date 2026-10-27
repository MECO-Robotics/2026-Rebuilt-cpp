#!/usr/bin/env python3
"""Check Remy pose telemetry through local HAL/NT4; always disable in cleanup."""
import asyncio,json,math,time
from pathlib import Path
import ntcore
from wpimath.geometry import Pose3d
from websockets.asyncio.client import connect
async def main():
 nt=ntcore.NetworkTableInstance.create();nt.startClient4('remy-port-validation');nt.setServer('127.0.0.1',5810)
 robot=nt.getStructTopic('/Visualization/RobotRemy/RobotPose',Pose3d).subscribe(Pose3d())
 comp=nt.getStructArrayTopic('/Visualization/RobotRemy/ComponentPoses',Pose3d).subscribe([])
 fuel=nt.getStructArrayTopic('/FieldSimulation/FuelPositions',Pose3d).subscribe([])
 rack=nt.getDoubleTopic('/SmartDashboard/IntakeRack/Position').subscribe(float('nan'))
 async with connect('ws://127.0.0.1:3300/wpilibws',max_size=None) as ws:
  async def drain():
   async for _ in ws:pass
  task=asyncio.create_task(drain())
  async def control(enable,acquire=False):
   await ws.send(json.dumps({'type':'Joystick','device':'0','data':{'>axes':[0,0,0,.8 if acquire else 0,0,0],'>buttons':[False]*10,'>povs':[-1]}}))
   await ws.send(json.dumps({'type':'DriverStation','device':'','data':{'>enabled':enable,'>autonomous':False,'>test':False,'>estop':False,'>fms':False,'>ds':True,'>new_data':True}}))
  try:
   for _ in range(60):await control(False);await asyncio.sleep(.02)
   initial=comp.get();assert len(initial)==6
   for _ in range(180):await control(True,True);await asyncio.sleep(.02)
   poses=comp.get();extension=rack.get()
   result={'component_count':len(poses),'rack_metres':extension,'rack_component_x':poses[2].X(),'rack_component_z':poses[2].Z(),'floor_and_hopper_fuel_poses':len(fuel.get()),'robot_xyz':[robot.get().X(),robot.get().Y(),robot.get().Z()]}
   assert extension>.26 and abs(poses[2].X()-extension*math.cos(math.radians(7.5)))<.005
   assert abs(poses[2].Z()+extension*math.sin(math.radians(7.5)))<.005
   assert poses[2].X()>initial[2].X()+.1 and len(fuel.get())>0
   result['passed']=True;Path('artifacts/remy-visualization-validation.json').write_text(json.dumps(result,indent=2)+'\n');print(result)
  finally:
   for _ in range(10):await control(False);await asyncio.sleep(.02)
   task.cancel()
 nt.stopClient()
asyncio.run(main())
