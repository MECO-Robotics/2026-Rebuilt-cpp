#!/usr/bin/env python3
"""Select and run Choreo in the local native HAL simulator; never contacts a roboRIO."""
import argparse
import asyncio
import contextlib
import json
import math
from pathlib import Path
import time
import ntcore
from websockets.asyncio.client import connect


async def run():
    result={"transport":"127.0.0.1 HAL websocket + NT4","checks":{}}
    instance=ntcore.NetworkTableInstance.create()
    instance.startClient4("season-cpp-auto-smoke");instance.setServer("127.0.0.1",5810)
    table=instance.getTable("SmartDashboard")
    pose=table.getDoubleArrayTopic("Drive/Pose").subscribe([])
    active=table.getStringTopic("Auto Choices/active").subscribe("")
    selection=table.getStringTopic("Auto Choices/selected").publish()
    name="Choreo LeftBlueBump + Shoot"
    async with connect("ws://127.0.0.1:3300/wpilibws",max_size=None) as ws:
        async def drain():
            async for _ in ws:pass
        reader=asyncio.create_task(drain())
        async def control(enabled):
            for port in ["0","1"]:
                await ws.send(json.dumps({"type":"Joystick","device":port,"data":{">axes":[0]*6,">buttons":[False]*10,">povs":[-1]}}))
            await ws.send(json.dumps({"type":"DriverStation","device":"","data":{">enabled":enabled,">autonomous":True,">test":False,">ds":True,">fms":False,">estop":False,">new_data":True}}))
        try:
            selection.set(name);instance.flush()
            for _ in range(50):await control(False);await asyncio.sleep(.02)
            selection.set(name);instance.flush()
            initial=None;endpoint=None
            started=time.monotonic()
            while time.monotonic()-started<6.5:
                await control(True)
                values=list(pose.get());elapsed=time.monotonic()-started
                if len(values)==3:
                    if .1<elapsed<.2:initial=values
                    if 3.1<elapsed<3.3:endpoint=values
                await asyncio.sleep(.02)
            result["selected"]=active.get();result["initialPose"]=initial;result["endPose"]=endpoint;result["finalPose"]=list(pose.get())
            # 2026 reflection is a half-turn about the official field center.
            expected=[(3.2206907272338867,5.213726997375488),(16.541-3.2206907272338867,8.0692-5.213726997375488)]
            error=min(math.hypot(endpoint[0]-x,endpoint[1]-y) for x,y in expected) if endpoint else float("inf")
            result["endpointErrorMeters"]=error
            result["checks"]={"chooser_selected_choreo":active.get()==name,"trajectory_endpoint_tracking":error<.4,"autonomous_pose_reset":initial is not None and initial[0]>2}
            result["passed"]=all(result["checks"].values())
        finally:
            with contextlib.suppress(Exception):await control(False)
            reader.cancel()
            with contextlib.suppress(asyncio.CancelledError,Exception):await reader
    instance.stopClient();return result


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument("--output",type=Path);args=parser.parse_args()
    try:result=asyncio.run(run())
    except Exception as exception:result={"passed":False,"error":str(exception)}
    rendered=json.dumps(result,indent=2);print(rendered)
    if args.output:args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(rendered+"\n")
    raise SystemExit(0 if result.get("passed") else 1)
