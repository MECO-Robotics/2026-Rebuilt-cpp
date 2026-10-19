#!/usr/bin/env python3
"""Exercise the local native season simulator's actual controller bindings and motor IO."""
import argparse
import asyncio
import contextlib
import json
import math
from pathlib import Path
import statistics
import time
import ntcore
from websockets.asyncio.client import connect


async def run():
    result={"transport":"127.0.0.1 HAL websocket + NT4","samples":{},"checks":{}}
    client=ntcore.NetworkTableInstance.create()
    client.startClient4("season-cpp-mechanism-smoke")
    client.setServer("127.0.0.1",5810)
    table=client.getTable("SmartDashboard")
    names=["IntakeRack/Position","IntakeRoller/Velocity","Conveyor/Velocity","BottomIndexer/Velocity","TopIndexer/Velocity","ShooterFlywheel/Velocity","ShooterFlywheel/Target","IntakeRoller/Voltage","Conveyor/Voltage","BottomIndexer/Voltage","TopIndexer/Voltage"]
    topics={n:table.getDoubleTopic(n).subscribe(float("nan")) for n in names}
    fuel=table.getDoubleArrayTopic("FieldSimulation/FuelPositionsXYZ").subscribe([])
    async with connect("ws://127.0.0.1:3300/wpilibws",max_size=None) as ws:
        async def drain():
            async for _ in ws:
                pass
        reader=asyncio.create_task(drain())
        async def control(enabled=True,button=None,intake=False):
            buttons=[False]*10
            if button is not None: buttons[button-1]=True
            await ws.send(json.dumps({"type":"Joystick","device":"0","data":{">axes":[0,0,0,.8 if intake else 0,0,0],">buttons":buttons,">povs":[-1]}}))
            await ws.send(json.dumps({"type":"DriverStation","device":"","data":{">enabled":enabled,">autonomous":False,">test":False,">estop":False,">fms":False,">ds":True,">new_data":True}}))
        async def phase(name,seconds,**controls):
            start=time.monotonic();samples=[]
            while time.monotonic()-start<seconds:
                await control(**controls)
                values={n:t.get() for n,t in topics.items()}
                if all(math.isfinite(v) for v in values.values()):samples.append(values)
                await asyncio.sleep(.02)
            if not samples:raise RuntimeError("Missing mechanism telemetry during "+name)
            tail=samples[-15:]
            median={n:statistics.median(s[n] for s in tail) for n in names}
            result["samples"][name]=median
            return median
        try:
            for _ in range(50):await control(False);await asyncio.sleep(.02)
            await phase("hub_button",.15,button=3)
            hub=await phase("hub_latched",3)
            acquire=await phase("acquire",3,intake=True)
            released=await phase("acquire_released",1.5)
            await phase("idle_button",.15,button=2)
            idle=await phase("idle",2)
            disabled=await phase("disabled",1,enabled=False)
            result["checks"]={
                "hub_preset_target":abs(hub["ShooterFlywheel/Target"]-30)<1e-6,
                "hub_velocity_tracks":abs(hub["ShooterFlywheel/Velocity"]-30)<3,
                "intake_rack_deployed":abs(acquire["IntakeRack/Position"]-.3)<.04,
                "acquire_roller_direction":acquire["IntakeRoller/Velocity"]< -1,
                "acquire_storage_directions":acquire["Conveyor/Velocity"]>1 and acquire["BottomIndexer/Velocity"]>1 and acquire["TopIndexer/Velocity"]< -1,
                "acquire_release_stops_outputs":max(abs(released[n]) for n in ["IntakeRoller/Voltage","Conveyor/Voltage","BottomIndexer/Voltage","TopIndexer/Voltage"])<.001,
                "idle_stops_shooter":abs(idle["ShooterFlywheel/Velocity"])<2,
                "disabled_stops_shooter":abs(disabled["ShooterFlywheel/Velocity"])<2,
                "fuel_model_publishes":len(fuel.get())>0,
            }
            result["passed"]=all(result["checks"].values())
        finally:
            with contextlib.suppress(Exception):await control(False)
            reader.cancel()
            with contextlib.suppress(asyncio.CancelledError,Exception):await reader
    client.stopClient()
    return result


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path)
    args=parser.parse_args()
    try:result=asyncio.run(run())
    except Exception as exception:result={"passed":False,"error":str(exception)}
    rendered=json.dumps(result,indent=2);print(rendered)
    if args.output:args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(rendered+"\n")
    raise SystemExit(0 if result.get("passed") else 1)
