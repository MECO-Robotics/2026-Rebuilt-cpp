#!/usr/bin/env python3
"""Check truth-backed localization on the localhost field-enabled simulator."""
import asyncio
import json
import math
from pathlib import Path

import ntcore
from websockets.asyncio.client import connect
from sim_evidence import running_sim_binary


async def main():
    running_sim_binary()  # Refuse mismatched simulator endpoints.
    client = ntcore.NetworkTableInstance.create()
    client.startClient4('localization-smoke')
    client.setServer('127.0.0.1', 5810)
    root = '/SmartDashboard/Localization/'
    aligned = client.getBooleanTopic(root + 'QuestAligned').subscribe(False)
    connected = client.getBooleanTopic(root + 'LimelightConnected').subscribe(False)
    tags = client.getDoubleTopic(root + 'LimelightTagCount').subscribe(0)
    error = client.getDoubleTopic(root + 'TruthErrorMetres').subscribe(float('nan'))
    seen_tags = False
    async with connect('ws://127.0.0.1:3300/wpilibws') as ws:
        async def drain():
            async for _ in ws:
                pass
        reader = asyncio.create_task(drain())

        async def hold(seconds, forward=0., turn=0., reset=False, enabled=True):
            nonlocal seen_tags
            for _ in range(round(seconds / .02)):
                buttons = [False] * 10
                buttons[6] = reset  # Back/View resets the scene.
                await ws.send(json.dumps({'type': 'Joystick', 'device': '0', 'data': {
                    '>axes': [0, -forward, 0, 0, turn, 0], '>buttons': buttons, '>povs': [-1]}}))
                await ws.send(json.dumps({'type': 'DriverStation', 'device': '', 'data': {
                    '>enabled': enabled, '>autonomous': False, '>test': False,
                    '>ds': True, '>station': 'blue1', '>new_data': True}}))
                seen_tags |= tags.get() > 0
                await asyncio.sleep(.02)
        try:
            await hold(.5, enabled=False)
            await hold(.1, reset=True)
            await hold(1)
            await hold(8, turn=.25)
            await hold(1)
            await hold(2, forward=.15)
            await hold(1)
            result = {'limelight_connected': connected.get(), 'saw_limelight_tags': seen_tags,
                      'quest_aligned': aligned.get(), 'translation_error_metres': error.get()}
            Path('artifacts/localization-smoke.json').write_text(json.dumps(result, indent=2) + '\n')
            print(json.dumps(result, indent=2))
            assert client.isConnected() and result['limelight_connected']
            assert seen_tags and result['quest_aligned']
            assert math.isfinite(result['translation_error_metres']) and result['translation_error_metres'] < .2
        finally:
            await hold(.2, enabled=False)
            reader.cancel()
            client.stopClient()
            ntcore.NetworkTableInstance.destroy(client)


if __name__ == '__main__':
    asyncio.run(main())
