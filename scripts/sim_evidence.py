"""Identify the actual Linux launcher snapshot, even after a concurrent rebuild."""
from pathlib import Path


def running_sim_binary():
    running = []
    for entry in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            command = entry.read_bytes().split(b'\0')[0].decode()
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
        if '/base-bot-halsim.' in command and command.endswith('/frcUserProgram'):
            running.append(Path(command))
    if len(running) != 1:
        raise RuntimeError('Expected one task-owned desktop simulator')
    return running[0]
