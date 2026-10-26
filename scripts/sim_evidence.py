"""Identify the Linux launcher snapshot owning both validation endpoints."""
from pathlib import Path
import re
import subprocess


def running_sim_binary(nt_port=5810, hal_port=3300):
    owners = {nt_port: set(), hal_port: set()}
    for row in subprocess.check_output(['ss', '-ltnpH'], text=True).splitlines():
        columns = row.split()
        if len(columns) < 5:
            continue
        try:
            port = int(columns[3].rsplit(':', 1)[1])
        except (ValueError, IndexError):
            continue
        if port in owners:
            owners[port].update(int(pid) for pid in re.findall(r'pid=(\d+)', row))
    if len(owners[hal_port]) != 1 or owners[nt_port] != owners[hal_port]:
        raise RuntimeError('HAL and NetworkTables must belong to the same task-owned simulator')
    pid = next(iter(owners[hal_port]))
    command = Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')[0].decode()
    if '/base-bot-halsim.' not in command or not command.endswith('/frcUserProgram'):
        raise RuntimeError('Expected a task-owned desktop launcher snapshot')
    return Path(command)
